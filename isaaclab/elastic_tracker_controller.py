"""Batched controllers for tracking Elastic-Tracker references."""

from __future__ import annotations

from dataclasses import dataclass

import torch


@dataclass
class ControllerGains:
    position: tuple[float, float, float] = (4.0, 4.0, 6.0)
    velocity: tuple[float, float, float] = (3.0, 3.0, 4.0)
    attitude: tuple[float, float, float] = (0.12, 0.12, 0.08)
    angular_rate: tuple[float, float, float] = (0.01, 0.01, 0.01)


@dataclass(frozen=True)
class CtbrControllerGains:
    """Gains for the position-reference to CTBR outer loop."""

    position: tuple[float, float, float] = (4.0, 4.0, 6.0)
    velocity: tuple[float, float, float] = (3.0, 3.0, 4.0)
    attitude: tuple[float, float, float] = (4.0, 4.0, 2.0)


def quaternion_to_matrix(quaternion_wxyz: torch.Tensor) -> torch.Tensor:
    """Convert normalized WXYZ quaternions to rotation matrices."""
    quaternion_wxyz = torch.nn.functional.normalize(quaternion_wxyz, dim=-1)
    real, i, j, k = torch.unbind(quaternion_wxyz, dim=-1)
    two = 2.0
    return torch.stack(
        (
            1.0 - two * (j * j + k * k),
            two * (i * j - k * real),
            two * (i * k + j * real),
            two * (i * j + k * real),
            1.0 - two * (i * i + k * k),
            two * (j * k - i * real),
            two * (i * k - j * real),
            two * (j * k + i * real),
            1.0 - two * (i * i + j * j),
        ),
        dim=-1,
    ).reshape(quaternion_wxyz.shape[:-1] + (3, 3))


def _vee(skew_matrix: torch.Tensor) -> torch.Tensor:
    return torch.stack(
        (
            skew_matrix[..., 2, 1],
            skew_matrix[..., 0, 2],
            skew_matrix[..., 1, 0],
        ),
        dim=-1,
    )


def _desired_rotation(
    commanded_acceleration_w: torch.Tensor, desired_yaw: torch.Tensor
) -> torch.Tensor:
    body_z_desired = torch.nn.functional.normalize(
        commanded_acceleration_w, dim=-1, eps=1.0e-6
    )
    yaw_heading = torch.stack(
        (
            torch.cos(desired_yaw),
            torch.sin(desired_yaw),
            torch.zeros_like(desired_yaw),
        ),
        dim=-1,
    )
    body_y_desired = torch.cross(body_z_desired, yaw_heading, dim=-1)
    degenerate = (
        torch.linalg.vector_norm(body_y_desired, dim=-1, keepdim=True) < 1.0e-6
    )
    fallback_x = torch.zeros_like(yaw_heading)
    fallback_x[..., 0] = 1.0
    fallback_y = torch.zeros_like(yaw_heading)
    fallback_y[..., 1] = 1.0
    fallback_heading = torch.where(
        (body_z_desired[..., :1].abs() < 0.9), fallback_x, fallback_y
    )
    fallback_body_y = torch.cross(body_z_desired, fallback_heading, dim=-1)
    body_y_desired = torch.where(degenerate, fallback_body_y, body_y_desired)
    body_y_desired = torch.nn.functional.normalize(
        body_y_desired, dim=-1, eps=1.0e-6
    )
    body_x_desired = torch.cross(body_y_desired, body_z_desired, dim=-1)
    return torch.stack(
        (body_x_desired, body_y_desired, body_z_desired), dim=-1
    )


def ctbr_controller(
    position_w: torch.Tensor,
    quaternion_wxyz: torch.Tensor,
    linear_velocity_w: torch.Tensor,
    desired_position_w: torch.Tensor,
    desired_velocity_w: torch.Tensor,
    desired_acceleration_w: torch.Tensor,
    desired_yaw: torch.Tensor,
    desired_yaw_rate: torch.Tensor | None = None,
    gravity: float = 9.81,
    gains: CtbrControllerGains = CtbrControllerGains(),
    max_collective_thrust: float = 3.0,
    max_body_rate: float | tuple[float, float, float] = 3.0,
) -> torch.Tensor:
    """Convert world-frame p/v/a/yaw references to TrackingEnv CTBR actions.

    The output order is normalized collective thrust followed by desired body
    roll, pitch, and yaw rates in rad/s. A collective-thrust value of one is
    one vehicle weight, matching ``Tracking-Direct-v2``.
    """
    device = position_w.device
    dtype = position_w.dtype
    position_gain = torch.as_tensor(gains.position, device=device, dtype=dtype)
    velocity_gain = torch.as_tensor(gains.velocity, device=device, dtype=dtype)
    attitude_gain = torch.as_tensor(gains.attitude, device=device, dtype=dtype)

    position_error = desired_position_w - position_w
    velocity_error = desired_velocity_w - linear_velocity_w
    commanded_acceleration_w = (
        desired_acceleration_w
        + position_gain * position_error
        + velocity_gain * velocity_error
    )
    commanded_acceleration_w = commanded_acceleration_w.clone()
    commanded_acceleration_w[..., 2] += gravity

    rotation = quaternion_to_matrix(quaternion_wxyz)
    desired_rotation = _desired_rotation(commanded_acceleration_w, desired_yaw)
    rotation_error_matrix = torch.matmul(
        desired_rotation.transpose(-1, -2), rotation
    ) - torch.matmul(rotation.transpose(-1, -2), desired_rotation)
    attitude_error = 0.5 * _vee(rotation_error_matrix)
    desired_body_rates = -attitude_gain * attitude_error
    if desired_yaw_rate is not None:
        desired_body_rates[..., 2] += desired_yaw_rate

    max_rate = torch.as_tensor(max_body_rate, device=device, dtype=dtype)
    desired_body_rates = torch.maximum(
        torch.minimum(desired_body_rates, max_rate), -max_rate
    )
    collective_thrust = torch.sum(
        commanded_acceleration_w * rotation[..., :, 2], dim=-1
    ) / gravity
    collective_thrust = collective_thrust.clamp(0.0, max_collective_thrust)
    return torch.cat((collective_thrust.unsqueeze(-1), desired_body_rates), dim=-1)


def geometric_controller(
    position_w: torch.Tensor,
    quaternion_wxyz: torch.Tensor,
    linear_velocity_w: torch.Tensor,
    angular_velocity_b: torch.Tensor,
    desired_position_w: torch.Tensor,
    desired_velocity_w: torch.Tensor,
    desired_acceleration_w: torch.Tensor,
    desired_yaw: torch.Tensor,
    mass: torch.Tensor | float,
    gravity: float = 9.81,
    gains: ControllerGains = ControllerGains(),
    max_thrust: float | None = None,
    max_torque: float | None = None,
) -> tuple[torch.Tensor, torch.Tensor]:
    """Return body-frame force and torque commands for a Z-up multirotor."""
    device = position_w.device
    dtype = position_w.dtype
    batch_shape = position_w.shape[:-1]
    mass_tensor = torch.as_tensor(mass, device=device, dtype=dtype)
    if mass_tensor.ndim == 0:
        mass_tensor = mass_tensor.expand(batch_shape)
    else:
        mass_tensor = mass_tensor.reshape(batch_shape)

    position_gain = torch.tensor(gains.position, device=device, dtype=dtype)
    velocity_gain = torch.tensor(gains.velocity, device=device, dtype=dtype)
    attitude_gain = torch.tensor(gains.attitude, device=device, dtype=dtype)
    angular_rate_gain = torch.tensor(gains.angular_rate, device=device, dtype=dtype)

    position_error = desired_position_w - position_w
    velocity_error = desired_velocity_w - linear_velocity_w
    commanded_acceleration = (
        desired_acceleration_w
        + position_gain * position_error
        + velocity_gain * velocity_error
    )
    commanded_acceleration[..., 2] += gravity
    commanded_force_w = mass_tensor[..., None] * commanded_acceleration

    desired_rotation = _desired_rotation(commanded_force_w, desired_yaw)

    rotation = quaternion_to_matrix(quaternion_wxyz)
    rotation_error_matrix = torch.matmul(
        desired_rotation.transpose(-1, -2), rotation
    ) - torch.matmul(rotation.transpose(-1, -2), desired_rotation)
    attitude_error = 0.5 * _vee(rotation_error_matrix)
    torque_b = -attitude_gain * attitude_error - angular_rate_gain * angular_velocity_b

    thrust = torch.sum(commanded_force_w * rotation[..., :, 2], dim=-1).clamp_min(0.0)
    if max_thrust is not None:
        thrust = thrust.clamp_max(max_thrust)
    force_b = torch.zeros_like(commanded_force_w)
    force_b[..., 2] = thrust
    if max_torque is not None:
        torque_b = torque_b.clamp(min=-max_torque, max=max_torque)
    return force_b, torque_b
