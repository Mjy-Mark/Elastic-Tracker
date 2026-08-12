import math
import unittest

import torch

from elastic_tracker_controller import CtbrControllerGains, ctbr_controller, geometric_controller


class GeometricControllerTest(unittest.TestCase):
    def setUp(self):
        self.zeros = torch.zeros(1, 3)
        self.identity = torch.tensor([[1.0, 0.0, 0.0, 0.0]])

    def test_hover_force(self):
        force, torque = geometric_controller(
            self.zeros,
            self.identity,
            self.zeros,
            self.zeros,
            self.zeros,
            self.zeros,
            self.zeros,
            torch.zeros(1),
            mass=0.5,
        )
        torch.testing.assert_close(force, torch.tensor([[0.0, 0.0, 4.905]]))
        torch.testing.assert_close(torque, self.zeros)

    def test_position_error_increases_thrust(self):
        force, _ = geometric_controller(
            self.zeros,
            self.identity,
            self.zeros,
            self.zeros,
            torch.tensor([[0.0, 0.0, 1.0]]),
            self.zeros,
            self.zeros,
            torch.zeros(1),
            mass=1.0,
        )
        self.assertGreater(force[0, 2].item(), 9.81)

    def test_horizontal_error_generates_pitch_torque(self):
        _, torque = geometric_controller(
            self.zeros,
            self.identity,
            self.zeros,
            self.zeros,
            torch.tensor([[1.0, 0.0, 0.0]]),
            self.zeros,
            self.zeros,
            torch.zeros(1),
            mass=1.0,
        )
        self.assertGreater(abs(torque[0, 1].item()), 1.0e-4)

    def test_yaw_reference_generates_yaw_torque(self):
        _, torque = geometric_controller(
            self.zeros,
            self.identity,
            self.zeros,
            self.zeros,
            self.zeros,
            self.zeros,
            self.zeros,
            torch.tensor([math.pi / 2.0]),
            mass=1.0,
        )
        self.assertGreater(torque[0, 2].item(), 0.0)


class CtbrControllerTest(unittest.TestCase):
    def setUp(self):
        self.zeros = torch.zeros(1, 3)
        self.identity = torch.tensor([[1.0, 0.0, 0.0, 0.0]])

    def command(
        self,
        desired_position=None,
        desired_yaw=0.0,
        desired_acceleration=None,
        gains=CtbrControllerGains(),
        max_collective_thrust=3.0,
        max_body_rate=3.0,
    ):
        return ctbr_controller(
            position_w=self.zeros,
            quaternion_wxyz=self.identity,
            linear_velocity_w=self.zeros,
            desired_position_w=(
                self.zeros if desired_position is None else desired_position
            ),
            desired_velocity_w=self.zeros,
            desired_acceleration_w=(
                self.zeros if desired_acceleration is None else desired_acceleration
            ),
            desired_yaw=torch.tensor([desired_yaw]),
            gains=gains,
            max_collective_thrust=max_collective_thrust,
            max_body_rate=max_body_rate,
        )

    def test_hover_is_one_vehicle_weight(self):
        torch.testing.assert_close(
            self.command(), torch.tensor([[1.0, 0.0, 0.0, 0.0]])
        )

    def test_positive_x_error_commands_positive_pitch_rate(self):
        action = self.command(desired_position=torch.tensor([[1.0, 0.0, 0.0]]))
        self.assertGreater(action[0, 2].item(), 0.0)

    def test_positive_y_error_commands_negative_roll_rate(self):
        action = self.command(desired_position=torch.tensor([[0.0, 1.0, 0.0]]))
        self.assertLess(action[0, 1].item(), 0.0)

    def test_positive_yaw_error_commands_positive_yaw_rate(self):
        action = self.command(desired_yaw=math.pi / 2.0)
        self.assertGreater(action[0, 3].item(), 0.0)

    def test_action_limits(self):
        action = self.command(
            desired_position=torch.tensor([[100.0, 100.0, 100.0]]),
            desired_yaw=math.pi,
            max_collective_thrust=1.5,
            max_body_rate=(0.2, 0.3, 0.4),
        )
        self.assertLessEqual(action[0, 0].item(), 1.5)
        torch.testing.assert_close(
            action[0, 1:].abs().clamp_max(torch.tensor([0.2, 0.3, 0.4])),
            action[0, 1:].abs(),
        )


if __name__ == "__main__":
    unittest.main()
