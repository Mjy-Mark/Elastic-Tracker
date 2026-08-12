#include <traj_opt/traj_opt.h>

#include <Eigen/Core>
#include <iostream>
#include <vector>

namespace {

Eigen::MatrixXd makeBox(const Eigen::Vector3d& minimum,
                        const Eigen::Vector3d& maximum) {
  Eigen::MatrixXd box(6, 6);
  box.col(0) << 1.0, 0.0, 0.0, maximum.x(), 0.0, 0.0;
  box.col(1) << -1.0, 0.0, 0.0, minimum.x(), 0.0, 0.0;
  box.col(2) << 0.0, 1.0, 0.0, 0.0, maximum.y(), 0.0;
  box.col(3) << 0.0, -1.0, 0.0, 0.0, minimum.y(), 0.0;
  box.col(4) << 0.0, 0.0, 1.0, 0.0, 0.0, maximum.z();
  box.col(5) << 0.0, 0.0, -1.0, 0.0, 0.0, minimum.z();
  return box;
}

}  // namespace

int main() {
  traj_opt::TrajOptConfig config;
  config.max_velocity = 3.0;
  config.max_acceleration = 6.0;
  traj_opt::TrajOpt optimizer(config);

  Eigen::Matrix3d initial_state = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d final_state = Eigen::Matrix3d::Zero();
  initial_state.col(0) << 0.0, 0.0, 1.0;
  final_state.col(0) << 2.0, 0.0, 1.0;

  std::vector<Eigen::MatrixXd> corridors{
      makeBox(Eigen::Vector3d(-1.0, -2.0, 0.1),
              Eigen::Vector3d(3.0, 2.0, 2.5))};
  Trajectory trajectory;
  if (!optimizer.generate_traj(initial_state, final_state, corridors, trajectory)) {
    std::cerr << "trajectory optimization failed" << std::endl;
    return 1;
  }
  if ((trajectory.getPos(0.0) - initial_state.col(0)).norm() > 1e-6) {
    std::cerr << "trajectory initial state mismatch" << std::endl;
    return 2;
  }
  if ((trajectory.getPos(trajectory.getTotalDuration()) - final_state.col(0)).norm() > 1e-5) {
    std::cerr << "trajectory final state mismatch" << std::endl;
    return 3;
  }

  std::cout << "trajectory pieces=" << trajectory.getPieceNum()
            << " duration=" << trajectory.getTotalDuration() << std::endl;
  return 0;
}
