#include "safety_shield/obstacle_reach.h"

namespace safety_shield {

ObstacleReach::ObstacleReach(int n_obstacles, 
      std::vector<double>& pos,
      std::vector<double>& radius,
      std::vector<bool>& moves,
      double max_v,
      double measurement_error_pos, 
      double measurement_error_vel, 
      double delay):
  n_obstacles_(n_obstacles),
  measurement_error_pos_(measurement_error_pos),
  measurement_error_vel_(measurement_error_vel),
  delay_(delay)
{
  reach_lib::System system(measurement_error_pos, measurement_error_vel, delay);
  
  for (int i = 0; i < n_obstacles; i++) {
    obstacle_p_.push_back(reach_lib::PedestrianVel(system, 0.1, radius[i], moves[i]*max_v));
    joint_pos_.push_back(reach_lib::Point(pos[2*i], pos[2*i+1], 0.0));
    joint_vel_.push_back(reach_lib::Point(0.0, 0.0, 0.0));
  }
}

void ObstacleReach::reset() {
  last_meas_timestep_ = -1;
  for (int i = 0; i < n_obstacles_; i++) {
    joint_pos_.push_back(reach_lib::Point(0.0, 0.0, 0.0));
    joint_vel_.push_back(reach_lib::Point(0.0, 0.0, 0.0));
  }
}

void ObstacleReach::measurement(const std::vector<reach_lib::Point>& obstacle_pos, double time) {
  try {
    if (last_meas_timestep_ != -1) {
      double dt = time - last_meas_timestep_;
      for (int i = 0; i < obstacle_pos.size(); i++) {
        // If more than 1 measurement, calculate velocity
        joint_vel_[i] = (obstacle_pos[i] - joint_pos_[i]) * (1/dt);
      } 
      has_second_meas_ = true;
    }
    joint_pos_ = obstacle_pos;
    last_meas_timestep_ = time;
    //ROS_INFO_STREAM("Obstacle Mocap measurement received. Timestamp of meas was " << last_meas_timestep);
  } catch (const std::exception &exc) {
    spdlog::error("Exception in ObstacleReach::measurement: {}", exc.what());
  }
}


void ObstacleReach::obstacleReachabilityAnalysis(double t_command, double t_brake) {
  try {
    // Time between reach command msg and last measurement plus the t_brake time.
    double t_reach = t_command-last_meas_timestep_ + t_brake;
    // Calculate reachable set
    for(int i=0; i < obstacle_p_.size(); i++)
      obstacle_p_[i].update(0.0, t_reach, {joint_pos_[i]}, {joint_vel_[i]});
  } catch (const std::exception &exc) {
      spdlog::error("Exception in ObstacleReach::obstacleReachabilityAnalysis: {}", exc.what());
  }
}

} // namespace safety_shield


