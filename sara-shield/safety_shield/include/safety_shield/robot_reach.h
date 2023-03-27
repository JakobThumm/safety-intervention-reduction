// -*- lsst-c++ -*-
/**
 * @file robot_reach.h
 * @brief Define the class for robot reachability analysis
 * @version 0.1
 * @copyright This file is part of SaRA-Shield.
 * SaRA-Shield is free software: you can redistribute it and/or modify it under 
 * the terms of the GNU General Public License as published by the Free Software Foundation, 
 * either version 3 of the License, or (at your option) any later version.
 * SaRA-Shield is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 * See the GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License along with SaRA-Shield. 
 * If not, see <https://www.gnu.org/licenses/>. 
 */

#include <vector>
#include <algorithm>
#include <exception>

#include <Eigen/Dense>
#include "spdlog/spdlog.h" 

#include "reach_lib.hpp"

#include "safety_shield/motion.h"

#ifndef ROBOT_REACH_H
#define ROBOT_REACH_H

namespace safety_shield {

/**
 * @brief Class that calculates the robot reachable sets.
 */
class RobotReach {
 private:
  /**
   * @brief the number of joints of the robot
   */
  int nb_joints_;

  /**
   * @brief Expands the radius of the robot cylinder by this amount to
   *  account for measurement and modelling errors.
   */
  double secure_radius_;
  double radius_robot_;

  /**
   * @brief List of transforamtion matrices from joint to joint (fixed description, not including joint movements)
   */
  std::vector<Eigen::Matrix4d> transformation_matrices_;

  /**
   * @brief The enclosing cylinder 
   */
  std::vector<reach_lib::Cylinder> robot_cylinders_;

public:

  /**
   * @brief A robot empty constructor
   */
  RobotReach() {}
  
  /**
   * @brief A robot basic constructor 
   *
   * @param transformation_matrices the transformation matrices
   * @param nb_joints the number of joints of the robot
   * @param geom_param the robot occupancy matrix
   * @param x initial x position of base
   * @param y initial y position of base
   * @param z initial z position of base
   * @param roll initial roll of base
   * @param pitch initial pitch of base
   * @param yaw initial yaw of base
   * @param secure_radius Expand the radius of the robot cylinder by this amount to
   *  account for measurement and modelling errors.
   */
  RobotReach(double x, double y, double z, 
      double roll, double pitch, double yaw,
      double radius, double secure_radius);

  /**
   *  @brief A robot destructor
   */
  ~RobotReach() {}

  /**
   * @brief Reset the robot reach object.
   * 
   * @param x initial x position of base
   * @param y initial y position of base
   * @param z initial z position of base
   * @param roll initial roll of base
   * @param pitch initial pitch of base
   * @param yaw initial yaw of base
   */
  void reset(double x, double y, double z, 
      double roll, double pitch, double yaw);
    
  /**
   * @brief Computes the global transformation matrix of a given joint.
   *
   * @param q The joint angle
   * @param n_joint The number of joint 
   * @param T The current transformation matrix (Start with Identity). T will be modified in this function.
   */
  inline void forwardKinematic(const double &qx, const double &qy, const int& n_joint, Eigen::Matrix4d &T) {
    // Transform T to new joint coordinate system
    T = T * transformation_matrices_[n_joint+1];
    Eigen::Matrix4d Rz;
    Rz << 1, 0, 0, qx,
          0, 1, 0, qy,
          0, 0, 1, 0,
          0, 0, 0, 1;
    T = T * Rz;  
  }

  Eigen::Vector4d pointToVector(const reach_lib::Point& p) {
      Eigen::Vector4d vec;
      vec << p.x, p.y, p.z, 1.0;
      return vec;
  }

  reach_lib::Point vectorToPoint(const Eigen::Vector4d& vec) {
      return reach_lib::Point(vec(0), vec(1), vec(2));
  }

  /**
   * @brief Transform the cylinder of joint n by the transformation matrix T.
   * @return the transformed cylinder
   */
  reach_lib::Cylinder transformCylinder(const int& n_joint, const Eigen::Matrix4d &T);

  /**
   * @brief Calculates the reachable set from the new desired start and goal joint position.
   * 
   * Computes the reachable occupancy cylinder of the robot.
   * For a detailed proof of formality, please see: http://mediatum.ub.tum.de/doc/1443612/652879.pdf Chapter 3.4
   * 
   * @param[in] start_config The configuration of the robot in the beginning of the trajectory
   * @param[in] goal_config The configuration of the robot in the end of the trajectory
   * @param[in] s_diff The difference in the trajectory time parameter s for the given path
   * @param[in] alpha_i The maximum acceleration of each cylinder point with resprect to the time parameter s for the given path
   * 
   * @returns Array of cylinder
   */
  std::vector<reach_lib::Cylinder> reach(Motion& start_config, Motion& goal_config,
    double s_diff, std::vector<double> alpha_i);

  std::vector<reach_lib::Capsule> reach_path(std::vector<Motion> path, int start_on_path);

};
} // namespace safety_shield 

#endif // ROBOT_REACH_H
