// -*- lsst-c++ -*/
/**
 * @file verify_iso.h
 * @brief Defines the verify ISO class
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

#include "safety_shield/verify.h"

#include <algorithm>
#include <vector>

#include "spdlog/spdlog.h" // https://github.com/gabime/spdlog

#include "reach_lib.hpp"

#ifndef VERIFY_ISO_H
#define VERIFY_ISO_H

namespace safety_shield {

/**
 * @brief Verifies if a given robot motion is safe with respect to obstacle motions
 */
class VerifyISO : public Verify {
 public:
  /**
   * @brief A basic VerifyISO constructor
   */
  VerifyISO() : Verify() {}

  /**
   * @brief Check a set of robot capsules if they collide with a set of obstacle cylinder
   * 
   * @param[in] robot_capsules The robot capsules
   * @param[in] obstacle_cylinder The obstacle occupancy cylinder
   * 
   * @returns Whether a collision between any two cylinder of the robot and obstacle set occured
   */
  bool robotObstacleCollision(const std::vector<reach_lib::Capsule>& robot_capsules, 
      const std::vector<reach_lib::Cylinder>& obstacle_cylinder);

  /**
   * @brief Verify the robot motion againt the reachability analysis of the obstacle in position, velocity, and acceleration
   * 
   * @param[in] robot_capsules Reachable capsules of the robot
   * @param[in] obstacle_cylinder List of list of cylinder. Each list of cylinder corresponds to a obstacle reachable set model.
   * 
   * @returns True: if the robot cylinder do not collide with one set of the obstacle cylinder, i.e., the motion is safe. 
   *          False: Otherwise
   */
  bool verify_obstacle_reach(const std::vector<reach_lib::Capsule>& robot_capsules, 
      std::vector<reach_lib::Cylinder> obstacle_cylinders);
};
} // namespace safety_shield

#endif // VERIFY_ISO_H
