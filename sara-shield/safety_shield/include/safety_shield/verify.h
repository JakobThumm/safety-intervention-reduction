// -*- lsst-c++ -*/
/**
 * @file verify.h
 * @brief Defines the abstract verify class
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

#include <algorithm>
#include <vector>

#include "reach_lib.hpp"

#ifndef VERIFY_H
#define VERIFY_H

namespace safety_shield {

/**
 * @brief Verifies if a given robot motion is safe with respect to a obstacles motion
 */
class Verify {
 public:
  /**
   * @brief A basic VerifyISO constructor
   */
  Verify() {}

  /**
   * @brief Check two cylinder for collision
   * 
   * @param[in] cap1 Cylinder 1
   * @param[in] cap2 Cylinder 2
   * 
   * @returns true if cylinder collide, false else
   */
  inline bool cylinderCollisionCheck(const reach_lib::Cylinder& cap1, const reach_lib::Cylinder& cap2) {
    return reach_lib::intersections::cylinder_cylinder_intersection(cap2, cap1);
  }

  inline bool cylinderCapsuleCollisionCheck(const reach_lib::Cylinder& cyl1, const reach_lib::Capsule& cap2) {
    return reach_lib::intersections::cylinder_capsule_intersection(cyl1, cap2);
  }
  
  /**
   * @brief Verify the robot motion against the reachable occupancy of the obstacle in position, velocity, and acceleration
   * 
   * Pure virtual function.
   * 
   * @param[in] robot_capsules Reachable capsules of the robot
   * @param[in] obstacle_cylinder List of list of cylinder. Each list of cylinder corresponds to a obstacle reachable set model.
   * 
   * @returns Whether the robot movement is unsafe for the obstacle
   */
  virtual bool verify_obstacle_reach(const std::vector<reach_lib::Capsule>& robot_capsules, 
      std::vector<reach_lib::Cylinder> obstacle_cylinders) = 0;
};
} // namespace safety_shield

#endif // VERIFY_H
