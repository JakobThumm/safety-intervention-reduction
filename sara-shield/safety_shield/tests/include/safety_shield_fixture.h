// -*- lsst-c++ -*/
/**
 * @file robot_reach_fixture.h
 * @brief Defines the test fixture for the robot reach object
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

#include <filesystem>
#include <vector>
#include <string>
#include <iostream>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>
#include "reach_lib.hpp"

#include "safety_shield/safety_shield.h"

#ifndef SAFETY_SHIELD_FIXTURE_H
#define SAFETY_SHIELD_FIXTURE_H

namespace safety_shield {

class SafetyShieldExposed : public SafetyShield {
 public:
  using SafetyShield::calculateMaxAccJerk;
  
  using SafetyShield::planSafetyShield;

  SafetyShieldExposed() {}

  explicit SafetyShieldExposed(bool activate_shield,
      double sample_time,
      std::string trajectory_config_file,
      std::string robot_config_file,
      std::string mocap_config_file,
      double init_x, 
      double init_y, 
      double init_z, 
      double init_roll, 
      double init_pitch, 
      double init_yaw,
      const std::vector<double> &init_qpos) : SafetyShield(
        activate_shield,
        sample_time, 
        trajectory_config_file,
        robot_config_file,
        mocap_config_file,
        init_x, 
        init_y, 
        init_z, 
        init_roll, 
        init_pitch, 
        init_yaw,
        init_qpos) {}
};

/**
 * @brief Test fixture for safety shield class
 */
class SafetyShieldTest : public ::testing::Test {
 protected:
  /**
   * @brief The safety shield object
   */
  SafetyShieldExposed shield_;

  /**
   * @brief Create the safety shield object
   */
  void SetUp() override {
    bool activate_shield = true;
    double sample_time = 0.001; 
    std::string trajectory_config_file = std::string("../../safety_shield/config/trajectory_parameters_schunk.yaml");
    std::string robot_config_file = std::string("../../safety_shield/config/robot_parameters_schunk.yaml");
    std::string mocap_config_file = std::string("../../safety_shield/config/cmu_mocap_no_hand.yaml");
    double init_x = 0.0;
    double init_y = 0.0;
    double init_z = 0.0;
    double init_roll = 0.0;
    double init_pitch = 0.0;
    double init_yaw = 0.0;
    std::vector<double> init_qpos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    shield_ = SafetyShieldExposed(activate_shield,
      sample_time, 
      trajectory_config_file,
      robot_config_file,
      mocap_config_file,
      init_x, 
      init_y, 
      init_z, 
      init_roll, 
      init_pitch, 
      init_yaw,
      init_qpos);
  }
};
} // namespace safety_shield

#endif // SAFETY_SHIELD_FIXTURE_H    