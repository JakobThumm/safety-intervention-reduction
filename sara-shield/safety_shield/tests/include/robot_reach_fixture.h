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

#include "safety_shield/robot_reach.h"

#ifndef ROBOT_REACH_FIXTURE_H
#define ROBOT_REACH_FIXTURE_H

namespace safety_shield {

/**
 * @brief Test fixture for robot reach class
 */
class RobotReachTest : public ::testing::Test {
 protected:
  /**
   * @brief The robot reach object
   */
  RobotReach* robot_reach_;

  /**
   * @brief Create the robot reach object
   */
  void SetUp() override {
    std::filesystem::path config_file = std::filesystem::current_path().parent_path() / "config/robot_reach_test_single_joint.yaml";
    YAML::Node robot_config = YAML::LoadFile(config_file.string());
    std::string robot_name = robot_config["robot_name"].as<std::string>();
    double nb_joints = robot_config["nb_joints"].as<int>();
    std::vector<double> transformation_matrices = robot_config["transformation_matrices"].as<std::vector<double>>();
    std::vector<double> enclosures = robot_config["enclosures"].as<std::vector<double>>();
    double secure_radius = robot_config["secure_radius"].as<double>();
    robot_reach_ = new RobotReach(transformation_matrices, 
      nb_joints, 
      enclosures, 
      0.0, 0.0, 1.0, 
      0.0, 0.0, 0.0,
      secure_radius);
  }
};
} // namespace safety_shield

#endif // ROBOT_REACH_FIXTURE_H    