// -*- lsst-c++ -*/
/**
 * @file human_reach_fixture.h
 * @brief Defines the test fixture for the human reach object
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

#include "safety_shield/human_reach.h"

#ifndef HUMAN_REACH_FIXTURE_H
#define HUMAN_REACH_FIXTURE_H

namespace safety_shield {

/**
 * @brief Test fixture for human reach class
 */
class HumanReachTest : public ::testing::Test {
 protected:
  /**
   * @brief The human reach object
   */
  HumanReach* human_reach_;

  /**
   * @brief Create the human reach object
   */
  void SetUp() override {
    std::filesystem::path config_file = std::filesystem::current_path().parent_path() / "config/human_reach_test_single_joint.yaml";
    YAML::Node human_config = YAML::LoadFile(config_file.string());
    double measurement_error_pos = human_config["measurement_error_pos"].as<double>();
    double measurement_error_vel = human_config["measurement_error_vel"].as<double>();
    double delay = human_config["delay"].as<double>();

    std::vector<std::string> joint_name_vec = human_config["joint_names"].as<std::vector<std::string>>();
    std::map<std::string, int> joint_names;
    for(std::size_t i = 0; i < joint_name_vec.size(); ++i) {
        joint_names[joint_name_vec[i]] = i;
    }

    std::vector<double> joint_v_max = human_config["joint_v_max"].as<std::vector<double>>();
    std::vector<double> joint_a_max = human_config["joint_a_max"].as<std::vector<double>>();
    // Build bodies
    const YAML::Node& bodies = human_config["bodies"];
    std::map<std::string, reach_lib::jointPair> body_link_joints;
    std::map<std::string, double> thickness;
    for (YAML::const_iterator it = bodies.begin(); it != bodies.end(); ++it) {
      const YAML::Node& body = *it;
      body_link_joints[body["name"].as<std::string>()] = reach_lib::jointPair(joint_names[body["proximal"].as<std::string>()], joint_names[body["distal"].as<std::string>()]);
      thickness[body["name"].as<std::string>()] = body["thickness"].as<double>(); 
    }
    // Build extremities
    const YAML::Node& extremities = human_config["extremities"];
    std::vector<std::string> extremity_base_names;
    std::vector<std::string> extremity_end_names; 
    std::vector<double> extremity_length;
    for (YAML::const_iterator it = extremities.begin(); it != extremities.end(); ++it) {
      const YAML::Node& extremity = *it;
      extremity_base_names.push_back(extremity["base"].as<std::string>());
      extremity_end_names.push_back(extremity["end"].as<std::string>());
      extremity_length.push_back(extremity["length"].as<double>());
    }
    human_reach_ = new HumanReach(joint_names.size(), 
      body_link_joints, 
      thickness, 
      joint_v_max, 
      joint_a_max,
      extremity_base_names, 
      extremity_end_names, 
      extremity_length,
      measurement_error_pos, 
      measurement_error_vel, 
      delay);
  }
};

/**
 * @brief Test fixture for human reach class with errors and delay
 */
class HumanReachTestError : public ::testing::Test {
 protected:
  /**
   * @brief The human reach object
   */
  HumanReach* human_reach_;

  /**
   * @brief Create the human reach object
   */
  void SetUp() override {
    std::filesystem::path config_file = std::filesystem::current_path().parent_path() / "config/human_reach_test_single_joint_error.yaml";
    YAML::Node human_config = YAML::LoadFile(config_file.string());
    double measurement_error_pos = human_config["measurement_error_pos"].as<double>();
    double measurement_error_vel = human_config["measurement_error_vel"].as<double>();
    double delay = human_config["delay"].as<double>();

    std::vector<std::string> joint_name_vec = human_config["joint_names"].as<std::vector<std::string>>();
    std::map<std::string, int> joint_names;
    for(std::size_t i = 0; i < joint_name_vec.size(); ++i) {
        joint_names[joint_name_vec[i]] = i;
    }

    std::vector<double> joint_v_max = human_config["joint_v_max"].as<std::vector<double>>();
    std::vector<double> joint_a_max = human_config["joint_a_max"].as<std::vector<double>>();
    // Build bodies
    const YAML::Node& bodies = human_config["bodies"];
    std::map<std::string, reach_lib::jointPair> body_link_joints;
    std::map<std::string, double> thickness;
    for (YAML::const_iterator it = bodies.begin(); it != bodies.end(); ++it) {
      const YAML::Node& body = *it;
      body_link_joints[body["name"].as<std::string>()] = reach_lib::jointPair(joint_names[body["proximal"].as<std::string>()], joint_names[body["distal"].as<std::string>()]);
      thickness[body["name"].as<std::string>()] = body["thickness"].as<double>(); 
    }
    // Build extremities
    const YAML::Node& extremities = human_config["extremities"];
    std::vector<std::string> extremity_base_names;
    std::vector<std::string> extremity_end_names; 
    std::vector<double> extremity_length;
    for (YAML::const_iterator it = extremities.begin(); it != extremities.end(); ++it) {
      const YAML::Node& extremity = *it;
      extremity_base_names.push_back(extremity["base"].as<std::string>());
      extremity_end_names.push_back(extremity["end"].as<std::string>());
      extremity_length.push_back(extremity["length"].as<double>());
    }
    human_reach_ = new HumanReach(joint_names.size(), 
      body_link_joints, 
      thickness, 
      joint_v_max, 
      joint_a_max,
      extremity_base_names, 
      extremity_end_names, 
      extremity_length,
      measurement_error_pos, 
      measurement_error_vel, 
      delay);
  }
};

/**
 * @brief Test fixture for human reach class for the position model
 */
class HumanReachTestPos : public ::testing::Test {
 protected:
  /**
   * @brief The human reach object
   */
  HumanReach* human_reach_;

  /**
   * @brief Create the human reach object
   */
  void SetUp() override {
    std::filesystem::path config_file = std::filesystem::current_path().parent_path() / "config/human_reach_test_arm_pos.yaml";
    YAML::Node human_config = YAML::LoadFile(config_file.string());
    double measurement_error_pos = human_config["measurement_error_pos"].as<double>();
    double measurement_error_vel = human_config["measurement_error_vel"].as<double>();
    double delay = human_config["delay"].as<double>();

    std::vector<std::string> joint_name_vec = human_config["joint_names"].as<std::vector<std::string>>();
    std::map<std::string, int> joint_names;
    for(std::size_t i = 0; i < joint_name_vec.size(); ++i) {
        joint_names[joint_name_vec[i]] = i;
    }

    std::vector<double> joint_v_max = human_config["joint_v_max"].as<std::vector<double>>();
    std::vector<double> joint_a_max = human_config["joint_a_max"].as<std::vector<double>>();
    // Build bodies
    const YAML::Node& bodies = human_config["bodies"];
    std::map<std::string, reach_lib::jointPair> body_link_joints;
    std::map<std::string, double> thickness;
    for (YAML::const_iterator it = bodies.begin(); it != bodies.end(); ++it) {
      const YAML::Node& body = *it;
      body_link_joints[body["name"].as<std::string>()] = reach_lib::jointPair(joint_names[body["proximal"].as<std::string>()], joint_names[body["distal"].as<std::string>()]);
      thickness[body["name"].as<std::string>()] = body["thickness"].as<double>(); 
    }
    // Build extremities
    const YAML::Node& extremities = human_config["extremities"];
    std::vector<std::string> extremity_base_names;
    std::vector<std::string> extremity_end_names; 
    std::vector<double> extremity_length;
    for (YAML::const_iterator it = extremities.begin(); it != extremities.end(); ++it) {
      const YAML::Node& extremity = *it;
      extremity_base_names.push_back(extremity["base"].as<std::string>());
      extremity_end_names.push_back(extremity["end"].as<std::string>());
      extremity_length.push_back(extremity["length"].as<double>());
    }
    human_reach_ = new HumanReach(joint_names.size(), 
      body_link_joints, 
      thickness, 
      joint_v_max, 
      joint_a_max,
      extremity_base_names, 
      extremity_end_names, 
      extremity_length,
      measurement_error_pos, 
      measurement_error_vel, 
      delay);
  }
};
} // namespace safety_shield

#endif // HUMAN_REACH_FIXTURE_H    