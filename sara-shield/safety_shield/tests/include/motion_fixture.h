// -*- lsst-c++ -*/
/**
 * @file motion_fixture.h
 * @brief Defines the test fixture for verify motion class
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

#include <gtest/gtest.h>

#include "safety_shield/motion.h"

#ifndef MOTION_FIXTURE_H
#define MOTION_FIXTURE_H

namespace safety_shield {

/**
 * @brief Test fixture for verify motion class (simple)
 */
class MotionTestSimple : public ::testing::Test {
 protected:
  /**
   * @brief The Motion object
   */
  Motion motion_;

  /**
   * @brief Create the Motion object
   */
  void SetUp() override {
    motion_ = Motion(3);
  }
};

/**
 * @brief Test fixture for verify motion class
 */
class MotionTest : public ::testing::Test {
 protected:
  /**
   * @brief The Motion object
   */
  Motion motion_;

  /**
   * @brief Create the Motion object
   */
  void SetUp() override {
    int n_joints = 3;
    std::vector<double> p0;
    std::vector<double> v0;
    std::vector<double> a0;
    std::vector<double> j0;
    p0.push_back(0); p0.push_back(1); p0.push_back(2);
    v0.push_back(1); v0.push_back(1); v0.push_back(1);
    a0.push_back(10); a0.push_back(10); a0.push_back(10);
    j0.push_back(0); j0.push_back(5); j0.push_back(0);
    motion_ = Motion(0, p0, v0, a0, j0);
  }
};
} // namespace safety_shield

#endif // MOTION_FIXTURE_H