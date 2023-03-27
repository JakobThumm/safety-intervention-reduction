// -*- lsst-c++ -*/
/**
 * @file long_term_traj_fixture.h
 * @brief Defines the test fixture for verify long term trajectory class
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
#include "safety_shield/long_term_traj.h"

#ifndef LONG_TERM_TRAJ_FIXTURE_H
#define LONG_TERM_TRAJ_FIXTURE_H

namespace safety_shield {

/**
 * @brief Test fixture for verify long term trajectory class
 */
class LongTermTrajTest : public ::testing::Test {
 protected:
  /**
   * @brief The LTT object
   */
  LongTermTraj long_term_trajectory_;

  /**
   * @brief Create the LTT object
   */
  void SetUp() override {
    long_term_trajectory_ = LongTermTraj();
  }
};

/**
 * @brief Test fixture for verify long term trajectory class with starting index
 */
class LongTermTrajTestIdx : public ::testing::Test {
 protected:
  /**
   * @brief The LTT object
   */
  LongTermTraj long_term_trajectory_;

  /**
   * @brief Create the LTT object
   */
  void SetUp() override {
    std::vector<Motion> mo_vec;
    int n_joints = 2;
    std::vector<double> p0;
    std::vector<double> v0;
    std::vector<double> j0;
    for (int i = 0; i < n_joints; i++) {
        p0.push_back(0.0);
        v0.push_back(0.0);
        j0.push_back(0.0);
    }
    std::vector<double> a0;
    a0.push_back(12);
    a0.push_back(13);
    Motion m0(0, p0, v0, a0, j0);
    mo_vec.push_back(m0);
    std::vector<double> a1;
    a1.push_back(1);
    a1.push_back(2);
    Motion m1(1, p0, v0, a1, j0);
    mo_vec.push_back(m1);
    std::vector<double> a2;
    a2.push_back(78);
    a2.push_back(79);
    Motion m2(2, p0, v0, a2, j0);
    mo_vec.push_back(m2);
    std::vector<double> a3;
    a3.push_back(90);
    a3.push_back(91);
    Motion m3(3, p0, v0, a3, j0);
    mo_vec.push_back(m3);
    long_term_trajectory_ = LongTermTraj(mo_vec, 0.001, 3);
  }
};
} // namespace safety_shield

#endif // LONG_TERM_TRAJ_FIXTURE_H