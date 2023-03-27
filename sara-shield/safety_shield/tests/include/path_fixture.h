// -*- lsst-c++ -*/
/**
 * @file path_fixture.h
 * @brief Defines the test fixture for verify path class
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

#include "safety_shield/path.h"

#ifndef PATH_FIXTURE_H
#define PATH_FIXTURE_H

namespace safety_shield {

/**
 * @brief Test fixture for verify path class (simple)
 */
class PathTest : public ::testing::Test {
 protected:
  /**
   * @brief The Path object
   */
  Path path_;

  /**
   * @brief Create the Path object
   */
  void SetUp() override {
    path_ = Path();
    std::array<double,6> phases = {1, 2, 3, 1, 0, -1};
    path_.setPhases(phases);
  }
};
} // namespace safety_shield

#endif // PATH_FIXTURE_H