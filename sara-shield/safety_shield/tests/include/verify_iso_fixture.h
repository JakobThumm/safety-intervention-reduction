// -*- lsst-c++ -*/
/**
 * @file verify_iso_fixture.h
 * @brief Defines the test fixture for verify ISO class
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

#include "safety_shield/verify_iso.h"

#ifndef VERFIY_ISO_FIXTURE_H
#define VERFIY_ISO_FIXTURE_H

namespace safety_shield {

/**
 * @brief Test fixture for verify ISO class
 */
class VerifyIsoTest : public ::testing::Test {
 protected:
  /**
   * @brief The verify iso object
   */
  VerifyISO verify_iso_;

  /**
   * @brief Create the verify iso object
   */
  void SetUp() override {
      verify_iso_ = VerifyISO();
  }
};
} // namespace safety_shield

#endif // VERFIY_ISO_FIXTURE_H