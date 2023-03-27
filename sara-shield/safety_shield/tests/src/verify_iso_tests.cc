#include <gtest/gtest.h>
#include <math.h> 

#include <Eigen/Dense>
#include "spdlog/spdlog.h" 

#include "verify_iso_fixture.h"
#include "safety_shield/verify_iso.h"
#include "reach_lib.hpp"

namespace safety_shield {

TEST_F(VerifyIsoTest, InitializationTest) {
  EXPECT_DOUBLE_EQ(0, 0.0);
}

TEST_F(VerifyIsoTest, CapsuleCollisionTest) {
  reach_lib::Capsule cap1(reach_lib::Point(0.0, 0.0, 0.0), reach_lib::Point(0.0, 0.0, 1.0), 1.0);
  reach_lib::Capsule cap2(reach_lib::Point(1.0, 0.0, 0.0), reach_lib::Point(1.0, 0.0, 1.0), 1.0);
  reach_lib::Capsule cap3(reach_lib::Point(2.01, 0.0, 0.0), reach_lib::Point(2.01, 0.0, 1.0), 1.0);
  EXPECT_TRUE(verify_iso_.capsuleCollisionCheck(cap1, cap2));
  EXPECT_FALSE(verify_iso_.capsuleCollisionCheck(cap1, cap3));
}

TEST_F(VerifyIsoTest, RobotHumanCollisionTest) {
  reach_lib::Capsule r_cap1(reach_lib::Point(0.0, 0.0, 0.0), reach_lib::Point(0.0, 0.0, 1.0), 0.1);
  reach_lib::Capsule r_cap2(reach_lib::Point(0.0, 0.0, 1.0), reach_lib::Point(0.0, 0.0, 2.0), 0.1);
  std::vector<reach_lib::Capsule> r_caps = {r_cap1, r_cap2};
  reach_lib::Capsule h_cap1(reach_lib::Point(1.0, 0.0, 0.0), reach_lib::Point(1.0, 0.0, 1.0), 0.5);
  reach_lib::Capsule h_cap2(reach_lib::Point(1.0, 0.0, 1.0), reach_lib::Point(1.0, 0.0, 2.0), 0.5);
  std::vector<reach_lib::Capsule> h_caps = {h_cap1, h_cap2};
  EXPECT_FALSE(verify_iso_.robotHumanCollision(r_caps, h_caps));
}

TEST_F(VerifyIsoTest, VerifyHumanReachTest) {
  reach_lib::Capsule r_cap1(reach_lib::Point(0.0, 0.0, 0.0), reach_lib::Point(0.0, 0.0, 1.0), 0.1);
  reach_lib::Capsule r_cap2(reach_lib::Point(0.0, 0.0, 1.0), reach_lib::Point(0.0, 0.0, 2.0), 0.1);
  std::vector<reach_lib::Capsule> r_caps = {r_cap1, r_cap2};
  reach_lib::Capsule h_cap11(reach_lib::Point(1.0, 0.0, 0.0), reach_lib::Point(1.0, 0.0, 1.0), 0.5);
  reach_lib::Capsule h_cap12(reach_lib::Point(1.0, 0.0, 1.0), reach_lib::Point(1.0, 0.0, 2.0), 0.5);
  std::vector<reach_lib::Capsule> h_caps1 = {h_cap11, h_cap12};
  reach_lib::Capsule h_cap21(reach_lib::Point(1.0, 0.0, 0.0), reach_lib::Point(0.0, 0.0, 1.0), 0.5);
  reach_lib::Capsule h_cap22(reach_lib::Point(1.0, 0.0, 1.0), reach_lib::Point(0.0, 0.0, 2.0), 0.5);
  std::vector<reach_lib::Capsule> h_caps2 = {h_cap21, h_cap22};
  std::vector<std::vector<reach_lib::Capsule>> h_caps_list = {h_caps1, h_caps2};
  EXPECT_TRUE(verify_iso_.verify_human_reach(r_caps, h_caps_list));
}

} // namespace safety_shield

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}