#include <gtest/gtest.h>
#include <math.h> 

#include <Eigen/Dense>
#include "spdlog/spdlog.h" 

#include "safety_shield_fixture.h"
#include "safety_shield/safety_shield.h"

namespace safety_shield {

TEST_F(SafetyShieldTest, InitializationTest) {
  EXPECT_DOUBLE_EQ(0, 0.0);
}

TEST_F(SafetyShieldTest, CalculateMaxAccJerkTest) {
  std::vector<double> prev_speed = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::vector<double> a_max_part = {2.0, 2.0, 2.0, 2.0, 2.0, 2.0};
  std::vector<double> j_max_part = {10.0, 10.0, 10.0, 10.0, 10.0, 10.0};
  // max_s_stop: 0.2
  // a_max_allowed: [10, 10, 10, 10, 10, 10]
  // j_max_allowed: [400, 400, 400, 400, 400, 400]
  double a_max_manoeuvre, j_max_manoeuvre;
  shield_.calculateMaxAccJerk(prev_speed, a_max_part, j_max_part, a_max_manoeuvre, j_max_manoeuvre);
  EXPECT_NEAR(a_max_manoeuvre, (10.0 - 2.0)/(1.0 + 2.0 * 0.2), 1e-5);
  EXPECT_NEAR(j_max_manoeuvre, (400.0 - 10.0 - 3 * 2.0 * a_max_manoeuvre)/(1.0 + 2.0 * 0.2), 1e-5);
}

TEST_F(SafetyShieldTest, PlanSafetyShieldTest) {
  double pos = 0;
  double vel = 0;
  double acc = 0;
  double ve = 1;
  double a_max = 10;
  double j_max = 100;
  double final_pos, final_vel, final_acc;
  bool success;
  safety_shield::Path path;
  for (int i=-10; i<11; i++) {
    vel = double(i)/10.0;
    for (int j=-40; j<40; j++) {
      acc = double(j)/5.0;
      for (int k=0; k<101; k++) {
        ve = double(k)/100.0;
        success = shield_.planSafetyShield(pos, vel, acc, ve, a_max, j_max, path);
        EXPECT_TRUE(success);
        path.getFinalMotion(final_pos, final_vel, final_acc);
        EXPECT_NEAR(final_vel, ve, 1e-6);
        EXPECT_NEAR(final_acc, 0.0, 1e-6);
      }
    }
  }
}

} // namespace safety_shield

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}