#include <vector>

#include <gtest/gtest.h>
#include <spdlog/spdlog.h>

#include "path_fixture.h"
#include "safety_shield/path.h"
#include "safety_shield/motion.h"

namespace safety_shield {

TEST_F(PathTest, GetValuesTest){
  EXPECT_DOUBLE_EQ(path_.getPosition(), 0.0);
  EXPECT_DOUBLE_EQ(path_.getVelocity(), 0.0);
  EXPECT_DOUBLE_EQ(path_.getAcceleration(), 0.0);
  EXPECT_DOUBLE_EQ(path_.getJerk(0), 1.0);
  EXPECT_DOUBLE_EQ(path_.getJerk(1), 0.0);
  EXPECT_DOUBLE_EQ(path_.getJerk(2), -1.0);
}

TEST_F(PathTest, IncrementTest){
  // 0.5
  path_.increment(0.5);
  EXPECT_NEAR(path_.getPosition(), 0.02083333333, 1e-5);
  EXPECT_DOUBLE_EQ(path_.getVelocity(), 0.125);
  EXPECT_DOUBLE_EQ(path_.getAcceleration(), 0.5);
  // 1.0
  path_.increment(0.5);
  EXPECT_NEAR(path_.getPosition(), 0.16666666666, 1e-5);
  EXPECT_DOUBLE_EQ(path_.getVelocity(), 0.5);
  EXPECT_DOUBLE_EQ(path_.getAcceleration(), 1.0);
  // 1.5
  path_.increment(0.5);
  EXPECT_NEAR(path_.getPosition(), 0.54166666666, 1e-5);
  EXPECT_DOUBLE_EQ(path_.getVelocity(), 1.0);
  EXPECT_DOUBLE_EQ(path_.getAcceleration(), 1.0);
  // 2.0
  path_.increment(0.5);
  EXPECT_NEAR(path_.getPosition(), 1.16666666666, 1e-5);
  EXPECT_DOUBLE_EQ(path_.getVelocity(), 1.5);
  EXPECT_DOUBLE_EQ(path_.getAcceleration(), 1.0);
  // 2.5
  path_.increment(0.5);
  EXPECT_NEAR(path_.getPosition(), 2.02083333333, 1e-5);
  EXPECT_DOUBLE_EQ(path_.getVelocity(), 1.875);
  EXPECT_DOUBLE_EQ(path_.getAcceleration(), 0.5);
  // 3.0
  path_.increment(0.5);
  EXPECT_NEAR(path_.getPosition(), 3.0, 1e-5);
  EXPECT_DOUBLE_EQ(path_.getVelocity(), 2.0);
  EXPECT_DOUBLE_EQ(path_.getAcceleration(), 0.0);
  
  EXPECT_DOUBLE_EQ(path_.getJerk(0), 1.0);
  EXPECT_DOUBLE_EQ(path_.getJerk(1), 0.0);
  EXPECT_DOUBLE_EQ(path_.getJerk(2), -1.0);
}

TEST_F(PathTest, GetFinalMotionTest){
  double pos; double vel; double acc;
  path_.getFinalMotion(pos, vel, acc);
  EXPECT_DOUBLE_EQ(pos, 3.0);
  EXPECT_DOUBLE_EQ(vel, 2.0);
  EXPECT_DOUBLE_EQ(acc, 0.0);
}

} // namespace safety_shield

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}