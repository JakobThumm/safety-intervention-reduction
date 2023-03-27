#include <vector>

#include <gtest/gtest.h>
#include <spdlog/spdlog.h>

#include "motion_fixture.h"
#include "safety_shield/motion.h"

namespace safety_shield {

TEST_F(MotionTestSimple, SimpleGetTest){
  EXPECT_DOUBLE_EQ(motion_.getTime(), 0.0);
  EXPECT_DOUBLE_EQ(motion_.getS(), 0.0);
  EXPECT_EQ(motion_.getNbModules(), 3);
}

TEST_F(MotionTestSimple, IsStoppedTestSimple){
  EXPECT_TRUE(motion_.isStopped());
}

TEST_F(MotionTest, IsStoppedTest){
  EXPECT_FALSE(motion_.isStopped());
  EXPECT_TRUE(motion_.isStopped(10.1));
}

TEST_F(MotionTest, hasSamePosTest){
  int n_joints = 3;
  std::vector<double> p0;
  p0.push_back(0); p0.push_back(1); p0.push_back(2);
  Motion* mo = new Motion(0, p0);
  EXPECT_TRUE(motion_.hasSamePos(mo));
}

TEST_F(MotionTest, hasSameVelTest){
  int n_joints = 3;
  std::vector<double> p0;
  std::vector<double> v0;
  p0.push_back(0); p0.push_back(1); p0.push_back(2);
  v0.push_back(1); v0.push_back(1); v0.push_back(1);
  Motion* mo = new Motion(0, p0, v0);
  EXPECT_TRUE(motion_.hasSameVel(mo));
}

TEST_F(MotionTest, hasSameAccTest){
  int n_joints = 3;
  std::vector<double> p0;
  std::vector<double> v0;
  std::vector<double> a0;
  p0.push_back(0); p0.push_back(1); p0.push_back(2);
  v0.push_back(1); v0.push_back(1); v0.push_back(1);
  a0.push_back(10); a0.push_back(10); a0.push_back(10);
  Motion* mo = new Motion(0, p0, v0, a0);
  EXPECT_TRUE(motion_.hasSameAcc(mo));
}

} // namespace safety_shield

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}