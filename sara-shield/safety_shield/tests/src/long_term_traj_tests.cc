#include <vector>

#include <gtest/gtest.h>
#include <spdlog/spdlog.h>

#include "long_term_traj_fixture.h"
#include "safety_shield/long_term_traj.h"
#include "safety_shield/motion.h"

namespace safety_shield {

TEST_F(LongTermTrajTestIdx, GetLengthTest){
  EXPECT_EQ(long_term_trajectory_.getLength(), 4);
}

TEST_F(LongTermTrajTestIdx, GetCurrentPosTest){
  EXPECT_EQ(long_term_trajectory_.getCurrentPos(), 0);
  long_term_trajectory_.increasePosition();
  EXPECT_EQ(long_term_trajectory_.getCurrentPos(), 1);
  long_term_trajectory_.increasePosition();
  EXPECT_EQ(long_term_trajectory_.getCurrentPos(), 2);
  long_term_trajectory_.increasePosition();
  EXPECT_EQ(long_term_trajectory_.getCurrentPos(), 3);
  long_term_trajectory_.increasePosition();
  EXPECT_EQ(long_term_trajectory_.getCurrentPos(), 3);
  long_term_trajectory_.increasePosition();
  EXPECT_EQ(long_term_trajectory_.getCurrentPos(), 3);
}

TEST_F(LongTermTrajTestIdx, GetMotionTest){
  Motion mo = long_term_trajectory_.getCurrentMotion();
  for (int i = 0; i < 2; i++) {
    EXPECT_DOUBLE_EQ(mo.getAngle()[i], 0.0);
    EXPECT_DOUBLE_EQ(mo.getVelocity()[i], 0.0);
    EXPECT_DOUBLE_EQ(mo.getJerk()[i], 0.0);
  }
  EXPECT_DOUBLE_EQ(mo.getAcceleration()[0], 12.0);
  EXPECT_DOUBLE_EQ(mo.getAcceleration()[1], 13.0);
  mo = long_term_trajectory_.getNextMotion();
  for (int i = 0; i < 2; i++) {
    EXPECT_DOUBLE_EQ(mo.getAngle()[i], 0.0);
    EXPECT_DOUBLE_EQ(mo.getVelocity()[i], 0.0);
    EXPECT_DOUBLE_EQ(mo.getJerk()[i], 0.0);
  }
  EXPECT_DOUBLE_EQ(mo.getAcceleration()[0], 1.0);
  EXPECT_DOUBLE_EQ(mo.getAcceleration()[1], 2.0);
  mo = long_term_trajectory_.getNextMotionAtIndex(5);
  for (int i = 0; i < 2; i++) {
    EXPECT_DOUBLE_EQ(mo.getAngle()[i], 0.0);
    EXPECT_DOUBLE_EQ(mo.getVelocity()[i], 0.0);
    EXPECT_DOUBLE_EQ(mo.getJerk()[i], 0.0);
  }
  EXPECT_DOUBLE_EQ(mo.getAcceleration()[0], 78.0);
  EXPECT_DOUBLE_EQ(mo.getAcceleration()[1], 79.0);
}

TEST_F(LongTermTrajTestIdx, GetTrajectoryIndexTest){
  EXPECT_EQ(long_term_trajectory_.getTrajectoryIndex(-1), 0);
  EXPECT_EQ(long_term_trajectory_.getTrajectoryIndex(3), 0);
  EXPECT_EQ(long_term_trajectory_.getTrajectoryIndex(4), 1);
  EXPECT_EQ(long_term_trajectory_.getTrajectoryIndex(5), 2);
  EXPECT_EQ(long_term_trajectory_.getTrajectoryIndex(6), 3);
  EXPECT_EQ(long_term_trajectory_.getTrajectoryIndex(10), 3);
}

TEST_F(LongTermTrajTest, MaxAccWindowTest){
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
  std::vector<double> a4;
  a4.push_back(57);
  a4.push_back(58);
  Motion m4(4, p0, v0, a4, j0);
  mo_vec.push_back(m4);
  std::vector<double> a5;
  a5.push_back(89);
  a5.push_back(90);
  Motion m5(5, p0, v0, a5, j0);
  mo_vec.push_back(m5);
  std::vector<double> a6;
  a6.push_back(56);
  a6.push_back(57);
  Motion m6(6, p0, v0, a6, j0);
  mo_vec.push_back(m6);
  long_term_trajectory_.setLongTermTrajectory(mo_vec);
  long_term_trajectory_.calculate_max_acc_jerk_window(mo_vec, 3);

  EXPECT_EQ(long_term_trajectory_.getMaxAccelerationWindow(0)[0], 78);
  EXPECT_EQ(long_term_trajectory_.getMaxAccelerationWindow(1)[0], 90);
  EXPECT_EQ(long_term_trajectory_.getMaxAccelerationWindow(2)[0], 90);
  EXPECT_EQ(long_term_trajectory_.getMaxAccelerationWindow(3)[0], 90);
  EXPECT_EQ(long_term_trajectory_.getMaxAccelerationWindow(4)[0], 89);
  EXPECT_EQ(long_term_trajectory_.getMaxAccelerationWindow(5)[0], 89);
  EXPECT_EQ(long_term_trajectory_.getMaxAccelerationWindow(6)[0], 56);
  EXPECT_EQ(long_term_trajectory_.getMaxAccelerationWindow(0)[1], 79);
  EXPECT_EQ(long_term_trajectory_.getMaxAccelerationWindow(1)[1], 91);
  EXPECT_EQ(long_term_trajectory_.getMaxAccelerationWindow(2)[1], 91);
  EXPECT_EQ(long_term_trajectory_.getMaxAccelerationWindow(3)[1], 91);
  EXPECT_EQ(long_term_trajectory_.getMaxAccelerationWindow(4)[1], 90);
  EXPECT_EQ(long_term_trajectory_.getMaxAccelerationWindow(5)[1], 90);
  EXPECT_EQ(long_term_trajectory_.getMaxAccelerationWindow(6)[1], 57);
}

TEST_F(LongTermTrajTest, MaxJerkWindowTest){
  std::vector<Motion> mo_vec;
  int n_joints = 2;
  std::vector<double> p0;
  std::vector<double> v0;
  std::vector<double> a0;
  for (int i = 0; i < n_joints; i++) {
    p0.push_back(0.0);
    v0.push_back(0.0);
    a0.push_back(0.0);
  }
  std::vector<double> j0;
  j0.push_back(12);
  j0.push_back(13);
  Motion m0(0, p0, v0, a0, j0);
  mo_vec.push_back(m0);
  std::vector<double> j1;
  j1.push_back(1);
  j1.push_back(2);
  Motion m1(1, p0, v0, a0, j1);
  mo_vec.push_back(m1);
  std::vector<double> j2;
  j2.push_back(78);
  j2.push_back(79);
  Motion m2(2, p0, v0, a0, j2);
  mo_vec.push_back(m2);
  std::vector<double> j3;
  j3.push_back(90);
  j3.push_back(91);
  Motion m3(3, p0, v0, a0, j3);
  mo_vec.push_back(m3);
  std::vector<double> j4;
  j4.push_back(57);
  j4.push_back(58);
  Motion m4(4, p0, v0, a0, j4);
  mo_vec.push_back(m4);
  std::vector<double> j5;
  j5.push_back(89);
  j5.push_back(90);
  Motion m5(5, p0, v0, a0, j5);
  mo_vec.push_back(m5);
  std::vector<double> j6;
  j6.push_back(56);
  j6.push_back(57);
  Motion m6(6, p0, v0, a0, j6);
  mo_vec.push_back(m6);
  long_term_trajectory_.setLongTermTrajectory(mo_vec);
  long_term_trajectory_.calculate_max_acc_jerk_window(mo_vec, 3);

  EXPECT_EQ(long_term_trajectory_.getMaxJerkWindow(0)[0], 78);
  EXPECT_EQ(long_term_trajectory_.getMaxJerkWindow(1)[0], 90);
  EXPECT_EQ(long_term_trajectory_.getMaxJerkWindow(2)[0], 90);
  EXPECT_EQ(long_term_trajectory_.getMaxJerkWindow(3)[0], 90);
  EXPECT_EQ(long_term_trajectory_.getMaxJerkWindow(4)[0], 89);
  EXPECT_EQ(long_term_trajectory_.getMaxJerkWindow(5)[0], 89);
  EXPECT_EQ(long_term_trajectory_.getMaxJerkWindow(6)[0], 56);
  EXPECT_EQ(long_term_trajectory_.getMaxJerkWindow(0)[1], 79);
  EXPECT_EQ(long_term_trajectory_.getMaxJerkWindow(1)[1], 91);
  EXPECT_EQ(long_term_trajectory_.getMaxJerkWindow(2)[1], 91);
  EXPECT_EQ(long_term_trajectory_.getMaxJerkWindow(3)[1], 91);
  EXPECT_EQ(long_term_trajectory_.getMaxJerkWindow(4)[1], 90);
  EXPECT_EQ(long_term_trajectory_.getMaxJerkWindow(5)[1], 90);
  EXPECT_EQ(long_term_trajectory_.getMaxJerkWindow(6)[1], 57);
}

TEST_F(LongTermTrajTest, InterpolateTest) {
    std::vector<Motion> motions;
    int n_joints = 2;
    std::vector<double> p0 = {0.0, 0.0};
    std::vector<double> v0 = {1.0, 0.0};
    std::vector<double> a0 = {1.0, 0.0};
    std::vector<double> j0 = {-1.0, 0.0};
    Motion m0(0, p0, v0, a0, j0);
    motions.push_back(m0);
    std::vector<double> p1 = {0.104833333333333, 0.0};
    std::vector<double> v1 = {1.095, 0.0};
    std::vector<double> a1 = {0.9, 0.0};
    std::vector<double> j1 = {0.0, 0.0};
    Motion m1(0.1, p1, v1, a1, j1);
    motions.push_back(m1);
    long_term_trajectory_.setLongTermTrajectory(motions, 0.1);
    std::vector<double> v_max = {10.0, 10.0};
    std::vector<double> a_max = {10.0, 10.0};
    std::vector<double> j_max = {100.0, 100.0};
    Motion motion_int = long_term_trajectory_.interpolate(0.01, 1.0, 0.0, 0.0, v_max, a_max, j_max);
    EXPECT_NEAR(motion_int.getAngle()[0], 0.010049833333333, 1e-5);
    EXPECT_NEAR(motion_int.getVelocity()[0], 1.00995, 1e-5);
    EXPECT_NEAR(motion_int.getAcceleration()[0], 0.99, 1e-5);
    EXPECT_NEAR(motion_int.getJerk()[0], -1.0, 1e-5);
    Motion motion_int2 = long_term_trajectory_.interpolate(0.01, 0.5, 0.0, 0.0, v_max, a_max, j_max);
    EXPECT_NEAR(motion_int2.getAngle()[0], 0.010049833333333, 1e-5);
    EXPECT_NEAR(motion_int2.getVelocity()[0], 0.504975, 1e-5);
    EXPECT_NEAR(motion_int2.getAcceleration()[0], 0.2475, 1e-5);
    EXPECT_NEAR(motion_int2.getJerk()[0], -0.1250, 1e-5);
    Motion motion_int3 = long_term_trajectory_.interpolate(0.01, 0.5, 1.0, 0.2, v_max, a_max, j_max);
    EXPECT_NEAR(motion_int3.getAngle()[0], 0.010049833333333, 1e-5);
    EXPECT_NEAR(motion_int3.getVelocity()[0], 0.504975, 1e-5);
    EXPECT_NEAR(motion_int3.getAcceleration()[0], 1.25745, 1e-5);
    EXPECT_NEAR(motion_int3.getJerk()[0], 1.561990000000000, 1e-5);
}

} // namespace safety_shield

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}