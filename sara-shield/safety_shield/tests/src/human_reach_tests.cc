#include <gtest/gtest.h>
#include <math.h> 

#include "spdlog/spdlog.h" 

#include "human_reach_fixture.h"
#include "safety_shield/human_reach.h"

namespace safety_shield {

TEST_F(HumanReachTest, InitializationTest){
  EXPECT_DOUBLE_EQ(0, 0);
}

TEST_F(HumanReachTest, HumanReachMeasTest){
  reach_lib::Point p(1, 2, 3);
  std::vector<reach_lib::Point> human_joint_pos;
  human_joint_pos.push_back(p);
  double t_meas = 1.0;
  human_reach_->measurement(human_joint_pos, t_meas);
  EXPECT_DOUBLE_EQ(human_reach_->getLastMeasTimestep(), 1.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointPos()[0].x, 1.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointPos()[0].y, 2.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointPos()[0].z, 3.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointVel()[0].x, 0.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointVel()[0].y, 0.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointVel()[0].z, 0.0);
}

TEST_F(HumanReachTest, HumanReachMeasVel0Test){
  reach_lib::Point p(1, 2, 3);
  std::vector<reach_lib::Point> human_joint_pos;
  human_joint_pos.push_back(p);
  double t_meas = 1.0;
  human_reach_->measurement(human_joint_pos, t_meas);
  reach_lib::Point q(1, 2, 3);
  human_joint_pos.pop_back();
  human_joint_pos.push_back(q);
  t_meas = 2.0;
  human_reach_->measurement(human_joint_pos, t_meas);
  EXPECT_DOUBLE_EQ(human_reach_->getLastMeasTimestep(), 2.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointPos()[0].x, 1.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointPos()[0].y, 2.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointPos()[0].z, 3.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointVel()[0].x, 0.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointVel()[0].y, 0.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointVel()[0].z, 0.0);
}

TEST_F(HumanReachTest, HumanReachMeasVel1Test){
  reach_lib::Point p(1, 2, 3);
  std::vector<reach_lib::Point> human_joint_pos;
  human_joint_pos.push_back(p);
  double t_meas = 1.0;
  human_reach_->measurement(human_joint_pos, t_meas);
  reach_lib::Point q(2, 3, 3);
  human_joint_pos.pop_back();
  human_joint_pos.push_back(q);
  t_meas = 2.0;
  human_reach_->measurement(human_joint_pos, t_meas);
  EXPECT_DOUBLE_EQ(human_reach_->getLastMeasTimestep(), 2.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointPos()[0].x, 2.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointPos()[0].y, 3.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointPos()[0].z, 3.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointVel()[0].x, 1.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointVel()[0].y, 1.0);
  EXPECT_DOUBLE_EQ(human_reach_->getJointVel()[0].z, 0.0);
}

TEST_F(HumanReachTest, HumanReachAnalysisVelTest){
  reach_lib::Point p(1, 2, 3);
  std::vector<reach_lib::Point> human_joint_pos;
  human_joint_pos.push_back(p);
  double t_meas = 1.0;
  human_reach_->measurement(human_joint_pos, t_meas);
  double t_command = 1.01;
  double t_break = 0.1;
  human_reach_->humanReachabilityAnalysis(t_command, t_break);
  std::vector<reach_lib::Capsule> v_cap = human_reach_->getArticulatedVelCapsules();
  EXPECT_DOUBLE_EQ(v_cap[0].p1_.x, 1.0);
  EXPECT_DOUBLE_EQ(v_cap[0].p1_.y, 2.0);
  EXPECT_DOUBLE_EQ(v_cap[0].p1_.z, 3.0);
  EXPECT_DOUBLE_EQ(v_cap[0].p2_.x, 1.0);
  EXPECT_DOUBLE_EQ(v_cap[0].p2_.y, 2.0);
  EXPECT_DOUBLE_EQ(v_cap[0].p2_.z, 3.0);
  // radius = thickness + meas_err_pos + ((t_command-t_last) + t_break + t_delay) * v_max 
  EXPECT_DOUBLE_EQ(v_cap[0].r_, 0.21);
}

TEST_F(HumanReachTest, HumanReachAnalysisAccTest){
  reach_lib::Point p(1, 2, 3);
  std::vector<reach_lib::Point> human_joint_pos;
  human_joint_pos.push_back(p);
  double t_meas = 1.0;
  human_reach_->measurement(human_joint_pos, t_meas);
  reach_lib::Point q(2, 3, 3);
  human_joint_pos.pop_back();
  human_joint_pos.push_back(q);
  t_meas = 2.0;
  human_reach_->measurement(human_joint_pos, t_meas);
  double t_command = 2.01;
  double t_break = 0.1;
  human_reach_->humanReachabilityAnalysis(t_command, t_break);
  double t = (t_command - t_meas) + t_break + human_reach_->getDelay();
  double a_max = 10;
  double thickness = 0.1;
  double v0 = sqrt(2);
  double r = thickness +
      human_reach_->getMeasurementErrorPos() +
      human_reach_->getMeasurementErrorVel() * t +
      0.5 * a_max * pow(t, 2.0);
  reach_lib::Point next_pos = q + (q-p) * t;
  std::vector<reach_lib::Capsule> a_cap = human_reach_->getArticulatedAccelCapsules();
  EXPECT_DOUBLE_EQ(a_cap[0].p1_.x, next_pos.x);
  EXPECT_DOUBLE_EQ(a_cap[0].p1_.y, next_pos.y);
  EXPECT_DOUBLE_EQ(a_cap[0].p1_.z, next_pos.z);
  EXPECT_DOUBLE_EQ(a_cap[0].p2_.x, next_pos.x);
  EXPECT_DOUBLE_EQ(a_cap[0].p2_.y, next_pos.y);
  EXPECT_DOUBLE_EQ(a_cap[0].p2_.z, next_pos.z);
  EXPECT_DOUBLE_EQ(a_cap[0].r_, r);
}

TEST_F(HumanReachTestError, HumanReachAnalysisVelTestError){
  reach_lib::Point p(1, 2, 3);
  std::vector<reach_lib::Point> human_joint_pos;
  human_joint_pos.push_back(p);
  double t_meas = 1.0;
  human_reach_->measurement(human_joint_pos, t_meas);
  double t_command = 1.01;
  double t_break = 0.1;
  double thickness = 0.1;
  double v_max = 1;
  human_reach_->humanReachabilityAnalysis(t_command, t_break);
  double t = (t_command - t_meas) + t_break + human_reach_->getDelay();
  // radius = thickness + meas_err_pos + ((t_command-t_last) + t_break + t_delay) * v_max 
  double r = thickness +
      human_reach_->getMeasurementErrorPos() +
      t * v_max;
  std::vector<reach_lib::Capsule> v_cap = human_reach_->getArticulatedVelCapsules();
  EXPECT_DOUBLE_EQ(v_cap[0].p1_.x, 1.0);
  EXPECT_DOUBLE_EQ(v_cap[0].p1_.y, 2.0);
  EXPECT_DOUBLE_EQ(v_cap[0].p1_.z, 3.0);
  EXPECT_DOUBLE_EQ(v_cap[0].p2_.x, 1.0);
  EXPECT_DOUBLE_EQ(v_cap[0].p2_.y, 2.0);
  EXPECT_DOUBLE_EQ(v_cap[0].p2_.z, 3.0);
  EXPECT_DOUBLE_EQ(v_cap[0].r_, r);
}

TEST_F(HumanReachTestError, HumanReachAnalysisAccTestError){
  reach_lib::Point p(1, 2, 3);
  std::vector<reach_lib::Point> human_joint_pos;
  human_joint_pos.push_back(p);
  double t_meas = 1.0;
  human_reach_->measurement(human_joint_pos, t_meas);
  reach_lib::Point q(2, 3, 3);
  human_joint_pos.pop_back();
  human_joint_pos.push_back(q);
  t_meas = 2.0;
  human_reach_->measurement(human_joint_pos, t_meas);
  double t_command = 2.01;
  double t_break = 0.1;
  human_reach_->humanReachabilityAnalysis(t_command, t_break);
  double t = (t_command - t_meas) + t_break + human_reach_->getDelay();
  double a_max = 10;
  double thickness = 0.1;
  double v0 = sqrt(2);
  double r = thickness +
      human_reach_->getMeasurementErrorPos() +
      human_reach_->getMeasurementErrorVel() * t +
      0.5 * a_max * pow(t, 2.0);
  reach_lib::Point next_pos = q + (q-p) * t;
  std::vector<reach_lib::Capsule> a_cap = human_reach_->getArticulatedAccelCapsules();
  EXPECT_DOUBLE_EQ(a_cap[0].p1_.x, next_pos.x);
  EXPECT_DOUBLE_EQ(a_cap[0].p1_.y, next_pos.y);
  EXPECT_DOUBLE_EQ(a_cap[0].p1_.z, next_pos.z);
  EXPECT_DOUBLE_EQ(a_cap[0].p2_.x, next_pos.x);
  EXPECT_DOUBLE_EQ(a_cap[0].p2_.y, next_pos.y);
  EXPECT_DOUBLE_EQ(a_cap[0].p2_.z, next_pos.z);
  EXPECT_DOUBLE_EQ(a_cap[0].r_, r);
}

TEST_F(HumanReachTestPos, HumanReachAnalysisPos){
  reach_lib::Point p1(0, 0, 0);
  reach_lib::Point p2(0.2, 0.2, 0.2);
  reach_lib::Point p3(0.4, 0.4, 0.4);
  std::vector<reach_lib::Point> human_joint_pos;
  human_joint_pos.push_back(p1); human_joint_pos.push_back(p2); human_joint_pos.push_back(p3);
  double t_meas = 1.0;
  human_reach_->measurement(human_joint_pos, t_meas);
  double t_command = 1.01;
  double t_break = 0.1;
  human_reach_->humanReachabilityAnalysis(t_command, t_break);
  double t = (t_command - t_meas) + t_break + human_reach_->getDelay();
  double length = 0.725;
  double thickness = 0.208;
  double v_max = 2;
  double r = length + 
      thickness +
      human_reach_->getMeasurementErrorPos() +
      v_max*t;
  reach_lib::Point next_pos = p1;
  std::vector<reach_lib::Capsule> p_cap = human_reach_->getArticulatedPosCapsules();
  EXPECT_DOUBLE_EQ(p_cap[0].p1_.x, next_pos.x);
  EXPECT_DOUBLE_EQ(p_cap[0].p1_.y, next_pos.y);
  EXPECT_DOUBLE_EQ(p_cap[0].p1_.z, next_pos.z);
  EXPECT_DOUBLE_EQ(p_cap[0].p2_.x, next_pos.x);
  EXPECT_DOUBLE_EQ(p_cap[0].p2_.y, next_pos.y);
  EXPECT_DOUBLE_EQ(p_cap[0].p2_.z, next_pos.z);
  EXPECT_DOUBLE_EQ(p_cap[0].r_, r);
}

} // namespace safety_shield

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}