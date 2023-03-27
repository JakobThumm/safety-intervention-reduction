// -*- lsst-c++ -*/
/**
 * @file safety_shield.h
 * @brief Defines the online verification class
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

#include <cmath>
#include <string>
#include <sstream>
#include <math.h>
#include <algorithm>
#include <vector>
#include <set>
#include <time.h>
#include <stdio.h>
#include <assert.h>

#include "spdlog/spdlog.h"
#include <yaml-cpp/yaml.h>
#include "long_term_planner/long_term_planner.h"
#include "reach_lib.hpp"

#include "safety_shield/long_term_traj.h"
#include "safety_shield/path.h"
#include "safety_shield/motion.h"
#include "safety_shield/robot_reach.h"
#include "safety_shield/obstacle_reach.h"
#include "safety_shield/verify.h"
#include "safety_shield/verify_iso.h"
#include "safety_shield/exceptions.h"
#include "safety_shield/gym_traj_planner.h"

#ifndef safety_shield_H
#define safety_shield_H

namespace safety_shield
{

  /**
   * @brief Computes the failsafe trajectory
   */
  class SafetyShield
  {
  private:
    /**
     * @brief Robot reachable set calculation object
     *
     */
    RobotReach *robot_reach_;

    /**
     * @brief Obstacle reachable set calcualtion object
     *
     */
    ObstacleReach *obstacle_reach_;

    /**
     * @brief Verifier object
     *
     * Takes the robot and obstacle cylinders as input and checks them for collision.
     */
    Verify *verify_;

    /**
     * @brief The visualization of reachable sets
     *
     * TODO: Write a visualization for mujoco
     */
    // RvizMarker* rviz_;

    /**
     * @brief path to go back to the long term plan
     */
    Path recovery_path_;

    /**
     * @brief fail-safe path of the current path
     */
    Path failsafe_path_;

    /**
     * @brief fail-safe path of the repair path
     */
    Path failsafe_path_2_;

    /**
     * @brief verified safe path
     */
    Path safe_path_;

    /**
     * @brief the constructed failsafe path
     */
    Path potential_path_;

    /**
     * @brief Whether or not to use the formal verification.
     *
     * If this is set to false, every action is executed regardless of safety.
     */
    bool activate_shield_;

    /**
     * @brief Number of joints of the robot
     */
    int nb_joints_;

    /**
     * @brief sampling time
     */
    double sample_time_;

    /**
     * @brief the number of samples since start
     */
    int path_s_discrete_;

    /**
     * @brief Time since start
     */
    double path_s_;

    /**
     * @brief Was the last timestep safe
     */
    bool is_safe_;

    /**
     * @brief Indicates if the last replanning was successful or not.
     *
     * Indicates problems in the following statements:
     * - It is not strictly guaranteed that the manoeuvres generated maintain 0 ≤ ṡ ≤ 1. In
     * practice, this is only a problem when s̈ ˙ max or s̈ max change rapidly from one timestep to
     * the next, causing the trajectory of s̈ to “overshoot” 0 or 1. Since at all times we have a
     * failsafe trajectory available, verified in advance, which brings the robot to a safe state, if a
     * proposed short-term plan were to overshoot at any point during the plan, this short-term
     * plan is verified as unsafe and the failsafe trajectory is chosen.
     * - Again, when s̈ ˙ m or s̈ m change rapidly between timesteps, it may occur that |s̈| > s̈ m at
     * the start of a proposed short-term plan. Again, if this occurs, that particular short-term
     * plan is verified as unsafe and the failsafe trajectory is chosen.
     */
    bool recovery_path_correct_ = false;

    /**
     * @brief The last published motion
     */
    Motion next_motion_;

    /**
     * @brief The new long term goal
     */
    Motion new_goal_motion_;

    /**
     * @brief the maximum time to stop
     */
    double max_s_stop_;

    /**
     * @brief the maximum time to stop in timesteps (discrete)
     */
    int sliding_window_k_;

    /**
     * @brief Minimum angle (absolute)
     */
    std::vector<double> q_min_allowed_;

    /**
     * @brief Maximum angle (absolute)
     */
    std::vector<double> q_max_allowed_;

    /**
     * @brief maximum velocity allowed
     */
    std::vector<double> v_max_allowed_;

    /**
     * @brief maximum acceleration allowed
     */
    std::vector<double> a_max_allowed_;

    /**
     * @brief maximum jerk allowed
     */
    std::vector<double> j_max_allowed_;

    /**
     * @brief maximum acceleration along the long term plan
     */
    std::vector<double> a_max_ltt_;

    /**
     * @brief maximum jerk along the long term plan
     */
    std::vector<double> j_max_ltt_;

    /**
     * @brief maximum cartesian acceleration of robot joints (+ end effector!)
     *
     * alpha_i_.size() = nb_joints_ + 1
     * TODO: Calculate this as overapproximation.
     */
    std::vector<double> alpha_i_;

    /**
     * @brief the stored long_term_trajectory
     */
    LongTermTraj long_term_trajectory_;

    /**
     * @brief new LTT that wants to override the current LTT
     */
    LongTermTraj new_long_term_trajectory_;

    /**
     * @brief indicates that there is a potential new LTT
     */
    bool new_ltt_ = false;

    /**
     * @brief indicates that there is a new goal to compute a new LTT.
     *
     * We need a differentation between new goal and new LTT because an LTT to a new goal can only be calculated if the accerlation and jerk values are within the LTT planning bounds.
     */
    bool new_goal_ = false;

    /**
     * @brief indicates that the new LTT was passed to the safety verification at least once.
     */
    bool new_ltt_processed_ = false;

    /**
     * @brief the last starting position of the replanning
     *
     * If the last starting position of the replanning is very close to this position, we can skip the replanning and use the previously planned trajectory.
     */
    Motion last_replan_start_motion_;

    /**
     * @brief the time when the loop begins
     */
    double cycle_begin_time_;

    //////// Reachable sets of obstacles and robot //////
    /**
     * @brief Vector of robot reachable set capsules (get updated in every step()).
     */
    std::vector<reach_lib::Capsule> robot_capsules_;

    /**
     * @brief Vector of obstacle reachable set cylinders (get updated in every step()).
     */
    std::vector<reach_lib::Cylinder> obstacle_cylinders_;

    //////// For replanning new trajectory //////
    /**
     * @brief Trajecory planner
     */
    long_term_planner::LongTermPlanner ltp_;

  protected:
    /**
     * @brief Calculate max acceleration and jerk based on previous velocity
     * @details Mathematical explanation in http://mediatum.ub.tum.de/doc/1443612/652879.pdf eq. 2.6a and b (P.20).
     *
     * @param[in] prev_speed vector of previous joint velocities
     * @param[in] a_max_part max acceleration for this part of the LTT
     * @param[in] j_max_part max jerk for this part of the LTT
     * @param[out] a_max_manoeuvre Maximum path acceleration
     * @param[out] j_max_manoeuvre Maximum path jerk
     */
    void calculateMaxAccJerk(const std::vector<double> &prev_speed, const std::vector<double> &a_max_part, const std::vector<double> &j_max_part, double &a_max_manoeuvre, double &j_max_manoeuvre);

    /**
     * @brief Computes the fail-safe path
     *
     * @param[in] pos,vel,acc the starting point caracteristics
     * @param[in] ve the desired final velocity
     * @param[in] a_max the maximum acceleration allowed
     * @param[in] j_max the maximum jerk allowed
     * @param[out] path the new path
     * @return Whether the planning was successful or not
     */
    bool planSafetyShield(double pos, double vel, double acc, double ve, double a_max, double j_max,
                          Path &path);

    /**
     * @brief Calculate the next desired joint position based on verification of recovery path.
     * @param is_safe Last recovery path + potential path are verified safe.
     * @return next motion
     */
    Motion determineNextMotion(bool is_safe);

    /**
     * @brief Check a given motion if it exceeds the joint limits.
     *
     * @param motion Motion to check
     * @return true if path does NOT exceed joint limits
     * @return false if path exceeds joint limits
     */
    bool checkMotionForJointLimits(Motion &motion);

    /**
     * @brief round a continuous time to a timestep
     * @param t continuous time
     * @return timestep
     */
    inline double roundToTimestep(double t) { return ceil(t / sample_time_) * sample_time_; }

    /**
     * @brief Calculates and returns the current motion
     */
    Motion getCurrentMotion();

    /**
     * @brief Determines if the current motion is in the acceleration bounds for replanning
     *
     * @param current_motion current motion
     * @returns bool: if the current motion lies in the bounds for replanning
     */
    bool checkCurrentMotionForReplanning(Motion &current_motion);

    /**
     * @brief Calculates a new trajectory from current joint state to desired goal state.
     * @param start_q The current joint angles
     * @param start_dq The current joint velocities
     * @param start_ddq The current joint accelerations
     * @param goal_q The desired joint angles
     * @param ltt The calculated long-term trajectory
     * @return True if success, false otherwise
     */
    bool calculateLongTermTrajectory(const std::vector<double> &start_q, const std::vector<double> start_dq, const std::vector<double> start_ddq,
                                     const std::vector<double> &goal_q, LongTermTraj &ltt);

    /**
     * @brief Convert a cylinder to a vector containing [p1.x, p1.y, p1.z, p2.x, p2.y, p2.z, r]
     * p1: Center point of half sphere 1
     * p2: Center point of half sphere 2
     * r: Radius of half spheres and cylinder
     *
     * @param cap Cylinder
     * @return std::vector<double>
     */
    inline std::vector<double> convertCylinder(const reach_lib::Cylinder &cap)
    {
      std::vector<double> cylinder(7);
      cylinder[0] = cap.p1_.x;
      cylinder[1] = cap.p1_.y;
      cylinder[2] = cap.p1_.z;
      cylinder[3] = cap.p2_.x;
      cylinder[4] = cap.p2_.y;
      cylinder[5] = cap.p2_.z;
      cylinder[6] = cap.r_;
      return cylinder;
    }

    inline std::vector<double> convertCapsule(const reach_lib::Capsule &cap)
    {
      std::vector<double> capsule(7);
      capsule[0] = cap.p1_.x;
      capsule[1] = cap.p1_.y;
      capsule[2] = cap.p1_.z;
      capsule[3] = cap.p2_.x;
      capsule[4] = cap.p2_.y;
      capsule[5] = cap.p2_.z;
      capsule[6] = cap.r_;
      return capsule;
    }

  public:
    /**
     * @brief Default contructor
     *
     */
    SafetyShield();

    /**
     * @brief Construct a new Safety Shield object
     *
     * @param activate_shield Wether to activate the safety functionality or not.
     * @param nb_joints Number of joints of the robot
     * @param sample_time Sample time of safety shield
     * @param max_s_stop Maximal path length to stop the robot
     * @param v_max_allowed Maximal allowed joint speed
     * @param a_max_allowed Maximal allowed joint acceleration
     * @param j_max_allowed Maximal allowed joint jerk
     * @param a_max_path Maximal allowed relative path acceleration
     * @param j_max_path Maximal allowed relative path jerk
     * @param long_term_trajectory Fixed trajectory to execute (will be overwritten by new intended goals)
     *    This also defines the initial qpos.
     * @param robot_reach Robot reachable set calculation object
     * @param obstacle_reach Obstacle reachable set calculation object
     * @param verify Verification of reachable sets object
     */
    SafetyShield(bool activate_shield,
                 int nb_joints,
                 double sample_time,
                 double max_s_stop,
                 const std::vector<double> &v_max_allowed,
                 const std::vector<double> &a_max_allowed,
                 const std::vector<double> &j_max_allowed,
                 const std::vector<double> &a_max_path,
                 const std::vector<double> &j_max_path,
                 const LongTermTraj &long_term_trajectory,
                 RobotReach *robot_reach,
                 ObstacleReach *obstacle_reach,
                 Verify *verify);

    /**
     * @brief Construct a new Safety Shield object from config files.
     *
     * @param activate_shield If the safety function should be active or not.
     * @param sample_time Sample time of shield
     * @param trajectory_config_file Path to config file defining the trajectory parameters
     * @param robot_config_file Path to config file defining the robot transformation matrices and cylinders
     * @param init_x Base x pos
     * @param init_y Base y pos
     * @param init_z Base z pos
     * @param init_roll Base roll
     * @param init_pitch Base pitch
     * @param init_yaw Base yaw
     * @param init_qpos Initial joint position of the robot
     * @param obstacle_pos initial obstacle position
     * @param obstacle_r obstacle radius
     * @param obstacle_moves defines if obstacle static or not
     * @param obstacle_vmax defines max obstacle velocity (for non static)
     */
    explicit SafetyShield(bool activate_shield,
                          double sample_time,
                          std::string trajectory_config_file,
                          std::string robot_config_file,
                          double init_x,
                          double init_y,
                          double init_z,
                          double init_roll,
                          double init_pitch,
                          double init_yaw,
                          const std::vector<double> &init_qpos,
                          std::vector<double> &obstacle_pos,
                          std::vector<double> &obstacle_r,
                          std::vector<bool> &obstacle_moves,
                          double obstacle_vmax);

    /**
     * @brief A SafetyShield destructor
     */
    ~SafetyShield(){};

    /**
     * @brief Resets the safety shield completely.
     *
     * @param activate_shield If the safety function should be active or not.
     * @param init_x Base x pos
     * @param init_y Base y pos
     * @param init_z Base z pos
     * @param init_roll Base roll
     * @param init_pitch Base pitch
     * @param init_yaw Base yaw
     * @param init_qpos Initial joint position of the robot
     * @param current_time Initial time
     */
    void reset(bool activate_shield,
               double init_x,
               double init_y,
               double init_z,
               double init_roll,
               double init_pitch,
               double init_yaw,
               const std::vector<double> &init_qpos,
               double current_time);

    /**
     * @brief Computes the new trajectory depending on dq and if the previous path is safe and publishes it
     * @param v is the previous path safe
     * @param prev_speed the velocity of the previous point
     * @returns goal position, velocity, acceleration and time of the computed trajectory to execute.
     */
    Motion computesPotentialTrajectory(bool v, const std::vector<double> &prev_speed);

    /**
     * @brief Gets the information that the next simulation cycle (sample time) has started
     * @param cycle_begin_time timestep of begin of current cycle in seconds.
     *
     * @return next motion to be executed
     */
    Motion step(double cycle_begin_time, Motion current_motion);

    /**
     * @brief Calculates a new trajectory from current joint state to desired goal state.
     * Sets new trajectory as desired new long term trajectory.
     * @param goal_position Desired joint angles to move to
     * @param goal_velocity Desired joint velocities at the goal position
     * @param goal_acceleration Desired joint acceleration at the goal position
     */
    void newLongTermTrajectory(const std::vector<std::vector<double>> &goal_position,
                               const std::vector<std::vector<double>> &goal_velocity,
                               const std::vector<std::vector<double>> &goal_acceleration);


    void newLongTermTrajectoryFromMotion(std::vector<Motion> &motion_list);

    /**
     * @brief Overrides the current long-term trajectory.
     * @details Requires the robot to be at a complete stop, i.e. v=a=j=0.0 for all joints
     *    Requires the LTT to end in a complete stop.
     *    Requires the LTT to start in the same position as the robot.
     *    Requires the LTT to start with v=0
     *
     * @param traj New long-term trajectory
     *
     * @throws RobotMovementException Robot is not v=a=j=0
     * @throws TrajectoryException Incorrect LTT
     */
    void setLongTermTrajectory(LongTermTraj &traj);

    /**
     * @brief Receive a new obstacle measurement
     * @param[in] obstacle_measurement A vector of obstacle joint measurements (list of reach_lib::Points)
     * @param[in] time The timestep of the measurement in seconds.
     */
    inline void obstacleMeasurement(const std::vector<reach_lib::Point> obstacle_measurement, double time)
    {
      obstacle_reach_->measurement(obstacle_measurement, time);
    }

    /**
     * @brief Receive a new obstacle measurement.
     * Calls obstacleMeasurement(const std::vector<reach_lib::Point> obstacle_measurement, double time).
     * @param[in] obstacle_measurement A vector of obstacle joint measurements (flattened list of doubles [x, y])
     * @param[in] time The timestep of the measurement in seconds.
     */
    inline void setObstacleMeasurement(const std::vector<double> obstacle_measurement, double time)
    {
      assert(obstacle_measurement.size() > 0);
      std::vector<reach_lib::Point> converted_vec;
      for (int i = 0; i < obstacle_measurement.size() / 2; i++)
      {
        converted_vec.push_back(reach_lib::Point(obstacle_measurement[2 * i], obstacle_measurement[2 * i + 1], 0));
      }
      obstacleMeasurement(converted_vec, time);
    }

    /**
     * @brief Get the Robot Reach Cylinders as a vector of [p1[0:3], p2[0:3], r]
     * p1: Center point of half sphere 1
     * p2: Center point of half sphere 2
     * r: Radius of half spheres and cylinder
     *
     * @return std::vector<std::vector<double>> Cylinders
     */
    inline std::vector<std::vector<double>> getRobotReachCapsules()
    {
      std::vector<std::vector<double>> capsules(robot_capsules_.size(), std::vector<double>(7));
      for (int i = 0; i < robot_capsules_.size(); i++)
      {
        capsules[i] = convertCapsule(robot_capsules_[i]);
      }
      return capsules;
    }

    /**
     * @brief Get the Obstacle Reach Cylinders as a vector of [p1[0:3], p2[0:3], r]
     * p1: Center point of half sphere 1
     * p2: Center point of half sphere 2
     * r: Radius of half spheres and cylinder
     *
     * @param type Type of cylinder. Select 0 for POS, 1 for VEL, and 2 for ACCEL
     *
     * @return std::vector<std::vector<double>> Cylinders
     */
    inline std::vector<std::vector<double>> getObstacleReachCylinders()
    {
      std::vector<std::vector<double>> cylinders(obstacle_cylinders_.size(), std::vector<double>(7));
      for (int i = 0; i < obstacle_cylinders_.size(); i++)
      {
        cylinders[i] = convertCylinder(obstacle_cylinders_[i]);
      }
      return cylinders;
    }

    inline bool getSafety()
    {
      return is_safe_;
    }
  };
} // namespace safety_shield

#endif // safety_shield_H
