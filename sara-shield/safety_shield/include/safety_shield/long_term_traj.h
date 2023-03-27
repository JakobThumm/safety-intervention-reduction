// -*- lsst-c++ -*/
/**
 * @file long_term_traj.h
 * @brief Defines the long term trajectory class
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

#include <algorithm>
#include <deque>
#include <utility>
#include <vector>
#include <assert.h>

#include "spdlog/spdlog.h" // https://github.com/gabime/spdlog

#include "safety_shield/motion.h"

#ifndef LONG_TERM_TRAJ_H
#define LONG_TERM_TRAJ_H

namespace safety_shield {

/**
 * @brief A long term trajectory has a series of motions that the robot should move to.
 * 
 * When the trajectory is over, it will output its last position forever.
 */
class LongTermTraj {
 private:
  /**
   * @brief Trajectory length.
   */
  int length_;

  /**
   * @brief Time between two timesteps.
   */
  double sample_time_;

  /**
   * @brief The long term trajectory.
   */
  std::vector<Motion> long_term_traj_;

  /**
   * @brief Maximum joint acceleration for next k steps.
   */
  std::vector<std::vector<double>> max_acceleration_window_;

  /**
   * @brief Maximum joint jerk for next k steps.
   */
  std::vector<std::vector<double>> max_jerk_window_;

  /**
   * @brief The current timestamp.
   */
  int current_pos_;

  /**
   * @brief Starting index if the trajectory is not initialized at s=0.
   */
  int starting_index_;

 public:
  /**
   * @brief Construct a new Long Term Traj object.
   */
  LongTermTraj():
    current_pos_(0),
    starting_index_(0),
    length_(1),
    sample_time_(0.01)
  {
      long_term_traj_.push_back(Motion(1));
  }

  /**
   * @brief Construct a new Long Term Traj object.
   * 
   * @param long_term_traj Vector of motions that make up the LTT.
   * @param sliding_window_k Size of sliding window for max acc and jerk calculationa
   */
  LongTermTraj(const std::vector<Motion> &long_term_traj, double sample_time, int starting_index=0, int sliding_window_k=10):
    long_term_traj_(long_term_traj),
    sample_time_(sample_time),
    current_pos_(0),
    starting_index_(starting_index)
  {
    length_ = long_term_traj.size();
    calculate_max_acc_jerk_window(long_term_traj_, sliding_window_k);
  }

  /**
   * @brief Destroy the Long Term Traj object
   */
  ~LongTermTraj() {}

  /**
   * @brief Interpolates the position, velocity, and acceleration from the trajectory at the given s position.
   * @details Mathematical explanation in http://mediatum.ub.tum.de/doc/1443612/652879.pdf eq. 2.4a and b (P.19).
   * 
   * @param s the point's time
   * @param ds the percentage of the maximum path velocity, 0 = stand still, 1 = full velocity
   * @param dds the derivative of ds to time, 1 = accelerate from v=0 to full velocity in 1 second
   * @param ddds the derivative of dds to time
   * @param v_max_allowed vector of maximum allowed velocities
   * @param a_max_allowed vector of maximum allowed accelerations
   * @param j_max_allowed vector of maximum allowed jerks
   * @return Motion 
   */
  Motion interpolate(double s, double ds, double dds, double ddds, 
        std::vector<double>& v_max_allowed, std::vector<double>& a_max_allowed, std::vector<double>& j_max_allowed);

  /**
   * @brief Set the Long Term Trajectory object
   * 
   * @param long_term_traj vector of motions to form the new long-term trajectory 
   */
  inline void setLongTermTrajectory(const std::vector<Motion>& long_term_traj) {
    long_term_traj_ = long_term_traj;
    length_ = long_term_traj_.size();
  }

  /**
   * @brief Set the Long Term Trajectory object with new sample time
   * 
   * @param long_term_traj vector of motions to form the new long term trajectory 
   * @param sample_time the sample time of the new long-term trajectory
   */
  inline void setLongTermTrajectory(const std::vector<Motion>& long_term_traj, double sample_time) {
    long_term_traj_ = long_term_traj;
    length_ = long_term_traj_.size();
    sample_time_ = sample_time;
  }

  /**
   * @brief Get the length of the LTT
   * 
   * @return int length
   */
  inline int getLength() const { return length_; }

  /**
   * @brief Get the current index of the LTT
   * 
   * @return int current_pos_
   */
  inline int getCurrentPos() const { return current_pos_; }

  /**
   * @brief Get the current motion of the LTT.
   * 
   * Equals getNextMotionAtIndex(0);
   * 
   * @return Current motion 
   */
  inline Motion getCurrentMotion() const { return long_term_traj_[current_pos_]; }

  /**
   * @brief Get the next motion of the LTT
   * 
   * Equals getNextMotionAtIndex(1);
   * 
   * @return Next motion 
   */
  inline Motion getNextMotion() const { return long_term_traj_[std::min(current_pos_+1, length_-1)]; }

  /**
   * @brief Get the motion at index from current pos
   * 
   * @param index steps from current pos
   * @return Motion 
   */
  inline Motion getNextMotionAtIndex(int index) const {
    return long_term_traj_[getTrajectoryIndex(index)];
  }

  /**
   * @brief Get the last motion of the ltt
   * 
   * @return Motion
   */
  inline Motion getLastMotion() {
    return long_term_traj_[long_term_traj_.size()-1];
  }
  
  inline int getTrajectoryIndex(int index) const {
    int desired_pos = std::min(index-starting_index_, length_-1);
    if (desired_pos < current_pos_) {
      //spdlog::debug("In LongTermTraj::getNextMotionAtIndex: desired_pos ({:08d}) < current_pos({:08d})", desired_pos, current_pos_);
      desired_pos = current_pos_;
    }
    return desired_pos;
  }

  /**
   * @brief Increment the current pos
   */
  inline void increasePosition() { current_pos_ = std::min(current_pos_+1, length_-1); }

  /**
   * @brief Get the Max Acceleration Window at index i
   * 
   * @param index 
   * @return std::vector<double> 
   */
  inline std::vector<double> getMaxAccelerationWindow(int index) const { 
    return max_acceleration_window_[getTrajectoryIndex(index)]; 
  }

  /**
   * @brief Get the Max Jerk Window at index i
   * 
   * @param index 
   * @return std::vector<double> 
   */
  inline std::vector<double> getMaxJerkWindow(int index) const { 
    return max_jerk_window_[getTrajectoryIndex(index)]; 
  }

  /**
   * @brief Calculate the maximum acceleration and jerk for each timestep with a sliding window approach.
   * 
   * Sets the `max_acceleration_window_` and `max_jerk_window_` values
   * 
   * @param long_term_traj Long term trajectory
   * @param k sliding window size
   */
  void calculate_max_acc_jerk_window(std::vector<Motion> &long_term_traj, int k);


  inline std::vector<Motion> getPath() {return long_term_traj_;}
};
}
#endif // LONG_TERM_TRAJ_H
