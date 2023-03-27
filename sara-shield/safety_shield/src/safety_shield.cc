#include "safety_shield/safety_shield.h"

namespace safety_shield
{

  SafetyShield::SafetyShield() : max_s_stop_(0),
                                 v_max_allowed_({0, 0, 0}),
                                 a_max_allowed_({0, 0, 0}),
                                 j_max_allowed_({0, 0, 0}),
                                 a_max_ltt_({0, 0, 0}),
                                 j_max_ltt_({0, 0, 0})
  {
    spdlog::info("Safety shield created.");
  }

  SafetyShield::SafetyShield(bool activate_shield,
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
                             Verify *verify) : activate_shield_(activate_shield),
                                               nb_joints_(nb_joints),
                                               max_s_stop_(max_s_stop),
                                               v_max_allowed_(v_max_allowed),
                                               a_max_allowed_(a_max_allowed),
                                               j_max_allowed_(j_max_allowed),
                                               a_max_ltt_(a_max_path),
                                               j_max_ltt_(j_max_path),
                                               sample_time_(sample_time),
                                               path_s_(0),
                                               path_s_discrete_(0),
                                               long_term_trajectory_(long_term_trajectory),
                                               robot_reach_(robot_reach),
                                               obstacle_reach_(obstacle_reach),
                                               verify_(verify)
  {
    sliding_window_k_ = (int)std::floor(max_s_stop_ / sample_time_);
    std::vector<double> prev_dq;
    for (int i = 0; i < 6; i++)
    {
      prev_dq.push_back(0.0);
      alpha_i_.push_back(1.0);
    }
    alpha_i_.push_back(1.0);

    is_safe_ = !activate_shield_;
    computesPotentialTrajectory(is_safe_, prev_dq);
    next_motion_ = determineNextMotion(is_safe_);
    std::vector<double> q_min(nb_joints, -3.141);
    std::vector<double> q_max(nb_joints, -3.141);
    ltp_ = long_term_planner::LongTermPlanner(nb_joints, sample_time, q_min, q_max, v_max_allowed, a_max_path, j_max_path);
    spdlog::info("Safety shield created.");
  }

  SafetyShield::SafetyShield(bool activate_shield,
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
                             double obstacle_vmax) : activate_shield_(activate_shield),
                                                     sample_time_(sample_time),
                                                     path_s_(0),
                                                     path_s_discrete_(0)
  {
    ///////////// Build robot reach
    YAML::Node robot_config = YAML::LoadFile(robot_config_file);
    std::string robot_name = robot_config["robot_name"].as<std::string>();
    nb_joints_ = robot_config["nb_joints"].as<int>();
    double secure_radius = robot_config["secure_radius"].as<double>();
    double radius = robot_config["radius"].as<double>();
    robot_reach_ = new RobotReach(init_x, init_y, init_z,
                                  init_roll, init_pitch, init_yaw, radius, secure_radius);
    ////////////// Setting trajectory variables
    // YAML::Node trajectory_config = YAML::LoadFile(trajectory_config_file);
    // max_s_stop_ = trajectory_config["max_s_stop"].as<double>();
    // q_min_allowed_ = trajectory_config["q_min_allowed"].as<std::vector<double>>();
    // q_max_allowed_ = trajectory_config["q_max_allowed"].as<std::vector<double>>();
    // v_max_allowed_ = trajectory_config["v_max_allowed"].as<std::vector<double>>();
    // a_max_allowed_ = trajectory_config["a_max_allowed"].as<std::vector<double>>();
    // j_max_allowed_ = trajectory_config["j_max_allowed"].as<std::vector<double>>();
    // a_max_ltt_ = trajectory_config["a_max_ltt"].as<std::vector<double>>();
    // j_max_ltt_ = trajectory_config["j_max_ltt"].as<std::vector<double>>();
    // ltp_ = long_term_planner::LongTermPlanner(nb_joints_, sample_time, q_min_allowed_, q_max_allowed_, v_max_allowed_, a_max_ltt_, j_max_ltt_);
    // Initialize the long term trajectory
    std::vector<Motion> long_term_traj;
    // long_term_traj.push_back(Motion(0.0, init_qpos));
    // long_term_trajectory_ = LongTermTraj(long_term_traj, sample_time_);
    //////////// Build obstacle reach
    double measurement_error_pos = 0.0;
    double measurement_error_vel = 0.0;
    double delay = 0.0;

    obstacle_reach_ = new ObstacleReach(obstacle_moves.size(),
                                        obstacle_pos,
                                        obstacle_r,
                                        obstacle_moves,
                                        obstacle_vmax,
                                        measurement_error_pos,
                                        measurement_error_vel,
                                        delay);
    ///////////// Build verifier
    verify_ = new safety_shield::VerifyISO();
    /////////// Other settings
    sliding_window_k_ = (int)std::floor(max_s_stop_ / sample_time_);
    std::vector<double> prev_dq;
    for (int i = 0; i < 6; i++)
    {
      prev_dq.push_back(0.0);
      alpha_i_.push_back(1.0);
    }
    alpha_i_.push_back(1.0);
    is_safe_ = !activate_shield_;
    // computesPotentialTrajectory(is_safe_, prev_dq);
    // next_motion_ = determineNextMotion(is_safe_);

    obstacle_reach_->obstacleReachabilityAnalysis(cycle_begin_time_, 0.2);
    obstacle_cylinders_ = obstacle_reach_->getPosCylinder();

    // spdlog::info("Safety shield created.");
  }

  void SafetyShield::reset(bool activate_shield,
                           double init_x,
                           double init_y,
                           double init_z,
                           double init_roll,
                           double init_pitch,
                           double init_yaw,
                           const std::vector<double> &init_qpos,
                           double current_time)
  {
    robot_reach_->reset(init_x, init_y, init_z, init_roll, init_pitch, init_yaw);
    obstacle_reach_->reset();
    std::vector<double> prev_dq;
    for (int i = 0; i < 6; i++)
    {
      prev_dq.push_back(0.0);
      alpha_i_.push_back(1.0);
    }
    alpha_i_.push_back(1.0);
    is_safe_ = !activate_shield_;
    new_ltt_ = false;
    new_goal_ = false;
    new_ltt_processed_ = false;
    recovery_path_correct_ = false;
    path_s_ = 0;
    path_s_discrete_ = 0;
    cycle_begin_time_ = current_time;
    recovery_path_ = Path();
    failsafe_path_ = Path();
    failsafe_path_2_ = Path();
    safe_path_ = Path();
    // Initialize the long term trajectory
    std::vector<Motion> long_term_traj;
    long_term_traj.push_back(Motion(0.0, init_qpos));
    long_term_trajectory_ = LongTermTraj(long_term_traj, sample_time_);
    computesPotentialTrajectory(is_safe_, prev_dq);
    next_motion_ = determineNextMotion(is_safe_);
    spdlog::info("Safety shield resetted.");
  }

  bool SafetyShield::planSafetyShield(double pos, double vel, double acc, double ve, double a_max, double j_max, Path &path)
  {
    if (a_max < 0 || fabs(acc) > a_max)
    {
      return false;
    }
    try
    {
      path.setCurrent(false);
      path.setPosition(pos);
      path.setVelocity(vel);
      path.setAcceleration(acc);
      double epsilon = 1e-6;

      // If current velocity is close to goal and acceleration is approx zero -> Do nothing
      if (fabs(vel - ve) < epsilon && fabs(acc) < epsilon)
      {
        std::array<double, 6> new_phases = {sample_time_, sample_time_, sample_time_, 0.0, 0.0, 0.0};
        path.setPhases(new_phases);
        return true;
      }
      // Time to reach zero acceleration with maximum jerk
      double t_to_a_0 = roundToTimestep(abs(acc) / j_max);
      // Velocity at zero acceleration
      double v_at_a_0 = vel + acc * t_to_a_0 / 2;

      // If the velocity at zero acceleration is the goal velocity, use t_to_a_0 and max jerk and return
      if (fabs(v_at_a_0 - ve) < epsilon)
      {
        std::array<double, 6> new_phases = {t_to_a_0, t_to_a_0, t_to_a_0, -acc / t_to_a_0, 0, 0};
        path.setPhases(new_phases);
        return true;
      }

      double a_peak, t01, t12, t23, v01, v12, v23, correct_int;
      // If we are accelerating
      if (acc > 0)
      {
        // Reducing the acceleration to zero with max jerk would lead to a velocity larger than target
        if (v_at_a_0 > ve + epsilon)
        {
          if (vel - (a_max * a_max + acc * acc / 2) / j_max > ve)
          {
            a_peak = -a_max;
            t01 = roundToTimestep((acc - a_peak) / j_max);
            t23 = roundToTimestep(-a_peak / j_max);
            v01 = (acc + a_peak) * t01 / 2;
            v23 = a_peak * t23 / 2;
            t12 = roundToTimestep((ve - vel - v01 - v23) / a_peak);
            v12 = a_peak * t12;
          }
          else
          {
            a_peak = -sqrt(fabs((vel - ve) * j_max + acc * acc / 2));
            t01 = roundToTimestep((acc - a_peak) / j_max);
            t23 = roundToTimestep(-a_peak / j_max);
            v01 = (acc + a_peak) * t01 / 2;
            v23 = a_peak * t23 / 2;
            t12 = 0;
            v12 = 0;
          }
        }
        else
        {
          if (vel + (a_max * a_max - acc * acc / 2) / j_max < ve)
          {
            a_peak = a_max;
            t01 = roundToTimestep((a_peak - acc) / j_max);
            t23 = roundToTimestep(a_peak / j_max);
            v01 = (acc + a_peak) * t01 / 2;
            v23 = a_peak * t23 / 2;
            t12 = roundToTimestep((ve - vel - v01 - v23) / a_peak);
            v12 = a_peak * t12;
          }
          else
          {
            a_peak = sqrt(fabs((ve - vel) * j_max + acc * acc / 2));
            t01 = roundToTimestep((a_peak - acc) / j_max);
            t23 = roundToTimestep(a_peak / j_max);
            v01 = (acc + a_peak) * t01 / 2;
            v23 = a_peak * t23 / 2;
            t12 = 0;
            v12 = 0;
          }
        }
      }
      // If we are decelerating (a <= 0)
      else
      {
        if (v_at_a_0 > ve + epsilon)
        {
          if (vel - (a_max * a_max - acc * acc / 2) / j_max > ve)
          {
            a_peak = -a_max;
            t01 = roundToTimestep((acc - a_peak) / j_max);
            t23 = roundToTimestep(-a_peak / j_max);
            v01 = (acc + a_peak) * t01 / 2;
            v23 = a_peak * t23 / 2;
            t12 = roundToTimestep((ve - vel - v01 - v23) / (a_peak));
            v12 = a_peak * t12;
          }
          else
          {
            a_peak = -sqrt(fabs((vel - ve) * j_max + acc * acc / 2));
            t01 = roundToTimestep((acc - a_peak) / j_max);
            t23 = roundToTimestep(-a_peak / j_max);
            v01 = (acc + a_peak) * t01 / 2;
            v23 = a_peak * t23 / 2;
            t12 = 0;
            v12 = 0;
          }
        }
        else
        {
          if (vel + (a_max * a_max + acc * acc / 2) / j_max < ve)
          {
            a_peak = a_max;
            t01 = roundToTimestep((a_peak - acc) / j_max);
            t23 = roundToTimestep(a_peak / j_max);
            v01 = (acc + a_peak) * t01 / 2;
            v23 = a_peak * t23 / 2;
            t12 = roundToTimestep((ve - vel - v01 - v23) / a_peak);
            v12 = a_peak * t12;
          }
          else
          {
            a_peak = sqrt(fabs((ve - vel) * j_max + acc * acc / 2));
            t01 = roundToTimestep((a_peak - acc) / j_max);
            t23 = roundToTimestep(a_peak / j_max);
            v01 = (acc + a_peak) * t01 / 2;
            v23 = a_peak * t23 / 2;
            t12 = 0;
            v12 = 0;
          }
        }
      }
      correct_int = ve - vel - v01 - v12 - v23;
      a_peak += correct_int / ((t01 + t23) / 2 + t12 + epsilon);
      double j01;
      if (t01 >= epsilon)
      {
        j01 = (a_peak - acc) / (t01);
      }
      else
      {
        j01 = 0;
      }
      double j12 = 0;
      double j23;
      if (t23 >= epsilon)
      {
        j23 = -a_peak / (t23);
      }
      else
      {
        j23 = 0;
      }
      if (t01 < 0 || t12 < 0 || t23 < 0)
      {
        spdlog::debug("planSafetyShield calculated time negative. t01 = {}, t12 = {}, t23 = {}", t01, t12, t23);
        return false;
      }
      std::array<double, 6> new_phases = {t01, t01 + t12, t01 + t12 + t23, j01, j12, j23};
      path.setPhases(new_phases);
      return true;
    }
    catch (const std::exception &exc)
    {
      spdlog::error("Exception in SafetyShield::planSafetyShield: {}", exc.what());
      return false;
    }
  }

  void SafetyShield::calculateMaxAccJerk(const std::vector<double> &prev_speed, const std::vector<double> &a_max_part, const std::vector<double> &j_max_part, double &a_max_manoeuvre, double &j_max_manoeuvre)
  {
    double new_c, new_d;
    // Calculate dds_max
    double denom = std::abs(prev_speed[0]) + std::abs(a_max_part[0]) * max_s_stop_ + 1E-9;
    double min_c = (a_max_allowed_[0] - std::abs(a_max_part[0])) / denom;
    for (int i = 1; i < a_max_allowed_.size(); i++)
    {
      denom = std::abs(prev_speed[i]) + std::abs(a_max_part[i]) * max_s_stop_;
      new_c = (a_max_allowed_[i] - std::abs(a_max_part[i])) / denom;
      min_c = (new_c < min_c) ? new_c : min_c;
    }
    a_max_manoeuvre = (min_c < 0) ? 0 : min_c;
    // Calculate ddds_max
    denom = std::abs(prev_speed[0]) + std::abs(a_max_part[0]) * max_s_stop_ + 1E-9;
    double min_d = (j_max_allowed_[0] - 3 * std::abs(a_max_part[0]) * a_max_manoeuvre - std::abs(j_max_part[0])) / denom;
    for (int i = 1; i < a_max_allowed_.size(); i++)
    {
      denom = std::abs(prev_speed[i]) + std::abs(a_max_part[i]) * max_s_stop_;
      new_d = (j_max_allowed_[i] - 3 * std::abs(a_max_part[i]) * a_max_manoeuvre - std::abs(j_max_part[i])) / denom;
      min_d = (new_d < min_d) ? new_d : min_d;
    }
    j_max_manoeuvre = (min_d < 0) ? 0 : min_d;
  }

  Motion SafetyShield::computesPotentialTrajectory(bool v, const std::vector<double> &prev_speed)
  {
    try
    {
      // s_int indicates the index of the entire traveled way
      while (path_s_ >= (path_s_discrete_ + 1) * sample_time_)
      {
        long_term_trajectory_.increasePosition();
        if (new_ltt_)
        {
          new_long_term_trajectory_.increasePosition();
        }
        path_s_discrete_++;
      }
      // If verified safe, take the recovery path, otherwise, take the failsafe path
      if (v && recovery_path_correct_)
      {
        recovery_path_.setCurrent(true);
        // discard old FailsafePath and replace with new one
        failsafe_path_ = failsafe_path_2_;
        // repair path already incremented
      }
      else
      {
        failsafe_path_.setCurrent(true);
        // discard RepairPath
        recovery_path_.setCurrent(false);
        failsafe_path_.increment(sample_time_);
      }
      // find maximum acceleration and jerk authorised
      double a_max_manoeuvre, j_max_manoeuvre;
      // One could use new_long_term_trajectory_.getMaxAccelerationWindow(path_s_discrete_) instead of a_max_ltt but there are many bugs to be solved before.
      if (!new_ltt_)
      {
        calculateMaxAccJerk(prev_speed, a_max_ltt_, j_max_ltt_, a_max_manoeuvre, j_max_manoeuvre);
      }
      else
      {
        calculateMaxAccJerk(prev_speed, a_max_ltt_, j_max_ltt_, a_max_manoeuvre, j_max_manoeuvre);
      }

      // Desired movement, one timestep
      //  if not already on the repair path, plan a repair path
      if (!recovery_path_.isCurrent())
      {
        // plan repair path and replace
        recovery_path_correct_ = planSafetyShield(failsafe_path_.getPosition(), failsafe_path_.getVelocity(),
                                                  failsafe_path_.getAcceleration(), 1, a_max_manoeuvre, j_max_manoeuvre, recovery_path_);
      }
      // Only plan new failsafe trajectory if the recovery path planning was successful.
      if (recovery_path_correct_)
      {
        // advance one step on repair path
        recovery_path_.increment(sample_time_);

        // plan new failsafe path for STP
        bool failsafe_2_planning_success = planSafetyShield(recovery_path_.getPosition(), recovery_path_.getVelocity(), recovery_path_.getAcceleration(), 0, a_max_manoeuvre, j_max_manoeuvre, failsafe_path_2_);
        // Check the validity of the planned path
        if (!failsafe_2_planning_success || recovery_path_.getPosition() < failsafe_path_.getPosition())
        {
          recovery_path_correct_ = false;
        }
      }
      // If all planning was correct, use new failsafe path with single recovery step
      if (recovery_path_correct_)
      {
        potential_path_ = failsafe_path_2_;
      }
      else
      {
        // If planning failed, use previous failsafe path
        potential_path_ = failsafe_path_;
      }

      //// Calculate start and goal pos of intended motion
      // Fill potential buffer with position and velocity from last failsafe path. This value is not really used.
      double s_d = failsafe_path_.getPosition();
      double ds_d = failsafe_path_.getVelocity();
      double dds_d = failsafe_path_.getAcceleration();
      double ddds_d = failsafe_path_.getJerk();
      // Calculate goal
      potential_path_.getFinalMotion(s_d, ds_d, dds_d);
      Motion goal_motion;
      if (new_ltt_)
      {
        goal_motion = new_long_term_trajectory_.interpolate(s_d, ds_d, dds_d, ddds_d, v_max_allowed_, a_max_allowed_, j_max_allowed_);
      }
      else
      {
        goal_motion = long_term_trajectory_.interpolate(s_d, ds_d, dds_d, ddds_d, v_max_allowed_, a_max_allowed_, j_max_allowed_);
      }
      goal_motion.setTime(potential_path_.getPhase(3));
      return goal_motion;
    }
    catch (const std::exception &exc)
    {
      spdlog::error("Exception in SafetyShield::computesPotentialTrajectory: {}", exc.what());
      throw exc;
    }
  }

  bool SafetyShield::checkMotionForJointLimits(Motion &motion)
  { // no joint limits in Safety Gym
    return true;
  }

  Motion SafetyShield::determineNextMotion(bool is_safe)
  {
    Motion next_motion;
    double s_d, ds_d, dds_d, ddds_d;
    if (is_safe)
    {
      // Fill potential buffer with position and velocity from recovery path
      if (recovery_path_.getPosition() >= failsafe_path_.getPosition())
      {
        s_d = recovery_path_.getPosition();
        ds_d = recovery_path_.getVelocity();
        dds_d = recovery_path_.getAcceleration();
        ddds_d = recovery_path_.getJerk();
      }
      else
      {
        potential_path_.increment(sample_time_);
        s_d = potential_path_.getPosition();
        ds_d = potential_path_.getVelocity();
        dds_d = potential_path_.getAcceleration();
        ddds_d = potential_path_.getJerk();
      }

      // Interpolate from new long term buffer
      if (new_ltt_)
      {
        next_motion = new_long_term_trajectory_.interpolate(s_d, ds_d, dds_d, ddds_d, v_max_allowed_, a_max_allowed_, j_max_allowed_);
      }
      else
      {
        next_motion = long_term_trajectory_.interpolate(s_d, ds_d, dds_d, ddds_d, v_max_allowed_, a_max_allowed_, j_max_allowed_);
      }
      // Set potential path as new verified safe path
      safe_path_ = potential_path_;
    }
    else
    {
      // interpolate from old safe path
      safe_path_.increment(sample_time_);
      s_d = safe_path_.getPosition();
      ds_d = safe_path_.getVelocity();
      dds_d = safe_path_.getAcceleration();
      ddds_d = safe_path_.getJerk();
      next_motion = long_term_trajectory_.interpolate(s_d, ds_d, dds_d, ddds_d, v_max_allowed_, a_max_allowed_, j_max_allowed_);
    }
    /// !!! Set s to the new path position !!!
    path_s_ = s_d;
    // Return the calculated next motion
    return next_motion;
  }

  Motion SafetyShield::step(double cycle_begin_time, Motion current_motion)
  {
    cycle_begin_time_ = cycle_begin_time;
    try
    {
      is_safe_ = false;
      
      Motion goal_motion = long_term_trajectory_.getLastMotion();

      if (activate_shield_)
      {
        // Compute the robot reachable set for the potential trajectory
        // robot_cylinders_ = robot_reach_->reach(current_motion, goal_motion, (goal_motion.getS() - current_motion.getS()), alpha_i_);
        robot_capsules_ = robot_reach_->reach_path(long_term_trajectory_.getPath(), long_term_trajectory_.getCurrentPos());
        // Compute the obstacle reachable sets for the potential trajectory
        // obstacleReachabilityAnalysis(t_command, t_brake)
        obstacle_reach_->obstacleReachabilityAnalysis(cycle_begin_time_, goal_motion.getTime());
        obstacle_cylinders_ = obstacle_reach_->getPosCylinder();
        // Verify if the robot and obstacle reachable sets are collision free
        is_safe_ = verify_->verify_obstacle_reach(robot_capsules_, obstacle_cylinders_);

        // std::cout << "compute safety : " << is_safe_ << std::endl;
      }
      else
      {
        is_safe_ = true;
      }

      if (is_safe_) long_term_trajectory_.increasePosition();

      // Select the next motion based on the verified safety
      next_motion_ = determineNextMotion(is_safe_);
      next_motion_.setTime(cycle_begin_time);
    }
    catch (const std::exception &exc)
    {
      spdlog::error("Exception in SafetyShield::getNextCycle: {}", exc.what());
    }
    return next_motion_;
  }

  Motion SafetyShield::getCurrentMotion()
  {
    Motion current_pos;
    if (!recovery_path_.isCurrent())
    {
      current_pos = long_term_trajectory_.interpolate(failsafe_path_.getPosition(),
                                                      failsafe_path_.getVelocity(),
                                                      failsafe_path_.getAcceleration(),
                                                      failsafe_path_.getJerk(),
                                                      v_max_allowed_,
                                                      a_max_allowed_,
                                                      j_max_allowed_);
    }
    else
    {
      current_pos = long_term_trajectory_.interpolate(recovery_path_.getPosition(),
                                                      recovery_path_.getVelocity(),
                                                      recovery_path_.getAcceleration(),
                                                      recovery_path_.getJerk(),
                                                      v_max_allowed_,
                                                      a_max_allowed_,
                                                      j_max_allowed_);
    }
    return current_pos;
  }

  bool SafetyShield::checkCurrentMotionForReplanning(Motion &current_motion)
  {
    for (int i = 0; i < nb_joints_ * 2; i++)
    {
      if (std::abs(current_motion.getAcceleration()[i]) > a_max_ltt_[i])
      {
        return false;
      }
    }
    return true;
  }

  void SafetyShield::newLongTermTrajectory(const std::vector<std::vector<double>> &goal_position,
                                           const std::vector<std::vector<double>> &goal_velocity,
                                           const std::vector<std::vector<double>> &goal_acceleration)
  {
    try
    {
      new_goal_motion_ = Motion(cycle_begin_time_, goal_position[goal_position.size() - 1], goal_velocity[goal_velocity.size() - 1]);
      new_goal_ = true;
      new_ltt_ = true;
      new_ltt_processed_ = true;
      std::vector<Motion> new_motions;

      double new_time = path_s_;
      for (int i = 0; i < goal_position.size(); i++)
      {
        Motion m = Motion(new_time, goal_position[i], goal_velocity[i], goal_acceleration[i]);
        new_motions.push_back(m);
        new_time += sample_time_;
      }
      new_long_term_trajectory_ = LongTermTraj(new_motions, path_s_);

      last_replan_start_motion_ = new_motions[0];
    }
    catch (const std::exception &exc)
    {
      spdlog::error("Exception in SafetyShield::newLongTermTrajectory: {}", exc.what());
    }
  }

  void SafetyShield::newLongTermTrajectoryFromMotion(std::vector<Motion> &motion_list){
    try
    {
      new_goal_motion_ = Motion(motion_list[motion_list.size() - 1]);
      new_goal_motion_.setTime(cycle_begin_time_);
      new_goal_ = true;
      new_ltt_ = true;
      new_ltt_processed_ = true;

      double new_time = path_s_;
      for (int i = 0; i < motion_list.size(); i++)
      {
        motion_list[i].setTime(new_time);
        new_time += sample_time_;
      }
      // new_long_term_trajectory_ = LongTermTraj(motion_list, path_s_);
      long_term_trajectory_ = LongTermTraj(motion_list, path_s_);

      last_replan_start_motion_ = motion_list[0];
    }
    catch (const std::exception &exc)
    {
      spdlog::error("Exception in SafetyShield::newLongTermTrajectory: {}", exc.what());
    }
  }

  void SafetyShield::setLongTermTrajectory(LongTermTraj &traj)
  {
    Motion current_motion = getCurrentMotion();
    // Check if robot is at stop
    if (!current_motion.isStopped())
    {
      throw RobotMovementException();
    }
    Motion start = traj.getNextMotionAtIndex(0);
    // Check if traj starts at the same position
    if (!current_motion.hasSamePos(&start))
    {
      throw TrajectoryException("Given LTT does not start at current robot position.");
    }
    // Check if traj starts at v=0
    if (!current_motion.hasSameVel(&start))
    {
      std::stringstream ss;
      ss << "Given LTT does not start with velocity 0.0. Start velocity is [";
      for (size_t i = 0; i < start.getVelocity().size(); ++i)
      {
        if (i != 0)
          ss << ",";
        ss << start.getVelocity()[i];
      }
      ss << "].";
      std::string s = ss.str();
      throw TrajectoryException(s);
    }
    // Check if traj ends in stop
    if (!traj.getNextMotionAtIndex(traj.getLength() - 1).isStopped())
    {
      throw TrajectoryException("Given LTT does not end in a complete stop of the robot (v = a = j = 0.0)");
    }
    // Replace LTT
    new_long_term_trajectory_ = traj;
    new_ltt_ = true;
    new_ltt_processed_ = true;
  }

  bool SafetyShield::calculateLongTermTrajectory(const std::vector<double> &start_q,
                                                 const std::vector<double> start_dq,
                                                 const std::vector<double> start_ddq,
                                                 const std::vector<double> &goal_q,
                                                 LongTermTraj &ltt)
  {
    long_term_planner::Trajectory trajectory;
    bool success = ltp_.planTrajectory(goal_q, start_q, start_dq, start_ddq, trajectory);
    if (!success)
      return false;
    std::vector<Motion> new_traj(trajectory.length);
    double new_time = path_s_;
    std::vector<double> q(nb_joints_ * 2);
    std::vector<double> dq(nb_joints_ * 2);
    std::vector<double> ddq(nb_joints_ * 2);
    std::vector<double> dddq(nb_joints_ * 2);
    for (int i = 0; i < trajectory.length; i++)
    {
      for (int j = 0; j < nb_joints_ * 2; j++)
      {
        q[j] = trajectory.q[j][i];
        dq[j] = trajectory.v[j][i];
        ddq[j] = trajectory.a[j][i];
        dddq[j] = trajectory.j[j][i];
      }
      new_traj[i] = Motion(new_time, q, dq, ddq, dddq);
      new_time += sample_time_;
    }
    ltt = LongTermTraj(new_traj, sample_time_, path_s_discrete_, sliding_window_k_);
    return true;
  }

} // namespace safety_shield