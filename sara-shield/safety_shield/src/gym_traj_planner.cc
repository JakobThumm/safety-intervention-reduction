#include "safety_shield/gym_traj_planner.h"

// #include <iostream>

namespace safety_shield
{
    std::vector<Motion> GymTrajPlanner::planner_point(Eigen::Vector2d action, Eigen::Vector2d robot_vel, Eigen::Matrix2d robot_rot, Eigen::Vector2d robot_com, bool policy_step)
    {
        // std::cout << "action = "<< action.transpose() << std::endl;
        Eigen::Map<Eigen::Vector2d>(selected_action_.data(), selected_action_.size()) = action;

        std::vector<Motion> planned_motions;
        planned_motions.reserve(steps_ahead_);


        double safety_radius = safety_buffer_ + 0.15 * policy_step;
        bool initial_loop_ok = planner_point_loop(action, robot_vel, robot_rot, robot_com, policy_step, safety_radius, planned_motions);

        if (!policy_step || initial_loop_ok || n_tries_ < 1)
            return planned_motions;


        Motion original_goal = planned_motions[planned_motions.size() - 1];
        float best_dist = 10000000;

        if(resample_strat_ == 0) {
            auto rd = std::random_device{};
            auto rng = std::default_random_engine{rd()};
            std::shuffle(std::begin(possible_actions_), std::end(possible_actions_), rng);
            for(Eigen::Vector2d resampled_action : possible_actions_) {
                std::vector<Motion> replanned_motions;
                replanned_motions.reserve(steps_ahead_);
                bool loop_ok = planner_point_loop(resampled_action, robot_vel, robot_rot, robot_com, policy_step, safety_buffer_, replanned_motions);

                if (loop_ok)
                {
                    planned_motions = replanned_motions;
                    Eigen::Map<Eigen::Vector2d>(selected_action_.data(), selected_action_.size()) = resampled_action;
                }
            }

        }
        else if(resample_strat_ == 1) {
            int valid_actions_found = 0;
            int total_tries = 0;
            while (valid_actions_found < n_tries_ && total_tries < max_tries_)
            {
                total_tries++;
                std::vector<Motion> replanned_motions;
                replanned_motions.reserve(steps_ahead_);
                Eigen::Vector2d resampled_action = Eigen::Vector2d::Random(); // select random action in [-0.05; 0.05] x [-1,1], ctrl range limits
                resampled_action(0) *= 0.05;

                bool loop_ok = planner_point_loop(resampled_action, robot_vel, robot_rot, robot_com, policy_step, safety_buffer_, replanned_motions);

                if (loop_ok)
                {
                    valid_actions_found++;
                    Motion new_goal = replanned_motions[replanned_motions.size() - 1];
                    float dist = new_goal.squaredDist(original_goal);
                    if (dist < best_dist)
                    {
                        planned_motions = replanned_motions;
                        best_dist = dist;
                        Eigen::Map<Eigen::Vector2d>(selected_action_.data(), selected_action_.size()) = resampled_action;
                    }
                }
            }

        } else if(resample_strat_ == 2){
            int valid_actions_found = 0;
            int total_tries = 0;
            while (total_tries < max_tries_)
            {
                total_tries++;
                std::vector<Motion> replanned_motions;
                replanned_motions.reserve(steps_ahead_);
                Eigen::Vector2d resampled_action = Eigen::Vector2d::Random(); // select random action in [-0.05; 0.05] x [-1,1], ctrl range limits
                resampled_action(0) = -10 * action(0); 

                if(resampled_action(1) < -0.33) {resampled_action(1) = -1;}
                else if(resampled_action(1) > 0.33)  {resampled_action(1) = 1;}
                else {{resampled_action(1) = 0;}}

                bool loop_ok = planner_point_loop(resampled_action, robot_vel, robot_rot, robot_com, policy_step, safety_buffer_, replanned_motions);

                if (loop_ok)
                {
                    planned_motions = replanned_motions;
                    Eigen::Map<Eigen::Vector2d>(selected_action_.data(), selected_action_.size()) = resampled_action;
                    break;
                }
            }
        }
        return planned_motions;
    }

    bool GymTrajPlanner::planner_point_loop(Eigen::Vector2d action, Eigen::Vector2d robot_vel, Eigen::Matrix2d robot_rot, Eigen::Vector2d robot_com, bool policy_step, double safety_radius, std::vector<Motion> &planned_motions)
    {
        bool no_collision = true;
        Eigen::Vector2d acceleration(0, 0);
        std::vector<double> pos(robot_com.data(), robot_com.data() + robot_com.size());
        std::vector<double> vel(robot_vel.data(), robot_vel.data() + robot_vel.size());
        std::vector<double> acc(acceleration.data(), acceleration.data() + acceleration.size());
        std::vector<double> action_vector(action.data(), action.data() + action.size());

        planned_motions.push_back(Motion(0, pos, vel, acc));
        planned_motions[planned_motions.size() - 1].setAction(action_vector);

        for (int i = 1; i < steps_ahead_; i++)
        {
            const double action_0_clip = std::clamp(action(0), -0.05, 0.05); // clip a_0 to Mujoco actuator range
            double acc_h = gear_ * action_0_clip / point_mass_;

            const double theta = regression_angle_point(action(1));

            // integrate rotation matrix
            Eigen::Matrix2d thetadot;
            double ct = std::cos(theta);
            double st = std::sin(theta);
            thetadot << ct, -st, st, ct;
            robot_rot = robot_rot * thetadot;

            Eigen::Vector2d acc_h_vector;
            acc_h_vector << acc_h, 0;
            acceleration = robot_rot * acc_h_vector;
            robot_vel += timestep_ * acceleration - friction_ * robot_vel;
            robot_com += timestep_ * robot_vel;

            for (int j = 0; j < obstacles_.size(); j++)
                if ((robot_com - obstacles_[j]).squaredNorm() < (obstacles_radius_[j] + safety_radius) * (obstacles_radius_[j] + safety_radius))
                    no_collision = false;

            Eigen::Map<Eigen::Vector2d>(pos.data(), pos.size()) = robot_com;
            Eigen::Map<Eigen::Vector2d>(vel.data(), vel.size()) = robot_vel;
            Eigen::Map<Eigen::Vector2d>(acc.data(), acc.size()) = acceleration;
            Eigen::Map<Eigen::Vector2d>(action_vector.data(), action_vector.size()) = action;
            planned_motions.push_back(Motion(0, pos, vel, acc));
            planned_motions[planned_motions.size() - 1].setAction(action_vector);

            // First sim step is robot action, the rest is a failsafe
            // However if validation trajectory, we execute action for longer
            if (!policy_step || i > 10)
            {
                action = point_slowdown(robot_vel, action(0), action(1), robot_rot);
            }

            if (robot_vel.squaredNorm() < 0.001)
                break;
        }
        return no_collision;
    }

    // Eigen::Vector2d point_slowdown(Eigen::Vector2d robot_vel, Eigen::Vector2d planned_action, Eigen::Matrix2d robot_rot)
    Eigen::Vector2d GymTrajPlanner::point_slowdown(Eigen::Vector2d robot_vel, double action_0, double action_1, Eigen::Matrix2d robot_rot)
    {
        Eigen::Vector2d action;
        action << 0, 0;

        const double heading = atan2(robot_vel(1), robot_vel(0));
        const double phi = atan2(robot_rot(1, 0), robot_rot(0, 0));
        double acc_opt = -1/timestep_ * (robot_vel(0) * cos(phi) + robot_vel(1) * sin(phi));
        double v_abs = robot_vel.norm();
        double delta_theta = 0;
        if (v_abs > 0.01) {
            action(0) = std::clamp(acc_opt * point_mass_ / gear_, -0.05, 0.05);
            delta_theta = heading - phi;
            if (abs(delta_theta) >= M_PI) {
                delta_theta -= 2 * M_PI * sign(delta_theta);
            }
        }
        if (abs(delta_theta) > 0.01 && v_abs > 0.1) {
            double act_theta = delta_theta;
            if (abs(delta_theta) > M_PI/2) {
                if (delta_theta > 0) {
                    act_theta = delta_theta - M_PI;
                } else {
                    act_theta = delta_theta + M_PI;
                }
            }
            action(1) = std::clamp(act_theta / (gear_rot_ * timestep_), -1.0, 1.0);
        }
        return action;
    }

    Eigen::Vector2d GymTrajPlanner::point_slowdown_old(Eigen::Vector2d robot_vel, double action_0, double action_1, Eigen::Matrix2d robot_rot)
    {
        Eigen::Vector2d v_h = robot_rot.inverse() * robot_vel; // project velocity on x axis
        double sign_vx = (v_h(0) > 0.01) - (v_h(0) < -0.01);
        Eigen::Vector2d action;
        action << -sign_vx, 0;

        if (sign_vx > 0.5 || sign_vx < 0.5)
        {
            action_1 = std::clamp(action_1, -0.05, 0.05);
            double l1 = v_h(0) * timestep_ + timestep_ * timestep_ * (action_0 / 2);       // distance traveled in next timestep with planned action
            double l2 = v_h(0) * timestep_ - timestep_ * timestep_ * (sign_vx * 0.05 / 2); // distance traveled in next timestep with slowdown
            action(1) = action_1 * (l2 / l1);
        }
        return action;
    }
}