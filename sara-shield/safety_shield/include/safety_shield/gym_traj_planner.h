#include <vector>
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <random>
#include <algorithm>
#include <math.h>
#include "safety_shield/motion.h"

#ifndef GYM_TRAJ_H
#define GYM_TRAJ_H

// #DEFINE GRID 0
// #DEFINE OPPOSITE 1
// #DEFINE RANDOM 2

namespace safety_shield
{

    class GymTrajPlanner
    {
    private:
        /**
         * @brief How many steps to compute the trajectory for
         */
        int steps_ahead_;
        /**
         * @brief How many safe actions to sample
         */
        int n_tries_;
        /**
         * @brief How many actions to try for safe sampling
         */
        int max_tries_;

        // point data
        /**
         * @brief point robot mass
         */
        const double point_mass_ = 0.00518879;

        /**
         * @brief rotational gear ratio
         */
        const double gear_rot_ = 3;
        
        const double angle_b_ = 0.000426;
        inline double regression_angle_point(double action)
        {
            if (abs(action) < 0.1)
            {
                return 0;
            }
            if (action > 0)
            {
                return gear_rot_ * timestep_ * action + angle_b_;
            }
            return gear_rot_ * timestep_ * action - angle_b_;
        }

        /**
         * @brief values for velocity computation
         */
        const double gear_ = 0.3;
        const double friction_ = 0.0; // 0.00384;

        /**
         * @brief Mujoco timestep
         */
        double timestep_;

        // Static obstacle data
        /**
         * @brief List of static obstacle position
         */
        std::vector<Eigen::Vector2d> obstacles_;
        /**
         * @brief List of static obstacle radiuses
         */
        std::vector<float> obstacles_radius_;

        /**
         * @brief Additional radius for obstacle distance computation
         */
        double safety_buffer_;

        /**
         * @brief Action to take (either agent, slowdown, or resampled action)
         */
        std::vector<double> selected_action_;

        std::vector<Eigen::Vector2d> possible_actions_;

        /**
        * @brief Resample strategy: 0 = Grid, 1 = Random, 2 = Opposite
        */
        int resample_strat_ = 0;

    public:
        /**
         * @brief Construct a new GymTrajPlanner object
         *
         * @param steps_ahead the maximum number of steps to compute
         * @param timestep the simulation time step
         * @param obstacles statci obstacle positions
         * @param obstacles_radius static obstacle radiuses
         * @param n_tries number of safe action to resample in case a collision is detected
         */
        GymTrajPlanner(int steps_ahead, double timestep, std::vector<Eigen::Vector2d> obstacles, std::vector<float> obstacles_radius, int n_tries, int max_tries, double safety_buffer, int resample_strat) : steps_ahead_(steps_ahead), timestep_(timestep), obstacles_(obstacles), obstacles_radius_(obstacles_radius), n_tries_(n_tries), max_tries_(max_tries), safety_buffer_(safety_buffer), resample_strat_(resample_strat)
        {
            selected_action_ = {0, 0};
            possible_actions_.clear();
            Eigen::MatrixXd actions{
                {-1, -1},
                {-1, 0},
                {-1, 1},
                {0, -1},
                {0, 0},
                {0, 1},
                {1, -1},
                {1, 0},
                {1, 1}};
            for(int i = 0; i < actions.rows(); i++)
                possible_actions_.push_back(actions.row(i));
        }

        /**
         * @brief Plan a trajectory for the point robot.
         * Can resample actions if a collision is detected during trajectory computation.
         *
         * @param action chosen action by the agent
         * @param robot_vel 2D-Velocity of the robot
         * @param robot_rot Rotation matrix of the robot on the 2D plane
         * @param robot_com 2D-Position of the center of the robot
         *
         * @return a list of motions describing the trajectory
         */
        std::vector<Motion> planner_point(Eigen::Vector2d action, Eigen::Vector2d robot_vel, Eigen::Matrix2d robot_rot, Eigen::Vector2d robot_com, bool policy_step);
        /**
         * @brief Plan a trajectory for the point robot.
         *
         * @param action chosen action by the agent
         * @param robot_vel 2D-Velocity of the robot
         * @param robot_rot Rotation matrix of the robot on the 2D plane
         * @param robot_com 2D-Position of the center of the robot
         * @param planned_motions a list of motions describing the trajectory
         *
         * @return true if computed trajectory is collision-free, else false.
         */
        bool planner_point_loop(Eigen::Vector2d action, Eigen::Vector2d robot_vel, Eigen::Matrix2d robot_rot, Eigen::Vector2d robot_com, bool policy_step, double safety_radius, std::vector<Motion> &planned_motions);

        /**
         * @brief Computes a slowdown action
         *
         * @param robot_vel 2D-Velocity of the robot
         * @param action_0 acceleration action
         * @param action_1 rotation action
         * @param robot_rot Rotation matrix of the robot on the 2D plane
         *
         * @return the slowdown action following the trajectory set by (action_0, action_1)
         */
        Eigen::Vector2d point_slowdown(Eigen::Vector2d robot_vel, double action_0, double action_1, Eigen::Matrix2d robot_rot);
        Eigen::Vector2d point_slowdown_old(Eigen::Vector2d robot_vel, double action_0, double action_1, Eigen::Matrix2d robot_rot);

        /**
         * @brief Returns the action selected after trajectory computation
         *
         * @return action
         */
        inline std::vector<double> get_action() { return selected_action_; };

        /**
         * @brief Returns the sign of the argument, or 0 if abs(x) < 0.0001
         */
        inline double sign(double x) { return (x > 0.0001) - (x < 0.0001); }
    };
}

#endif // GYM_TRAJ_H
