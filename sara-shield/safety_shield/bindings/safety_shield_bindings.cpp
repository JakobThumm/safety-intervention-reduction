/**
 * @file safety_shield_bindings.cpp
 * @author Jakob Thumm (jakob.thumm@tum.de)
 * @brief Python bindings for the safety shield.
 * @version 0.1
 * @date 2022-06-08
 * 
 * @copyright This file is part of SaRA-Shield.
 * SaRA-Shield is free software: you can redistribute it and/or modify it under 
 * the terms of the GNU General Public License as published by the Free Software Foundation, 
 * either version 3 of the License, or (at your option) any later version.
 * SaRA-Shield is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 * See the GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License along with SaRA-Shield. 
 * If not, see <https://www.gnu.org/licenses/>. 
 * 
 */
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include "safety_shield/safety_shield.h"

namespace py = pybind11;

PYBIND11_MODULE(safety_shield_py, handle) {
  handle.doc() = "This module contains the python bindings for the safety shield.";
  // Motion class
  py::class_<safety_shield::Motion>(handle, "Motion")
    .def(py::init<>())
    .def(py::init<int>(), py::arg("nb_modules"))
    .def(py::init<double, std::vector<double>, double>(), 
      py::arg("time"), py::arg("q"), py::arg("s") = 0.0)
    .def(py::init<double, std::vector<double>, std::vector<double>, double>(), 
      py::arg("time"), py::arg("q"), py::arg("dq"), py::arg("s") = 0.0)
    .def(py::init<double, std::vector<double>, std::vector<double>, std::vector<double>, double>(), 
      py::arg("time"), py::arg("q"), py::arg("dq"), py::arg("ddq"), py::arg("s") = 0.0)
    .def(py::init<double, std::vector<double>, std::vector<double>,std::vector<double>, std::vector<double>, double>(), 
      py::arg("time"), py::arg("q"), py::arg("dq"), py::arg("ddq"), py::arg("dddq"), py::arg("s") = 0.0)
    .def("isStopped", &safety_shield::Motion::isStopped, py::arg("threshold") = 1e-4)
    .def("hasSamePos", &safety_shield::Motion::hasSamePos, py::arg("motion"), py::arg("threshold") = 1e-4)
    .def("hasSameVel", &safety_shield::Motion::hasSameVel, py::arg("motion"), py::arg("threshold") = 1e-4)
    .def("hasSameAcc", &safety_shield::Motion::hasSameAcc, py::arg("motion"), py::arg("threshold") = 1e-4)
    .def("getTime", &safety_shield::Motion::getTime)
    .def("getS", &safety_shield::Motion::getS)
    .def("getPos", &safety_shield::Motion::getPos)
    .def("getAction", &safety_shield::Motion::getAction)
    .def("getVelocity", &safety_shield::Motion::getVelocity)
    .def("getAcceleration", &safety_shield::Motion::getAcceleration)
    .def("getJerk", &safety_shield::Motion::getJerk)
    .def("setTime", &safety_shield::Motion::setTime, py::arg("new_time"))
    .def("setS", &safety_shield::Motion::setS, py::arg("new_s"))
    .def("setPos", &safety_shield::Motion::setPos, py::arg("new_q"))
    .def("setVelocity", &safety_shield::Motion::setVelocity, py::arg("new_dq"))
    .def("setAcceleration", &safety_shield::Motion::setAcceleration, py::arg("new_ddq"))
    .def("setJerk", &safety_shield::Motion::setJerk, py::arg("new_dddq"))
    ;
  // Long-term trajectory class
  py::class_<safety_shield::LongTermTraj>(handle, "LongTermTraj")
    .def(py::init<>())
    .def(py::init<std::vector<safety_shield::Motion>, double, int, int>(), py::arg("long_term_traj"), py::arg("sample_time"), py::arg("starting_index") = 0, py::arg("sliding_window_k") = 10)
    .def("interpolate", &safety_shield::LongTermTraj::interpolate, py::arg("s"), py::arg("ds"), py::arg("dds"), py::arg("ddds"), py::arg("v_max_allowed"), py::arg("a_max_allowed"), py::arg("j_max_allowed"))
    .def("setLongTermTrajectory", py::overload_cast<const std::vector<safety_shield::Motion>&>(&safety_shield::LongTermTraj::setLongTermTrajectory), py::arg("long_term_traj"))
    .def("setLongTermTrajectory", py::overload_cast<const std::vector<safety_shield::Motion>&, double>(&safety_shield::LongTermTraj::setLongTermTrajectory), py::arg("long_term_traj"), py::arg("sample_time"))
    .def("getLength", &safety_shield::LongTermTraj::getLength)
    .def("getCurrentPos", &safety_shield::LongTermTraj::getCurrentPos)
    .def("getCurrentMotion", &safety_shield::LongTermTraj::getCurrentMotion)
    .def("getNextMotion", &safety_shield::LongTermTraj::getNextMotion)
    .def("getNextMotionAtIndex", &safety_shield::LongTermTraj::getNextMotionAtIndex, py::arg("index"))
    .def("getTrajectoryIndex", &safety_shield::LongTermTraj::getTrajectoryIndex, py::arg("index"))
    .def("increasePosition", &safety_shield::LongTermTraj::increasePosition)
    .def("getMaxAccelerationWindow", &safety_shield::LongTermTraj::getMaxAccelerationWindow)
    .def("getMaxJerkWindow", &safety_shield::LongTermTraj::getMaxJerkWindow)
    .def("calculate_max_acc_jerk_window", &safety_shield::LongTermTraj::calculate_max_acc_jerk_window, py::arg("long_term_traj"), py::arg("k"))
    ;
  // Safety shield class
  py::class_<safety_shield::SafetyShield>(handle, "SafetyShield")
    .def(py::init<>())
    .def(py::init<bool, double, std::string, std::string, double, double, double, double, double, double, const std::vector<double>&, std::vector<double>&, std::vector<double>&, std::vector<bool>&, double>(),
      py::arg("activate_shield"),
      py::arg("sample_time"),
      py::arg("trajectory_config_file"),
      py::arg("robot_config_file"),
      py::arg("init_x"),
      py::arg("init_y"),
      py::arg("init_z"),
      py::arg("init_roll"),
      py::arg("init_pitch"),
      py::arg("init_yaw"),
      py::arg("init_qpos"),
      py::arg("obstacle_pos"),
      py::arg("obstacle_r"),
      py::arg("obstacle_moves"),
      py::arg("obstacle_vmax"))
    .def("reset", &safety_shield::SafetyShield::reset, 
      py::arg("activate_shield"),
      py::arg("init_x"),
      py::arg("init_y"),
      py::arg("init_z"),
      py::arg("init_roll"),
      py::arg("init_pitch"),
      py::arg("init_yaw"),
      py::arg("init_qpos"),
      py::arg("current_time"))
    .def("step", &safety_shield::SafetyShield::step, py::arg("cycle_begin_time"), py::arg("current_motion"))
    .def("newLongTermTrajectory", &safety_shield::SafetyShield::newLongTermTrajectory, py::arg("goal_position"), py::arg("goal_velocity"), py::arg("goal_acceleration"))
    .def("newLongTermTrajectoryFromMotion", &safety_shield::SafetyShield::newLongTermTrajectoryFromMotion, py::arg("motion_list"))
    .def("setLongTermTrajectory", &safety_shield::SafetyShield::setLongTermTrajectory, py::arg("traj"))
    .def("setObstacleMeasurement", &safety_shield::SafetyShield::setObstacleMeasurement, py::arg("obstacle_measurement"), py::arg("time"))
    .def("getRobotReachCapsules", &safety_shield::SafetyShield::getRobotReachCapsules)
    .def("getObstacleReachCylinders", &safety_shield::SafetyShield::getObstacleReachCylinders)
    .def("getSafety", &safety_shield::SafetyShield::getSafety)
  ;
  // Gym_traj_planner class
  py::class_<safety_shield::GymTrajPlanner>(handle, "GymTrajPlanner")
    .def(py::init<int, double, std::vector<Eigen::Vector2d>, std::vector<float>, int, int, double, int>(),
      py::arg("steps_ahead"),
      py::arg("timestep"),
      py::arg("obstacles"),
      py::arg("obstacles_radius"),
      py::arg("n_tries"),
      py::arg("max_tries"),
      py::arg("safety_buffer"),
      py::arg("resample_strat"))
    .def("get_action", &safety_shield::GymTrajPlanner::get_action)
    .def("planner_point", &safety_shield::GymTrajPlanner::planner_point, py::arg("action"), py::arg("robot_vel"), py::arg("robot_rot"), py::arg("robot_com"), py::arg("policy_step"))
    .def("point_slowdown", &safety_shield::GymTrajPlanner::point_slowdown, py::arg("robot_vel"), py::arg("action_0"), py::arg("action_1"), py::arg("robot_rot")) // TODO FIX EIGEN DECLARATION BUG
  ;
}
