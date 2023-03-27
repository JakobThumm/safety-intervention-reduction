// -*- lsst-c++ -*/
/**
 * @file obstacle_reach.h
 * @brief Defines the obstacle reach class
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

#include <string>
#include <vector>
#include <map>
#include <exception>
#include <assert.h>

#include "spdlog/spdlog.h"

#include "reach_lib.hpp"

#ifndef OBSTACLE_REACH_H
#define OBSTACLE_REACH_H

namespace safety_shield
{

  /**
   * @brief Class handling the reachability analysis of the obstacles.
   *
   * This class holds position pedestrian models for each obstacle,
   * and handels incoming measurements.
   */
  class ObstacleReach
  {
  private:
    /**
     * @brief Last measured timestep.
     */
    double last_meas_timestep_ = -1;

    /**
     * @brief Number of joint measurements.
     */
    int n_obstacles_;

    /**
     * @brief Joint position measurements
     */
    std::vector<reach_lib::Point> joint_pos_;

    /**
     * @brief Calculated velocities
     */
    std::vector<reach_lib::Point> joint_vel_;

    /**
     * @brief The object for calculating the velocity based reachable set.
     */
    std::vector<reach_lib::PedestrianVel> obstacle_p_;

    /**
     * @brief We need two measurements for velocity calculation.
     */
    bool has_second_meas_ = false;

    /**
     * @brief Maximal positional measurement error
     *
     */
    double measurement_error_pos_;

    /**
     * @brief Maximal velocity measurement error
     *
     */
    double measurement_error_vel_;

    /**
     * @brief Delay in measurement processing pipeline
     *
     */
    double delay_;

  public:
    /**
     * @brief Empty constructor
     */
    ObstacleReach() {}

    /**
     * @brief ObstacleReach constructor
     * @param[in] n_obstacles Number of joints in the measurement
     * @param[in] measurement_error_pos Maximal positional measurement error
     * @param[in] measurement_error_vel Maximal velocity measurement error
     * @param[in] delay Delay in measurement processing pipeline
     * @param[in] radius Defines the thickness of the obstacle
     * @param[in] pos Defines the initial position (2D coords) of the obstacle
     * @param[in] moves Defines if the obstacle is static
     * @param[in] max_v The maximum velocity of the joints
     */
    ObstacleReach(int n_obstacles,
                  std::vector<double> &pos,
                  std::vector<double> &radius,
                  std::vector<bool> &moves,
                  double max_v,
                  double measurement_error_pos,
                  double measurement_error_vel,
                  double delay);

    /**
     * @brief Destructor
     */
    ~ObstacleReach() {}

    /**
     * @brief Reset the obstacle reach object.
     *
     */
    void reset();

    /**
     * @brief Update the joint measurements.
     * @param[in] obstacle_pos The positions of the obstacles.
     * @param[in] time The simulation time.
     */
    void measurement(const std::vector<reach_lib::Point> &obstacle_pos, double time);

    /**
     * @brief Calculate reachability analysis for given breaking time.
     *
     * Updates the values in obstacle_p_.
     * Get the values afterwards with the getter functions!
     *
     * @param[in] t_command Current time
     * @param[in] t_brake Time horizon of reachability analysis
     */
    void obstacleReachabilityAnalysis(double t_command, double t_brake);

    /**
     * @brief Get the Articulated Pos cylinders
     *
     * @return reach_lib::PedestrianVel cylinders
     */
    inline std::vector<reach_lib::Cylinder> getPosCylinder()
    {
      std::vector<reach_lib::Cylinder> caps;
      for (reach_lib::PedestrianVel obstacle : obstacle_p_) {
        std::vector<reach_lib::Cylinder> temp = reach_lib::get_cylinders(obstacle);
        caps.insert(caps.end(), temp.begin(), temp.end());
      }
      return caps;
    }

    /**
     * @brief Get the Last Meas Timestep object
     *
     * @return double
     */
    inline double getLastMeasTimestep()
    {
      return last_meas_timestep_;
    }

    /**
     * @brief Get the Joint Pos object
     *
     * @return std::vector<reach_lib::Point>
     */
    inline std::vector<reach_lib::Point> getJointPos()
    {
      return joint_pos_;
    }

    /**
     * @brief Get the Joint Vel object
     *
     * @return std::vector<reach_lib::Point>
     */
    inline std::vector<reach_lib::Point> getJointVel()
    {
      return joint_vel_;
    }

    /**
     * @brief Get the Measurement Error Pos object
     *
     * @return double
     */
    inline double getMeasurementErrorPos()
    {
      return measurement_error_pos_;
    }

    /**
     * @brief Get the Measurement Error Vel object
     *
     * @return double
     */
    inline double getMeasurementErrorVel()
    {
      return measurement_error_vel_;
    }

    /**
     * @brief Get the Delay object
     *
     * @return double
     */
    inline double getDelay()
    {
      return delay_;
    }
  };
} // namespace safety_shield
#endif // OBSTACLE_REACH_H
