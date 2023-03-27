#include "safety_shield/robot_reach.h"

namespace safety_shield
{

  RobotReach::RobotReach(double x = 0, double y = 0, double z = 0, double roll = 0, double pitch = 0, double yaw = 0, double radius = 0.5, double secure_radius = 0.0) : nb_joints_(1),
                                                                                                                                                                         radius_robot_(radius),
                                                                                                                                                                         secure_radius_(secure_radius)
  {
    Eigen::Matrix4d transformation_matrix;
    double cr = cos(roll);
    double sr = sin(roll);
    double cp = cos(pitch);
    double sp = sin(pitch);
    double cy = cos(yaw);
    double sy = sin(yaw);
    transformation_matrix << cr * cp, cr * sp * sy - sr * cy, cr * sp * cy + sr * sy, x,
        sr * cp, sr * sp * sy + cr * cy, sr * sp * cy - cr * sy, y,
        -sp, cp * sy, cp * cy, z,
        0, 0, 0, 1;
    transformation_matrices_.push_back(transformation_matrix);
    transformation_matrices_.push_back(transformation_matrix);

    // Fill cylinders

    Eigen::Vector4d p1;
    Eigen::Vector4d p2;
    p1(0) = x;
    p1(1) = y;
    p1(2) = z;
    p2 = p1;
    p2(2) += 0.1;
    reach_lib::Cylinder cylinder(vectorToPoint(p1), vectorToPoint(p2), radius);
    robot_cylinders_.push_back(cylinder);
  }

  void RobotReach::reset(double x, double y, double z,
                         double roll, double pitch, double yaw)
  {
    Eigen::Matrix4d transformation_matrix;
    double cr = cos(roll);
    double sr = sin(roll);
    double cp = cos(pitch);
    double sp = sin(pitch);
    double cy = cos(yaw);
    double sy = sin(yaw);
    transformation_matrix << cr * cp, cr * sp * sy - sr * cy, cr * sp * cy + sr * sy, x,
        sr * cp, sr * sp * sy + cr * cy, sr * sp * cy - cr * sy, y,
        -sp, cp * sy, cp * cy, z,
        0, 0, 0, 1;
    transformation_matrices_[0] = transformation_matrix;
  }

  reach_lib::Cylinder RobotReach::transformCylinder(const int &n_joint, const Eigen::Matrix4d &T)
  {
    Eigen::Vector4d p1 = T * pointToVector(robot_cylinders_[n_joint].p1_);
    Eigen::Vector4d p2 = T * pointToVector(robot_cylinders_[n_joint].p2_);
    reach_lib::Cylinder c(
        vectorToPoint(p1),
        vectorToPoint(p2),
        robot_cylinders_[n_joint].r_);
    return c;
  }

  std::vector<reach_lib::Cylinder> RobotReach::reach(Motion &start_config, Motion &goal_config,
                                                     double s_diff, std::vector<double> alpha_i)
  {
    try
    {
      Eigen::Matrix4d T_before = transformation_matrices_[0];
      Eigen::Matrix4d T_after = transformation_matrices_[0];
      std::vector<reach_lib::Cylinder> reach_cylinders;
      std::vector<double> q1 = start_config.getPos();
      std::cout<<"pos begin : "<< q1[0]<< " , " << q1[1] <<std::endl;
      std::vector<double> q2 = goal_config.getPos();
      std::cout<<"pos end : "<< q2[0]<< " , " << q2[1] <<std::endl;
      for (int i = 0; i < nb_joints_; i++)
      {
        // build cylinder before
        forwardKinematic(q1[2 * i], q1[2 * i + 1], i, T_before);
        reach_lib::Cylinder before = transformCylinder(i, T_before);
        // build cylinder after
        forwardKinematic(q2[2 * i], q2[2 * i + 1], i, T_after);
        reach_lib::Cylinder after = transformCylinder(i, T_after);

        // Caculate center of ball enclosing point p1 before and after
        reach_lib::Point p_1_k = (before.p1_ + after.p1_) * 0.5;
        // Caculate center of ball enclosing point p2 before and after
        reach_lib::Point p_2_k = (before.p2_ + after.p2_) * 0.5;
        // Calculate radius of ball enclosing point p1 before and after
        double r_1 = reach_lib::Point::norm(before.p1_ - after.p1_) / 2 + alpha_i[i] * s_diff * s_diff / 8 + robot_cylinders_[i].r_;
        // Calculate radius of ball enclosing point p2 before and after
        double r_2 = reach_lib::Point::norm(before.p2_ - after.p2_) / 2 + alpha_i[i + 1] * s_diff * s_diff / 8 + robot_cylinders_[i].r_;
        // Final radius is maximum of r_1 and r_2 plus the radius expansion for modelling errors.
        double radius = std::max(r_1, r_2) + secure_radius_;
        // Enclosure cylinder radius is max of ball around p1 and ball around p2
        reach_cylinders.push_back(reach_lib::Cylinder(p_1_k, p_2_k, radius));
      }
      return reach_cylinders;
    }
    catch (const std::exception &exc)
    {
      spdlog::error("Exception in RobotReach::reach: {}", exc.what());
      return {};
    }
  }

  std::vector<reach_lib::Capsule> RobotReach::reach_path(std::vector<Motion> path, int start_on_path)
  {
    try
    {
      std::vector<reach_lib::Capsule> reach_capsules;
      reach_capsules.reserve(path.size()-start_on_path-1);
      for (int i = start_on_path; i < path.size()-1; i++)
      {
        // build cylinder
        std::vector<double> p1 = path[i].getPos();
        reach_lib::Point p_1_k = reach_lib::Point(p1[0], p1[1], 0);
        // Caculate center of ball enclosing point p2 before and after
        std::vector<double> p2 = path[i+1].getPos();
        reach_lib::Point p_2_k = reach_lib::Point(p2[0], p2[1], 0);
        double radius = radius_robot_ + secure_radius_;
        reach_capsules.push_back(reach_lib::Capsule(p_1_k, p_2_k, radius));
      }
      return reach_capsules;
    }
    catch (const std::exception &exc)
    {
      spdlog::error("Exception in RobotReach::reach: {}", exc.what());
      return {};
    }
  }

} // namespace safety_shield