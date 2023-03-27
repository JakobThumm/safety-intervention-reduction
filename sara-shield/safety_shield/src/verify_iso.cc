#include "safety_shield/verify_iso.h"

#include <iostream>
namespace safety_shield
{

  bool VerifyISO::robotObstacleCollision(const std::vector<reach_lib::Capsule> &robot_capsules,
                                         const std::vector<reach_lib::Cylinder> &obstacle_cylinders)
  {
    // Check position cylinder
    for (auto &obstacle_cylinder : obstacle_cylinders)
    {
      for (auto &robot_capsule : robot_capsules)
      {
        // If there is a collision, return true
        if (cylinderCapsuleCollisionCheck(obstacle_cylinder, robot_capsule))
        {
          return true;
        }
      }
    }
    return false;
  }

  bool VerifyISO::verify_obstacle_reach(const std::vector<reach_lib::Capsule> &robot_capsules,
                                        std::vector<reach_lib::Cylinder> obstacle_cylinders)
  {
    try
    {
      // If no collision occured, we are safe and don't have to check the rest.
      if (!robotObstacleCollision(robot_capsules, obstacle_cylinders))
      {
        return true;
      }
      return false;
    }
    catch (const std::exception &exc)
    {
      spdlog::error("Exception in VerifyISO::verify_obstacle_reach: {}", exc.what());
      return false;
    }
  }
} // namespace safety_shield