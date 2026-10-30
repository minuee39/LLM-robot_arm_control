#pragma once

#include <string>
#include <moveit/collision_detection/collision_matrix.hpp>

inline bool palletObstaclePairForbidden(
    const collision_detection::AllowedCollisionMatrix& matrix,
    const std::string& obstacle, const std::string& other)
{
  collision_detection::AllowedCollision::Type allowed = collision_detection::AllowedCollision::NEVER;
  return !matrix.getAllowedCollision(obstacle, other, allowed) ||
         allowed == collision_detection::AllowedCollision::NEVER;
}
