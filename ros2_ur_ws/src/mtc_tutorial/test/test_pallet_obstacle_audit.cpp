#include <gtest/gtest.h>
#include <mtc_tutorial/pallet_obstacle_audit.hpp>

TEST(PalletObstacleAudit, MissingPairIsForbiddenByDefault)
{
  collision_detection::AllowedCollisionMatrix matrix;
  EXPECT_TRUE(palletObstaclePairForbidden(matrix, "green_block", "J1"));
}

TEST(PalletObstacleAudit, ExplicitPermissionIsRejected)
{
  collision_detection::AllowedCollisionMatrix matrix;
  matrix.setEntry("green_block", "J1", true);
  EXPECT_FALSE(palletObstaclePairForbidden(matrix, "green_block", "J1"));
}

TEST(PalletObstacleAudit, DefaultPermissionIsRejectedUnlessExplicitlyForbidden)
{
  collision_detection::AllowedCollisionMatrix matrix;
  matrix.setDefaultEntry("red_block", true);
  EXPECT_FALSE(palletObstaclePairForbidden(matrix, "green_block", "red_block"));
  matrix.setEntry("green_block", "red_block", false);
  EXPECT_TRUE(palletObstaclePairForbidden(matrix, "green_block", "red_block"));
}

TEST(PalletObstacleAudit, ConditionalPermissionIsRejected)
{
  collision_detection::AllowedCollisionMatrix matrix;
  collision_detection::DecideContactFn condition = [](collision_detection::Contact&) { return true; };
  matrix.setEntry("green_block", "J1", condition);
  EXPECT_FALSE(palletObstaclePairForbidden(matrix, "green_block", "J1"));
}

TEST(PalletObstacleAudit, ObstacleDefaultNeverOverridesTargetDefaultPermission)
{
  collision_detection::AllowedCollisionMatrix matrix;
  matrix.setDefaultEntry("red_block", true);
  matrix.setDefaultEntry("green_block", false);
  EXPECT_TRUE(palletObstaclePairForbidden(matrix, "green_block", "red_block"));
}

TEST(PalletObstacleAudit, SerializedAllForbiddenRowsCanBeOmittedWithoutPermission)
{
  moveit_msgs::msg::AllowedCollisionMatrix msg;
  msg.entry_names = { "J1", "green_block" };
  msg.entry_values.resize(2);
  for (auto& row : msg.entry_values)
    row.enabled = { false, false };
  collision_detection::AllowedCollisionMatrix matrix(msg);
  EXPECT_TRUE(palletObstaclePairForbidden(matrix, "green_block", "J1"));
  moveit_msgs::msg::AllowedCollisionMatrix round_trip;
  matrix.getMessage(round_trip);
  EXPECT_TRUE(round_trip.entry_names.empty());
}
