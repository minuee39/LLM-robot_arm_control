#include <limits>
#include <gtest/gtest.h>
#include <mtc_tutorial/pallet_release.hpp>

TEST(PalletRelease, DefaultPreservesNamedOpeningForOtherProfiles)
{
  EXPECT_TRUE(palletReleasePositionValid(-1.0, "robotiq_finger_joint", "world", 0.10, 0.05));
}

TEST(PalletRelease, CandidateHasPositiveClearanceWithinConfirmedBounds)
{
  EXPECT_TRUE(palletReleasePositionValid(0.020, "J8", "base_footprint", 0.03, 0.03));
  EXPECT_TRUE(palletReleasePositionValid(0.0335, "J8", "base_footprint", 0.03, 0.03));
}

TEST(PalletRelease, RejectsNonfiniteOutOfBoundsOrNoClearance)
{
  for (const double value : { std::numeric_limits<double>::quiet_NaN(),
                              std::numeric_limits<double>::infinity(), -0.5, 0.0, 0.015, 0.0336 })
    EXPECT_FALSE(palletReleasePositionValid(value, "J8", "base_footprint", 0.03, 0.03));
  EXPECT_FALSE(palletReleasePositionValid(0.020, "J8", "base_footprint", 0.045, 0.03));
  EXPECT_FALSE(palletReleasePositionValid(0.020, "J8", "base_footprint", 0.03, 0.045));
}

TEST(PalletRelease, RejectsUnrelatedProfileOrInvalidDimensions)
{
  EXPECT_FALSE(palletReleasePositionValid(0.020, "robotiq_finger_joint", "world", 0.03, 0.03));
  EXPECT_FALSE(palletReleasePositionValid(0.020, "J8", "world", 0.03, 0.03));
  EXPECT_FALSE(palletReleasePositionValid(0.020, "J8", "base_footprint", 0, 0.03));
  EXPECT_FALSE(palletReleasePositionValid(0.020, "J8", "base_footprint", 0.03,
                                        std::numeric_limits<double>::quiet_NaN()));
}
