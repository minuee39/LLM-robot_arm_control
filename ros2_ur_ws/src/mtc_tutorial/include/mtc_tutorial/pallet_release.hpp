#pragma once

#include <algorithm>
#include <cmath>
#include <string>

inline bool palletReleasePositionValid(double value, const std::string& joint,
                                      const std::string& frame, double size_x, double size_y)
{
  if (value == -1.0)
    return true;  // Preserve named open pose for every existing profile.
  return std::isfinite(value) && joint == "J8" && frame == "base_footprint" &&
         std::isfinite(size_x) && std::isfinite(size_y) && size_x > 0.0 && size_y > 0.0 &&
         value > 0.5 * std::max(size_x, size_y) && value <= 0.0335;
}
