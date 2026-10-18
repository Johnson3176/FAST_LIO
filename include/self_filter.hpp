#pragma once
#include <cmath>

// Bounds are expressed in the raw LiDAR frame, in meters.
struct SelfFilterBox
{
    bool enabled = true;
    double min_x = -1.0, max_x = 0.1;
    double min_y = -0.5, max_y = 0.5;
    double min_z = -0.5, max_z = 2.0;

    bool valid() const
    {
        return std::isfinite(min_x) && std::isfinite(max_x) && min_x < max_x &&
               std::isfinite(min_y) && std::isfinite(max_y) && min_y < max_y &&
               std::isfinite(min_z) && std::isfinite(max_z) && min_z < max_z;
    }

    bool excludes(double x, double y, double z) const
    {
        return enabled && x >= min_x && x <= max_x &&
               y >= min_y && y <= max_y && z >= min_z && z <= max_z;
    }
};
