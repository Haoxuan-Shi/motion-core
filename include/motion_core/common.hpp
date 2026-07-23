#ifndef MOTION_CORE_COMMON_HPP
#define MOTION_CORE_COMMON_HPP

#include <cstddef>

namespace motion_core {

constexpr double pi = 3.141592653589793238462643383279502884;

struct Pose2D {
  double x{0.0};
  double y{0.0};
  double heading{0.0};
};

struct PathPoint {
  double x{0.0};
  double y{0.0};
};

double normalize_angle(double angle);
bool is_finite(const Pose2D& pose) noexcept;
bool is_finite(const PathPoint& point) noexcept;

}  // namespace motion_core

#endif

