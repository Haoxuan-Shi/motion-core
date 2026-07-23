#ifndef MOTION_CORE_MODELS_HPP
#define MOTION_CORE_MODELS_HPP

#include "motion_core/common.hpp"

namespace motion_core {

struct DifferentialDriveLimits {
  double max_linear_speed{2.0};
  double max_angular_speed{1.5};
};

struct DifferentialDriveStep {
  Pose2D pose;
  double linear_speed{0.0};
  double angular_speed{0.0};
};

class DifferentialDrive {
 public:
  explicit DifferentialDrive(DifferentialDriveLimits limits = {});
  DifferentialDriveStep step(const Pose2D& pose, double linear_speed,
                             double angular_speed, double dt) const;
  const DifferentialDriveLimits& limits() const noexcept { return limits_; }

 private:
  DifferentialDriveLimits limits_;
};

struct BicycleLimits {
  double wheelbase{2.7};
  double max_speed{15.0};
  double max_steering_angle{0.6};
};

struct BicycleStep {
  Pose2D pose;
  double speed{0.0};
  double steering_angle{0.0};
};

class KinematicBicycle {
 public:
  explicit KinematicBicycle(BicycleLimits limits = {});
  BicycleStep step(const Pose2D& pose, double speed, double steering_angle,
                   double dt) const;
  const BicycleLimits& limits() const noexcept { return limits_; }

 private:
  BicycleLimits limits_;
};

}  // namespace motion_core

#endif

