#include "motion_core/models.hpp"

#include "numeric.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace motion_core {
namespace {

void require_positive_finite(double value, const char* name) {
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(std::string(name) + " must be finite and positive");
  }
}

void require_step_inputs(const Pose2D& pose, double first, double second,
                         double dt) {
  if (!is_finite(pose) || !std::isfinite(first) || !std::isfinite(second)) {
    throw std::invalid_argument("state and control inputs must be finite");
  }
  require_positive_finite(dt, "dt");
}

struct IntegratedHeading {
  double initial_heading{0.0};
  double reduced_increment{0.0};
  double direct_increment{0.0};
  bool direct_increment_available{false};
  double normalized_heading{0.0};
};

IntegratedHeading integrate_heading(double heading, double yaw_rate,
                                    double dt) {
  constexpr double two_pi = 2.0 * pi;
  const double initial_heading = normalize_angle(heading);
  double heading_delta = 0.0;
  double reduced_delta = 0.0;
  bool direct_increment_available = detail::try_multiply(
      yaw_rate, dt, heading_delta);
  if (direct_increment_available) {
    reduced_delta = detail::require_finite_result(
        std::remainder(heading_delta, two_pi), "reduced heading increment");
  } else {
    const double rate_period =
        detail::checked_divide(two_pi, dt, "heading-rate period");
    const double reduced_rate = detail::require_finite_result(
        std::remainder(yaw_rate, rate_period), "reduced heading rate");
    reduced_delta = detail::checked_multiply(
        reduced_rate, dt, "reduced heading increment");
  }
  const double next_heading = detail::checked_add(
      initial_heading, reduced_delta, "reduced heading sum");
  return {initial_heading, reduced_delta, heading_delta,
          direct_increment_available, normalize_angle(next_heading)};
}

double sinc(double value) {
  if (std::abs(value) < 1e-4) {
    const double square = value * value;
    return 1.0 - square / 6.0 + square * square / 120.0;
  }
  return detail::checked_divide(std::sin(value), value, "sinc");
}

double cosc(double value) {
  if (std::abs(value) < 1e-4) {
    const double square = value * value;
    return value * (0.5 - square / 24.0 + square * square / 720.0);
  }
  return detail::checked_divide(1.0 - std::cos(value), value, "cosc");
}

Pose2D integrate_unicycle(const Pose2D& pose, double speed,
                          double yaw_rate, double dt) {
  Pose2D result = pose;
  const IntegratedHeading heading = integrate_heading(pose.heading, yaw_rate, dt);
  if (yaw_rate == 0.0) {
    const double x_velocity = detail::checked_multiply(
        speed,
        detail::require_finite_result(std::cos(heading.initial_heading),
                                      "longitudinal heading cosine"),
        "longitudinal x velocity");
    const double y_velocity = detail::checked_multiply(
        speed,
        detail::require_finite_result(std::sin(heading.initial_heading),
                                      "longitudinal heading sine"),
        "longitudinal y velocity");
    const double x_displacement = detail::checked_multiply(
        x_velocity, dt, "longitudinal x displacement");
    const double y_displacement = detail::checked_multiply(
        y_velocity, dt, "longitudinal y displacement");
    result.x =
        detail::checked_add(pose.x, x_displacement, "integrated x position");
    result.y =
        detail::checked_add(pose.y, y_displacement, "integrated y position");
  } else if (heading.direct_increment_available &&
             std::abs(heading.direct_increment) < 1e-4) {
    const double forward_displacement = detail::checked_multiply(
        speed, dt, "small-turn forward displacement");
    const double sine = detail::require_finite_result(
        std::sin(heading.initial_heading), "initial-heading sine");
    const double cosine = detail::require_finite_result(
        std::cos(heading.initial_heading), "initial-heading cosine");
    const double sinc_value = sinc(heading.direct_increment);
    const double cosc_value = cosc(heading.direct_increment);
    const double x_factor = detail::checked_subtract(
        detail::checked_multiply(cosine, sinc_value,
                                 "small-turn x sinc component"),
        detail::checked_multiply(sine, cosc_value,
                                 "small-turn x cosc component"),
        "small-turn x factor");
    const double y_factor = detail::checked_add(
        detail::checked_multiply(sine, sinc_value,
                                 "small-turn y sinc component"),
        detail::checked_multiply(cosine, cosc_value,
                                 "small-turn y cosc component"),
        "small-turn y factor");
    const double x_displacement = detail::checked_multiply(
        forward_displacement, x_factor, "small-turn x displacement");
    const double y_displacement = detail::checked_multiply(
        forward_displacement, y_factor, "small-turn y displacement");
    result.x =
        detail::checked_add(pose.x, x_displacement, "integrated x position");
    result.y =
        detail::checked_add(pose.y, y_displacement, "integrated y position");
  } else {
    const double half_increment = detail::checked_multiply(
        0.5, heading.reduced_increment, "half heading increment");
    const double midpoint = normalize_angle(detail::checked_add(
        heading.initial_heading, half_increment, "midpoint heading"));
    const double twice_half_sine = detail::checked_multiply(
        2.0,
        detail::require_finite_result(std::sin(half_increment),
                                      "half-increment sine"),
        "twice half-increment sine");
    const double sine_difference = detail::checked_multiply(
        twice_half_sine,
        detail::require_finite_result(std::cos(midpoint),
                                      "midpoint heading cosine"),
        "heading sine difference");
    const double cosine_difference = detail::checked_multiply(
        twice_half_sine,
        detail::require_finite_result(std::sin(midpoint),
                                      "midpoint heading sine"),
        "heading cosine difference");
    const double x_displacement = detail::checked_quotient_product(
        speed, yaw_rate, sine_difference, "curved x displacement");
    const double y_displacement = detail::checked_quotient_product(
        speed, yaw_rate, cosine_difference, "curved y displacement");
    result.x =
        detail::checked_add(pose.x, x_displacement, "integrated x position");
    result.y =
        detail::checked_add(pose.y, y_displacement, "integrated y position");
  }
  result.heading = heading.normalized_heading;
  if (!is_finite(result)) {
    detail::throw_unrepresentable("integrated pose");
  }
  return result;
}

}  // namespace

double normalize_angle(double angle) {
  if (!std::isfinite(angle)) {
    throw std::invalid_argument("angle must be finite");
  }
  constexpr double two_pi = 2.0 * pi;
  double normalized = detail::require_finite_result(
      std::remainder(angle, two_pi), "normalized angle");
  if (normalized >= pi) {
    normalized =
        detail::checked_subtract(normalized, two_pi, "normalized angle");
  }
  if (normalized < -pi) {
    normalized = detail::checked_add(normalized, two_pi, "normalized angle");
  }
  return detail::require_finite_result(normalized, "normalized angle");
}

bool is_finite(const Pose2D& pose) noexcept {
  return std::isfinite(pose.x) && std::isfinite(pose.y) &&
         std::isfinite(pose.heading);
}

bool is_finite(const PathPoint& point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y);
}

DifferentialDrive::DifferentialDrive(DifferentialDriveLimits limits)
    : limits_(limits) {
  require_positive_finite(limits_.max_linear_speed, "max_linear_speed");
  require_positive_finite(limits_.max_angular_speed, "max_angular_speed");
}

DifferentialDriveStep DifferentialDrive::step(const Pose2D& pose,
                                               double linear_speed,
                                               double angular_speed,
                                               double dt) const {
  require_step_inputs(pose, linear_speed, angular_speed, dt);
  const double applied_linear = std::clamp(
      linear_speed, -limits_.max_linear_speed, limits_.max_linear_speed);
  const double applied_angular = std::clamp(
      angular_speed, -limits_.max_angular_speed, limits_.max_angular_speed);
  DifferentialDriveStep result{
      integrate_unicycle(pose, applied_linear, applied_angular, dt),
      applied_linear,
      applied_angular};
  if (!is_finite(result.pose) || !std::isfinite(result.linear_speed) ||
      !std::isfinite(result.angular_speed)) {
    detail::throw_unrepresentable("differential-drive step");
  }
  return result;
}

KinematicBicycle::KinematicBicycle(BicycleLimits limits) : limits_(limits) {
  require_positive_finite(limits_.wheelbase, "wheelbase");
  require_positive_finite(limits_.max_speed, "max_speed");
  require_positive_finite(limits_.max_steering_angle, "max_steering_angle");
  if (limits_.max_steering_angle >= pi / 2.0) {
    throw std::invalid_argument("max_steering_angle must be less than pi/2");
  }
}

BicycleStep KinematicBicycle::step(const Pose2D& pose, double speed,
                                   double steering_angle, double dt) const {
  require_step_inputs(pose, speed, steering_angle, dt);
  const double applied_speed =
      std::clamp(speed, -limits_.max_speed, limits_.max_speed);
  const double applied_steering =
      std::clamp(steering_angle, -limits_.max_steering_angle,
                 limits_.max_steering_angle);
  const double steering_tangent = detail::require_finite_result(
      std::tan(applied_steering), "steering tangent");
  const double yaw_rate = detail::checked_product_quotient(
      applied_speed, steering_tangent, limits_.wheelbase,
      "bicycle yaw rate");
  BicycleStep result{integrate_unicycle(pose, applied_speed, yaw_rate, dt),
                     applied_speed, applied_steering};
  if (!is_finite(result.pose) || !std::isfinite(result.speed) ||
      !std::isfinite(result.steering_angle)) {
    detail::throw_unrepresentable("bicycle step");
  }
  return result;
}

}  // namespace motion_core
