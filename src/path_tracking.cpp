#include "motion_core/path_tracking.hpp"

#include "numeric.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace motion_core {
namespace {

struct Projection {
  PathPoint point;
  std::size_t segment{0};
  double distance{0.0};
  double signed_error{0.0};
  double heading{0.0};
};

void validate_path(const std::vector<PathPoint>& path) {
  if (path.size() < 2) {
    throw std::invalid_argument("path must contain at least two points");
  }
  for (const auto& point : path) {
    if (!is_finite(point)) {
      throw std::invalid_argument("path points must be finite");
    }
  }
  for (std::size_t i = 1; i < path.size(); ++i) {
    if (path[i].x == path[i - 1].x && path[i].y == path[i - 1].y) {
      throw std::invalid_argument("adjacent path points must be distinct");
    }
  }
}

Projection nearest_projection(const Pose2D& pose,
                              const std::vector<PathPoint>& path) {
  Projection best;
  bool has_best = false;
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const double dx = detail::checked_subtract(
        path[i + 1].x, path[i].x, "path segment x difference");
    const double dy = detail::checked_subtract(
        path[i + 1].y, path[i].y, "path segment y difference");
    const double length =
        detail::checked_hypot(dx, dy, "path segment length");
    const double unit_x =
        detail::checked_divide(dx, length, "path segment x direction");
    const double unit_y =
        detail::checked_divide(dy, length, "path segment y direction");
    const double offset_x = detail::checked_subtract(
        pose.x, path[i].x, "path projection x offset");
    const double offset_y = detail::checked_subtract(
        pose.y, path[i].y, "path projection y offset");

    double t = 0.0;
    double dx_square = 0.0;
    double dy_square = 0.0;
    double length_squared = 0.0;
    double x_projection = 0.0;
    double y_projection = 0.0;
    double projection_numerator = 0.0;
    const bool direct_projection =
        detail::try_multiply(dx, dx, dx_square) &&
        detail::try_multiply(dy, dy, dy_square) &&
        detail::try_add(dx_square, dy_square, length_squared) &&
        length_squared > 0.0 &&
        detail::try_multiply(offset_x, dx, x_projection) &&
        detail::try_multiply(offset_y, dy, y_projection) &&
        detail::try_add(
            x_projection, y_projection, projection_numerator);
    const double projection_length = direct_projection
        ? detail::require_finite_result(std::sqrt(length_squared),
                                        "path segment length")
        : length;
    if (direct_projection) {
      t = std::clamp(
          detail::checked_divide(projection_numerator, length_squared,
                                 "path projection fraction"),
          0.0, 1.0);
    } else {
      const double along = detail::checked_add(
          detail::checked_multiply(offset_x, unit_x,
                                   "scaled path x projection"),
          detail::checked_multiply(offset_y, unit_y,
                                   "scaled path y projection"),
          "scaled path projection");
      t = std::clamp(
          detail::checked_divide(along, length,
                                 "scaled path projection fraction"),
          0.0, 1.0);
    }

    const PathPoint projected{
        detail::checked_add(
            path[i].x,
            detail::checked_multiply(t, dx, "projected path x increment"),
            "projected path x coordinate"),
        detail::checked_add(
            path[i].y,
            detail::checked_multiply(t, dy, "projected path y increment"),
            "projected path y coordinate")};
    const double ex = detail::checked_subtract(
        pose.x, projected.x, "path cross-track x difference");
    const double ey = detail::checked_subtract(
        pose.y, projected.y, "path cross-track y difference");
    const double distance =
        detail::checked_hypot(ex, ey, "path projection distance");

    double signed_error = 0.0;
    double direct_y = 0.0;
    double direct_x = 0.0;
    double direct_cross = 0.0;
    const bool direct_cross_track =
        detail::try_multiply(dy, ex, direct_y) &&
        detail::try_multiply(dx, ey, direct_x) &&
        detail::try_add(direct_y, -direct_x, direct_cross);
    if (direct_cross_track) {
      signed_error = detail::checked_divide(
          direct_cross, projection_length, "signed cross-track error");
    } else {
      signed_error = detail::checked_subtract(
          detail::checked_multiply(unit_y, ex,
                                   "scaled cross-track y term"),
          detail::checked_multiply(unit_x, ey,
                                   "scaled cross-track x term"),
          "scaled signed cross-track error");
    }

    if (!has_best || distance < best.distance) {
      best = {projected,
              i,
              distance,
              signed_error,
              detail::require_finite_result(std::atan2(dy, dx),
                                            "path segment heading")};
      has_best = true;
    }
  }
  if (!has_best || !is_finite(best.point) ||
      !std::isfinite(best.distance) ||
      !std::isfinite(best.signed_error) ||
      !std::isfinite(best.heading)) {
    detail::throw_unrepresentable("nearest path projection");
  }
  return best;
}

void require_positive(double value, const char* message) {
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(message);
  }
}

}  // namespace

PurePursuit::PurePursuit(PurePursuitConfig config) : config_(config) {
  require_positive(config_.wheelbase, "wheelbase must be finite and positive");
  require_positive(config_.lookahead_distance,
                   "lookahead_distance must be finite and positive");
  require_positive(config_.max_steering_angle,
                   "max_steering_angle must be finite and positive");
  if (config_.max_steering_angle >= pi / 2.0) {
    throw std::invalid_argument("max_steering_angle must be less than pi/2");
  }
}

TrackingResult PurePursuit::compute(
    const Pose2D& pose, const std::vector<PathPoint>& path) const {
  if (!is_finite(pose)) {
    throw std::invalid_argument("pose must be finite");
  }
  validate_path(path);
  const Projection nearest = nearest_projection(pose, path);

  double remaining = config_.lookahead_distance;
  PathPoint cursor = nearest.point;
  PathPoint target = path.back();
  std::size_t target_index = path.size() - 1;
  for (std::size_t segment = nearest.segment;
       segment + 1 < path.size(); ++segment) {
    const PathPoint end = path[segment + 1];
    const double dx = detail::checked_subtract(
        end.x, cursor.x, "lookahead segment x difference");
    const double dy = detail::checked_subtract(
        end.y, cursor.y, "lookahead segment y difference");
    const double length =
        detail::checked_hypot(dx, dy, "lookahead segment length");
    if (remaining <= length && length > 0.0) {
      const double fraction = detail::checked_divide(
          remaining, length, "lookahead interpolation fraction");
      target = {
          detail::checked_add(
              cursor.x,
              detail::checked_multiply(
                  fraction, dx, "lookahead target x increment"),
              "lookahead target x coordinate"),
          detail::checked_add(
              cursor.y,
              detail::checked_multiply(
                  fraction, dy, "lookahead target y increment"),
              "lookahead target y coordinate")};
      target_index = segment + 1;
      break;
    }
    remaining = detail::checked_subtract(
        remaining, length, "remaining lookahead distance");
    cursor = end;
  }

  const double target_dx = detail::checked_subtract(
      target.x, pose.x, "pursuit target x difference");
  const double target_dy = detail::checked_subtract(
      target.y, pose.y, "pursuit target y difference");
  const double target_heading = detail::require_finite_result(
      std::atan2(target_dy, target_dx), "pursuit target heading");
  const double alpha = normalize_angle(detail::checked_subtract(
      target_heading, pose.heading, "pursuit heading error"));
  const double target_distance = std::max(
      1e-9, detail::checked_hypot(target_dx, target_dy,
                                  "pursuit target distance"));
  const double alpha_sine = detail::require_finite_result(
      std::sin(alpha), "pursuit heading-error sine");
  double doubled_wheelbase = 0.0;
  double raw_steering = 0.0;
  if (detail::try_multiply(2.0, config_.wheelbase, doubled_wheelbase)) {
    raw_steering = detail::checked_atan2_product(
        doubled_wheelbase, alpha_sine, target_distance,
        "pure-pursuit steering");
  } else {
    raw_steering = detail::checked_atan2_product(
        config_.wheelbase,
        detail::checked_multiply(2.0, alpha_sine,
                                 "scaled pursuit sine"),
        target_distance, "scaled pure-pursuit steering");
  }
  const double steering = std::clamp(raw_steering,
                                     -config_.max_steering_angle,
                                     config_.max_steering_angle);
  TrackingResult result{
      steering,
      nearest.signed_error,
      normalize_angle(detail::checked_subtract(
          nearest.heading, pose.heading, "path heading error")),
      target,
      target_index};
  if (!std::isfinite(result.steering_angle) ||
      !std::isfinite(result.cross_track_error) ||
      !std::isfinite(result.heading_error) || !is_finite(result.target)) {
    detail::throw_unrepresentable("pure-pursuit result");
  }
  return result;
}

StanleyController::StanleyController(StanleyConfig config) : config_(config) {
  if (!std::isfinite(config_.gain) || config_.gain < 0.0) {
    throw std::invalid_argument("gain must be finite and non-negative");
  }
  require_positive(config_.speed_softening,
                   "speed_softening must be finite and positive");
  require_positive(config_.max_steering_angle,
                   "max_steering_angle must be finite and positive");
  if (config_.max_steering_angle >= pi / 2.0) {
    throw std::invalid_argument("max_steering_angle must be less than pi/2");
  }
}

TrackingResult StanleyController::compute(
    const Pose2D& pose, double speed,
    const std::vector<PathPoint>& path) const {
  if (!is_finite(pose) || !std::isfinite(speed)) {
    throw std::invalid_argument("pose and speed must be finite");
  }
  validate_path(path);
  const Projection nearest = nearest_projection(pose, path);
  const double heading_error = normalize_angle(detail::checked_subtract(
      nearest.heading, pose.heading, "Stanley heading error"));
  const double softened_speed = detail::checked_add(
      std::abs(speed), config_.speed_softening, "Stanley softened speed");
  const double correction = detail::checked_atan2_product(
      config_.gain, nearest.signed_error, softened_speed,
      "Stanley cross-track correction");
  const double steering = std::clamp(
      normalize_angle(detail::checked_add(
          heading_error, correction, "Stanley steering sum")),
      -config_.max_steering_angle, config_.max_steering_angle);
  TrackingResult result{steering, nearest.signed_error, heading_error,
                        nearest.point, nearest.segment + 1};
  if (!std::isfinite(result.steering_angle) ||
      !std::isfinite(result.cross_track_error) ||
      !std::isfinite(result.heading_error) || !is_finite(result.target)) {
    detail::throw_unrepresentable("Stanley result");
  }
  return result;
}

}  // namespace motion_core
