#ifndef MOTION_CORE_PATH_TRACKING_HPP
#define MOTION_CORE_PATH_TRACKING_HPP

#include "motion_core/common.hpp"

#include <cstddef>
#include <vector>

namespace motion_core {

struct TrackingResult {
  double steering_angle{0.0};
  double cross_track_error{0.0};
  double heading_error{0.0};
  PathPoint target{};
  std::size_t target_index{0};
};

struct PurePursuitConfig {
  double wheelbase{2.7};
  double lookahead_distance{3.0};
  double max_steering_angle{0.6};
};

class PurePursuit {
 public:
  explicit PurePursuit(PurePursuitConfig config = {});
  TrackingResult compute(const Pose2D& pose,
                         const std::vector<PathPoint>& path) const;

 private:
  PurePursuitConfig config_;
};

struct StanleyConfig {
  double gain{1.0};
  double speed_softening{0.5};
  double max_steering_angle{0.6};
};

class StanleyController {
 public:
  explicit StanleyController(StanleyConfig config = {});
  TrackingResult compute(const Pose2D& pose, double speed,
                         const std::vector<PathPoint>& path) const;

 private:
  StanleyConfig config_;
};

}  // namespace motion_core

#endif

