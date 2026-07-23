#ifndef MOTION_CORE_PREDICTIVE_CONTROLLER_HPP
#define MOTION_CORE_PREDICTIVE_CONTROLLER_HPP

#include <cstddef>
#include <vector>

namespace motion_core {

struct LongitudinalMPCConfig {
  std::size_t horizon{20};
  double dt{0.1};
  double speed_weight{2.0};
  double terminal_speed_weight{4.0};
  double acceleration_weight{0.1};
  double jerk_weight{0.2};
  double min_acceleration{-4.0};
  double max_acceleration{2.5};
  double max_jerk{3.0};
  std::size_t iterations{160};
};

struct LongitudinalPlan {
  double acceleration{0.0};
  double objective{0.0};
  std::vector<double> accelerations;
  std::vector<double> predicted_speeds;
};

class LongitudinalPredictiveController {
 public:
  explicit LongitudinalPredictiveController(LongitudinalMPCConfig config = {});
  LongitudinalPlan plan(double current_speed, double target_speed,
                        double previous_acceleration = 0.0) const;
  const LongitudinalMPCConfig& config() const noexcept { return config_; }

 private:
  LongitudinalMPCConfig config_;
};

}  // namespace motion_core

#endif

