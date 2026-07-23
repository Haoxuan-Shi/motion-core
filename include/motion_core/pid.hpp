#ifndef MOTION_CORE_PID_HPP
#define MOTION_CORE_PID_HPP

namespace motion_core {

struct PIDConfig {
  double kp{1.0};
  double ki{0.0};
  double kd{0.0};
  double output_min{-1.0};
  double output_max{1.0};
  double integral_min{-10.0};
  double integral_max{10.0};
  double anti_windup_gain{1.0};
};

struct PIDResult {
  double output{0.0};
  double error{0.0};
  double derivative{0.0};
  double integral{0.0};
  bool saturated{false};
};

class PIDController {
 public:
  explicit PIDController(PIDConfig config = {});
  PIDResult update(double setpoint, double measurement, double dt);
  void reset(double integral = 0.0);
  const PIDConfig& config() const noexcept { return config_; }
  double integral() const noexcept { return integral_; }

 private:
  PIDConfig config_;
  double integral_{0.0};
  double previous_error_{0.0};
  bool has_previous_{false};
};

}  // namespace motion_core

#endif

