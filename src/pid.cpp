#include "motion_core/pid.hpp"

#include "numeric.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace motion_core {
namespace {

void require_finite(double value, const char* name) {
  if (!std::isfinite(value)) {
    throw std::invalid_argument(std::string(name) + " must be finite");
  }
}

}  // namespace

PIDController::PIDController(PIDConfig config) : config_(config) {
  require_finite(config_.kp, "kp");
  require_finite(config_.ki, "ki");
  require_finite(config_.kd, "kd");
  require_finite(config_.output_min, "output_min");
  require_finite(config_.output_max, "output_max");
  require_finite(config_.integral_min, "integral_min");
  require_finite(config_.integral_max, "integral_max");
  require_finite(config_.anti_windup_gain, "anti_windup_gain");
  if (config_.output_min >= config_.output_max) {
    throw std::invalid_argument("output_min must be less than output_max");
  }
  if (config_.integral_min > config_.integral_max) {
    throw std::invalid_argument("integral_min must not exceed integral_max");
  }
  if (config_.anti_windup_gain < 0.0) {
    throw std::invalid_argument("anti_windup_gain must be non-negative");
  }
}

PIDResult PIDController::update(double setpoint, double measurement,
                                double dt) {
  require_finite(setpoint, "setpoint");
  require_finite(measurement, "measurement");
  if (!std::isfinite(dt) || dt <= 0.0) {
    throw std::invalid_argument("dt must be finite and positive");
  }

  const double error =
      detail::checked_subtract(setpoint, measurement, "PID error");
  const double derivative = has_previous_
      ? detail::checked_divide(
            detail::checked_subtract(error, previous_error_,
                                     "PID error difference"),
            dt, "PID derivative")
      : 0.0;
  const double integral_increment =
      detail::checked_multiply(error, dt, "PID integral increment");
  double candidate = std::clamp(
      detail::checked_add(integral_, integral_increment,
                          "PID integral candidate"),
      config_.integral_min, config_.integral_max);

  auto raw_output = [&](double integral) {
    const double proportional =
        detail::checked_multiply(config_.kp, error, "PID proportional term");
    const double integral_term = detail::checked_multiply(
        config_.ki, integral, "PID integral term");
    const double derivative_term = detail::checked_multiply(
        config_.kd, derivative, "PID derivative term");
    return detail::checked_add(
        detail::checked_add(proportional, integral_term,
                            "PID proportional-integral sum"),
        derivative_term, "PID control sum");
  };

  const double previous_unsaturated = raw_output(integral_);
  double unsaturated = raw_output(candidate);
  double output =
      std::clamp(unsaturated, config_.output_min, config_.output_max);
  const bool pushes_high = unsaturated > config_.output_max &&
                           unsaturated > previous_unsaturated;
  const bool pushes_low = unsaturated < config_.output_min &&
                          unsaturated < previous_unsaturated;
  if (pushes_high || pushes_low) {
    candidate = integral_;
    unsaturated = raw_output(candidate);
    output = std::clamp(unsaturated, config_.output_min, config_.output_max);
  }

  if (config_.anti_windup_gain > 0.0 && config_.ki != 0.0) {
    const double back_calculation_error = detail::checked_subtract(
        output, unsaturated, "PID anti-windup error");
    double weighted_correction = detail::checked_multiply(
        config_.anti_windup_gain, back_calculation_error,
        "PID anti-windup weighted correction");
    if (config_.ki < 0.0) {
      weighted_correction = -weighted_correction;
    }
    const double correction = detail::checked_multiply(
        weighted_correction, dt, "PID anti-windup correction");
    candidate = detail::checked_add(
        candidate, correction, "PID anti-windup integral");
    candidate = std::clamp(candidate, config_.integral_min,
                           config_.integral_max);
    unsaturated = raw_output(candidate);
    output = std::clamp(unsaturated, config_.output_min, config_.output_max);
  }

  const bool saturated = unsaturated < config_.output_min ||
                         unsaturated > config_.output_max;
  PIDResult result{output, error, derivative, candidate, saturated};
  if (!std::isfinite(result.output) || !std::isfinite(result.error) ||
      !std::isfinite(result.derivative) || !std::isfinite(result.integral)) {
    detail::throw_unrepresentable("PID result");
  }
  integral_ = candidate;
  previous_error_ = error;
  has_previous_ = true;
  return result;
}

void PIDController::reset(double integral) {
  require_finite(integral, "integral");
  integral_ =
      std::clamp(integral, config_.integral_min, config_.integral_max);
  previous_error_ = 0.0;
  has_previous_ = false;
}

}  // namespace motion_core
