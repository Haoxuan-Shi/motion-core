#include "motion_core/predictive_controller.hpp"

#include "numeric.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace motion_core {
namespace {

void require_nonnegative(double value, const char* name) {
  if (!std::isfinite(value) || value < 0.0) {
    throw std::invalid_argument(std::string(name) +
                                " must be finite and non-negative");
  }
}

void project_controls(std::vector<double>& accelerations,
                       double previous_acceleration,
                       const LongitudinalMPCConfig& config) {
  double previous = std::clamp(previous_acceleration,
                               config.min_acceleration,
                               config.max_acceleration);
  const double max_delta = detail::checked_multiply(
      config.max_jerk, config.dt, "predictive jerk-step limit");
  for (double& acceleration : accelerations) {
    const double lower = std::max(
        config.min_acceleration,
        detail::checked_subtract(previous, max_delta,
                                 "predictive lower jerk bound"));
    const double upper = std::min(
        config.max_acceleration,
        detail::checked_add(previous, max_delta,
                            "predictive upper jerk bound"));
    acceleration = std::clamp(acceleration, lower, upper);
    detail::require_finite_result(acceleration,
                                  "projected predictive acceleration");
    previous = acceleration;
  }
}

double solver_lipschitz(const LongitudinalMPCConfig& config) {
  const double n = static_cast<double>(config.horizon);
  const double acceleration_term = detail::checked_multiply(
      2.0, config.acceleration_weight,
      "predictive acceleration Lipschitz term");
  double jerk_term = 0.0;
  if (config.jerk_weight != 0.0) {
    const double dt_squared = detail::checked_multiply(
        config.dt, config.dt, "predictive jerk time-step square");
    jerk_term = detail::checked_divide(
        detail::checked_multiply(
            8.0, config.jerk_weight,
            "predictive jerk Lipschitz numerator"),
        dt_squared, "predictive jerk Lipschitz term");
  }
  double speed_term = 0.0;
  if (config.speed_weight != 0.0) {
    speed_term = detail::checked_multiply(
        2.0, config.speed_weight, "predictive speed Lipschitz weight");
    speed_term = detail::checked_multiply(
        speed_term, config.dt, "predictive speed Lipschitz first time term");
    speed_term = detail::checked_multiply(
        speed_term, config.dt, "predictive speed Lipschitz second time term");
    speed_term = detail::checked_multiply(
        speed_term, n, "predictive speed Lipschitz first horizon term");
    speed_term = detail::checked_multiply(
        speed_term, n, "predictive speed Lipschitz second horizon term");
  }
  double terminal_term = 0.0;
  if (config.terminal_speed_weight != 0.0) {
    terminal_term = detail::checked_multiply(
        2.0, config.terminal_speed_weight,
        "predictive terminal Lipschitz weight");
    terminal_term = detail::checked_multiply(
        terminal_term, config.dt,
        "predictive terminal Lipschitz first time term");
    terminal_term = detail::checked_multiply(
        terminal_term, config.dt,
        "predictive terminal Lipschitz second time term");
    terminal_term = detail::checked_multiply(
        terminal_term, n, "predictive terminal Lipschitz horizon term");
  }

  return detail::checked_add(
      detail::checked_add(
          detail::checked_add(acceleration_term, jerk_term,
                              "predictive Lipschitz control sum"),
          speed_term, "predictive Lipschitz running-cost sum"),
      terminal_term, "predictive Lipschitz constant");
}

double evaluate(const std::vector<double>& accelerations,
                double current_speed, double target_speed,
                double previous_acceleration,
                const LongitudinalMPCConfig& config,
                std::vector<double>* speeds) {
  double objective = 0.0;
  double speed = current_speed;
  double previous = previous_acceleration;
  if (speeds != nullptr) {
    speeds->clear();
    speeds->reserve(accelerations.size() + 1);
    speeds->push_back(speed);
  }
  for (double acceleration : accelerations) {
    speed = detail::checked_add(
        speed,
        detail::checked_multiply(config.dt, acceleration,
                                 "predictive rollout speed increment"),
        "predictive rollout speed");
    double speed_cost = 0.0;
    if (config.speed_weight != 0.0) {
      const double speed_error = detail::checked_subtract(
          speed, target_speed, "predictive speed error");
      speed_cost = detail::checked_weighted_square(
          config.speed_weight, speed_error, "predictive speed cost");
    }
    const double acceleration_cost = detail::checked_weighted_square(
        config.acceleration_weight, acceleration,
        "predictive acceleration cost");
    double jerk_cost = 0.0;
    if (config.jerk_weight != 0.0) {
      const double jerk = detail::checked_divide(
          detail::checked_subtract(acceleration, previous,
                                   "predictive acceleration difference"),
          config.dt, "predictive jerk");
      jerk_cost = detail::checked_weighted_square(
          config.jerk_weight, jerk, "predictive jerk cost");
    }
    const double stage_cost = detail::checked_add(
        detail::checked_add(speed_cost, acceleration_cost,
                            "predictive stage speed-control cost"),
        jerk_cost, "predictive stage cost");
    objective = detail::checked_add(
        objective, stage_cost, "predictive running objective");
    previous = acceleration;
    if (speeds != nullptr) {
      speeds->push_back(speed);
    }
  }
  if (config.terminal_speed_weight != 0.0) {
    const double terminal_error = detail::checked_subtract(
        speed, target_speed, "predictive terminal speed error");
    objective = detail::checked_add(
        objective,
        detail::checked_weighted_square(
            config.terminal_speed_weight, terminal_error,
            "predictive terminal speed cost"),
        "predictive objective");
  }
  return detail::require_finite_result(objective, "predictive objective");
}

}  // namespace

LongitudinalPredictiveController::LongitudinalPredictiveController(
    LongitudinalMPCConfig config)
    : config_(config) {
  if (config_.horizon == 0 || config_.horizon > 10000) {
    throw std::invalid_argument("horizon must be in [1, 10000]");
  }
  if (!std::isfinite(config_.dt) || config_.dt <= 0.0) {
    throw std::invalid_argument("dt must be finite and positive");
  }
  require_nonnegative(config_.speed_weight, "speed_weight");
  require_nonnegative(config_.terminal_speed_weight,
                      "terminal_speed_weight");
  require_nonnegative(config_.acceleration_weight, "acceleration_weight");
  require_nonnegative(config_.jerk_weight, "jerk_weight");
  if (config_.speed_weight == 0.0 &&
      config_.terminal_speed_weight == 0.0 &&
      config_.acceleration_weight == 0.0 && config_.jerk_weight == 0.0) {
    throw std::invalid_argument("at least one cost weight must be positive");
  }
  if (!std::isfinite(config_.min_acceleration) ||
      !std::isfinite(config_.max_acceleration) ||
      config_.min_acceleration >= config_.max_acceleration) {
    throw std::invalid_argument(
        "acceleration bounds must be finite and strictly ordered");
  }
  if (!std::isfinite(config_.max_jerk) || config_.max_jerk <= 0.0) {
    throw std::invalid_argument("max_jerk must be finite and positive");
  }
  if (config_.iterations == 0 || config_.iterations > 100000) {
    throw std::invalid_argument("iterations must be in [1, 100000]");
  }

  (void)detail::checked_multiply(
      config_.max_jerk, config_.dt, "predictive jerk-step limit");
  (void)detail::checked_multiply(
      config_.dt, config_.min_acceleration,
      "predictive minimum speed increment");
  (void)detail::checked_multiply(
      config_.dt, config_.max_acceleration,
      "predictive maximum speed increment");
  (void)solver_lipschitz(config_);
}

LongitudinalPlan LongitudinalPredictiveController::plan(
    double current_speed, double target_speed,
    double previous_acceleration) const {
  if (!std::isfinite(current_speed) || current_speed < 0.0 ||
      !std::isfinite(target_speed) || target_speed < 0.0 ||
      !std::isfinite(previous_acceleration)) {
    throw std::invalid_argument(
        "speeds must be finite and non-negative; acceleration must be finite");
  }

  const double speed_difference = detail::checked_subtract(
      target_speed, current_speed, "predictive initial speed difference");
  const double n = static_cast<double>(config_.horizon);
  double horizon_duration = 0.0;
  double initial = 0.0;
  if (detail::try_multiply(config_.dt, n, horizon_duration)) {
    initial = detail::checked_divide(
        speed_difference, horizon_duration,
        "predictive initial acceleration");
  } else {
    initial = detail::checked_divide(
        detail::checked_divide(
            speed_difference, config_.dt,
            "scaled predictive initial acceleration"),
        n, "scaled predictive horizon acceleration");
  }
  const double effective_previous = std::clamp(
      previous_acceleration, config_.min_acceleration, config_.max_acceleration);
  std::vector<double> accelerations(config_.horizon, initial);
  project_controls(accelerations, effective_previous, config_);

  const double lipschitz = solver_lipschitz(config_);
  const double step_size = detail::checked_divide(
      0.8, std::max(lipschitz, 1e-12),
      "predictive gradient step size");
  std::vector<double> speeds(config_.horizon);
  std::vector<double> gradient(config_.horizon);

  for (std::size_t iteration = 0; iteration < config_.iterations;
       ++iteration) {
    double speed = current_speed;
    for (std::size_t k = 0; k < config_.horizon; ++k) {
      speed = detail::checked_add(
          speed,
          detail::checked_multiply(config_.dt, accelerations[k],
                                   "predictive gradient speed increment"),
          "predictive gradient rollout speed");
      speeds[k] = speed;
      gradient[k] = detail::checked_multiply(
          detail::checked_multiply(
              2.0, config_.acceleration_weight,
              "predictive acceleration gradient weight"),
          accelerations[k], "predictive acceleration gradient");
    }

    double future_error_sum = 0.0;
    if (config_.terminal_speed_weight != 0.0) {
      future_error_sum = detail::checked_multiply(
          config_.terminal_speed_weight,
          detail::checked_subtract(
              speeds.back(), target_speed,
              "predictive terminal gradient speed error"),
          "predictive terminal gradient error");
    }
    for (std::size_t reverse = config_.horizon; reverse > 0; --reverse) {
      const std::size_t k = reverse - 1;
      if (config_.speed_weight != 0.0) {
        future_error_sum = detail::checked_add(
            future_error_sum,
            detail::checked_multiply(
                config_.speed_weight,
                detail::checked_subtract(
                    speeds[k], target_speed,
                    "predictive running gradient speed error"),
                "predictive weighted running speed error"),
            "predictive future error sum");
      }
      if (future_error_sum != 0.0) {
        gradient[k] = detail::checked_add(
            gradient[k],
            detail::checked_multiply(
                detail::checked_multiply(
                    2.0, config_.dt,
                    "predictive doubled gradient time step"),
                future_error_sum, "predictive tracking gradient"),
            "predictive control-tracking gradient");
      }
    }

    double jerk_scale = 0.0;
    if (config_.jerk_weight != 0.0) {
      jerk_scale = detail::checked_divide(
          detail::checked_multiply(
              2.0, config_.jerk_weight,
              "predictive jerk-gradient numerator"),
          detail::checked_multiply(
              config_.dt, config_.dt,
              "predictive jerk-gradient time square"),
          "predictive jerk-gradient scale");
    }
    for (std::size_t k = 0; k < config_.horizon; ++k) {
      const double before =
          k == 0 ? effective_previous : accelerations[k - 1];
      if (jerk_scale != 0.0) {
        gradient[k] = detail::checked_add(
            gradient[k],
            detail::checked_multiply(
                jerk_scale,
                detail::checked_subtract(
                    accelerations[k], before,
                    "predictive backward acceleration difference"),
                "predictive backward jerk gradient"),
            "predictive gradient with backward jerk");
        if (k + 1 < config_.horizon) {
          gradient[k] = detail::checked_subtract(
              gradient[k],
              detail::checked_multiply(
                  jerk_scale,
                  detail::checked_subtract(
                      accelerations[k + 1], accelerations[k],
                      "predictive forward acceleration difference"),
                  "predictive forward jerk gradient"),
              "predictive gradient with forward jerk");
        }
      }
    }

    for (std::size_t k = 0; k < config_.horizon; ++k) {
      accelerations[k] = detail::checked_subtract(
          accelerations[k],
          detail::checked_multiply(step_size, gradient[k],
                                   "predictive gradient update"),
          "updated predictive acceleration");
    }
    project_controls(accelerations, effective_previous, config_);
  }

  LongitudinalPlan result;
  result.accelerations = accelerations;
  result.acceleration = accelerations.front();
  result.objective = evaluate(accelerations, current_speed, target_speed,
                              effective_previous, config_,
                              &result.predicted_speeds);
  if (!std::isfinite(result.acceleration) ||
      !std::isfinite(result.objective)) {
    detail::throw_unrepresentable("predictive plan");
  }
  for (double acceleration : result.accelerations) {
    detail::require_finite_result(acceleration,
                                  "predictive acceleration sequence");
  }
  for (double speed : result.predicted_speeds) {
    detail::require_finite_result(speed, "predictive speed sequence");
  }
  return result;
}

}  // namespace motion_core
