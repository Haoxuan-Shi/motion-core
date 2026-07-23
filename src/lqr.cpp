#include "motion_core/lqr.hpp"

#include "numeric.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace motion_core {
namespace {

void require_nonnegative_finite(double value, const char* name) {
  if (!std::isfinite(value) || value < 0.0) {
    throw std::invalid_argument(std::string(name) +
                                " must be finite and non-negative");
  }
}

}  // namespace

FiniteHorizonLQR::FiniteHorizonLQR(LQRConfig config) : config_(config) {
  if (config_.horizon == 0 || config_.horizon > 100000) {
    throw std::invalid_argument("horizon must be in [1, 100000]");
  }
  if (!std::isfinite(config_.dt) || config_.dt <= 0.0) {
    throw std::invalid_argument("dt must be finite and positive");
  }
  require_nonnegative_finite(config_.position_weight, "position_weight");
  require_nonnegative_finite(config_.velocity_weight, "velocity_weight");
  require_nonnegative_finite(config_.terminal_position_weight,
                             "terminal_position_weight");
  require_nonnegative_finite(config_.terminal_velocity_weight,
                             "terminal_velocity_weight");
  if (!std::isfinite(config_.control_weight) ||
      config_.control_weight <= 0.0) {
    throw std::invalid_argument("control_weight must be finite and positive");
  }
  if (!std::isfinite(config_.max_acceleration) ||
      config_.max_acceleration <= 0.0) {
    throw std::invalid_argument("max_acceleration must be finite and positive");
  }

  gains_.resize(config_.horizon);
  const double dt = config_.dt;
  const double b0 = detail::checked_multiply(
      detail::checked_multiply(0.5, dt, "LQR half time step"), dt,
      "LQR position input coefficient");
  const double b1 = dt;
  double p00 = config_.terminal_position_weight;
  double p01 = 0.0;
  double p11 = config_.terminal_velocity_weight;

  for (std::size_t reverse = config_.horizon; reverse > 0; --reverse) {
    const std::size_t k = reverse - 1;
    const double r0 = detail::checked_add(
        detail::checked_multiply(b0, p00, "LQR r0 position term"),
        detail::checked_multiply(b1, p01, "LQR r0 velocity term"),
        "LQR r0 sum");
    const double p00_dt =
        detail::checked_multiply(p00, dt, "LQR p00 time product");
    const double p01_dt =
        detail::checked_multiply(p01, dt, "LQR p01 time product");
    const double r1 = detail::checked_add(
        detail::checked_multiply(
            b0, detail::checked_add(p00_dt, p01, "LQR r1 position sum"),
            "LQR r1 position term"),
        detail::checked_multiply(
            b1, detail::checked_add(p01_dt, p11, "LQR r1 velocity sum"),
            "LQR r1 velocity term"),
        "LQR r1 sum");

    const double denominator_position = detail::checked_multiply(
        detail::checked_multiply(b0, b0, "LQR denominator b0 square"),
        p00, "LQR denominator position term");
    const double denominator_cross = detail::checked_multiply(
        detail::checked_multiply(
            detail::checked_multiply(2.0, b0,
                                     "LQR denominator doubled b0"),
            b1, "LQR denominator input cross product"),
        p01, "LQR denominator cross term");
    const double denominator_velocity = detail::checked_multiply(
        detail::checked_multiply(b1, b1, "LQR denominator b1 square"),
        p11, "LQR denominator velocity term");
    double denominator = detail::checked_add(
        config_.control_weight, denominator_position,
        "LQR denominator position sum");
    denominator = detail::checked_add(
        denominator, denominator_cross, "LQR denominator cross sum");
    denominator = detail::checked_add(
        denominator, denominator_velocity, "LQR denominator");
    if (denominator <= 0.0) {
      throw std::invalid_argument("Riccati denominator must remain positive");
    }
    gains_[k] = {
        detail::checked_divide(r0, denominator, "LQR position gain"),
        detail::checked_divide(r1, denominator, "LQR velocity gain")};

    const double apa00 = p00;
    const double apa01 =
        detail::checked_add(p00_dt, p01, "LQR propagated cost cross term");
    const double apa11_position = detail::checked_multiply(
        detail::checked_multiply(dt, dt, "LQR time-step square"), p00,
        "LQR propagated position cost");
    const double apa11_cross = detail::checked_multiply(
        detail::checked_multiply(2.0, dt, "LQR doubled time step"), p01,
        "LQR propagated cross cost");
    const double apa11 = detail::checked_add(
        detail::checked_add(apa11_position, apa11_cross,
                            "LQR propagated velocity partial sum"),
        p11, "LQR propagated velocity cost");
    p00 = detail::checked_subtract(
        detail::checked_add(config_.position_weight, apa00,
                            "LQR position cost sum"),
        detail::checked_square_quotient(
            r0, denominator, "LQR position Riccati correction"),
        "LQR updated position cost");
    p01 = detail::checked_subtract(
        apa01,
        detail::checked_product_quotient(
            r0, r1, denominator, "LQR cross Riccati correction"),
        "LQR updated cross cost");
    p11 = detail::checked_subtract(
        detail::checked_add(config_.velocity_weight, apa11,
                            "LQR velocity cost sum"),
        detail::checked_square_quotient(
            r1, denominator, "LQR velocity Riccati correction"),
        "LQR updated velocity cost");
  }
  for (const auto& gain : gains_) {
    detail::require_finite_result(gain[0], "LQR position gain sequence");
    detail::require_finite_result(gain[1], "LQR velocity gain sequence");
  }
}

double FiniteHorizonLQR::control(const DoubleIntegratorState& state,
                                 std::size_t step) const {
  if (!std::isfinite(state.position) || !std::isfinite(state.velocity)) {
    throw std::invalid_argument("state must be finite");
  }
  if (step >= gains_.size()) {
    throw std::out_of_range("LQR step is outside the configured horizon");
  }
  const auto& gain = gains_[step];
  const double raw = -detail::checked_add(
      detail::checked_multiply(gain[0], state.position,
                               "LQR position feedback"),
      detail::checked_multiply(gain[1], state.velocity,
                               "LQR velocity feedback"),
      "LQR feedback sum");
  return detail::require_finite_result(
      std::clamp(raw, -config_.max_acceleration, config_.max_acceleration),
      "LQR control");
}

DoubleIntegratorState FiniteHorizonLQR::propagate(
    const DoubleIntegratorState& state, double acceleration) const {
  if (!std::isfinite(state.position) || !std::isfinite(state.velocity) ||
      !std::isfinite(acceleration)) {
    throw std::invalid_argument("state and acceleration must be finite");
  }
  const double applied = std::clamp(acceleration, -config_.max_acceleration,
                                    config_.max_acceleration);
  const double velocity_displacement = detail::checked_multiply(
      config_.dt, state.velocity, "LQR velocity displacement");
  const double position_input = detail::checked_multiply(
      detail::checked_multiply(
          detail::checked_multiply(0.5, config_.dt, "LQR half time step"),
          config_.dt, "LQR position input coefficient"),
      applied, "LQR acceleration displacement");
  DoubleIntegratorState result{
      detail::checked_add(
          detail::checked_add(state.position, velocity_displacement,
                              "LQR propagated position without control"),
          position_input, "LQR propagated position"),
      detail::checked_add(
          state.velocity,
          detail::checked_multiply(config_.dt, applied,
                                   "LQR velocity input"),
          "LQR propagated velocity")};
  if (!std::isfinite(result.position) || !std::isfinite(result.velocity)) {
    detail::throw_unrepresentable("LQR propagated state");
  }
  return result;
}

}  // namespace motion_core
