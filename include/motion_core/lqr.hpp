#ifndef MOTION_CORE_LQR_HPP
#define MOTION_CORE_LQR_HPP

#include <array>
#include <cstddef>
#include <vector>

namespace motion_core {

struct DoubleIntegratorState {
  double position{0.0};
  double velocity{0.0};
};

struct LQRConfig {
  std::size_t horizon{30};
  double dt{0.1};
  double position_weight{1.0};
  double velocity_weight{0.2};
  double control_weight{0.1};
  double terminal_position_weight{5.0};
  double terminal_velocity_weight{1.0};
  double max_acceleration{10.0};
};

class FiniteHorizonLQR {
 public:
  explicit FiniteHorizonLQR(LQRConfig config = {});

  double control(const DoubleIntegratorState& state,
                 std::size_t step = 0) const;
  DoubleIntegratorState propagate(const DoubleIntegratorState& state,
                                  double acceleration) const;
  const std::vector<std::array<double, 2>>& gains() const noexcept {
    return gains_;
  }
  const LQRConfig& config() const noexcept { return config_; }

 private:
  LQRConfig config_;
  std::vector<std::array<double, 2>> gains_;
};

}  // namespace motion_core

#endif

