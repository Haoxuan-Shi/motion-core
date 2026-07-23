#include <motion_core/motion_core.hpp>

#include <cmath>

int main() {
  motion_core::DifferentialDrive model({2.0, 1.0});
  const auto result = model.step({}, 1.0, 0.0, 0.1);
  return std::abs(result.pose.x - 0.1) < 1e-12 ? 0 : 1;
}

