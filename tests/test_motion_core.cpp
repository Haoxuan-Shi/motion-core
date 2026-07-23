#include "motion_core/motion_core.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

int assertions = 0;

void expect(bool condition, const std::string& message) {
  ++assertions;
  if (!condition) {
    throw std::runtime_error(message);
  }
}

void expect_near(double actual, double expected, double tolerance,
                 const std::string& message) {
  expect(std::abs(actual - expected) <= tolerance,
         message + ": actual=" + std::to_string(actual) +
             " expected=" + std::to_string(expected));
}

void expect_relative_near(double actual, double expected, double tolerance,
                          const std::string& message) {
  expect_near(actual, expected,
              tolerance * std::max(1.0, std::abs(expected)), message);
}

template <typename Exception, typename Callable>
void expect_throws(Callable&& callable, const std::string& message) {
  ++assertions;
  try {
    callable();
  } catch (const Exception&) {
    return;
  } catch (...) {
    throw std::runtime_error(message + " (wrong exception type)");
  }
  throw std::runtime_error(message + " (no exception)");
}

template <typename Exception, typename Callable>
void expect_throws_message(Callable&& callable,
                           const std::string& expected_message,
                           const std::string& message) {
  ++assertions;
  try {
    callable();
  } catch (const Exception& error) {
    if (error.what() == expected_message) {
      return;
    }
    throw std::runtime_error(
        message + " (message=\"" + error.what() + "\")");
  } catch (...) {
    throw std::runtime_error(message + " (wrong exception type)");
  }
  throw std::runtime_error(message + " (no exception)");
}

std::vector<motion_core::PathPoint> straight_path() {
  return {{0.0, 0.0}, {5.0, 0.0}, {10.0, 0.0}, {15.0, 0.0}};
}

void nominal() {
  using namespace motion_core;
  expect_near(normalize_angle(3.0 * pi), -pi, 1e-12,
              "angle normalization uses [-pi, pi)");

  DifferentialDrive differential({3.0, 2.0});
  const auto drive_step = differential.step({0.0, 0.0, 0.0}, 2.0, 0.0, 0.5);
  expect_near(drive_step.pose.x, 1.0, 1e-12,
              "differential drive advances straight");
  expect_near(drive_step.pose.y, 0.0, 1e-12,
              "straight drive has no lateral drift");

  KinematicBicycle bicycle({2.5, 10.0, 0.6});
  const auto bicycle_step = bicycle.step({0.0, 0.0, 0.0}, 4.0, 0.2, 0.1);
  expect(bicycle_step.pose.x > 0.0 && bicycle_step.pose.heading > 0.0,
         "bicycle advances and turns");

  PIDController pid({1.0, 0.5, 0.1, -10.0, 10.0, -5.0, 5.0, 1.0});
  const auto first = pid.update(2.0, 0.0, 0.1);
  const auto second = pid.update(2.0, 0.5, 0.1);
  expect(first.output > 0.0 && std::isfinite(second.output),
         "PID produces finite corrective output");

  FiniteHorizonLQR lqr;
  DoubleIntegratorState state{4.0, 0.0};
  const double control = lqr.control(state);
  expect(control < 0.0, "LQR drives positive position error toward zero");
  state = lqr.propagate(state, control);
  expect(state.velocity < 0.0, "LQR propagation applies acceleration");
  expect(lqr.gains().size() == lqr.config().horizon,
         "Riccati recursion emits one gain per horizon step");

  PurePursuit pure_pursuit({2.7, 3.0, 0.55});
  const auto pursuit =
      pure_pursuit.compute({1.0, -1.0, 0.0}, straight_path());
  expect(pursuit.steering_angle > 0.0,
         "pure pursuit steers toward a path above the vehicle");

  StanleyController stanley({1.2, 0.5, 0.55});
  const auto stanley_result =
      stanley.compute({1.0, 1.0, 0.0}, 3.0, straight_path());
  expect(stanley_result.steering_angle < 0.0,
         "Stanley steers toward a path below the vehicle");

  LongitudinalPredictiveController predictive;
  const auto plan = predictive.plan(1.0, 5.0, 0.0);
  expect(plan.acceleration > 0.0, "predictive controller accelerates toward target");
  expect(plan.predicted_speeds.back() > plan.predicted_speeds.front(),
         "predictive rollout increases speed");
  expect(plan.predicted_speeds.size() ==
             predictive.config().horizon + 1,
         "predictive rollout includes initial speed");
}

void saturation() {
  using namespace motion_core;
  DifferentialDrive drive({1.0, 0.5});
  const auto drive_step = drive.step({0.0, 0.0, 0.0}, 99.0, -99.0, 0.1);
  expect_near(drive_step.linear_speed, 1.0, 0.0,
              "linear command is clamped");
  expect_near(drive_step.angular_speed, -0.5, 0.0,
              "angular command is clamped");

  KinematicBicycle bicycle({2.0, 4.0, 0.4});
  const auto bicycle_step = bicycle.step({0.0, 0.0, 0.0}, -10.0, 2.0, 0.1);
  expect_near(bicycle_step.speed, -4.0, 0.0, "bicycle speed is clamped");
  expect_near(bicycle_step.steering_angle, 0.4, 0.0,
              "bicycle steering is clamped");

  PIDController pid({2.0, 1.0, 0.0, -0.5, 0.5, -0.2, 0.2, 2.0});
  PIDResult result;
  for (int i = 0; i < 200; ++i) {
    result = pid.update(100.0, 0.0, 0.1);
  }
  expect_near(result.output, 0.5, 0.0, "PID output saturates at upper bound");
  expect(pid.integral() >= -0.2 && pid.integral() <= 0.2,
         "anti-windup keeps integral bounded");

  PIDConfig tiny_gain_config;
  tiny_gain_config.kp = 1e-13;
  tiny_gain_config.ki = 0.0;
  tiny_gain_config.kd = 0.0;
  tiny_gain_config.output_min = -5e-14;
  tiny_gain_config.output_max = 5e-14;
  tiny_gain_config.anti_windup_gain = 0.0;
  PIDController tiny_gain(tiny_gain_config);
  const auto tiny_saturation = tiny_gain.update(1.0, 0.0, 1.0);
  expect_near(tiny_saturation.output, 5e-14, 0.0,
              "tiny-gain PID output is clamped exactly");
  expect(tiny_saturation.saturated,
         "tiny-gain PID reports saturation at its configured scale");

  tiny_gain_config.kp = tiny_gain_config.output_max;
  PIDController exact_boundary(tiny_gain_config);
  const auto boundary_result = exact_boundary.update(1.0, 0.0, 1.0);
  expect(!boundary_result.saturated,
         "PID output exactly on a clamp boundary is not saturated");

  tiny_gain_config.kp =
      std::nextafter(tiny_gain_config.output_max,
                     std::numeric_limits<double>::infinity());
  PIDController one_ulp_over_boundary(tiny_gain_config);
  const auto one_ulp_result =
      one_ulp_over_boundary.update(1.0, 0.0, 1.0);
  expect_near(one_ulp_result.output, tiny_gain_config.output_max, 0.0,
              "one-ULP-high PID output is clamped");
  expect(one_ulp_result.saturated,
         "one-ULP-high PID output is reported as saturated");

  PIDConfig reverse_integral_config;
  reverse_integral_config.kp = 0.0;
  reverse_integral_config.ki = -1.0;
  reverse_integral_config.kd = 0.0;
  reverse_integral_config.output_min = -1.0;
  reverse_integral_config.output_max = 1.0;
  reverse_integral_config.anti_windup_gain = 0.0;
  PIDController reverse_integral(reverse_integral_config);
  reverse_integral.reset(2.0);
  const auto reverse_held = reverse_integral.update(1.0, 0.0, 1.0);
  expect_near(reverse_held.integral, 2.0, 0.0,
              "negative integral gain cannot deepen lower saturation");
  expect_near(reverse_held.output, -1.0, 0.0,
              "negative integral gain remains output-limited");
  const auto reverse_recovery = reverse_integral.update(-1.0, 0.0, 1.0);
  expect_near(reverse_recovery.integral, 1.0, 0.0,
              "negative integral gain accepts recovery integration");

  reverse_integral_config.anti_windup_gain = 1.0;
  PIDController reverse_back_calculation(reverse_integral_config);
  reverse_back_calculation.reset(2.0);
  const auto reverse_corrected =
      reverse_back_calculation.update(1.0, 0.0, 1.0);
  expect_near(reverse_corrected.integral, 1.0, 0.0,
              "back-calculation follows the sign of a negative integral gain");
  expect_near(reverse_corrected.output, -1.0, 0.0,
              "signed back-calculation returns to the lower output boundary");

  PIDConfig forward_integral_config = reverse_integral_config;
  forward_integral_config.ki = 1.0;
  PIDController forward_back_calculation(forward_integral_config);
  forward_back_calculation.reset(2.0);
  const auto forward_corrected =
      forward_back_calculation.update(1.0, 0.0, 1.0);
  expect_near(forward_corrected.integral, 1.0, 0.0,
              "positive-gain back-calculation retains symmetric behavior");
  expect_near(forward_corrected.output, 1.0, 0.0,
              "positive-gain correction returns to the upper output boundary");

  LQRConfig lqr_config;
  lqr_config.max_acceleration = 0.25;
  FiniteHorizonLQR lqr(lqr_config);
  expect_near(lqr.control({1000.0, 1000.0}), -0.25, 0.0,
              "LQR respects acceleration limit");

  PurePursuit pursuit({2.7, 0.2, 0.1});
  const auto lateral = pursuit.compute({0.0, -10.0, 0.0}, straight_path());
  expect(std::abs(lateral.steering_angle) <= 0.1,
         "path tracker respects steering limit");

  LongitudinalMPCConfig config;
  config.min_acceleration = -1.25;
  config.max_acceleration = 0.75;
  config.max_jerk = 0.5;
  config.dt = 0.2;
  LongitudinalPredictiveController controller(config);
  const auto plan = controller.plan(0.0, 20.0, 0.0);
  double previous = 0.0;
  for (double acceleration : plan.accelerations) {
    expect(acceleration >= config.min_acceleration - 1e-12 &&
               acceleration <= config.max_acceleration + 1e-12,
           "MPC acceleration is feasible");
    expect(std::abs(acceleration - previous) <=
               config.max_jerk * config.dt + 1e-12,
           "MPC jerk is feasible");
    previous = acceleration;
  }
}

void invalid() {
  using namespace motion_core;
  expect_throws<std::invalid_argument>(
      [] { DifferentialDrive({0.0, 1.0}); },
      "zero differential-drive speed limit is rejected");
  expect_throws<std::invalid_argument>(
      [] { KinematicBicycle({0.0, 2.0, 0.5}); },
      "zero wheelbase is rejected");
  expect_throws<std::invalid_argument>(
      [] { normalize_angle(std::numeric_limits<double>::infinity()); },
      "non-finite angle is rejected");
  expect_throws<std::invalid_argument>(
      [] { DifferentialDrive().step({}, 1.0, 0.0, 0.0); },
      "zero integration interval is rejected");
  expect_throws<std::invalid_argument>(
      [] { PIDController({1.0, 0.0, 0.0, 1.0, -1.0}); },
      "reversed PID limits are rejected");
  expect_throws<std::invalid_argument>(
      [] {
        LQRConfig config;
        config.horizon = 0;
        FiniteHorizonLQR ignored(config);
      },
      "zero LQR horizon is rejected");
  expect_throws<std::out_of_range>(
      [] {
        FiniteHorizonLQR lqr;
        (void)lqr.control({}, lqr.config().horizon);
      },
      "LQR step beyond horizon is rejected");
  expect_throws<std::invalid_argument>(
      [] { PurePursuit().compute({}, {{0.0, 0.0}}); },
      "one-point path is rejected");
  expect_throws<std::invalid_argument>(
      [] {
        StanleyController().compute({}, 1.0,
                                    {{0.0, 0.0}, {0.0, 0.0}});
      },
      "duplicate path segment is rejected");
  expect_throws<std::invalid_argument>(
      [] {
        LongitudinalMPCConfig config;
        config.max_jerk = 0.0;
        LongitudinalPredictiveController ignored(config);
      },
      "zero jerk limit is rejected");
  expect_throws<std::invalid_argument>(
      [] { (void)LongitudinalPredictiveController().plan(1.0, -1.0); },
      "negative target speed is rejected");
}

void numerical_boundaries() {
  using namespace motion_core;
  const double maximum = std::numeric_limits<double>::max();

  const auto expected_constant_turn = [](double initial_heading, double speed,
                                         double yaw_rate, double dt) {
    const double final_heading = initial_heading + yaw_rate * dt;
    return Pose2D{
        speed / yaw_rate *
            (std::sin(final_heading) - std::sin(initial_heading)),
        speed / yaw_rate *
            (std::cos(initial_heading) - std::cos(final_heading)),
        normalize_angle(final_heading)};
  };

  const auto below_old_cutoff =
      DifferentialDrive({2.0, 2.0}).step({}, 1.0, 1e-11, 1e12);
  const auto below_old_expected =
      expected_constant_turn(0.0, 1.0, 1e-11, 1e12);
  expect_relative_near(below_old_cutoff.pose.x, below_old_expected.x, 2e-15,
                       "tiny nonzero yaw keeps curved x integration");
  expect_relative_near(below_old_cutoff.pose.y, below_old_expected.y, 2e-15,
                       "tiny nonzero yaw keeps curved y integration");
  expect_relative_near(below_old_cutoff.pose.heading,
                       below_old_expected.heading, 2e-15,
                       "tiny nonzero yaw keeps heading integration");

  for (const double yaw_rate : {9e-11, 1.1e-10, -9e-11, -1.1e-10}) {
    constexpr double initial_heading = 0.7;
    constexpr double dt = 1e10;
    const auto actual = DifferentialDrive({2.0, 2.0})
                            .step({0.0, 0.0, initial_heading}, 1.0,
                                  yaw_rate, dt)
                            .pose;
    const auto expected =
        expected_constant_turn(initial_heading, 1.0, yaw_rate, dt);
    expect_relative_near(actual.x, expected.x, 4e-15,
                         "former yaw cutoff has continuous x behavior");
    expect_relative_near(actual.y, expected.y, 4e-15,
                         "former yaw cutoff has continuous y behavior");
    expect_relative_near(actual.heading, expected.heading, 4e-15,
                         "former yaw cutoff has continuous heading behavior");
  }

  const auto underflow_turn = DifferentialDrive({2.0, 2.0})
                                  .step({0.0, 0.0, 0.3}, 1.0,
                                        std::numeric_limits<double>::denorm_min(),
                                        1.0)
                                  .pose;
  expect_relative_near(underflow_turn.x, std::cos(0.3), 2e-15,
                       "underflow-scale angular increment uses straight limit x");
  expect_relative_near(underflow_turn.y, std::sin(0.3), 2e-15,
                       "underflow-scale angular increment uses straight limit y");

  constexpr double tiny_steering = 2e-11;
  constexpr double bicycle_speed = 1.0;
  constexpr double bicycle_wheelbase = 2.0;
  constexpr double bicycle_dt = 1e12;
  const double bicycle_yaw =
      bicycle_speed * std::tan(tiny_steering) / bicycle_wheelbase;
  const auto bicycle_turn =
      KinematicBicycle({bicycle_wheelbase, 2.0, 0.6})
          .step({}, bicycle_speed, tiny_steering, bicycle_dt)
          .pose;
  const auto bicycle_expected = expected_constant_turn(
      0.0, bicycle_speed, bicycle_yaw, bicycle_dt);
  expect_relative_near(bicycle_turn.x, bicycle_expected.x, 3e-15,
                       "bicycle-derived tiny yaw keeps curved x integration");
  expect_relative_near(bicycle_turn.y, bicycle_expected.y, 3e-15,
                       "bicycle-derived tiny yaw keeps curved y integration");

  expect_throws_message<std::overflow_error>(
      [] { (void)DifferentialDrive().step({}, 2.0, 0.0, 1e308); },
      "longitudinal x displacement is not representable as a finite double",
      "differential-drive displacement overflow is rejected");
  expect_throws<std::overflow_error>(
      [maximum] {
        (void)DifferentialDrive().step(
            {maximum, 0.0, 0.0}, 2.0, 0.0, maximum / 2.0);
      },
      "differential-drive pose sum overflow is rejected");
  expect_throws<std::overflow_error>(
      [maximum] {
        (void)KinematicBicycle().step(
            {maximum, 0.0, 0.0}, 15.0, 0.0, maximum / 15.0);
      },
      "bicycle pose sum overflow is rejected");
  const auto scaled_turn =
      DifferentialDrive().step({}, 1.0, 1.5, maximum);
  expect(is_finite(scaled_turn.pose),
         "overflowing raw heading increment is reduced before integration");

  expect_throws<std::overflow_error>(
      [maximum] { (void)PIDController().update(maximum, -maximum, 1.0); },
      "PID error subtraction overflow is rejected");
  expect_throws<std::overflow_error>(
      [maximum] {
        PIDConfig config;
        config.kp = 0.0;
        config.kd = 1.0;
        config.output_min = -maximum;
        config.output_max = maximum;
        config.integral_min = -maximum;
        config.integral_max = maximum;
        config.anti_windup_gain = 0.0;
        PIDController controller(config);
        (void)controller.update(maximum, 0.0, 1.0);
        (void)controller.update(-maximum, 0.0, 1.0);
      },
      "PID derivative difference overflow is rejected");
  expect_throws<std::overflow_error>(
      [] {
        PIDConfig config;
        config.kp = 0.0;
        config.kd = 1.0;
        config.output_min = -std::numeric_limits<double>::max();
        config.output_max = std::numeric_limits<double>::max();
        config.anti_windup_gain = 0.0;
        PIDController controller(config);
        (void)controller.update(0.0, 0.0, 1.0);
        (void)controller.update(
            1.0, 0.0, std::numeric_limits<double>::denorm_min());
      },
      "PID derivative division overflow is rejected");
  expect_throws<std::overflow_error>(
      [maximum] {
        PIDConfig config;
        config.kp = 0.0;
        config.ki = 0.0;
        config.kd = 0.0;
        PIDController controller(config);
        (void)controller.update(maximum, 0.0, 2.0);
      },
      "PID integral product overflow is rejected");
  expect_throws<std::overflow_error>(
      [maximum] {
        PIDConfig config;
        config.kp = 0.0;
        config.ki = 0.0;
        config.kd = 0.0;
        config.integral_min = -maximum;
        config.integral_max = maximum;
        PIDController controller(config);
        controller.reset(maximum);
        (void)controller.update(maximum, 0.0, 1.0);
      },
      "PID integral sum overflow is rejected");
  expect_throws<std::overflow_error>(
      [maximum] {
        PIDConfig config;
        config.kp = maximum;
        config.output_min = -maximum;
        config.output_max = maximum;
        config.anti_windup_gain = 0.0;
        PIDController controller(config);
        (void)controller.update(2.0, 0.0, 1.0);
      },
      "PID proportional product overflow is rejected");
  PIDConfig atomic_config;
  atomic_config.kp = 0.0;
  atomic_config.ki = 0.0;
  atomic_config.kd = 0.0;
  atomic_config.integral_min = -maximum;
  atomic_config.integral_max = maximum;
  PIDController atomic_pid(atomic_config);
  const double saved_integral =
      atomic_pid.update(1.0, 0.0, 1.0).integral;
  expect_throws<std::overflow_error>(
      [&atomic_pid, maximum] {
        (void)atomic_pid.update(maximum, -maximum, 1.0);
      },
      "failed PID arithmetic reports overflow");
  expect(atomic_pid.integral() == saved_integral,
         "failed PID arithmetic leaves controller state unchanged");

  expect_throws<std::overflow_error>(
      [maximum] {
        LQRConfig config;
        config.dt = maximum;
        (void)FiniteHorizonLQR(config);
      },
      "LQR discretization overflow is rejected");
  expect_throws<std::overflow_error>(
      [maximum] {
        (void)FiniteHorizonLQR().propagate({maximum, maximum}, 0.0);
      },
      "LQR state propagation sum overflow is rejected");
  expect_throws<std::overflow_error>(
      [maximum] {
        (void)FiniteHorizonLQR().control({maximum, maximum});
      },
      "LQR control arithmetic overflow is rejected");

  const std::vector<PathPoint> overflowing_segment = {
      {-maximum, 0.0}, {maximum, 0.0}};
  expect_throws<std::overflow_error>(
      [&overflowing_segment] {
        (void)PurePursuit().compute({}, overflowing_segment);
      },
      "pure-pursuit path-coordinate difference overflow is rejected");
  expect_throws<std::overflow_error>(
      [&overflowing_segment] {
        (void)StanleyController().compute({}, 1.0, overflowing_segment);
      },
      "Stanley path-coordinate difference overflow is rejected");

  const std::vector<PathPoint> large_representable_path = {
      {0.0, 0.0}, {maximum, 0.0}};
  const auto scaled_tracking =
      PurePursuit().compute({1.0, 1.0, 0.0}, large_representable_path);
  expect(std::isfinite(scaled_tracking.steering_angle) &&
             is_finite(scaled_tracking.target),
         "representable large-path geometry is evaluated with scaling");
  const auto scaled_pursuit =
      PurePursuit({maximum, 3.0, 0.6})
          .compute({1.0, -1.0, 0.0}, straight_path());
  expect(std::isfinite(scaled_pursuit.steering_angle),
         "pure-pursuit steering product is scaled for atan2");
  const auto scaled_stanley =
      StanleyController({maximum, 0.5, 0.6})
          .compute({1.0, 2.0, 0.0}, 1.0, straight_path());
  expect(std::isfinite(scaled_stanley.steering_angle),
         "Stanley correction product is scaled for atan2");

  expect_throws<std::overflow_error>(
      [maximum] {
        LongitudinalMPCConfig config;
        config.horizon = 1;
        config.dt = 1.0;
        config.min_acceleration = -1e308;
        config.max_acceleration = 1e308;
        config.max_jerk = 1e307;
        config.iterations = 1;
        LongitudinalPredictiveController controller(config);
        (void)controller.plan(maximum, maximum, 1e308);
      },
      "predictive speed rollout overflow is rejected");
  expect_throws<std::overflow_error>(
      [] {
        LongitudinalMPCConfig config;
        config.horizon = 1;
        config.dt = 1.0;
        config.speed_weight = 1.0;
        config.terminal_speed_weight = 0.0;
        config.acceleration_weight = 0.0;
        config.jerk_weight = 0.0;
        config.min_acceleration = -1.0;
        config.max_acceleration = 1.0;
        config.max_jerk = 2.0;
        config.iterations = 1;
        LongitudinalPredictiveController controller(config);
        (void)controller.plan(1e200, 0.0);
      },
      "predictive objective square overflow is rejected");

  LongitudinalMPCConfig scaled_objective_config;
  scaled_objective_config.horizon = 1;
  scaled_objective_config.dt = 1.0;
  scaled_objective_config.speed_weight = 1e-300;
  scaled_objective_config.terminal_speed_weight = 0.0;
  scaled_objective_config.acceleration_weight = 0.0;
  scaled_objective_config.jerk_weight = 0.0;
  scaled_objective_config.min_acceleration = -1.0;
  scaled_objective_config.max_acceleration = 1.0;
  scaled_objective_config.max_jerk = 2.0;
  scaled_objective_config.iterations = 1;
  const auto scaled_objective =
      LongitudinalPredictiveController(scaled_objective_config)
          .plan(1e200, 0.0);
  expect(std::isfinite(scaled_objective.objective),
         "representable weighted square is evaluated with scaling");

  LongitudinalMPCConfig large_dt_config;
  large_dt_config.horizon = 1;
  large_dt_config.dt = maximum;
  large_dt_config.speed_weight = 0.0;
  large_dt_config.terminal_speed_weight = 0.0;
  large_dt_config.acceleration_weight = 1.0;
  large_dt_config.jerk_weight = 0.0;
  large_dt_config.min_acceleration = -1e-308;
  large_dt_config.max_acceleration = 1e-308;
  large_dt_config.max_jerk = 1e-308;
  large_dt_config.iterations = 1;
  const auto large_dt_plan =
      LongitudinalPredictiveController(large_dt_config).plan(0.0, 0.0);
  expect(std::isfinite(large_dt_plan.objective) &&
             std::isfinite(large_dt_plan.predicted_speeds.back()),
         "large dt remains valid when all required products are representable");
}

void determinism() {
  using namespace motion_core;
  LongitudinalPredictiveController controller;
  const auto first = controller.plan(2.0, 7.0, -0.1);
  const auto second = controller.plan(2.0, 7.0, -0.1);
  expect(first.accelerations == second.accelerations,
         "MPC control sequence is exactly repeatable");
  expect(first.predicted_speeds == second.predicted_speeds,
         "MPC rollout is exactly repeatable");
  expect(first.objective == second.objective,
         "MPC objective is exactly repeatable");

  KinematicBicycle model;
  Pose2D a{0.0, 0.0, 0.0};
  Pose2D b = a;
  for (int i = 0; i < 100; ++i) {
    a = model.step(a, 3.0, 0.15, 0.05).pose;
    b = model.step(b, 3.0, 0.15, 0.05).pose;
  }
  expect(a.x == b.x && a.y == b.y && a.heading == b.heading,
         "model rollout is exactly repeatable");

  PurePursuit tracker;
  const auto left = tracker.compute({1.0, -0.5, 0.1}, straight_path());
  const auto right = tracker.compute({1.0, -0.5, 0.1}, straight_path());
  expect(left.steering_angle == right.steering_angle &&
             left.target_index == right.target_index,
         "tracking result is exactly repeatable");
}

void finite_outputs() {
  using namespace motion_core;
  expect(std::isfinite(normalize_angle(1e300)),
         "normalizing a large finite angle remains finite");

  KinematicBicycle model({2.7, 15.0, 0.55});
  PurePursuit tracker({2.7, 2.5, 0.55});
  LongitudinalMPCConfig config;
  config.horizon = 12;
  LongitudinalPredictiveController controller(config);
  std::vector<PathPoint> path;
  for (int i = 0; i < 80; ++i) {
    const double x = static_cast<double>(i);
    path.push_back({x, std::sin(x * 0.05)});
  }
  Pose2D pose{0.0, -0.5, 0.0};
  double speed = 0.0;
  double previous_acceleration = 0.0;
  for (int i = 0; i < 300; ++i) {
    const auto plan = controller.plan(speed, 4.0, previous_acceleration);
    const auto tracking = tracker.compute(pose, path);
    speed = std::max(0.0, speed + config.dt * plan.acceleration);
    pose = model.step(pose, speed, tracking.steering_angle, config.dt).pose;
    previous_acceleration = plan.acceleration;
    expect(is_finite(pose) && std::isfinite(speed) &&
               std::isfinite(plan.objective) &&
               std::isfinite(tracking.steering_angle),
           "closed-loop values remain finite");
  }
}

}  // namespace

int main(int argc, char** argv) {
  const std::string group = argc > 1 ? argv[1] : "";
  const std::vector<std::pair<std::string, std::function<void()>>> groups = {
      {"nominal", nominal},       {"saturation", saturation},
      {"invalid", invalid},       {"numerical", numerical_boundaries},
      {"determinism", determinism}, {"finite", finite_outputs}};
  try {
    for (const auto& entry : groups) {
      if (group.empty() || group == entry.first) {
        entry.second();
        std::cout << "PASS " << entry.first << '\n';
      }
    }
    if (!group.empty()) {
      bool found = false;
      for (const auto& entry : groups) {
        found = found || group == entry.first;
      }
      if (!found) {
        std::cerr << "unknown test group: " << group << '\n';
        return 2;
      }
    }
    std::cout << assertions << " assertions passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "FAIL " << (group.empty() ? "all" : group) << ": "
              << error.what() << '\n';
    return 1;
  }
}
