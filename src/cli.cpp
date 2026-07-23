#include "motion_core/motion_core.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <locale>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

struct DemoOptions {
  std::string output{"-"};
  std::size_t steps{160};
  double dt{0.05};
};

void print_usage(std::ostream& out) {
  out << "motion-core - deterministic autonomy-control demo\n\n"
      << "Usage:\n"
      << "  motion-core demo [--output FILE|-] [--steps N] [--dt SECONDS]\n"
      << "  motion-core --help\n\n"
      << "The demo writes a deterministic CSV trace. '-' selects standard output.\n";
}

std::size_t parse_size(const std::string& text, const char* option) {
  std::size_t consumed = 0;
  unsigned long long value = 0;
  try {
    value = std::stoull(text, &consumed);
  } catch (const std::exception&) {
    throw std::invalid_argument(std::string(option) + " requires an integer");
  }
  if (consumed != text.size() || value == 0 || value > 1000000ULL) {
    throw std::invalid_argument(std::string(option) +
                                " must be in [1, 1000000]");
  }
  return static_cast<std::size_t>(value);
}

double parse_positive(const std::string& text, const char* option) {
  std::size_t consumed = 0;
  double value = 0.0;
  try {
    value = std::stod(text, &consumed);
  } catch (const std::exception&) {
    throw std::invalid_argument(std::string(option) + " requires a number");
  }
  if (consumed != text.size() || !std::isfinite(value) || value <= 0.0 ||
      value > 10.0) {
    throw std::invalid_argument(std::string(option) +
                                " must be in (0, 10]");
  }
  return value;
}

DemoOptions parse_options(int argc, char** argv) {
  DemoOptions options;
  int index = 1;
  if (index < argc && std::string(argv[index]) == "demo") {
    ++index;
  }
  while (index < argc) {
    const std::string argument = argv[index++];
    if (argument == "--help" || argument == "-h") {
      print_usage(std::cout);
      std::exit(0);
    }
    if (argument == "--output" && index < argc) {
      options.output = argv[index++];
    } else if (argument == "--steps" && index < argc) {
      options.steps = parse_size(argv[index++], "--steps");
    } else if (argument == "--dt" && index < argc) {
      options.dt = parse_positive(argv[index++], "--dt");
    } else {
      throw std::invalid_argument("unknown or incomplete option: " + argument);
    }
  }
  return options;
}

std::vector<motion_core::PathPoint> make_reference_path() {
  std::vector<motion_core::PathPoint> path;
  path.reserve(101);
  for (std::size_t i = 0; i <= 100; ++i) {
    const double x = 0.6 * static_cast<double>(i);
    path.push_back({x, 2.0 * std::sin(x / 12.0)});
  }
  return path;
}

void run_demo(const DemoOptions& options, std::ostream& output) {
  const std::vector<motion_core::PathPoint> path = make_reference_path();
  motion_core::PurePursuit tracker({2.7, 3.0, 0.55});
  motion_core::KinematicBicycle bicycle({2.7, 12.0, 0.55});
  motion_core::LongitudinalMPCConfig mpc_config;
  mpc_config.dt = options.dt;
  mpc_config.horizon = 24;
  motion_core::LongitudinalPredictiveController speed_controller(mpc_config);

  motion_core::Pose2D pose{0.0, -1.0, 0.0};
  double speed = 0.0;
  double previous_acceleration = 0.0;
  const double target_speed = 5.0;

  output.imbue(std::locale::classic());
  output << std::setprecision(12);
  output << "step,time,x,y,heading,speed,acceleration,steering,target_index\n";
  for (std::size_t step = 0; step < options.steps; ++step) {
    const auto longitudinal = speed_controller.plan(
        speed, target_speed, previous_acceleration);
    const auto lateral = tracker.compute(pose, path);
    speed = std::max(0.0, speed + longitudinal.acceleration * options.dt);
    const auto model_step =
        bicycle.step(pose, speed, lateral.steering_angle, options.dt);
    pose = model_step.pose;
    previous_acceleration = longitudinal.acceleration;
    output << step << ',' << (static_cast<double>(step + 1) * options.dt)
           << ',' << pose.x << ',' << pose.y << ',' << pose.heading << ','
           << speed << ',' << longitudinal.acceleration << ','
           << lateral.steering_angle << ',' << lateral.target_index << '\n';
  }
  if (!output) {
    throw std::runtime_error("failed while writing trace output");
  }
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const DemoOptions options = parse_options(argc, argv);
    if (options.output == "-") {
      run_demo(options, std::cout);
    } else {
      std::ofstream file(options.output, std::ios::out | std::ios::trunc);
      if (!file) {
        throw std::runtime_error("cannot open output file: " + options.output);
      }
      run_demo(options, file);
    }
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "motion-core: " << error.what() << '\n';
    std::cerr << "Run 'motion-core --help' for usage.\n";
    return 2;
  }
}
