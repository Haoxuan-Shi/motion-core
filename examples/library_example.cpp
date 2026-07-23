#include "motion_core/motion_core.hpp"

#include <iostream>
#include <vector>

int main() {
  const std::vector<motion_core::PathPoint> path{
      {0.0, 0.0}, {5.0, 0.5}, {10.0, 0.0}, {15.0, -0.5}};
  motion_core::KinematicBicycle model({2.7, 10.0, 0.55});
  motion_core::PurePursuit lateral({2.7, 2.5, 0.55});
  motion_core::LongitudinalPredictiveController longitudinal;

  motion_core::Pose2D pose{0.0, -0.5, 0.0};
  double speed = 0.0;
  double acceleration = 0.0;
  for (int step = 0; step < 20; ++step) {
    const auto speed_plan = longitudinal.plan(speed, 3.0, acceleration);
    const auto steering = lateral.compute(pose, path);
    acceleration = speed_plan.acceleration;
    speed += longitudinal.config().dt * acceleration;
    pose = model.step(pose, speed, steering.steering_angle,
                      longitudinal.config().dt).pose;
  }
  std::cout << pose.x << ',' << pose.y << ',' << pose.heading << ',' << speed
            << '\n';
}

