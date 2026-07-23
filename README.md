# motion-core

[English](README.md) | [简体中文](README.zh-CN.md)

`motion-core` is a small C++17 library and command-line demo for deterministic,
CPU-only motion control. It is intended for teaching, simulation, regression
tests, and controller prototyping--not direct control of safety-critical
hardware.

The library has no third-party runtime dependency. It provides:

- exact constant-input integration for constrained differential-drive and
  kinematic-bicycle models, with headings normalized to `[-pi, pi)`;
- PID control with output/integral limits, exact pre-clamp saturation
  reporting, and gain-sign-aware conditional integration and back-calculation
  anti-windup;
- finite-horizon LQR for a discrete double integrator, solved by backward
  Riccati recursion;
- Pure Pursuit and Stanley path trackers;
- a longitudinal predictive controller with a configurable horizon and fixed,
  deterministic projected-gradient iterations under acceleration and jerk
  bounds; and
- a CLI that emits a reproducible closed-loop CSV trace.

## Quickstart (Windows PowerShell)

Requirements: CMake 3.16+, Ninja, and a C++17 compiler (Visual Studio 2019 or
newer is sufficient).

```powershell
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
.\build\motion-core.exe demo --steps 200 --dt 0.05 --output trace.csv
```

For a multi-config generator, omit `-DCMAKE_BUILD_TYPE`, pass `--config Release`
to build, test, and install commands, and run the executable from that
generator's configuration directory. Run `motion-core --help` for CLI details.
The CSV contains time, pose, speed, acceleration, steering, and target-waypoint
index.

## Library use

```cpp
#include <motion_core/motion_core.hpp>

motion_core::KinematicBicycle model({2.7, 12.0, 0.55});
motion_core::Pose2D pose{};
pose = model.step(pose, 3.0, 0.1, 0.05).pose;
```

[`examples/library_example.cpp`](examples/library_example.cpp) combines path
tracking, longitudinal prediction, and bicycle propagation. All public
functions reject non-finite inputs and invalid physical/configuration limits
with `std::invalid_argument`. An out-of-horizon LQR step throws
`std::out_of_range`. If all supplied values are finite but a required derived
quantity cannot be represented as a finite `double`, the operation throws
`std::overflow_error` with the failing quantity in the message; no public
result containing NaN or infinity is returned.

## Install and consume with CMake

```powershell
cmake --install build --config Release --prefix install
```

An installed consumer can use `find_package(motion-core CONFIG REQUIRED)` and
link `motion_core::motion_core`. Headers, the library, the CLI, export targets,
and package version metadata are installed.

## Numerical and safety scope

The predictive controller solves a convex quadratic tracking cost with a
fixed number of projected-gradient updates. Its returned sequence is feasible
for the configured acceleration and step-to-step jerk bounds, but this compact
solver does not claim the convergence certificates or feature set of a
production MPC package. Models are kinematic and omit tire, actuator, latency,
and state-estimation effects. Results are deterministic for the same binary,
platform, and inputs; bitwise equality across different compilers or floating-
point architectures is not promised.

Large values are handled according to the operation rather than by a global
magnitude limit. Angle increments, path geometry, steering ratios, and
weighted squares use scaled formulations where the mathematical result is
representable. A finite input is rejected only when a required result or
intermediate state for that primitive is not representable.

Constant-turn propagation never classifies motion from an absolute yaw-rate
cutoff. Exactly zero yaw uses the straight solution; small angular increments
use `sinc`/`cosc` series, and larger or multiply-overflowing increments use a
reduced-angle midpoint identity. Consequently, a tiny nonzero yaw accumulated
over a long step remains a turn rather than being silently linearized.

`PIDResult::saturated` is true exactly when the final raw controller output is
strictly below `output_min` or above `output_max` before clamping. An output
exactly on either boundary is not saturated, and no scale-independent
near-equality tolerance is used. Conditional integration and back-calculation
retain their gain-sign-aware behavior.

See [`docs/architecture.md`](docs/architecture.md) for equations and design
boundaries. The project is MIT licensed.
