# Changelog

All notable changes are documented here. The format follows Keep a Changelog,
and versions use semantic versioning.

## [Unreleased]

### Fixed

- Make the Windows quickstart select Ninja explicitly and use its actual
  single-config executable path while documenting the separate multi-config
  command convention.
- Derive `PIDResult::saturated` from strict pre-clamp output inequalities
  instead of a fixed `1e-12` near-equality tolerance, so saturation is reported
  correctly for tiny controller gains without changing anti-windup behavior.
- Make conditional integration and back-calculation anti-windup follow the
  actual integral contribution, including controllers with negative `ki`.
- Preserve analytic constant-turn motion for every nonzero yaw rate, including
  tiny rates accumulated over long time steps, using stable series and
  half-angle formulations instead of an absolute rate cutoff.
- Reject unrepresentable finite model, PID, LQR, path-tracking, and predictive
  arithmetic with deterministic `std::overflow_error` failures instead of
  returning NaN or infinity.
- Scale large-but-representable angle, path, steering, and weighted-objective
  calculations without imposing global input magnitude caps.

## [0.1.0] - 2026-07-23

### Added

- Constrained differential-drive and kinematic-bicycle models.
- Anti-windup PID and finite-horizon double-integrator LQR.
- Pure Pursuit and Stanley path tracking.
- Acceleration- and jerk-constrained longitudinal predictive control.
- Deterministic CSV demo CLI, CMake install exports, examples, and CTest suite.
