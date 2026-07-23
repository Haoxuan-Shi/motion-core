# Architecture and numerical design

## Boundaries

Public headers live in `include/motion_core`; implementations are in `src`.
The library owns no threads, clocks, files, or global mutable state. The CLI is
a thin deterministic simulation that writes CSV. Tests use only the C++
standard library.

## Models and conventions

`Pose2D` uses metres and radians in a right-handed planar frame. Positive yaw
is counter-clockwise. Requested controls are clamped before propagation.
Constant speed and yaw rate are integrated analytically; the bicycle yaw rate
is `v * tan(steering) / wheelbase`. Headings are normalized to `[-pi, pi)`.
Exactly zero yaw takes the straight-line limit. Nonzero yaw is never discarded
by an absolute rate tolerance: small angular increments use stable
`sinc`/`cosc` series, while larger increments use the equivalent half-angle
identity after range reduction. This keeps tiny-rate, long-duration turns and
bicycle-derived yaw consistent with the analytic constant-twist solution.

Every public time step and configuration scalar is checked for finiteness and
physical sign/range. Adjacent path points must be distinct. These checks make
invalid-domain failures explicit rather than allowing NaNs into a rollout.

Derived floating-point arithmetic is checked at the operation that produces
it. Adds, differences, products, quotients, Riccati terms, controller sums,
rollout states, gradients, and objective accumulations must remain finite.
Where an equivalent scaled formulation preserves a representable result, the
implementation uses it: large heading increments are reduced before angle
addition, constant-turn differences use cancellation-resistant series or
half-angle forms, path projection uses normalized segment directions and
`hypot`, steering `atan2` inputs are exponent-scaled, and weighted squares are
evaluated with exponent scaling. This avoids arbitrary global magnitude caps.

## Feedback controllers

The PID stores integral and previous-error state. It applies integral limits,
compares the candidate and prior raw controller outputs, and stops integration
only when the actual integral contribution would deepen output saturation.
This remains correct for either sign of `ki`. Configurable back-calculation is
mapped through the sign of `ki`, so its correction always moves the integral
contribution toward the clamped output rather than farther into saturation.
The returned saturation flag is evaluated from the final raw output before
clamping: strict inequality outside the configured output interval means
saturated, while an exact boundary value does not. No absolute near-equality
tolerance participates in that classification.

The LQR discretizes the double integrator with
`A=[[1,dt],[0,1]]` and `B=[dt^2/2,dt]`. Starting from the terminal diagonal
cost, it computes one gain per stage using the finite-horizon discrete Riccati
recursion. The public control is clamped to the configured acceleration limit.

Pure Pursuit projects onto the nearest segment, walks the polyline by the
lookahead distance, and computes a bicycle steering command. Stanley combines
wrapped path-heading error with signed nearest-segment cross-track error.
Nearest-element scans and ties follow input order.

## Longitudinal predictive control

The decision vector is acceleration over the finite horizon. Predicted speed
uses `v[k+1] = v[k] + dt*a[k]`. The quadratic objective penalizes speed error,
terminal speed error, acceleration, and finite-difference jerk. Analytic
gradients and a conservative fixed step size are used for a configured, fixed
number of iterations. After every update, a forward projection enforces
acceleration limits and `|a[k]-a[k-1]| <= max_jerk*dt`, including the measured
previous acceleration (clamped to the acceleration domain).

This design favors small size and repeatability over solver sophistication.
There is no warm start, adaptive convergence stopping, obstacle model, or
guarantee of matching an exact quadratic-program optimum.

## Determinism and failure behavior

Algorithms use stable input-order scans, no random numbers, no wall clock, and
fixed iteration counts. Non-finite inputs and invalid domains throw
`std::invalid_argument`; an LQR step outside its horizon throws
`std::out_of_range`; finite inputs or configurations whose required arithmetic
is not representable throw `std::overflow_error` naming the failed quantity.
Public result structs and sequences are validated before return. PID arithmetic
failures occur before its stored state is committed.

Tests cover nominal operation, saturation, invalid inputs, operation-specific
representability boundaries, tiny yaw rates on both sides of historical
floating-point cutoffs, exact repeatability within one process, and long finite
rollouts.
