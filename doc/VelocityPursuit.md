# Spatial velocity pursuit

This document retains the future-speed experiment and its historical results.
Normal following has reverted to [current-error proportional velocity control](VelocityFeedback.md).
The removed `velocityPursuitTimeSeconds` setting is ignored when loading saved profiles.

The experimental controller chooses acceleration to converge toward a feasible
speed farther along the path. The selected experiment horizon was 0.35 s.
It replaces the separate tangential velocity power correction in those comparisons.
Heading, cross-track damping and endpoint capture still perform their existing roles.
Both-feedback-disabled motor-model tests still use an independent time reference.
Tank control is unchanged.

## Why change the controller

The previous online planner repeatedly chose the maximum reachable acceleration
from the latest measured speed. A separate proportional velocity correction tracked
an integrated reference. The scalar error derivation in `VelocityFeedback.md`
assumed synchronous sensing, a fixed tangent, exact inversion and no allocation.
Those assumptions do not describe a delayed, coupled twisting path. Fast changes
between accelerating and braking can combine with delayed measurements to create
large corrections. The unchanged native plant's cruise-noise experiment independently
confirms the sensitivity of that feedback bandwidth to delayed sensor noise.

The new controller uses a future spatial goal to choose the entire tangential
acceleration demand. It does not add another tangential P correction to that demand.
The measured velocity remains the state used by the acceleration inverse.

The motivation is analogous to choosing a lookahead point in
[Coulter's pure-pursuit path tracking report](https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf).
The speed law below is our longitudinal adaptation; Coulter's geometric steering
law does not establish its stability.

## Future goal and energy equation

Let `s` be current path progress, `v` the measured tangential speed, `T` the configured
convergence horizon, and `q(s) = v_feasible(s)^2` the independent feasible envelope.
Choose a distance

```text
L = min(distance remaining in the planning window, max(2 inches, v*T))
s_g = s + L
v_g = sqrt(q(s_g)).
```

The minimum distance avoids dividing by zero at launch. The bounded stopping window
is sized separately from braking authority, or by the path's existing lookahead setting.
Online execution obtains `q` from its backward reachability solve, before starting
the forward solve at measured speed. Stored execution also caps goals with its existing
precomputed speed profile. It uses allocation-free squared-speed queries. Neither
mode replaces a future goal with `v_measured + 0.05*a`.

For constant acceleration over distance `L`, `v_g^2 = v^2 + 2*a*L`, so

```text
a_goal = (v_g^2 - v^2)/(2*L).
```

For example, at 20 in/s a goal of 16 in/s seven inches ahead gives
`a_goal = (256 - 400)/14 = -10.29 in/s^2`. The robot brakes gradually toward that
future state. A nearby mandatory speed restriction can require stronger braking.

Squared speed is interpolated spatially because the energy equation is linear in
`v^2`, rather than in `v`, during constant acceleration. If an intervening local
speed minimum requires more braking than the farther goal, that minimum replaces
the goal. The first prototype omitted this rule and accelerated toward the faster
section beyond a slow corner; it failed there. The final implementation retains
the corner and passes the same path. A longer lookahead cannot skip a slow section.

## Damping and the local convergence argument

Factor the energy equation:

```text
a_goal = [(v_g + v)/(2L)] * (v_g - v).
```

Near constant cruising speed, the bracket becomes `K ≈ v/L ≈ 1/T`. Without damping,
an ideal acceleration-controlled plant has `e_dot = -K*e`, where `e = v_g - v`.
Its speed converges to the independently selected goal. This also explains why
longer horizons reduce reaction to instantaneous speed noise.

An acceleration state smooths commands:

```text
Ta*a_dot + a = a_goal
Ta = 0.25 * min(T, L/max(1 in/s, v))
w = 1 - exp(-h/Ta)
a_filtered[k] = a_applied[k-1] + w*(a_goal[k] - a_applied[k-1]).
```

At steady cruise `Ta = 1/(4K)`. Combining the ideal plant `v_dot = a` with the
acceleration filter gives

```text
Ta*e_ddot + e_dot + K*e = 0.
```

The characteristic discriminant is `1 - 4*Ta*K = 0`, giving two equal negative
poles, `p = -2K`. This is critical damping in that local continuous approximation.
The discrete production helper is tested for monotonic convergence from above
and below a fixed speed goal at 20 ms. The damping state stores the projected
planning command, resets on pause/new movement, and resets after invalid timing
or a gap over 250 ms. It never integrates a velocity-error objective.

This is not a global proof for sensor delay, changing goals, wheel saturation or
heading coupling. Those effects are checked with the native plant tests below.

## Braking and twisting constraints

Filtering cannot postpone compulsory braking. The existing backward/forward
reachability solve supplies a maximum feasible first-interval acceleration.
The filtered pursuit command is capped by that acceleration. More braking is
allowed only within current translational acceleration, angular acceleration
and signed wheel-power bounds, and without a negative squared successor speed.
Infeasible extra braking is projected back toward
the solver's known feasible command.

For heading `theta(s)` and signed curvature `kappa(s)`, the published motion uses

```text
omega = theta'(s)*v
alpha = theta''(s)*v^2 + theta'(s)*a
field acceleration = tangent*a + left_normal*kappa(s)*v^2.
```

Thus changing longitudinal acceleration also updates twisting acceleration and
the motor prediction. The same coupled wheel model computes feedforward before
heading/lateral corrections, endpoint blending and allocation. Power plots compare
this prediction with the delivered peak absolute motor command. Their units are
normalized motor command, not electrical watts.

Stored execution retains its precomputed speed goals but now runs a bounded
feasibility check at actual speed. It therefore does more runtime work than the
previous stored-field-only lookup. Online execution does not build a full trajectory.
In the final controlled-clock clean tests, complete follower updates had p95 times
of 0.15 ms online and 0.16 ms stored on this desktop; these are measurements, not
hardware timing guarantees.

## Tests and choice of horizon

The same stock physics, motor calibration, geometry and existing localization
filters are used for every comparison. Acceleration variation is
`sum(abs(a[k]-a[k-1])) / traversal_seconds`, excluding endpoint blending. The speed
RMS compares physical plant speed with a separately generated nominal profile at
the same path distance; it does not compare against the planner's measured-state
velocity and call that tracking accuracy.

Clean online twisting course:

| Convergence horizon | Duration, s | Acceleration variation/s | Spatial speed RMS error, in/s |
|---|---:|---:|---:|
| Legacy controller | 10.000 | 114.50 | 0.590 |
| 0.10 s | 9.980 | 97.08 | 0.608 |
| 0.20 s | 10.080 | 88.07 | 0.997 |
| **0.35 s** | **10.240** | **84.48** | **1.664** |
| 0.50 s | 10.560 | 78.51 | 2.588 |

At 0.35 s, acceleration variation falls 26.2%; total acceleration variation also
falls about 24.4%, so this is not explained solely by the longer traversal. Speed
departs farther from the nominal profile because braking begins earlier. The
0.50 s option further reduces acceleration variation but slows the route more.
The 0.35 s default balances smoothing and completion time.

An isolated native-plant 16 in/s cruise with 60 ms sensing delay and 0.3 in/s,
7 Hz velocity noise measured:

| Controller | Physical speed RMS error, in/s | Acceleration command RMS, in/s² | Motor command standard deviation |
|---|---:|---:|---:|
| Previous feedback bandwidth | 0.04045 | 1.72233 | 0.01177 |
| Future convergence, 0.35 s | 0.00391 | 0.15544 | 0.00106 |

These reductions apply to this cruise-noise experiment. Whole-course power
variation is nearly unchanged online at 0.35 s: 1.085 versus 1.086 per second.
Power spikes remain around corner exits, where heading/lateral corrections and
wheel allocation also act. This change does not claim to remove those spikes.

The full twisting course also passes imposed extra drag of 4 in/s², a 5 in/s speed
impulse with 10/30/20 ms loop jitter, and 60 ms delayed pose/velocity packets with
added velocity noise. The delayed legacy online comparison fails with an infeasible
overspeed braking state; all tested future horizons (0.20/0.35/0.50 s) complete.
At 0.35 s, delayed online heading RMS is 2.41 degrees and endpoint error is 0.169 in;
stored values are 2.30 degrees and 0.043 in. Both modes pass their callbacks and
independent spatial speed/heading/endpoint checks.

The selected regression run has 162 tests: 161 passed, one skipped. The existing
wall-clock feedforward-only fixture skips when a host timing gap invalidates that
experiment; controlled-clock feedforward-only tests pass.

Wall-timed ApexAutoTest online passes in 11.342 s, with 0.533 in endpoint error,
0.632 degree endpoint heading error and a 22.77 ms average loop. A first stored
run fails after multiple 0.47–0.56 s command gaps. After removing per-node lookup
allocations, the stored rerun passes in 10.597 s, with 0.170 in endpoint error,
0.281 degree heading error and a 20.82 ms average loop. Both records are retained.
The rerun does not prove the source of those earlier stalls or robustness to
arbitrarily long command gaps. The timing guards and completion checks were retained.

The final combined wall run completes both paths. Online passes all checks in
10.265 s (0.197 in endpoint error, 0.435 degree heading error, 21.43 ms average
loop). Stored completes in 11.016 s (0.108 in endpoint error, 0.280 degree heading
error), but its test fails the unchanged 23 ms loop budget at 26.10 ms. The final
wall-test suite therefore has one pass and one timing failure. This timing
limitation remains unresolved; it is not counted as a passing regression.

## Configuration and artifacts

Experimental comparisons explicitly pass a nonzero pursuit horizon to the
four-argument `OnlineTrajectoryPlanner` constructor. Normal following uses zero
and stored execution consumes the stored trajectory field. `velocityFeedbackGain`
controls proportional velocity feedback again. The old saved pursuit setting
cannot enable the experiment; heading gains and endpoint capture remain separate.

`getTrackingVelocityTarget()` reports the future speed goal while pursuit is active.
`getVelocityGoalDistanceIn()`, `getVelocityGoal()` and `getCurrentSpeedCeiling()` expose
the goal location, speed and independent current feasibility ceiling. The goal is
NaN during overspeed recovery. A speed goal ahead of the robot is not an expected
instantaneous speed at its current location.

`AutoTest` logs `velocity_goal_distance_in` and `current_speed_ceiling_in_s` alongside
the existing prediction and actual motor commands. Desktop velocity graphs use a
separate nominal profile at current progress. `VelocityFeedbackSimulationTest`
also logs the future goal, the independent plant and endpoint blend.

Generated XChart PNGs, CSVs and regression XML are under
`TeamCode/build/path-power-graphs/velocity-pursuit-final/`.
Wall-timed results are under `velocity-pursuit-wall/`; `first-run/` retains the
original wall-test XML and `second-run/` contains the stored rerun graphs.
`final-run/` contains the final combined wall run, including its timing failure.
