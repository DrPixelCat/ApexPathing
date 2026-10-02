# Online planning and stored vector fields

The builder has two terminal methods. `quickBuild()` compiles geometry and plans online
while following. `buildToMemory()` solves the nominal rest-to-rest trajectory and builds
an interpolated field in RAM. `profiledBuild()` has been removed. Holonomic paths, tank
paths and dual-actuated modes use this path contract. Stationary turns use direct PID
heading feedback with `quickBuild()` and a precomputed angular profile with
`buildToMemory()`. Quick turns perform no trajectory solves.

```java
Path online = factory.holonomicPath(start, corner, end)
        .setLookaheadDistance(Dist.fromIn(24)) // optional; otherwise automatically sized
        .quickBuild();

Path stored = factory.holonomicPath(start, corner, end)
        .setMemoryGridSpacing(Dist.fromIn(1))
        .setMemoryGridPadding(Dist.fromIn(12))
        .buildToMemory();

Turn pidTurn = factory.turn(start).turnTo(Angle.fromDeg(90)).quickBuild();
Turn storedTurn = factory.turn(start).turnTo(Angle.fromDeg(90)).buildToMemory();
```

Online traversal projects the measured pose onto the route. The coupled solver samples
only its visible arc-length window, exact boundaries, and authored constraint transitions.
Its initial speed is the measured tangential velocity; its window boundary has a zero-speed
bound. That boundary advances with the robot, then shortens at the actual endpoint.
A short horizon therefore still imposes a stopping-speed bound. The published near-term
reference uses up to 50 milliseconds of feasible velocity preview and full acceleration.
The initial solver state is never replaced with a lower envelope speed. Overspeed recovery
reports `overspeed` and brakes from the measured state, with bounded normal acceleration
and a feasible angular command from measured angular rate. An infeasible solve stops the
drivetrain, records a diagnostic, and throws; it never resumes the former quick controller.

The automatic horizon is fixed for one traversal. It includes the worst-direction,
power-limited stopping distance with 50% distance headroom and 50 milliseconds of travel.
Mecanum sizing checks its full directional LUT and diagonal wheel-power demand.
New traversals capture fresh planning inputs. Pause/resume clears timing and execution
feedback history; the next update uses fresh measured velocity. Stored turns unwrap
measured heading continuously and sample their angular LUT by heading progress,
including after a pause. Quick turns target the final heading directly with existing
heading gains, measured-rate damping, and endpoint capture. Their diagnostic sample
reports the final heading and zero profile velocity/acceleration. Custom angular
velocity and acceleration limits apply to stored turn profiles.

Stored path bounds determine the rectangular grid, including negative and translated
coordinates. Defaults are 1-inch spacing and 12-inch padding. Each node packs velocity,
full acceleration, displacement toward the route, tangent, progress, and angular targets
in primitive arrays. Queries clamp coordinates before computing indices and bilinear
weights. Outside the grid, the interpolated boundary point plus its stored displacement
reconstructs the represented route point; displacement is then measured from the actual
robot location. Launch and braking intervals preserve their kinematic boundary equations;
a stored stopping envelope also bounds interpolation across constraint reductions.
Heading is evaluated by the existing interpolator at interpolated progress, preserving wrap.
Tangent heading styles use the interpolated tangent so heading and motion agree.
Crossings remain unsupported. No field layers or disk serialization are added.

Both path sources feed the same follower. Velocity and acceleration drive calibrated motion
feedforward; velocity error and `toPath` drive feedback. Acceleration includes tangential
and normal components. The normal component contributes centripetal feedforward once.
Tank control uses the existing Ramsete conversion into achievable forward and angular
commands. Existing motor allocation, endpoint capture, and completion policies remain.
Holonomic heading feedforward retains the existing compensation for measured translation
lag; the published sample includes that angular-rate adjustment.

`follower.getTrajectoryTarget()` exposes the latest physical reference, also delivered to
`FollowerDiagnostics.recordTrajectoryTarget`. Path distances are inches; linear velocity
and acceleration are inches/second and inches/second². Angular quantities use radians,
radians/second, and radians/second². Turns report `angularProgressRad` and `remainingAngleRad`
and leave linear progress/distance zero. Motor demand after feedback and allocation remains
available through the existing command diagnostics. `getTrajectoryFailure()` and
`recordTrajectoryFailure` describe failures.

`buildToMemory()` precomputes a permanent trajectory snapshot once. Following uses that
data directly, with no configuration signatures, invalidation checks, or rebuilding.
Later changes to planning settings do not replace the stored field or angular LUT.
Reversal creates a new movement, retaining mode and settings and precomputing its own
stored data when needed. Traversal callbacks are not copied.
Custom mutable heading interpolators implement `copyForTrajectory()` to detach their
stored configuration; custom curves can override `getBoundsIn()` with exact bounds.
Legacy scalar generators and LUT access remain available for regression analysis; followers
and tracking logs use the shared trajectory sample. Calibration JSON keys are unchanged.

`trajectory.TrajectoryTest` benchmarks build cost, packed field bytes, and query/update
latency and writes `TeamCode/build/trajectory-benchmark.csv`. It also checks solve counts:
stored queries perform zero solves; each online query solves only its bounded window.
Desktop timings do not establish robot-controller performance.
See [validation results and remaining fixture failures](TrajectoryValidation.md).
