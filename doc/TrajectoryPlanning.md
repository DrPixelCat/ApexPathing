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


All path profiles use one concrete `PathProfileGenerator`; the former base class and
forwarding generator have been combined. `DriveModel` supplies physical velocity and
acceleration limits, wheel projections, and analytic acceleration bounds.

Each follower owns a drive model and its `MotionPowerCalculator`. The tracking update
calls `calculate(...)` once for signed translation, rotation, and normal reference power.
The planner uses the same calculator with a settings snapshot to evaluate hypothetical
states. Planning and runtime policies retain the conservative kV reserve, tank reverse
reserve, and runtime normal-power cap. Characterized mecanum wheels use the shared
`MecanumWheelModel.solve(...)` kernel in both paths. Profile reporting reuses each sample's
power evaluator instead of reconstructing a second calculation. Feedback, Ramsete
corrections, endpoint capture, braking pulse history, and motor output remain in the follower.

Use a follower-backed factory to capture the physical drivetrain and selected settings:

```java
GeometryFactory factory = new GeometryFactory(follower);
Path path = factory.holonomicPath(start, corner, end).buildToMemory();
// Direct generation, when needed:
MotionParameters[] profile = path.createGenerator().generateRestToRest();
```

`factory.tankPath(...)` captures the tank profile and `factory.holonomicPath(...)` captures
the holonomic profile. Neither building nor reversing a path actuates a dual-actuated drive
or selects its runtime profile. Each stored path retains its own settings and physical model.
Online following captures current settings and the actual drivetrain model at traversal start.
Standalone builders still work: `setPlanningConstants(constants)` selects the default model
for those settings, or `setDriveModel(model)` binds a custom model and a settings snapshot.
For explicit use, `follower.getDriveModel(Profile.TANK)` returns a hardware-free snapshot;
`follower.getDriveModel()` uses the current mode. Circular omni models copy the constructed
wheel geometry, so changing the original motor configuration cannot alter an existing model.

Tank and holonomic profiles share the entire sampled solver: both advance scalar distance
with coupled heading velocity/acceleration constraints. Their physical limits and power
metrics differ, which the drive models encapsulate. Separate generators would duplicate
window planning, reachability, constraints, caching, and validation without simplifying
those differences. `TurnProfileGenerator` remains separate because stationary turns use
angular displacement instead of distance along a path.

`HolonomicDriveModel` supports mecanum directional limits, isotropic swerve limits, and
circular omni limits. Omni planning and allocation use the exact peak of the combined
signed translation and rotation wheel commands for any wheel count or mounting offset.
Its analytic acceleration bound intersects each wheel's affine power interval. Swerve planning continues to assume steering alignment and
does not constrain module steering slew. `TankDriveModel` reserves the worse calibrated
forward/reverse demand because scalar profile speed does not encode travel direction.
The optional conservative profile kV changes planning only. Runtime feedforward uses
identified kV. Custom mutable drive models use live geometry evaluation unless they
explicitly opt into solver sample caching. Custom directional providers also default to
uncached evaluation; built-in providers opt in. The drivetrain supplies its model through
`createDriveModel(settings, mode)` for runtime and planning alike. Directional overrides
are recreated against each settings snapshot. Stopping-horizon sizing checks all 360
body directions for every holonomic model, including custom providers and omni layouts.
See [custom drivetrains](CustomDrivetrains.md) for the extension contract.

The former mecanum, swerve, tank, omni, and isotropic generator classes have been removed.
Replace direct construction with `new PathProfileGenerator(path)` or
`new PathProfileGenerator(path, model, powerLimit)`. Calibration JSON keys are unchanged.

Online traversal projects the measured pose onto the route. The coupled solver samples
only its visible arc-length window, exact boundaries, and authored constraint transitions.
Online windows retain every geometry LUT node (about 0.75 nodes per inch
overall), reusing those precomputed points across updates. Exact window boundaries,
the midpoint, and constraint transitions are added as needed. Geometry projection
and stored trajectories continue to use the full-resolution LUT. The measured-state
trajectory solve is recalculated each update. Retained LUT nodes cache
their limits, heading derivatives, and power evaluators; moving boundary nodes are
evaluated afresh. Built-in constraint and heading mutations invalidate those caches.
Physical planning limits are checked once per window. Custom mutable implementations
use live evaluation unless they provide the appropriate revision or change hooks.
Closest-point queries seed a bounding-box hierarchy with the previous LUT index,
then certify the global nearest LUT point before applying the existing Newton refinement.
Heading splines retain an interval cursor and use binary search after larger jumps.
Constraint schedules retain stable authored order and prefix limits, selecting only
transitions inside the current window. Window arrays and controllability storage
are reused. Online solving publishes only the first squared-speed successor while
still validating future transitions. Before searching predecessor speeds, each
interval bounds squared speed by the next stopping bound plus twice distance times
its maximum acceleration, and by necessary signed angular-acceleration limits.
A power-feasible maximum acceleration bypasses branch
splitting for all drivetrain types. Command publication reuses the first window
point instead of reconstructing its geometry. Mecanum and tank affine power branches use
analytic bounds verified against the signed power evaluator; eligible swerve branches
use verified quadratic bounds. Unsupported branches retain bounded numerical searches.
Windows use full geometry resolution on the first solve; there is no coarse pass
or coarse-to-fine retry. Overspeed still uses measured-state braking recovery.
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
Scalar profile and LUT access remain available for regression analysis; followers
and tracking logs use the shared trajectory sample.

`trajectory.TrajectoryTest` benchmarks build cost, packed field bytes, and query/update
latency and writes `TeamCode/build/trajectory-benchmark.csv`. It also checks solve counts:
stored queries perform zero solves; each online query solves only its bounded window.
Desktop timings do not establish robot-controller performance.
See [validation results and remaining fixture failures](TrajectoryValidation.md).
See [online planning optimizations and comparison measurements](QuickBuildPerformance.md).
See [full-resolution planning and shared solver improvements](OnlinePlanningEfficiency.md).
