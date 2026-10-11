# Custom paths and followers

Use the normal `GeometryFactory` builders and `Follower.create(...)` unless you need
different behavior. Each override below is optional and independent. Custom geometry,
heading and constraints work with both `quickBuild()` (online planning) and
`buildToMemory()` (a precomputed spatial field).

## Supply a curve

Implement the existing `geometry.ParametricSegment` interface. Positions are field
coordinates in **inches**, even when your factory uses another distance unit.
Derivatives are with respect to `t`, where `0 <= t <= 1`.

```java
import geometry.DistUnit;
import geometry.ParametricSegment;
import geometry.Vector;

final class Wave implements ParametricSegment {
    private Vector vector(double x, double y) { return Vector.of(x, y, DistUnit.IN); }
    public Vector getPosition(double t) {
        return vector(40 * t, 5 * Math.sin(Math.PI * t));
    }
    public Vector getFirstDerivative(double t) {
        return vector(40, 5 * Math.PI * Math.cos(Math.PI * t));
    }
    public Vector getSecondDerivative(double t) {
        return vector(0, -5 * Math.PI * Math.PI * Math.sin(Math.PI * t));
    }
}
```

Supply exactly two endpoint poses. Their positions must match the curve's endpoints;
the builder checks these and samples for finite positions and derivatives. Providers
must remain immutable and supply smooth, valid geometry throughout the domain.
`PathSegment` still handles arc length, projection, tangents and curvature. The default
`getBoundsIn()` samples the curve; override it for exact or conservative bounds when
those samples could miss sharp extrema.

```java
GeometryFactory geometry = new GeometryFactory(follower).setDistUnit(DistUnit.IN);
Path path = geometry.holonomicPath(geometry.pose(0, 0, 0), geometry.pose(40, 0, 0))
        .setCurve(new Wave())
        .quickBuild();
follower.follow(path);
// In the OpMode loop:
follower.update();
```

Omitting `setCurve(...)` retains the standard spline behavior. Tank builders accept
custom curves too, with the same tangent-heading requirements as normal tank paths.

## Supply heading interpolation

Implement `paths.heading.HeadingInterpolator`, then call
`builder.setHeadingInterpolator(provider)` before building. This takes precedence over
built-in interpolation styles and heading nodes. It also supplies the path's final
heading for completion checks.

All `s` parameters mean **remaining inches**. Return heading in radians through
`Angle`; return first and second derivatives with respect to **forward distance
traveled**, in rad/in and rad/in squared. Thus a heading that increases with traveled
distance has a positive first derivative, although its remaining-distance argument
decreases. The planner uses `omega = headingPrime * velocity` and
`alpha = headingPrime * acceleration + headingSecond * velocity * velocity`.

The builder calls `copyForTrajectory()` and then `setPathLength(lengthInches)` on
that copy. Mutable providers must return a detached copy containing all configuration;
immutable providers may use the default returning `this` if `setPathLength` is a no-op.
Stored fields and reversed paths also take heading copies. Mutable providers may
retain the default revision of `-1` to force fresh online evaluations, or report a
nonnegative revision that changes whenever their configuration changes.

Tank providers must explicitly return `true` from `supportsTank()`. This promises
forward or backward tangent heading and corresponding curvature derivatives; it
does not enable arbitrary independent tank rotation. `TankInterpolator` already
implements this contract.

## Supply constraint values

Implement `paths.constraint.PathConstraint` without inheriting from either built-in
constraint class. Provide:

- `getS()` and `setS(...)`: normalized arc-length position where the limit takes effect.
- `getDomain()`: `TRANSLATIONAL` or `ANGULAR`.
- `getType()`: `VELOCITY` or `ACCELERATION`.
- `getLimit()`: in/s, in/s squared, rad/s, or rad/s squared according to domain/type.
- `copyForTrajectory()`: a detached copy for mutable constraints, used in reversal.

Add it with `builder.addConstraint(customLimit)`. Limits form the existing step
schedule: a value remains active until another constraint in that domain/type replaces
it. Builder positions must be in `[0, 1)`. Return finite, nonnegative values.

Custom constraints default to live evaluation, so online planning observes mutations
without a listener. For caching, implement `addChangeListener(...)`, return `true`,
and notify every registered listener whenever progress or value changes. Stored fields
retain their precomputed values. Reversal copies constraints and preserves their
schedule at the same field locations.

## Replace profile generation

Most physical differences belong in a `DriveModel` or directional kinematics provider;
see [custom drivetrains](CustomDrivetrains.md). To change profile generation itself,
subclass `trajectory.generators.PathProfileGenerator` and supply a per-path factory:

```java
builder.setProfileGeneratorFactory((path, settings, model) ->
        new MyGenerator(settings, path, model));
```

The factory receives the selected path, settings and drive model. Bind your generator
to these arguments. It must return a fresh non-null generator and must not read sensors
or write motors. Override `generateRestToRest()` for stored planning and the six-argument
`generateWindow(...)` for normal online planning. Direct `generate()` and experimental
`generatePursuitWindow(...)` calls remain separate entry points when you use those features.
Preserve the returned motion units, feasibility limits and failure behavior.

All `Path.createGenerator(...)` overloads use this factory, including follower planning,
stored precomputation and reversal. Reversed paths retain the factory and pass the
reversed path to it. Replacing the factory on a built path invalidates its stored data;
the next stored access rebuilds it. Captured factory configuration should be immutable.

## Replace tracking feedback

Extend `follower.drivetrains.HolonomicFollower` and override `updatePath(Path)` to
implement a different feedback law. This example uses its own position/velocity
correction instead of invoking Apex's built-in path controller:

```java
import com.qualcomm.robotcore.hardware.HardwareMap;
import config.ApexConstants;
import follower.drivetrains.HolonomicFollower;
import geometry.Vector;
import paths.movements.Path;
import trajectory.TrajectorySample;

public final class MyFollower extends HolonomicFollower {
    public MyFollower(ApexConstants config, HardwareMap hardwareMap) {
        super(config, hardwareMap);
    }

    @Override protected MovementOutcome updatePath(Path path) {
        TrajectorySample target = samplePathTarget(path);
        Vector correction = target.toPath.times(.04)
                .plus(target.velocity.minus(getVelocity().getVec()).times(.01));
        double headingCorrection = .8
                * getPose().getHeading().getShortestAngleTo(target.heading).getRad();
        return executeHolonomicPath(path, target, correction, headingCorrection);
    }
}
```

Construct `new MyFollower(robotConfig, hardwareMap)` directly, then use the same
`GeometryFactory(follower)`, `follow(path)` and `update()` workflow. Gains above are
illustrative and require tuning on your robot.

Call `samplePathTarget` once per path tick. It selects online planning or the stored
field. Call `executeHolonomicPath` once and return its outcome. The helper calculates
body-axis feedforward with the shared power model, allocates bounded motor commands,
publishes diagnostics, runs callbacks, and applies the shared completion dwell and
stuck detection. All supplied corrections are normalized field-space **power**, not
velocity. They share the position-correction allocation stage. Controller disable
switches suppress feedback while retaining feedforward.

For your own feedforward, use the overload taking `fieldFeedforward` and
`headingFeedforward`. The convenience overload does not include the built-in tracker's
specialized inverse-wheel pulse compensation or centered braking state. Diagnostics
report the supplied command; they cannot infer how your custom algorithm divided
position and velocity feedback.

The final base `update()` continues to own localization, invalid-measurement handling,
pause/resume and outcomes. These helpers require the active holonomic path and a
holonomic drivetrain. For a completely different tracker, including tank tracking,
the existing protected `updatePath`, `holdPose` and `resetPathControllers` hooks remain
available; raw motor writes make your tracker responsible for its path services.

Executable external-package examples and regression coverage are in
`TeamCode/src/test/java/customization/` and `CustomFollowerIntegrationTest`.
