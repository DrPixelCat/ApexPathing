# Custom drivetrains and directional kinematics

Extend `BaseDrivetrain` and implement `moveWithVectors(x, y, turn)`. Its configuration
extends `BaseDrivetrainConstants` and builds the custom hardware. Existing `Mecanum`
and `Omni` classes can also be subclassed. `Follower.create(...)` recognizes their
subclasses and builds hardware once; other fixed drives use the generic holonomic
or tank follower according to their declared capability.

For custom tracking, extend `Follower` or `HolonomicFollower` and construct that follower
directly. The base lifecycle stays final; override `updatePath`, `holdPose`, and
`resetPathControllers` as needed. Concrete named followers remain final.

## Reading follower diagnostics

`Follower.getDiagnostics()` returns a read-only `FollowerTelemetry` view of tracking state.
The view stays live across updates and movements, without allocating a snapshot each tick:

```java
FollowerTelemetry tracking = follower.getDiagnostics();
follower.update();
telemetry.addData("Target speed", tracking.getTrackingVelocityTarget());
telemetry.addData("Cross-track error", tracking.getCrossTrackErrorIn());
```

Read it on the OpMode thread after `update()`. Retain individual values for history;
retaining the view retains access to current values. `isVelocitySampleAvailable()` is false
after pausing or resetting. Existing getters such as `follower.getTrackingVelocityTarget()`
remain available and read the same state. Trajectory targets and failures remain accessible
through `follower.getTrajectoryTarget()` and `follower.getTrajectoryFailure()`.
`Follower.setDiagnostics(FollowerDiagnostics)` still installs the optional visualization observer.

## Position-hold allocation

Position holding during a turn uses the same model's `commandDemand(...)` as path allocation.
Override `DriveModel.positionHoldCommand(Vector)` to shape a body-space position correction
before its power budget is applied. The default preserves the correction; the built-in mecanum
model preserves its forward/strafe velocity penalty. Omni uses its registered wheel projections
to budget the correction. Overrides must preserve zero and return finite command components;
they must not read sensors or actuate hardware. No allocator type branch is needed for custom models.

## Directional limits

`DirectionalKinematics.limitsAt(Angle)` returns immutable `DirectionalLimits` with
positive finite `maxVelocity` (in/s) and `maxAcceleration` (in/s squared). Direction is
robot-relative: 0 degrees is forward, 90 degrees is left. Acceleration is a symmetric
bound on acceleration/braking along that direction. It does not separately model
braking or impose a combined two-dimensional acceleration ellipse.

A mecanum or omni subclass can override the protected factory without replacing its
motor mixer or profile generator:

```java
public class MeasuredOmni extends Omni {
    public MeasuredOmni(Omni.Constants config, HardwareMap hardwareMap) {
        super(config, hardwareMap);
    }

    @Override
    protected DirectionalKinematics createDirectionalKinematics(FollowerConstants settings) {
        DirectionalKinematics defaults = super.createDirectionalKinematics(settings);
        return direction -> {
            DirectionalLimits base = defaults.limitsAt(direction);
            double sideways = Math.sin(direction.getRad());
            double scale = 1 - .2 * sideways * sideways;
            return new DirectionalLimits(base.maxVelocity * scale, base.maxAcceleration * scale);
        };
    }
}
```

Imports are `drivetrains.Omni`, `drivetrains.models.DirectionalKinematics`,
`drivetrains.models.DirectionalLimits`, `follower.FollowerConstants`, and the FTC hardware
types. The same override works in a `Mecanum` subclass. Return a measured lookup table
instead of a lambda when appropriate. To construct the subclass through robot configuration:

```java
Omni.Constants drive = new Omni.Constants() {
    @Override public Omni build(HardwareMap hardwareMap) {
        return new MeasuredOmni(this, hardwareMap);
    }
};
drive.setFirstWheelAngle(Angle.fromDeg(90));
drive.addWheel(new Motor("wheel0"));
drive.addWheel(new Motor("wheel1"));
drive.addWheel(new Motor("wheel2"));
```

Always bind the provider to the supplied `settings`, rather than the singleton or mutable
hardware configuration. Models recreate providers for settings snapshots and path reversal.
Providers must not read sensors, actuate hardware, or change drivetrain mode. Custom
providers default to uncached evaluation. Override `cacheSolverSamples()` only if geometry
is immutable and all changing inputs are covered by solver invalidation. A live provider
capturing mutable external data is unsuitable for an immutable stored trajectory.

The planner queries these limits before applying authored, calibrated-domain, angular,
curvature, and motor-power constraints. Directional overrides do not multiply feedforward
coefficients or motor power. Runtime tools can query `follower.getDriveModel().directionalLimits(angle)`.
Stopping-horizon sizing samples the same provider at one-degree intervals; a custom provider
with sharper features should choose a conservative explicit path lookahead distance.

## Default omni equations

For n uniformly spaced wheels (n >= 3), radial angle phi_i = phi_0 + 2*pi*i/n:

```
u_i = -sin(phi_i)*powerForward + cos(phi_i)*powerLeft + powerTurn
commandDemand = max_i(abs(u_i))
```

`WheelProjectionModel` owns these immutable projections and supplies motor mixing,
combined power demand, and exact affine acceleration bounds. Planning, runtime reference
power, and command allocation use the same geometry. Translation and rotation are combined
before taking absolute values, including when they cancel on a wheel. Allocation reserves
position correction first, then scales reference translation and rotation together, then
uses remaining budget for velocity correction. Separate stages retain a conservative sum
of their peak demands so their combined motor commands stay bounded.

For pure translation at body angle theta, let q(theta) be the largest absolute wheel
projection in that direction. Default directional limits are:

```
wheelVelocity = min(forwardVelocity*q(0), strafeVelocity*q(pi/2))
wheelAcceleration = min(forwardAcceleration*q(0), strafeAcceleration*q(pi/2))
maxVelocity(theta) = wheelVelocity / q(theta)
maxAcceleration(theta) = wheelAcceleration / q(theta)
```

This assumes matching wheel sizes/gearing and conservatively fits one common wheel limit
to both configured axis measurements. Override the provider for measured direction-specific
behavior. Rotation consumes normalized motor power; its physical radius is represented by
angular calibration. The normalized mixer is exact; predicting physical motion still depends
on feedforward calibration.

Three wheels use this same implementation. The former Kiwi classes and enum entry are
removed. Register motors in counterclockwise radial order and configure tangent directions
with `Motor.reverse()` as necessary. The old fixed motor ordering and turn convention should
not be copied blindly. Saved tuning JSON tagged `KIWI` migrates to `OMNI`.

## Full model overrides

For different physical limits or power behavior, override the public factory:

```java
@Override
public DriveModel createDriveModel(FollowerConstants settings, Path.PathType mode) {
    if (mode == Path.PathType.TANK) return super.createDriveModel(settings, mode);
    return new MyDriveModel(settings, getWheelProjectionModel());
}
```

This example applies to an `Omni` subclass. The default factory already uses the directional
hook; a complete custom model is responsible for consuming its own directional provider.
Both execution and online planning use the factory, and follower-backed builders snapshot
its model for stored profiles. The requested path mode is validated without actuating hardware.

A `DriveModel` implements physical velocity/acceleration limits, `translationDemand`,
`powerEvaluator`, and `withConstants`. Override the scalar `commandDemand(x,y,turn)` to
model combined translation/rotation demand. `withConstants` must preserve physical geometry
and recreate settings-dependent providers against its argument. For ordinary body-axis
feedforward, the inherited `PowerEvaluator` uses the shared `MotionPowerCalculator`;
analytic acceleration bounds are optional and otherwise use numerical search.

Command demand must be a convex, homogeneous budget (such as peak absolute wheel power):
scaling a command scales its demand, and the demand of summed commands does not exceed
the sum of their demands. Allocation relies on these properties. A custom physical wheel
mixer and its model must agree. Built-in mecanum wheel characterization retains its shared
inverse wheel solver. Standalone builders accept `setDriveModel(model)`; use a follower-backed
factory for the actual drivetrain's custom model and geometry.
