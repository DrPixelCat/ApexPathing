# Profile power ceiling

Set a normalized motor-power ceiling per path:

```java
Path path = new HolonomicPathBuilder(startPose, endPose)
        .setProfilePowerLimit(0.95)
        .buildToMemory();
```

`TankPathBuilder` accepts the same setting. Values must be finite and in `(0, 1]`.
It is a ceiling: velocity, acceleration, heading, and endpoint constraints can require
less power. The coupled solver searches against the requested ceiling; final validation
allows a numerical tolerance of 0.000001.

Without an explicit value, the existing defaults apply: 1.0 without characterization,
or 0.88 times calibrated available motor power with characterization. An explicit
value replaces the 0.88 reserve, while calibrated available motor power still caps
the result. For example, requesting 1.0 with calibrated available power 0.9 produces
a ceiling of 0.9. Characterized profiles keep measured velocity domains as hard
bounds and evaluate power jointly with signed acceleration, including braking.

For direct generation, mecanum, tank, and swerve generators also accept an override:

```java
MecanumProfileGenerator generator =
        new MecanumProfileGenerator(constants, path, 0.95);
MotionParameters[] profile = generator.generate();
double effectiveCeiling = generator.getProfilePowerLimit();
```

The constructor override takes precedence over the path setting. Settings are captured
when constructing a generator. Set the ceiling in the builder before building.
A stored trajectory permanently retains the ceiling used for its precomputation.
Reversing a path preserves its setting. These settings do not change the global
follower constants or standalone turn profiles.

Both `quickBuild()` and `buildToMemory()` honor this ceiling for all drivetrain types.
Online paths use planning settings when a traversal starts. Stored paths precompute once
and keep their original data. This API controls planned feedforward
utilization; the follower's feedback corrections can use additional available motor
power. See the [feedforward audit and fixes](ProfileGeneratorAudit.md) for measured
results and the limits of the sampled solver.
