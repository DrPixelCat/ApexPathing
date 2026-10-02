# Profile power ceiling

Set a normalized motor-power ceiling per path:

```java
Path path = new HolonomicPathBuilder(startPose, endPose)
        .setProfilePowerLimit(0.95)
        .profiledBuild();
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
when constructing a generator. To change an existing path, call
`path.setProfilePowerLimit(value)` followed by `path.rebuildMotionProfile()`.
Reversing a path preserves its setting. These settings do not change the global
follower constants or standalone turn profiles.

Holonomic `quickBuild()` generates no feedforward profile, so this setting has no
effect there. Tank quick profiles honor it. This API controls planned feedforward
utilization; the follower's feedback corrections can use additional available motor
power. See the [feedforward audit and fixes](ProfileGeneratorAudit.md) for measured
results and the limits of the sampled solver.
