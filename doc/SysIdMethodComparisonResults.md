# Controlled SysId comparison

October 5, 2026. The deciding difference in these simulator tests is the powered
back-EMF equation, rather than the choice of joint versus sequential regression.
The existing Apex joint regression works accurately when that one simulator term
is made compatible with its assumed model. This is not a hardware verdict.

## Experiment

`SysIdMethodComparisonTest` uses the installed FTCodeSim v0.1.0-alpha.1 physics,
four ramp/dynamic excitations in both directions, and one deterministic clock for
physics, Pinpoint and the follower. Physics steps at 5 ms; control/sensing at
20 ms. Native and filtered velocities are recorded over identical 100 ms windows,
with preceding delivered power and a common native-velocity movement gate.
Battery voltage is fixed at 13 V. Tests use isolated temporary tuning storage.

Each dataset is replayed through the production Apex joint regression and an
implementation of Road Runner's legacy sequential regression mathematics:
ramp power-versus-velocity OLS, then constant-power acceleration residual OLS.
This matched-data comparison does **not** execute a complete Road Runner tuner:
it keeps Apex's excitation, windows, direction normalization and accelerating
sample selection, instead of Road Runner's position differentiation/acquisition.
It therefore tests regression choice, not the performance of Road Runner itself.

Freshly identified gains then drive independent, open-loop three-second time
trapezoids in both directions: one second each of acceleration, cruise and braking,
with peaks of 24 in/s or 1.5 rad/s. Error is physical velocity minus target,
normalized by run direction and averaged across both directions. Cruise here
uses the **entire scheduled cruise second**, unlike the manual UI's first-target-
crossing gate. Braking error is not included in the reported means.

## The deciding difference

The installed simulator's `SimulatedDrivetrain.registerMotor()` builds this term:

```java
(v,d,b) -> Math.signum(v) == Math.signum(d) ? -v * Math.abs(d) : 0
```

For powered forward translation, the loaded coefficients and rolling friction
reduce to the following equation (in inches, seconds and normalized power u):

```
a = (183 - (150/65) * v) * u - 49
```

The velocity-dependent power multiplier cannot be represented exactly by the
constant-gain model `u = kS sign(v) + kV v + kA a`. At 24 in/s, stock physics
requires about 0.384 power to hold speed; the native joint fit predicts 0.434,
which independently produces +6.397 in/s² of acceleration at that speed.
A visually good regression fit on the excitation samples does not establish
that its holding-power prediction is correct at the profile's cruise speed.

Independent acceleration probes establish the difference without any regression
or sensor estimator. The change in acceleration per unit change in power is:

| Imposed speed (in/s) | Stock simulator | Test control |
| --- | ---: | ---: |
| 8 | 164.538 | 183.000 |
| 24 | 127.615 | 183.000 |
| 40 | 90.692 | 183.000 |

The test-only control replaces `-v * abs(d)` with `-v` in the powered,
same-direction term. All coefficients, friction, kinematics, sensing, sampling,
regression and opposite-direction/zero-power braking branches stay identical.
Each variant is freshly characterized before its profile test. This is an
ablation to isolate a cause, not a proposed production physics change.

## Profile results

Values below are mean cruise errors, in in/s for drive and rad/s for heading.

| Regression and measurement | Stock drive | Control drive | Stock heading | Control heading |
| --- | ---: | ---: | ---: | ---: |
| Apex joint, native velocity | +1.831 | -0.076 | +0.548 | -0.005 |
| Apex joint, filtered velocity | +1.754 | -0.141 | +0.481 | -0.108 |
| RR-style sequential, native velocity | +4.355 | +1.796 | +1.935 | +0.587 |
| RR-style sequential, filtered velocity | +4.316 | +1.679 | +1.905 | +0.567 |

For native joint fits, acceleration mean error changes from -0.735 to -0.051
in/s for drive, and +0.057 to -0.007 rad/s for heading. The control's independently
probed holding acceleration is zero for both axes. Assertions verify both holding
probes and cruise tracking within 3% of peak speed, as well as the acceleration-
per-power slopes above.

Native velocity alone does not remove the stock overspeed. After removing the
simulator cross term, the remaining filtered heading error makes measurement
filtering a secondary issue worth examining separately. The matched-data
sequential fit is not a fix; its remaining error is not evidence against Road
Runner's complete workflow, which uses different acquisition and excitation.

## Reproduction and scope

Run from the repository root:

```powershell
.\gradlew.bat :TeamCode:testDebugUnitTest --offline --init-script work/manual-sysid-build.gradle --tests org.firstinspires.ftc.teamcode.sim.SysIdMethodComparisonTest --max-workers=2
```

The existing init script redirects Gradle build products to avoid files locked by
the interactive simulator. It is optional when that simulator is closed.
The test writes `windows.csv`, `comparison.csv`, `profiles.csv` and
`authority-probes.csv` under `TeamCode/build/sysid-method-comparison/`.

The original automatic tuning algorithm and production simulator configuration
were not changed by this experiment. Its positive control establishes that the
existing joint method can work with matching physics and synchronized native
measurements. It does not establish real-robot behavior, combined translation/
rotation tracking, battery-voltage portability, or braking accuracy. Earlier
wall-clock harness results are not numerically interchangeable with this test.

The installed simulator source was read from its Gradle cached sources JAR
(`ca8602ac033d8d8e0cbbdba9fe9046ffd0f7555e`). Local extracts are under
`work/sim-reference/`; the deciding expression is in `SimulatedDrivetrain.java`.

## Temporary tuner-only workaround

At the user's request, automatic and manual drive/heading `FeedforwardTuner`
phases now temporarily remove the powered back-EMF duty multiplier on desktop
SimMotor instances. `SimulatorSysIdMotorOverride` lives only on the desktop test
classpath; the robot APK has no simulator dependency. The original term is
restored on phase exit, including Stop, through the existing `onExit` cleanup.
Stock simulator creation, coefficients and braking branches are unchanged.

The adapter contains a TODO to verify the model with the simulator developer and
REV Hub measurements before treating this workaround as permanent. Gains learned
with it characterize the temporary tuning model; outside these SysId phases the
simulator still uses its stock model, so identical open-loop tracking there is not
promised. This workaround does not resolve the separate filtering/timing issues.
