# Road Runner braking slope experiment

**Validation correction:** these results used a custom motor/chassis plant, which has now been
removed. They do not validate stock FTCodeSim. See [stock simulator validation](StockFTCodeSimValidation.md).

This report records the experiment before the production integration. The current follower now
uses the [continuous profiled braking implementation](ProfiledBrakingImplementation.md). Its
validation test writes a new gallery under `build/profiled-braking-validation`; the baseline
gallery linked below retains the earlier controller's results.

The updated simulator and the actual Road Runner library support using gentler braking. With a 20 ms loop and 40 ms velocity lag, Road Runner's displacement adaptation reduces braking RMS error from **1.89 in/s at −120 in/s²** to **0.67 at −30** and **0.34 at −15**. Acceleration stays at +60 in/s² throughout the comparison.

For Apex's displacement-based follower, **−30 in/s² is a sensible initial braking limit**. −60 also meets the experiment's targets across all tested loop periods and sensor conditions, but −30 has more margin. These results come from a configured nominal plant; they do not establish the physical robot's braking limit.

## Run graphs and data

After running the test, open:

- [Graph index: every run](../TeamCode/build/braking-slope-sweep/index.html)
- [Full results](../TeamCode/build/braking-slope-sweep/results.md)
- [Summary CSV](../TeamCode/build/braking-slope-sweep/summary.csv)

The index provides filters for drivetrain, braking limit, controller, loop period, and sensor condition. **All 315 runs have four plots each:** speed against distance, motor power, speed error against time, and requested acceleration. There are also 30 controller overlays and 12 plots comparing braking limits. Every run has its complete trace CSV.

![Mecanum error as braking becomes steeper](../TeamCode/build/braking-slope-sweep/MECANUM-rms-slope-s1.png)

![Controllers on the gentler −30 in/s² profile](../TeamCode/build/braking-slope-sweep/MECANUM-b30-overlay-s1.png)

Run the current implementation's validation from the repository root:

```powershell
.\gradlew.bat :TeamCode:testDebugUnitTest --tests org.firstinspires.ftc.teamcode.sim.BrakingSlopeSweepTest --tests org.firstinspires.ftc.teamcode.sim.ApexMotorPlantTest
```

## Simulator changes

The published FTCodeSim version is still `v0.1.0-alpha.1`. The [current upstream motor registration](https://github.com/jjophoven/FTCodeSim/blob/d5e7f965a5774ee019a5c55b0c3881547127746b/ftcodesim/src/main/java/org/codeblooded/ftcodesim/hardware/drivetrain/SimulatedDrivetrain.java) still activates a speed-dependent regenerative term when commanded power opposes motion, regardless of that power's magnitude. Updating the dependency alone would retain the discontinuity observed in the earlier experiment.

The project now installs its own desktop motor/chassis adapter:

- A continuous four-quadrant DC motor model uses applied voltage, duty-independent back EMF, and inertia. Small opposing power produces a small torque change rather than activating an unrelated braking term.
- Rolling and static friction act once at the chassis. Friction removes momentum without making an unpowered robot reverse at rest.
- At zero power, BRAKE retains electrical damping; FLOAT opens the bridge and retains rolling friction.
- Tank chassis friction uses configuration values, and chassis velocity obeys the tank's heading and lateral-motion constraint. Wheel roll velocities stay synchronized with chassis motion.
- Interactive simulation retains field collisions, hardware registration, localization adapters, and bounded physics steps. Unbounded characterization/path fixtures use those same equations with walls disabled.

The nominal configuration retains 75 in/s maximum loaded speed and 150 in/s² initial acceleration at full power. Its straight-line feedforward values at 13 V are `kS=0.06`, `kV=0.94/75`, and `kA=0.94/150`. Existing desktop tuning from the previous plant should be regenerated through the tuners. Robot tuning files were not overwritten.

The implementation is in `ApexMotorPlant`, `ApexChassisPhysics`, and the mecanum/tank adapters under the desktop test source tree. Road Runner is also a **test-only dependency**, so it is not packaged in the Robot Controller APK. The production Apex braking controller is unchanged.

## What the Road Runner source actually does

The experiment executes `com.acmerobotics.roadrunner:core:1.0.1`, rather than treating the earlier custom scalar controller as an exact implementation of Road Runner. The release source is pinned to commit `03367b9701fd37f98705d65af56e4d4cd27417a6`.

The [profile implementation](https://github.com/acmerobotics/road-runner/blob/03367b9701fd37f98705d65af56e4d4cd27417a6/road-runner/core/src/main/kotlin/com/acmerobotics/roadrunner/Profiles.kt) accepts independent negative and positive acceleration constraints. It builds forward/backward envelopes, merges them, and evaluates each displacement interval with constant acceleration using `v²=v0²+2*a*ds`. Its time representation evaluates position, velocity, and acceleration from the same intervals.

The [holonomic controller and motor feedforward](https://github.com/acmerobotics/road-runner/blob/03367b9701fd37f98705d65af56e4d4cd27417a6/road-runner/core/src/main/kotlin/com/acmerobotics/roadrunner/Control.kt) add position/velocity feedback to the requested velocity before motor feedforward. The reference acceleration remains signed. The feedforward implementation uses the commanded velocity's sign for the static term. It does not apply the earlier prototype's underspeed-dependent acceleration scaling or braking pulses.

The [standard mecanum execution pipeline](https://github.com/acmerobotics/road-runner-quickstart/blob/6e63a7792e9bb6958798bf46fc03d84765b50c51/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/MecanumDrive.java) samples the trajectory by elapsed time, runs the holonomic controller, converts its output to wheel commands, and divides feedforward voltage by measured battery voltage. Its sample configuration uses separate limits of −30 and +50 in/s²; those values are starting configuration, not a universal robot limit.

The sweep therefore distinguishes:

| Policy | Behavior |
|---|---|
| `APEX_CURRENT` | Actual production Apex follower, including its existing LUT interpolation, minimum target, coast overrides, and pulses |
| `PROPOSED_SCALED` | Earlier scalar proposal with underspeed-dependent negative-acceleration scaling |
| `RR_DISPLACEMENT` | Actual RR profile, holonomic controller, wheel kinematics, and motor feedforward, sampled at actual distance; axial position error is zero |
| `RR_TIME` | Same RR holonomic pipeline sampled by time, with axial position gain 2/s |

All policies use matched feedforward coefficients. RR's velocity correction gain is converted from Apex's power-error gain so the equivalent straight-line correction strength is equal. These are explicit experiment parameters, not a claim about the gains on a particular Road Runner robot.

`RR_TIME` is tested on the mecanum plant. `RR_DISPLACEMENT` on the tank plant is a straight-line adaptation of the holonomic scalar pipeline. It is **not a test of native Road Runner Ramsete**. The [native tank source](https://github.com/acmerobotics/road-runner-quickstart/blob/6e63a7792e9bb6958798bf46fc03d84765b50c51/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/TankDrive.java) uses Ramsete; in core 1.0.1 its returned command has constant dual-number derivatives, so it should not be silently treated as the same acceleration-feedforward pipeline.

## Experiment and acceleration prerequisite

The straight course contains three interior slowdowns, with rising speed ceilings between them:

| Distance | Speed ceiling |
|---:|---:|
| 0 in | 42 in/s |
| 90 in | 18 in/s |
| 120 in | 40 in/s |
| 210 in | 12 in/s |
| 240 in | 36 in/s |
| 330 in | 20 in/s |
| 360 in | 42 in/s |

The path is 480 inches long and execution stops at 420 inches, excluding endpoint capture. The longer course allows even −15 in/s² to reach all requested speeds and gives every policy recovery-acceleration segments.

For each braking limit, all controllers share the same profile generated by the actual RR profiler. Injecting that profile into Apex isolates its controller behavior and permits independent braking/acceleration limits without changing production profile generation. This is a controller comparison, not an end-to-end comparison of different planners.

The matrix tests braking magnitudes **120, 90, 60, 30, and 15 in/s²**, 10/20/40 ms control periods, and exact velocity / 40 ms first-order velocity lag / lag with deterministic ±0.2 in/s noise and loop jitter. There are 180 mecanum runs and 135 tank runs.

Before the sweep, a feedforward-only 6→36 in/s ramp gives **0.175 in/s RMS** on both plants. Every complete run must then pass positive-acceleration tracking, including recovery after each slowdown. **All 315 pass:** maximum acceleration RMS speed error is **1.228 in/s**, and maximum peak speed error during acceleration is **2.613 in/s**. The gates require RMS ≤4.2, peak ≤8.4, and course completion; the first 3 inches are excluded from the tracking metric.

Plant tests independently cover holding, reachable acceleration in both motion directions, saturation, continuity through opposing/zero power, passive stopping, kinematics, and a moving tank turning without lateral slip. Tank curve/turn/reverse tracking and the tank feedforward identification workflow also pass after updating the fixture calibration for the new plant. Accuracy tolerances were retained.

## Results and interpretation

At **20 ms with 40 ms velocity lag**, braking RMS speed error is:

| Negative acceleration limit | Apex mecanum | RR displacement adaptation | RR holonomic time reference | Apex tank |
|---:|---:|---:|---:|---:|
| −120 in/s² | 6.55 | 1.89 | 1.29 | 7.54 |
| −90 in/s² | 4.22 | 1.67 | 1.18 | 5.65 |
| −60 in/s² | 1.69 | 1.25 | 0.98 | 2.73 |
| −30 in/s² | 1.65 | 0.67 | 0.65 | 1.78 |
| −15 in/s² | 1.18 | 0.34 | 0.34 | 1.18 |

Values are in in/s. RR's displacement adaptation gives the same result on the nominal straight tank plant. The time-reference results include position feedback and a different reference progression; they should not be attributed solely to changing the motor braking formula.

At −30, RR's displacement adaptation has at most **0.80 in/s RMS** and **0.98 in/s boundary error** across all loop/sensor cases. At −60 the maxima are **1.52** and **2.33** respectively. Both limits pass the braking acceptance targets in every tested condition on both plants: RMS ≤3, peak absolute speed error ≤6, and boundary error ≤3 in/s.

Steeper braking is physically achievable on this plant, but it is less tolerant of measurement lag and discrete control updates. At −120, exact velocity produces only 0.84 in/s RMS error at 20 ms for the displacement adaptation; adding lag raises that to 1.89 and boundary error to 4.61. During deceleration, lagged velocity reads higher than instantaneous velocity, causing extra negative feedback. Reducing the deceleration makes that false overspeed smaller. This is an inference supported by the paired exact/lag cases, not a hardware measurement.

Apex's fixed opposing pulses have an additional limitation: their magnitude and cooldown do not continuously match requested acceleration. Slowing the profile helps its current controller too, but at −30 its power command still varies much more than RR's continuous command. The earlier scaling safeguard gives little benefit at these gentler slopes; at −30 and −15 it produces the same result as the unscaled RR displacement pipeline in this sweep.

## Recommended next production change

Keep displacement profiling, expose an independent braking-acceleration limit for profiled paths, and initially set it to **30 in/s²** without reducing positive acceleration. Use constant-acceleration interpolation and continuous signed acceleration feedforward, with feedback applied consistently to the commanded velocity. The evidence supports starting with the straightforward RR-style rule; add a stall/underspeed safeguard only if a separate disturbance experiment demonstrates a need.

Before committing that controller change to robot operation, retune the physical drivetrain's feedforward and run these same acceleration, boundary-speed, and loop/lag checks. Quick builds and endpoint capture remain separate work. No production braking change was made in this experiment.
