# Straight-line interior braking comparison

The continuous braking proposal works well on a motor plant that obeys its feedforward model. It performs worse than the current follower on Apex's current mecanum simulator. Acceleration passed before either result was interpreted. The experiment therefore does **not** support replacing production braking yet.

## Reproduce

From the repository root:

```powershell
.\gradlew.bat :TeamCode:testDebugUnitTest --tests org.firstinspires.ftc.teamcode.sim.StraightDecelerationComparisonTest --tests trajectory.profile.DynamicMotionProfileTest
```

The new test is `TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/StraightDecelerationComparisonTest.java`. It writes tables, profile knots, gain snapshots, acceleration traces, signed motor-response probes, complete controller traces, and speed/power plots to `TeamCode/build/straight-braking-comparison/`. The baseline tuning is frozen in `TeamCode/src/test/resources/straight-deceleration-baseline.json`, copied from the saved simulator tuning on October 1, 2026. Runtime tuning files and production control code are unchanged.

## Course and controls

A 360-inch straight profiled path starts from rest and contains three interior slowdowns:

| Requested distance | Speed change |
|---:|---:|
| 60 in | 42 → 18 in/s |
| 150 in | 40 → 12 in/s |
| 240 in | 36 → 20 in/s |

The speed ceilings rise again at 90, 180, and 270 inches, testing recovery acceleration after every slowdown. Execution stops at 300 inches, before endpoint braking. The existing generator rounds these constraint changes onto its sample grid: the low-speed boundaries occur at 60.448, 150.448, and 240.448 inches. Boundary metrics use those generated positions to avoid attributing profile discretization to the controller.

Three policies use the same profile knots, plant, gains, and initial state:

- **CURRENT:** the actual production `Follower`, including its coast logic, opposing-power pulses, and minimum target velocity.
- **CONTINUOUS_SAME_LUT:** the proposed scalar controller, using the existing LUT interpolation. This isolates the control-law change.
- **CONTINUOUS_CONSTANT_ACCEL:** the proposed scalar controller with `v = sqrt(v0² + 2*a*ds)` between knots. This also tests physically consistent displacement interpolation.

The proposed controller computes:

```text
aApplied = aTarget
if aTarget < 0:
    aApplied *= clamp(vMeasured / vTarget, 0, 1)²
power = kS*motionSign + kV*vTarget + kA*aApplied + K*(vTarget - vMeasured)
```

Power is clipped to [-1, 1]. There are no brake pulses, coast overrides, or minimum velocity floor in the prototype. All policies are scored against the same constant-acceleration displacement reference, including the existing follower.

The matrix contains **81 complete runs**: three policies × three plants × 10/20/40 ms control periods × exact velocity / 40 ms velocity lag / lag with deterministic noise and loop jitter. The known linear plant is tested with both mecanum and tank production followers. The third plant uses the current mecanum motor model and its forward chassis friction.

The bench advances the actual four `SimMotor` instances in steps of at most 1 ms, integrating an unbounded straight-line position. It excludes field walls, lateral slip, steering, battery variation, endpoint capture, and real-time localizer timing. Tank and mecanum prototypes are identical scalar laws on this straight course; this does not validate turning or Ramsete integration.

## Acceleration prerequisite

Before comparing braking, the test verifies the known plant's holding and positive-acceleration response, then runs a feedforward-only 6 → 36 in/s acceleration ramp at 30 in/s². The ramp errors were **0.175 in/s RMS** for both known plants and **0.189 in/s RMS** for the current mecanum plant. No gain recharacterization was needed in the final experiment.

Every full run also checks all positive-acceleration legs, including recovery after each slowdown. Its gate requires RMS error ≤4.2 in/s, peak absolute error ≤8.4 in/s, and completion. The first three inches are excluded from that tracking metric, but each run starts from rest and must complete. **All 81 runs passed; the largest acceleration RMS was 1.627 in/s and the largest peak absolute speed error during acceleration was 4.298 in/s.** A failed prerequisite prevents braking interpretation for that fixture.

During setup, a test-helper issue was discovered: `ApexSimulation.createTankHardware()` assigns coefficients intended for a different motor-term layout. The new benchmark explicitly installs the known plant's motor terms for both linear fixtures and checks their response, rather than calibrating away the error. That existing helper was left unchanged; its full chassis behavior is not validated by this experiment.

## Results

Representative results at a **20 ms control period with 40 ms velocity lag**:

| Plant | Policy | Acceleration RMS | Braking RMS | Worst boundary error | Power variation/s |
|---|---|---:|---:|---:|---:|
| Known linear, mecanum | Current | 1.080 | 1.671 | 1.879 | 20.990 |
| Known linear, mecanum | Continuous, constant acceleration | 1.013 | 1.232 | 1.881 | 0.773 |
| Known linear, tank | Current | 1.079 | 2.822 | 5.319 | 11.831 |
| Known linear, tank | Continuous, constant acceleration | 1.013 | 1.232 | 1.881 | 0.773 |
| Current mecanum simulator | Current | 0.882 | 2.416 | 1.777 | 11.693 |
| Current mecanum simulator | Continuous, constant acceleration | 0.518 | 8.989 | 7.998 | 5.346 |

Speed errors are in in/s. Boundary error is the largest absolute error at the three generated low-speed boundaries. Power variation/s measures total absolute command changes during braking divided by braking duration; it is **not** a physical jerk measurement.

With exact velocity at 20 ms, the known mecanum plant's braking RMS drops from **2.827 to 0.329 in/s**, and the tank follower's drops from **3.733 to 0.329 in/s**. With lag, the continuous law has a small underspeed bias. On the known plant its power command varies far less than the current pulse controller, and all nine constant-acceleration cases meet the diagnostic braking targets: RMS ≤3 in/s, peak absolute error ≤6 in/s, and boundary error ≤3 in/s.

On the current mecanum simulator, all nine constant-acceleration cases miss those targets. At 20 ms with lag, its largest braking underspeed is **15.566 in/s**. The current controller passes the targets in that particular case, but is not consistent across the full matrix. Full results, including the same-LUT comparison, are in the generated `results.md`.

![Known plant power commands](../TeamCode/build/straight-braking-comparison/LINEAR_MECANUM-power.png)

![Current simulator speed tracking](../TeamCode/build/straight-braking-comparison/DEFAULT_MECANUM-velocity.png)

## Why the current simulator rejects the simple model

After all acceleration gates passed, isolated signed-response probes were run using the unchanged feedforward gains. At 24 in/s:

| Requested acceleration (in/s²) | Feedforward power | Actual acceleration (in/s²) |
|---:|---:|---:|
| +30 | +0.3668 | +31.4 |
| +60 | +0.5733 | +54.4 |
| −30 | −0.0462 | −256.7 |
| −60 | −0.2527 | −289.7 |
| −120 | −0.6658 | −355.6 |

The installed FTCodeSim v0.1.0-alpha.1 motor source selects a regenerative term whenever power opposes velocity:

```java
(v, d, b) -> Math.signum(v) != Math.signum(d) && d != 0 ? -v : 0
```

That term is independent of power magnitude. Its activation changes the motor response sharply as power crosses zero. A single affine `kS/kV/kA` model fitted to positive acceleration cannot predict it. The proposed underspeed safeguard reduces negative acceleration after measured speed drops, but filtered velocity responds too late to prevent a large undershoot here.

This is evidence about the installed simulator and tested tuning, not proof that a physical robot has the same response. Positive-acceleration success alone does not establish the validity of negative-acceleration feedforward.

## Effect on the braking plan

1. Keep the displacement profile and its forward/backward sweeps. The controlled experiment supports continuous signed feedforward and constant-acceleration interpolation when the plant fits the model.
2. Verify holding, positive acceleration, zero-power deceleration, and opposing-power deceleration on the actual drivetrain before replacing the current braking. Use speed/power response probes to determine whether one feedforward model works across both signs.
3. Address the simulator's discontinuous opposing-power response and the tank fixture's coefficient mapping separately. A different constant braking gain alone cannot reliably represent a discontinuity or a range of accelerations the instantaneous plant cannot produce.
4. Rerun this comparison against the verified plant. Require the acceleration gates first, then braking accuracy, boundary speed, and command smoothness across loop rates and sensor lag. Integrate the continuous controller into production only after those checks pass.

No production braking change was made as part of this test.
