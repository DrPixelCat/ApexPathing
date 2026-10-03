# Continuous displacement-based profiled braking

**Validation correction:** the simulator results below used a custom motor/chassis plant. They
do not establish compatibility with stock FTCodeSim. That custom plant has been removed; see
[stock simulator retuning and validation](StockFTCodeSimValidation.md) for the corrected run.

Apex's profiled holonomic and tank followers now consume the existing generated LUT's velocity
and signed acceleration directly. Negative acceleration produces continuous braking feedforward,
with the same velocity feedback used during acceleration. Fixed opposing pulses, coast overrides,
underspeed-dependent acceleration scaling, and the 6 in/s profile velocity floor are no longer
used by either path follower.

## Preserving Apex's constrained profile

This change does not replace Apex's generator with Road Runner. The existing drivetrain-specific
generators, forward/backward reachability, velocity and acceleration constraints, coupled power
checks, sample positions, and LUT interpolation are unchanged. No acceleration or braking limits
were changed, and no new hardcoded braking limit was added. Execution continues to sample at
measured displacement along the path.

The profile remains the feasible reference. Tracking feedback can request correction around that
reference; it does not regenerate or overwrite the LUT. Existing power allocation and drivetrain
power clipping still bound delivered commands. A feasible reference alone cannot guarantee that
a physical robot with model error, disturbances, or delayed measurements never exceeds a limit.

Cross-track and heading control, centripetal correction, tank Ramsete pose correction, endpoint
capture, boosted-acceleration endpoint handling, callbacks, completion, drive-mode switching,
quick builds, and turn execution retain their existing algorithms. This is a Road Runner-style
translation control rule within Apex's architecture, rather than importing Road Runner's full
trajectory follower into the robot application. Road Runner remains test-only.

## Command calculation

For target velocity `v`, signed target acceleration `a`, and measured velocity `vm`:

```text
feedbackPower = velocityFeedbackGain * (v - vm)
correctedVelocity = v + feedbackPower / kV
power = kS * sign(correctedVelocity) + kV * v + kA * a + feedbackPower
```

This is equivalent to Road Runner applying velocity feedback before its motor feedforward:
`kS * sign(correctedVelocity) + kV * correctedVelocity + kA * a`. Apex keeps the linear
feedforward and feedback contributions separate for its existing diagnostics and allocator.
The reference acceleration is preserved even when the robot is underspeed. In reverse tank
travel, the profile's translation velocity and acceleration are signed by travel direction.

At rest, Apex retains acceleration-directed static breakaway, matching its generator's power
model. If `kV` is zero, feedback remains in power units and static direction uses the original
motion target, avoiding division by zero. Angular control remains Apex's existing implementation.

During a slowdown, the negative `kA*a` contribution reduces holding power immediately. Depending
on speed and slope, the resulting motor command may remain positive or become opposing power.
Overspeed adds braking continuously; underspeed relaxes it continuously. There is no pulse
cooldown or switch into a separate coast controller.

## Tuner compatibility

The profiled-tracking phase keeps its course, two measured baselines, candidate comparison,
repeat validation, manual review, acceptance gates, and active-profile storage. Its editor and
optimizer now vary the active velocity feedback gain, profile velocity power, and corner speed
scale. The latter two already existed; their generator behavior has not changed.

Legacy pulse/coast fields remain readable and writable in `constants.json` for compatibility,
but they no longer control profiled execution. Existing robot calibration files are not rewritten.
Recheck profiled tracking after this controller change rather than relying on a review of the
previous pulse controller.

## Validation and graphs

- [315-run comparison against actual Road Runner](../TeamCode/build/profiled-braking-validation/index.html).
  Each braking limit uses a common RR-generated profile to isolate controller behavior. All
  acceleration checks pass; the new follower passes every braking target at magnitudes 60, 30,
  and 15 in/s² across both plants and all loop/sensor cases. At magnitude 120, two of the 18
  Apex cases narrowly miss the 3 in/s boundary-error target, so aggressive braking is not
  treated as universally reliable.
- [18 runs using actual Apex-generated profiles](../TeamCode/build/apex-generated-braking-validation/index.html).
  The straight course has three interior slowdowns. Both positive and negative acceleration
  limits are 30 in/s². Every acceleration and braking check passes. Tests check the generated
  velocity/acceleration ceilings and confirm that LUT identity and all sampled kinematic values
  remain unchanged after execution.
- Direct command probes verify targets below 6 in/s, signed acceleration at rest/underspeed/
  overspeed, and equality with actual Road Runner motor feedforward. A separate oracle test
  covers both velocity directions and static-sign changes under feedback.
- Existing profile convergence, LUT interpolation, motor allocation, drivetrain-profile
  serialization, tank curve/turn/reverse tracking, and manual tuner review regressions pass.

At 20 ms with 40 ms velocity lag, the new follower's braking RMS errors are:

| Braking magnitude (in/s²) | New Apex (in/s) | RR displacement (in/s) |
|---:|---:|---:|
| 120 | 1.848 | 1.886 |
| 90 | 1.669 | 1.669 |
| 60 | 1.246 | 1.248 |
| 30 | 0.665 | 0.668 |
| 15 | 0.338 | 0.338 |

The small differences reflect Apex's retained LUT interpolation and allocation. The actual
Apex-generated profile at 30 gives 0.654 in/s braking RMS under the same loop/lag condition.
These are nominal simulator results, not physical robot measurements. The earlier baseline
gallery in `build/braking-slope-sweep` remains historical evidence of the old controller.

Reproduce the focused controller experiments:

```powershell
.\gradlew.bat :TeamCode:testDebugUnitTest --tests follower.ProfiledMotionFeedforwardTest --tests org.firstinspires.ftc.teamcode.sim.BrakingSlopeSweepTest
```
