# Apex SysId comparison with Road Runner

Reviewed October 5, 2026. This is a source comparison, not a hardware-performance
verdict. Simulator tracking errors do not establish that either identification
method is invalid. The original four-run Apex automatic workflow has been restored;
the experimental holding-speed runs and automatic profile refinement are removed.
The requested manual time-profile tuner and regression plots remain available.

## Reference implementations

- [Road Runner 1.0 FTC tuning code](https://raw.githubusercontent.com/acmerobotics/road-runner-ftc/3011c70e5a24526fd9f18be93dbaf2e6630d46fd/RoadRunner/src/main/java/com/acmerobotics/roadrunner/ftc/Tuning.kt)
- [Road Runner 1.0 ramp regression](https://raw.githubusercontent.com/acmerobotics/road-runner-ftc/3011c70e5a24526fd9f18be93dbaf2e6630d46fd/web/tuning/common.ts)
- [Road Runner motor feedforward](https://raw.githubusercontent.com/acmerobotics/road-runner/5cf18fb625dcb5b17ce8c47af16a80f47b0f6bb1/road-runner/core/src/main/kotlin/com/acmerobotics/roadrunner/Control.kt)
- [Legacy automatic tuner](https://raw.githubusercontent.com/acmerobotics/road-runner-quickstart/2f14a30cfbe287c492dd177a8ad4b4ceaa97b85a/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/drive/opmode/AutomaticFeedforwardTuner.java)
- [Legacy regression utility](https://raw.githubusercontent.com/acmerobotics/road-runner-quickstart/2f14a30cfbe287c492dd177a8ad4b4ceaa97b85a/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/RegressionUtil.java)
- [Current mecanum gain application](https://github.com/acmerobotics/road-runner-quickstart/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/MecanumDrive.java)

Version distinction matters: the legacy automatic tuner estimates kS/kV from a
slow ramp, then optionally estimates kA from constant-power acceleration with those
values fixed. The inspected 1.0 workflow supplies ramp regressions for kS/kV and a
manual feedforward profile tuner. It does not automatically estimate all three
gains with the same workflow as Apex.

## Differences to investigate first

| Area | Road Runner | Existing Apex implementation | Assessment |
| --- | --- | --- | --- |
| Identification velocity | Current ramp logger records encoder position/velocity; regression prepares those signals. | `FeedforwardTuner.measuredVelocity()` reads `Follower.getVelocity()`, which can be the joint Kalman estimate. | Confirmed measurement-path difference. Filtering can shift transient velocity/acceleration relative to delivered power. Compare native and filtered signals on the same recorded run before changing the estimator. |
| Sampling time | Current logger timestamps individual power, voltage and encoder acquisitions using `MidpointTimer`. | A phase timer timestamps the frame after the follower update; preceding delivered power and endpoint velocity are integrated in 100 ms windows. | Apex correctly associates power with the preceding interval. Remaining concern is sensor/filter delay and acquisition alignment, not a proven one-frame command shift. |
| Regression | Legacy kS/kV ramp regression is followed by a kA residual regression with kS/kV fixed. Current 1.0 ramp regression fits voltage versus velocity. | Four forward/reverse ramp and dynamic runs jointly fit sign(v), v and a. | A legitimate methodological difference. Joint identification is mathematically sound for the assumed model with sufficient excitation; departure from Road Runner does not itself prove an error. |
| Battery voltage | Current ramp fit uses power times measured voltage; execution divides feedforward voltage by current battery voltage. | Calibration and execution both use normalized motor power, with no voltage normalization in this path. | Confirmed portability gap across battery voltages. It cannot explain a mismatch in an ideal constant-voltage simulation by itself. |
| Units | Current quickstart stores wheel feedforward in encoder tick units, converts trajectory wheel velocity/acceleration, and uses voltage output. | Drive uses inches/s and inches/s²; heading uses radians/s and radians/s², with power output. | Different units are intentional. Copying Road Runner's numerical gains directly would be wrong. No degree/radian conversion mismatch or angular-velocity wrapping was found in the inspected path. |
| Where feedforward is applied | Current trajectory execution transforms a body command to wheel velocity/acceleration and evaluates motor feedforward per wheel. | Apex generates translation and heading feedforward separately, then combines them in drivetrain allocation. | A structural difference worth examining for combined translation/rotation. Pure single-axis SysId/manual tests do not establish mixed-motion behavior. |
| Manual reference | Current manual tuner builds a profile from a distance, then samples it by elapsed time and reverses the direction. | Requested manual tuner uses a fixed three-second time trapezoid and alternates direction on X. | Both use a time-indexed feedforward reference. Constructing a profile from a distance does not make it displacement-indexed. |
| Motor configuration | Current quickstart sets brake behavior and controls motor power directly. | Motor configuration defaults to brake and `RUN_WITHOUT_ENCODER`; drivetrain records limited delivered commands. | Defaults are compatible with direct characterization. Robot-specific motor configuration still needs checking on hardware. |

At a fixed reference voltage V and encoder scale s inches/tick, forward-only
equivalent power-domain gains are kS_Apex = kS_RR/V,
kV_Apex = kV_RR/(V*s), and kA_Apex = kA_RR/(V*s).
Heading gains also require the relevant wheel/heading kinematic conversion; the
forward conversion alone does not apply. No gains have been converted or replaced.

## Checks that did not reveal an obvious error

- The original regression uses the expected signed model:
  power = kS sign(v) + kV v + kA a. Columns are scaled before elimination.
  Existing exact-model tests recover all three coefficients in both directions.
- Acceleration windows derive from endpoint velocity change over the same interval
  whose velocity and delivered power are averaged. Dynamic windows opposing the
  run's direction are excluded, and each excitation direction requires coverage.
- The kS=0 boundary is refitted when an unconstrained negative intercept is found;
  the code does not merely clamp kS while leaving the old kV/kA unchanged.
- Gains are saved and applied in the same Apex units used during identification.
  Moving-friction feedforward kS and feedback breakaway power have separate fields.
  `setFeedforwardGains()` refreshes the turn controller as well as shared constants.
- For nonzero velocity, Road Runner's `MotorFeedforward` and Apex use the velocity
  direction for the static term. At exactly zero velocity Apex selects the
  acceleration direction; Road Runner's Kotlin `withSign` follows the velocity's
  signed zero. That is a launch convention difference rather than a cruise-error
  explanation.

These checks rule out specific source-level mistakes; they do not establish
tracking accuracy or sufficiently rich excitation for every real robot.

## Completed controlled comparison

The [deterministic matched-data experiment](SysIdMethodComparisonResults.md)
compared native/filtered sensing and joint/Road Runner-style sequential fits.
Changing those choices did not eliminate stock simulator overspeed. Independent
acceleration probes identified a speed-times-power term in the simulator's
powered back-EMF equation. Removing only its duty factor in a test-only control
allowed the existing native joint fit to track both axes accurately. Production
physics and the original automatic tuning method remain unchanged.

The sequential comparison replays the legacy regression mathematics on matched
Apex windows; it is not a full execution of Road Runner's acquisition workflow.
Filtered heading velocity still has a measurable secondary effect in the control.

The runtime path follower also applies feedback and allocation after generating
feedforward; that should be distinguished from the manual open-loop test when
comparing tracking with Road Runner's trajectory follower. The manual profile
alone does not measure the quality of the complete path-following controller.

No actual-robot data has been collected. Previous simulator evidence includes
harness timing/concurrency problems and should not be treated as a rejection of
the established identification methods.

## Follow-up: duty-dependent back-EMF compensation audit

The user's requested comparison is **Road Runner 1.0**. The current quickstart
declares core `1.0.1` and FTC `0.1.25`; the downloaded core source also declares
`libVersion = "1.0.1"`. The findings about current feedforward, manual tuning and
trajectory feedback below are specifically from that 1.0 implementation. Legacy
details are background only. The earlier matched-data sequential regression
experiment was legacy-inspired and must not be described as a Road Runner 1.0
workflow test.

Reviewed the complete official `road-runner`, `road-runner-ftc` and
`road-runner-quickstart` master source archives on October 5, 2026, plus core
v0.5.6 and quickstart revision `2f14a30cfbe287c492dd177a8ad4b4ceaa97b85a`.
Local copies are in `work/roadrunner-audit/`. Searched for duty/PWM/back-EMF
handling, and traced regression, manual profile power, trajectory power and
motor configuration rather than relying only on keyword absence.

No explicit correction for a velocity-times-duty term was found in these stock
feedforward paths. Current `MotorFeedforward.compute()` evaluates
`kS.withSign(vel) + kV * vel + kA * accel`; quickstart then divides the wheel
feedforward voltage by measured battery voltage before `setPower()`. Current
forward ramp regression fits commanded power times battery voltage versus wheel
velocity. None inverts a speed-dependent power coefficient or adjusts kV/kA
according to output duty. Legacy `Kinematics.calculateMotorFeedforward()` also
uses fixed gains, with a different static-term sign convention.

Important distinctions:

- The current `ManualFeedforwardTuner` directly applies feedforward divided by
  battery voltage; its tuning mode does not add trajectory feedback. The legacy
  manual tuner similarly applies feedforward with nominal/measured voltage
  compensation and warns when built-in motor velocity PID is enabled.
- Normal trajectory execution **does** add tracking feedback before per-wheel
  feedforward. This can reduce errors from an approximate model; successful path
  following does not establish perfect open-loop feedforward tracking.
- Legacy quickstart supports `RUN_USING_ENCODER` and Hub velocity PIDF, including
  voltage compensation of the F coefficient. That closed-loop motor mode can
  compensate disturbances, but it is not a duty-dependent back-EMF equation in
  Road Runner. The current inspected quickstart does not install that velocity
  PIDF path in its mecanum/tank constructors.
- Legacy `PIDFController` allows a user-supplied custom `kF` callback, whose
  default is zero. This extension point does not supply a stock correction.
- Empirical gain fitting/manual refinement can absorb some nonlinearity over a
  limited operating range; that is not exact compensation for the cross term.

This conclusion is about the inspected official implementations, not every fork,
custom controller or undocumented behavior inside REV firmware. It does not show
that Road Runner's full open-loop workflow succeeds on stock FTCodeSim; that
experiment has not been performed.
