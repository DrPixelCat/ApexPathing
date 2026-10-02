# Motion characterization

Follower Tuner runs **Motion Characterization** immediately after drive and heading
feedforward SysId, before PD tuning. It is automatic measurement, with A to start each
stationary, positioned run and A to save the final result. Stop cancels unsaved work.
Existing feedforward gains remain the fallback for older profiles and planned braking.

The new phase measures forward, reverse, both strafing directions on holonomic drives,
and both rotation directions. Tank profiles omit strafe. Leave 96 inches of travel plus
stopping room, or room for four full turns. Real robots must be repositioned between
runs; only the simulator staging helper can move the robot automatically.

## What is measured

Each direction has repeated acquisition runs at 0.65 and 1.0 normalized motor power.
Independent validation starts at 0.55 power to cover low speeds, then uses 0.95 power;
none of its measurements enter the fit. These levels scale to the drivetrain's configured
maximum motor power. The phase
records delivered power and measured robot-relative velocity, rather than a nominal
motor specification. Acceleration comes from a least-squares slope of measured velocity
over 150 ms windows. Timing gaps discard the affected window and its short recovery
interval. Validation repeats when too few usable windows survive.

The measured moving response is

`a = (gain0 + gainSlope * speed) * power - (drag0 + dragSlope * speed)`.

Consequently, `holdingPower(speed) = drag(speed) / gain(speed)` and
`kA(speed) = 1 / gain(speed)`. A conventional constant-gain motor is included by
`gainSlope = 0`. All coefficients come from measurements; no simulator coefficients
or FTC Code Sim equations are embedded in the tuner. This is a compact response model,
not a guarantee that every robot obeys that equation. Inadequate fits are rejected.

Acceptance requires identifiable acquisition data, at least 12 withheld windows,
high-speed validation coverage, at least two validation samples per speed band,
power RMS error <= 0.04 overall and <= 0.05 in each band, and peak error <= 0.10.
Validation evaluates the shared acquisition/validation speed range; samples beyond
that range cannot justify extrapolation or a higher speed ceiling.
Gain must remain positive and holding demand nonnegative, monotonic and below full
power over the accepted range. The speed ceiling is at most 95% of the smaller
acquisition/validation coverage; the planner does not extrapolate to unmeasured speeds.

Raw samples, fitted windows and accepted models are saved in the
`motion_characterization_*.csv` file beside the existing tuning logs. The accepted
models are stored under `motionCharacterization` in the active drivetrain profile.
Dual tank and holonomic profiles keep separate models. Reaccepting either feedforward
SysId invalidates the old characterization and schedules a new one in the normal flow.

## Planning and following

The existing forward/backward profile sweeps, kinematic constraints, heading chain
rule and LUT format remain. Their power evaluations now use the accepted speed-dependent
response. Translation, centripetal and heading requests share a conservative 0.88
fraction of the calibrated maximum motor power, leaving 12% for tracking corrections. Configured velocity,
acceleration, angular and traction limits continue to be hard caps.

The follower uses the same response for powered acceleration and cruise. Mecanum forward
and strafe feedforward are computed separately in the planned heading frame, matching
the profiler, and passed through the existing power allocator. Heading error is corrected
by feedback instead of adding an unplanned strafe holding-power request. Point turns use
the measured angular response. Existing displacement-based
braking, endpoint capture and signed braking feedforward remain; this phase does not
claim to identify reverse-power braking. The profiled-tracking phase continues to tune
velocity feedback and corner speed, but cannot override the measured power model with
the older arbitrary profile-kV adjustment.

Without a saved characterization, legacy constants keep their existing behavior.
The calibration describes the tested robot, load, floor and battery condition. Repeat it
after a meaningful change to those conditions; normalized duty is not universal voltage
compensation. Simulation validation uses the unchanged upstream stock motor/chassis.

The existing affine feedforward model follows the conventional
[SysId formulation](https://docs.wpilib.org/en/latest/docs/software/advanced-controls/controllers/feedforward.html).
The extra measured duty-speed interaction addresses the mismatch documented in
[the stock acceleration diagnosis](StockAccelerationDiagnosis.md).

## Stock simulator validation

The October 2, 2026 validation used the unchanged FTC Code Sim alpha.1 stock
mecanum motor/chassis and default project hardware parameters. It began with the
previously accepted drive/heading SysId constants, acquired all six directional maps,
ran both PD phases including X verification, limits, centripetal and profiled tracking,
then ran the actual Apex AutoTest. All subsequent phases saved successfully. SysId's
existing algorithm and quality gates were retained; this validation does not establish
fresh from-zero SysId acceptance.

| Measurement | Result |
|---|---:|
| Independent map prediction RMS, across six directions | 0.0047–0.0192 motor power |
| AutoTest, all five movements | 17.39 s |
| Outbound peak target / measured speed | 50.22 / 49.91 in/s |
| Outbound acceleration tracking RMS | 0.83 in/s |
| Outbound braking tracking RMS | 1.56 in/s |
| Moving-curve heading RMS / peak | 2.38 / 4.41 degrees |
| Moving-curve cross-track RMS / peak | 0.18 / 0.36 in |

The accepted forward map requests 0.731 holding power at 50 in/s, compared with
0.650 from the old affine fit. It also increases the required acceleration power
as speed rises. The new profile reaches near 50 in/s without assuming its low-speed
acceleration remains available there. These results combine the measured maps with
the subsequent controller/limit tuning and the 12% reserve; they are not a controlled
comparison changing just one coefficient.

The versioned simulator calibration includes these measured maps. Tests also cover
legacy fallback, per-profile serialization, rejection of unsupported fits, shared
translation/turn power budgets, lower drivetrain power caps, and profile limits.
Tank and swerve generator coverage is numerical; their physical characterization
still needs to be validated on the corresponding robot.

The recorded velocity graph is [AutoTest velocity tracking](../TeamCode/AutoTest_Velocity_Tracking.png).
Detailed CSVs and run logs are under
`TeamCode/build/motion-characterization-validation/withheld-workflow`.

An independent replay of the saved fixture completed in 17.48 s, with 0.80 in/s
acceleration RMS and 1.65 in/s braking RMS at a 21.55 ms average control loop.
A host-delayed run at 36.71 ms average failed the sustained-braking rebound check.
The check was kept unchanged. These results require a functioning control/update
cadence; the calibration does not make the follower immune to delayed measurements
or motor updates.
