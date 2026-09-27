# Profiled braking and velocity reachability

Profiled paths use one braking response for all drivetrains. During ordinary
acceleration, the follower uses `kS + kV*v + kA*a` and velocity feedback. During
planned deceleration, it uses `kS + kV*v_target` plus braking velocity feedback.
If measured speed exceeds the target by more than the configured tolerance,
the follower sends a single opposing brake command and then waits for its
cooldown before another pulse. The brake request is applied before position
and heading commands are allocated to wheels.

During sustained deceleration, the positive drive command is capped at
`kS + kV*v_measured - brakingNeutralPowerMargin`. This lets the robot relax
braking as it slows without commanding a recovery acceleration between brake
pulses. The cap releases near a flat velocity target so the robot can continue
through a corner. Endpoint capture still handles the final approach to a stop.

The response is built into `HolonomicPathFollower` and `TankPathFollower`.
Mecanum and swerve use the holonomic path. Tank applies the same rule in its
signed forward direction. Tuning values remain available for robot-specific
calibration: `brakingVelocityFeedbackGain`, `brakingOverspeedToleranceIn`,
`brakingCornerToleranceIn`, `tankBrakingCornerToleranceIn`,
`brakingPulsePower`, `tankBrakingPulsePower`, `brakingPulseCooldownCycles`,
and `brakingNeutralPowerMargin`. Saved configurations with an obsolete
`brakingResponse` key are accepted; the key has no effect and is omitted when
the configuration is saved again.

## Profile speed limits

The profile generators for mecanum, swerve, and tank perform forward and
backward reachability passes against drivetrain power limits. At high speed,
the FTCodeSim drivetrain needs more power than the follower's original
`translationKV` implied. `profileVelocityPowerCoefficient` provides an
effective speed-dependent power cost for profile generation while leaving
the follower's feedforward calibration unchanged. This value captures the
combined simulated drivetrain behavior, including back EMF; it is not a
measured motor constant. Other robots can use their calibrated `translationKV`
unless a separate profile coefficient is supplied.

`profileCornerSpeedScale` smoothly lowers the local speed ceiling as
curvature increases. The backward pass carries the lower ceiling into the
approach to a corner. A tighter corner brake tolerance can favor a small
underspeed at entry.

The selected FTCodeSim mecanum settings are saved in
`TeamCode/src/test/resources/simulator/mecanum-alpha1.json`:
`profileVelocityPowerCoefficient = 0.014`,
`profileCornerSpeedScale = 0.80`, and
`brakingCornerToleranceIn = -1.5`. Tank uses a 0.10 opposing pulse and a
3 in/s corner tolerance; mecanum and swerve use a 0.30 pulse by default.
These power and tolerance values remain tunable for each drivetrain.

## Simulation checks

The selected AutoTest profile reduced matched-position braking RMSE from
4.78 to 2.59 in/s and curved braking bias from +3.38 to -1.56 in/s compared
with the earlier event brake profile. The new peak targets were reachable in
the simulator: about 58/56, 58/56, and 56/53 in/s target/actual, rather than
64.8/56.2, 64.8/55.2, and 63.1/52.7 in/s. The outbound path took about
10.7 seconds rather than 9.6 seconds because the prior target was not
reachable in the available distance.

The saved corner fixture checks cross-track error, heading error, braking
RMS, and below-target corner entry. AutoTest checks that raw speed does not
rebound sharply during sustained deceleration. Tank and dual-actuated
simulation tests exercise their follower paths, while profile generation
tests cover mecanum, swerve, and tank. These checks establish behavior in
FTCodeSim; drivetrain constants still require tuning for a different robot.
