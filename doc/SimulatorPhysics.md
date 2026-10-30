# Mecanum simulator physics and calibration contract

Verified October 29, 2026 against the **loaded** FTCodeSim `v0.1.0-alpha.1` runtime. This describes desktop simulation, not a physical robot calibration.

The simulator is an empirical acceleration model. It does not expose a mass/inertia-based force model here, so the coefficients below should not be interpreted as newtons or motor torque constants.

## Provenance and verification

The dependency is declared in `TeamCode/build.gradle`. Equations were read from the locally cached upstream sources for `SimulatedDrivetrain.registerMotor`, `SimMotor.update`, and `SimulatedMecanum.update`, then checked against the loaded Java classes. In particular, the mecanum override must be read rather than assuming the base drivetrain's friction gate applies.

Runtime artifact SHA-256:

```text
AF3B3EDC54AAF9B5312C1171E55C25D42ED7F0FFA92D2E6DE0236115A892731C
```

The audit's runtime location, fitted values and numeric results are recorded in `TeamCode/build/test-results/simulatorPlantAudit/`. Local inspection copies of upstream source are in `tmp/simulator-plant-audit/upstream/`; they are not compiled or substituted for the dependency.

No simulator equations, registered motor coefficients, production controllers, or saved calibration values were changed for this verification.

## Units and measured coefficients

Chassis linear velocity/acceleration use inches/second and inches/second squared. Chassis angular quantities use radians/second and radians/second squared. `SimMotor` itself stores wheel angular quantities; multiply by wheel radius to obtain wheel-rim linear quantities. Duty is normalized motor power.

| Quantity | Verified value | Meaning |
| --- | ---: | --- |
| Supply voltage | 13 V | Registered voltage sensor for the verified setup |
| Wheel radius | 1.889765 in | Converts motor angular motion to wheel-rim linear motion |
| Powered wheel gain G | 183 in/s² per unit duty | Motor coefficient 0 × measured voltage × wheel radius |
| Duty-dependent speed loss B | 2.307692308 s⁻¹ | Motor coefficient 1; equals 150/65 for this configuration |
| Opposing-power braking D | 3.920001 s⁻¹ | Registered regenerative coefficient |
| Zero-power BRAKE loss D₀ | 3.920001 s⁻¹ | Registered neutral-brake coefficient |
| Forward chassis drag | 49 in/s² | Separate from the motor's wheel acceleration |
| Strafe chassis drag | 85 in/s² | Separate direction-dependent chassis drag |
| Angular chassis drag | 1 rad/s² | Separate chassis angular deceleration |
| Effective turn radius R | 9.251970 in | (wheelbase + track width)/2 |

The model is voltage dependent through G; the numeric values are specific to this supply and configuration. Tests of the equations use the actual registered voltage and coefficients.

## Motor response is piecewise

Let `w` be signed wheel-rim speed, `u` the effective signed duty after clipping/deadband, and `a_w` wheel-rim acceleration. The registered motor Coulomb coefficient is zero. The complete powered/neutral contract is:

| Regime | Wheel acceleration |
| --- | --- |
| Nonzero duty, same sign as wheel speed | `a_w = G*u - B*w*abs(u)` |
| Nonzero duty opposing wheel speed | `a_w = G*u - D*w` |
| Zero duty, BRAKE | `a_w = -D₀*w` |
| Zero duty, FLOAT | `a_w = 0` |
| Wheel at rest, nonzero duty | `a_w = G*u` |

`SimMotor.setPower` clips duty to [-1,1] and sets it to zero when its magnitude is **below 1e-5**. BRAKE/FLOAT and the exact neutral duty matter; the near-zero powered branch cannot be substituted for the zero-power branch.

For example, at +20 in/s:

- `u=+0.001` produces approximately **+0.136846 in/s²** at the wheel.
- `u=0` in BRAKE produces **−78.400020 in/s²**.
- `u=−0.001` produces **−78.583020 in/s²**.

The braking discontinuity depends on speed. A universal assertion that zero power adds more than 100 in/s² braking is false for this plant.

An unloaded FLOAT motor can coast with zero duty. A moving robot still needs power to overcome chassis drag. Conflating those two experiments produces invalid holding-power and kS expectations.

## Chassis response

For wheel accelerations in FL, FR, BL, BR order:

```text
ax    = (aFL + aFR + aBL + aBR) / 4
ay    = (-aFL + aFR + aBL - aBR) / 4
alpha = (-aFL + aFR - aBL + aBR) / (4*R)
```

The mecanum update then subtracts drag along current robot-frame translation. At direction theta, the drag magnitude is:

```text
drag(theta) = 1 / (abs(cos(theta))/49 + abs(sin(theta))/85)
```

It separately subtracts `1*sign(omega)` from angular acceleration. It rotates chassis acceleration into the field frame, integrates velocity and then position using explicit Euler, applies field-boundary handling, and writes chassis-implied wheel velocities back into the motor state.

The upstream stationary gate compares commanded linear acceleration with directional natural deceleration, and commanded angular acceleration with angular natural deceleration, while checking that both velocities are near zero. **`config.staticFriction=55` is not used by this mecanum override's stationary gate.** At exactly zero translation, `atan2(0,0)` chooses the forward drag for that gate, including an attempted strafe. After motion begins, drag follows the actual velocity direction.

For reliable breakaway checks, distinguish the initial stationary gate from the power needed for sustained movement. The corrected regression checks both directions on all three axes and chooses a below-gate command and an above-sustained-drag command.

## Implications for feedforward

For positive forward motion and positive duty, the chassis equation is:

```text
a = (183 - 2.307692308*v)*u - 49
u = (a + 49) / (183 - 2.307692308*v)
```

For strafe, replace 49 with 85. For pure positive rotation, replace the powered gain with `183/R`, the speed with omega, and the drag with 1.

Consequences:

- Forward startup power is about **49/183 = 0.26776**. Expecting the chassis to move at 0.08 forward power is inconsistent with the plant.
- The sustained strafe threshold extrapolates to **85/183 = 0.46448**; the initial rest gate has the separate behavior described above.
- Holding power and acceleration authority vary with speed. The local forward acceleration power coefficient is `1/(183-B*v)`, not a constant kA.
- An affine `kS + kV*v + kA*a` fit may approximate a limited speed range. Its fitted kS is not an independently imposed physical Coulomb-friction term.
- `maxVelocity=65` is a coefficient-generation input, not the complete chassis's forward steady speed. Under the forward powered equation at duty 1, zero acceleration occurs at approximately **58.07 in/s**. The analogous strafe value is **42.47 in/s**.
- Braking must select the appropriate powered, regenerative, or neutral regime. A single affine fit cannot represent the neutral discontinuity exactly.
- Combined translation/rotation requires wheel-space inversion and chassis drag; independent axis fits do not prove correctness under combined motion.

The saved `mecanumWheelModel` agrees with the loaded coefficients and independently measured coupled model. This result does **not** claim that every saved scalar axis fit is exact, that the follower always selects the right model, or that the complete tuning workflow succeeds.

## Corrected diagnostic expectations

Four previously failing tests were corrected without relaxing their old tolerances to hide errors:

1. **MecanumFrictionModelTest:** checks all three signed acceleration regimes against the registered coefficients, replacing the fixed >100 in/s² discontinuity assumption.
2. **ReleaseDrivetrainPhysicsTest:** checks commands below the real stationary gate and above sustained-drag thresholds instead of assuming 0.05 sticks and 0.08 moves on every axis.
3. **FeedforwardModelDiagnosticTest:** retains rejection of the obsolete affine fit. Its positive control now inverts the actual duty-dependent powered equation at a held-out state; the old calculation assumed duty-independent back EMF.
4. **FeedforwardCalibrationTest:** identifies speed-dependent unloaded motor authority from observed nonzero acceleration/power pairs, then checks independent speeds, accelerations and both directions. It separates unloaded FLOAT coasting from loaded chassis holding. The former unasserted feedback sweep was replaced by asserted held-out probes; constant-kA fitting is no longer presented as the plant's exact inverse.

New **MecanumPlantPhysicsTest** coverage checks clipping/deadband, powered/opposing/neutral regimes, BRAKE/FLOAT behavior, chassis response on three axes in both directions, and consistency among the source equations, measured coupled coefficients, and saved wheel calibration. It does not replace any upstream physics.

## Verification results

**28 focused tests passed**, including the four previously failing diagnostics and existing stock-plant, coupled-motion, wheel-model and motion-characterization checks.

| Check | Result |
| --- | --- |
| Observation-fitted motor gain | 183.000000000 |
| Observation-fitted gain slope | −2.307692308 |
| Held-out unloaded motor acceleration residual RMS | Below 1e-6 in/s² |
| Independent no-feedback motor ramp velocity RMS | **0.068208268 in/s** |
| Coupled held-out acceleration components | **1,947** |
| Coupled acceleration residual peak | **2×10⁻¹¹**, in respective component units |
| Existing no-feedback coupled traversal linear velocity RMS | **0.146509 in/s** |
| Existing no-feedback coupled traversal angular velocity RMS | **0.002593 rad/s** |
| Existing no-feedback coupled traversal maximum lateral drift | **0.020647 in** |

Those residuals validate equations and isolated reference execution; they do not establish full follower/tuner acceptance.

Reproduce the focused suite from the repository root in PowerShell:

```powershell
.\gradlew.bat :TeamCode:testDebugUnitTest --console=plain `
  --init-script tmp/simulator-plant-audit.init.gradle `
  --tests '*MecanumPlantPhysicsTest' `
  --tests '*StockSimulatorPlantTest' `
  --tests '*MeasuredMecanumExperimentTest' `
  --tests '*MecanumFrictionModelTest' `
  --tests '*ReleaseDrivetrainPhysicsTest' `
  --tests '*FeedforwardModelDiagnosticTest' `
  --tests '*FeedforwardCalibrationTest' `
  --tests '*MecanumWheelModelTest' `
  --tests '*MotionCharacterizationTest'
```

The temporary init script only separates audit result directories and forces execution. It does not change dependency versions or physics. For ordinary reports without that script, the same class filters work with Gradle's default result locations.

## Remaining work indicated by this evidence

- The legacy `FeedforwardPhaseSimulationTest` expects mecanum kS values of 0.06 and 0.03 from the same unchanged stock hardware. Its `expectedKS` argument changes the assertion, not the plant. Those expectations must be redesigned around an actual defined acquisition/validation contract, rather than replacing them with a one-run observed fit.
- The old motor-only holding experiment cannot determine loaded chassis friction. Subsequent calibration checks should use complete chassis measurements when asserting loaded authority.
- Characterization acquisition failures, sample shortages, endpoint stalls and tuning UI failures remain separate investigations. This work does not establish their root causes or repair them.
- Keep neutral-braking discontinuities and combined wheel motion in any further feedforward comparison. Matching one fitted constant is insufficient evidence of physical correctness.

The earlier complete test failure report is preserved at `tmp/all-tests-report.md`; the full suite and long tuning workflows were not rerun for this focused change.
