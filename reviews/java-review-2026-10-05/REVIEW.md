# Java source review — 2026-10-05

This is the original review before repairs. See [REPAIRS.md](REPAIRS.md) for implemented fixes and validation; reproduce.ps1 now runs the corrected regression suite.

Reviewed the current working tree under `TeamCode/src/main/java`: **154 files**, including package documentation. Existing uncommitted edits are included. No production source was changed. Severity: P1 = incorrect actuator behavior or blocked execution; P2 = reproducible defect or materially misleading API; cleanup notes are explicitly recommendations, not demonstrated failures.

## Assessment

The code's main weakness is overlapping ownership. Geometry, path builders, live planning, stored trajectories, follower allocation, braking adjustments, and tuners each own pieces of the same policy. Public mutable settings then require caches and compatibility branches to synchronize by convention. Fix the behavioral defects first; simplify by giving each policy one owner. Avoid a sweeping framework rewrite.

The small numerical/value classes are mostly manageable. The largest maintenance costs are BaseProfileGenerator, FollowerConstants, HolonomicPathFollower, the calibration adapter switchboard, and the combined device wrappers/drivers. File size alone is not a defect; the problem is mixing persistence, runtime mutation, numerical computation, compatibility, and diagnostics.

## Ranked findings

### F01 [P1] Pinpoint offsets are sent in the wrong unit

[localizers/Pinpoint.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/localizers/Pinpoint.java:381)

`setOffsets` writes `getM()` into both pod-offset registers. The device expects millimeters, so a 100 mm offset becomes 0.1 mm. Rotation compensation is wrong for every nonzero offset. Write `getMm()` and verify register values with a nonzero-offset test. The zero-offset example configuration hides this defect. Source: [goBILDA user guide](https://www.gobilda.com/content/user_manuals/3110-0002-0001%20User%20Guide.pdf), which specifies millimeters for these offsets.

### F02 [P1] The maximum-power setter always selects full power

[drivetrains/BaseDrivetrainConstants.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/drivetrains/BaseDrivetrainConstants.java:33)

`Math.max(Math.min(0.0, maxPower), 1.0)` returns 1 for every finite input. `setMaxPower(.3)` therefore permits full output. Validate finiteness and use `Math.max(0, Math.min(1, value))`, or reject values outside the allowed range. Confirmed by compiling and executing the current source.

### F03 [P1] Stopping a swerve drivetrain leaves steering servos powered

[drivetrains/CoaxialSwerve.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/drivetrains/CoaxialSwerve.java:100)

Steering power is written to four CRServos, but this class inherits `BaseDrivetrain.stop()` (line 157), which zeros only drive motors. A stop after a steering correction leaves that steering command active. Override stop to zero all actuators; account for superclass-constructor calls before servo initialization if applicable.

### F04 [P1] Octoquad initialization can wait forever

[localizers/Octoquad.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/localizers/Octoquad.java:55)

The constructor waits until status is RUNNING with no deadline or terminal-fault handling. `FAULT_NO_IMU` can never satisfy that condition. A disconnected or failed device can strand OpMode initialization. Use a bounded initialization wait, recognize fault states, and preserve interruption when aborting.

### F05 [P1] A stalled path is reported as completed

[follower/HolonomicPathFollower.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/follower/HolonomicPathFollower.java:273)

Both holonomic and tank executors return the same `true` for arrival and watchdog expiry (TankPathFollower line 205). `Follower.update()` then calls stop and marks the movement ended. An autonomous sequence that advances on `!isBusy()` can continue after a collision far from its endpoint. Return an explicit execution outcome and expose failure separately from successful completion.

### F06 [P1] Pose holding sends field error as robot-relative drive

[follower/Follower.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/follower/Follower.java:253)

The holonomic hold controller computes a field-coordinate error, then passes its X/Y directly to the three-argument `drive` overload, which uses heading zero. At a 90-degree robot heading, holding a point along field X drives along field Y. Rotate the error/command by the current heading and send a body command. The tank branch also projects using the target heading rather than the current heading.

### F07 [P1] Paused movement can still drive when hold is requested

[follower/Follower.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/follower/Follower.java:211)

The idle/paused branch calls `holdPose(lastPose)` whenever `update(true)` is used. A paused path therefore commands motion toward its endpoint. Treat paused execution as its own state that keeps output zero; apply idle holding only when its documented policy allows it.

### F08 [P2] Mirrored arc creation crashes

[geometry/GeometryFactory.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/geometry/GeometryFactory.java:100)

The cast is justified by the comment 'Will always return an ArcPose', but `Pose.mirrorX()` and `mirrorY()` construct ordinary Pose objects. Both mirrored arc modes throw ClassCastException. Mirror the ArcPose explicitly while preserving its radius, or implement covariant ArcPose mirroring. Confirmed by execution.

### F09 [P2] Completion silently loosens user tolerances and allows unsettled arrival

[follower/PathCompletionPolicy.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/follower/PathCompletionPolicy.java:13)

Distance and heading tolerances are widened to at least 1.5 inches and 2 degrees (lines 44–50), overriding defaults of .5 inches and 1 degree. A holonomic movement inside tolerance also completes after .10 seconds even if velocity remains unsettled; tank execution explicitly requires settling. Make acceptance use the configured limits and define one consistent settling rule. Both behaviors reproduced directly.

### F10 [P2] Controller-disable flags are ignored in tank paths and partially in turns

[follower/Follower.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/follower/Follower.java:223)

Holonomic execution receives both enable flags. Tank execution receives neither, and turn execution receives only the drive flag. Calling the public disable methods consequently does not have consistent effect across movement types. Pass a common controller-enable policy to all executors, or explicitly narrow and name the API.

### F11 [P2] Callbacks belong to the path object instead of its execution

[follower/MovementCallbackRunner.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/follower/MovementCallbackRunner.java:15)

Trigger state lives on Callback. `reset()` clears only the last heading, and starting a movement never resets callback flags. Following the same Path or Turn twice skips previously triggered callbacks. Keep fired-callback state in the execution context, or reset it explicitly at follow start. Also avoid sharing mutable callback instances between traversals.

### F12 [P2] Two-point rest-to-rest profiles have zero duration

[trajectory/profile/DynamicMotionProfile.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/trajectory/profile/DynamicMotionProfile.java:235)

The public factory accepts two samples. Both endpoint velocities are zero, and the time integration adds nothing when their sum is zero, even across positive distance. A 24-unit profile returns duration 0. Reject an impossible envelope or insert an interior acceleration/deceleration state; never silently turn travel into instantaneous movement. Confirmed by execution.

### F13 [P2] Characterized turn braking ignores the characterized power model

[trajectory/profile/DynamicMotionProfile.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/trajectory/profile/DynamicMotionProfile.java:132)

The measured-power overload consults the supplied model for cruising and forward reachability, but backward reachability always uses the legacy affine kV/kA/kS equation. TurnProfileGenerator supplies the measured angular model at lines 96–101, so its braking envelope can disagree with calibrated motor authority. A direct reproduction with measured power = .2 × absolute acceleration permits braking power 2 against budget 1. Apply the same measured model and power bound to the backward sweep; validate with asymmetric forward/braking coefficients.

### F14 [P2] Missing facing-point parameters cause NPE instead of fallback

[paths/builders/HolonomicPathBuilder.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/paths/builders/HolonomicPathBuilder.java:300)

Selecting FACING_POINT without a facing point dereferences `facingPoint` during the missing-parameter check. The advertised warning/fallback is never reached. Check null before finiteness, or reject the incomplete builder configuration with a clear IllegalArgumentException.

### F15 [P2] Three-wheel geometry can divide by its default zero baseline

[localizers/ThreeWheel.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/localizers/ThreeWheel.java:48)

Offsets default to Vector.zero(), and heading delta divides by `forwardOffsetIn`. Building with pod names but no baseline produces NaN/Infinity on updates. Validate a finite, physically meaningful nonzero baseline at build time. Add equivalent validation for encoder scales in ThreeWheel, TwoWheel, MecanumDriveEncoders, and OdometryPod rather than relying on saved-calibration parsing.

### F16 [P2] Pinpoint hides rejected samples as fresh valid localization

[localizers/Pinpoint.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/localizers/Pinpoint.java:328)

The driver returns old coordinates/velocity for rejected readings but the outer localizer still processes them as an ordinary new update. Status from bytes 0–3 is not consumed. A fixed velocity cutoff of 1000 mm/s rejects valid motion above about 39.37 in/s. Carry readiness, sample validity, and timestamp through the driver boundary; make physical limits configurable instead of silently freezing readings.

### F17 [P2] Octoquad ignores its own pose-validity predicate

[localizers/Octoquad.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/localizers/Octoquad.java:68)

The wrapper accepts `crcOk` alone, while LocalizerDataBlock.isPoseDataValid() at line 1064 also requires RUNNING status. Correct CRC does not make a fault-state pose usable. Use that predicate and mark invalid samples without feeding them to the estimator.

### F18 [P2] Localizer replacement destroys the usable state before construction succeeds

[localizers/LocalizerSet.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/localizers/LocalizerSet.java:89)

`select()` changes activeProfile before building its new localizer; `rebuildActive()` removes the old instance before building the replacement. Hardware/configuration exceptions leave the set pointing at no localizer. Construct and validate the replacement first, then swap the active state atomically.

### F19 [P2] Vector equality violates the intended value contract

[geometry/Vector.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/geometry/Vector.java:145)

Vector.equals delegates to Dist.equals, which is Object identity. Independently constructed equal coordinates and a vector's copy compare unequal. Vector also overrides equals without hashCode. Define consistent value equality/hashCode across the small value types, or remove this misleading equality method and provide a named approximate comparison. Confirmed by execution.

### F20 [P2] Mecanum limit caches are stale after live limit changes

[trajectory/generators/MecanumProfileGenerator.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/trajectory/generators/MecanumProfileGenerator.java:16)

The directional limit calculator is final and constructed once. The base generator can notice changed planning constants, but regenerating samples still uses the old calculator's velocity/acceleration limits. Either snapshot immutable planning settings for the generator's lifetime or rebuild all derived limit data together when a revision changes.

### F21 [P2] Spline endpoint evaluation deliberately misses the endpoint

[geometry/BSpline.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/geometry/BSpline.java:95)

Clamping t >= 1 to .999999 avoids indexing past the segment array, but changes the geometric endpoint. Its error grows with curve scale and segment count and disagrees with exact endpoint bounds. Choose the final segment with localT = 1; clamp the index without perturbing the parameter.

### F22 [P2] Manual drive tuning uses field X despite front/back instructions

[tuning/follower/feedback/DrivePhase.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/tuning/follower/feedback/DrivePhase.java:55)

The test endpoint adds `(target, 0)` in field coordinates and metrics read field X. With a robot heading other than zero, that is not forward/backward movement. Rotate the requested displacement by the starting heading and project measurements onto that same fixed axis.

### F23 [P2] Manual tuning response logs are not closed on early exit

[tuning/follower/feedback/DrivePhase.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/tuning/follower/feedback/DrivePhase.java:46)

DrivePhase and HeadingPhase own ManualResponseMetrics, which owns an open CSV until finish(). Neither phase overrides onExit to finish it when Stop or an exception interrupts a trial. TuningPhase already provides a finally/onExit hook: use it to close logs and restore controller state.

### F24 [P2] Settling metrics remember a transient pass as final settling

[tuning/follower/feedback/ManualResponseMetrics.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/tuning/follower/feedback/ManualResponseMetrics.java:74)

Once settlingTime becomes finite, leaving the tolerance band does not clear it. A response that briefly settles and then oscillates can still report the earlier value. Track the start of the final sustained tolerance interval and invalidate it whenever the response exits. Saturation should also be measured against the active power limit, not a fixed near-1 threshold.

### F25 [P2] Constructing a limits phase changes drivetrain output policy

[tuning/follower/limits/LimitsPhase.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/tuning/follower/limits/LimitsPhase.java:94)

The constructor writes maxPower = 1, before the trial starts, and never restores the previous setting. Merely selecting the phase changes manual-driving power during its mode menu. Save the original limit, change it at trial initialization, and restore it in onExit.

### F26 [P2] Covariance validation accepts impossible covariance matrices

[localizers/util/KalmanFilter.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/localizers/util/KalmanFilter.java:128)

Symmetry and nonnegative diagonal entries are insufficient for positive semidefiniteness. For example [[1,2],[2,1]] passes these checks but has a negative eigenvalue. A direct reproduction accepts it in reset, then fails the innovation-variance calculation for observation [1,-1]. Validate PSD with an appropriate small-matrix factorization, or restrict the public API to independently nonnegative diagonal noise and reject unsupported full covariances.

## Simplifications that would pay for themselves

1. **Use immutable settings at execution boundaries.** Build a validated planning snapshot once. Stop having paths fetch the global FollowerConstants singleton and profile generators reconstruct only selected caches. Store traversal state separately from Path/Turn definitions.
2. **Make frame and unit contracts explicit.** Use field/body command names and conversion at one boundary. Dist should not represent squared distance, encoder counts per inch, or dimensionless motor commands. Primitive doubles with precise names are often clearer than another generic unit hierarchy.
3. **Give the physical model one implementation.** Planner feasibility and follower command allocation should evaluate the same motor-power model. Name and measure braking/pulse parameters; remove legacy options after callers migrate instead of keeping silent no-op arguments.
4. **Use one numerical helper where the math is actually identical.** Several regressions carry their own normal-equation elimination. A small scaled solver with a documented conditioning/failure contract is enough; no dependency-injection framework is needed. Keep genuinely different estimators separate.
5. **Move device protocol code out of runtime wrappers.** Preserve vendor attribution and isolate register units/status decoding from geometry and filtering. A driver reading must carry validity, readiness, and sample time. Returning yesterday's value as today's measurement is misleading.
6. **Keep diagnostics off the hot path.** Avoid creating a FeedbackController merely to multiply a gain by a value (Follower lines 618 and 651). Reduce short-lived controller Calculation/EnumSet and matrix/pose allocations only after measuring loop allocations and timing. Do not guess at micro-optimizations.
7. **Use lifecycle hooks consistently.** TuningPhase already has finally/onExit. Extend that disciplined cleanup to manual metrics and OpMode CSV ownership. Restore temporary calibration settings on abort and distinguish 'trial finished' from 'trial passed'.
8. **Standardize surface-level Java style last.** Expand compressed multi-statement control flow, use private fields/accessors for invariants, trim region/author boilerplate from tiny classes, and organize imports consistently. Avoid gratuitous renaming or abstractions for simple enums/interfaces.

## File-by-file review

Every file appears below. 'No actionable issue' means this source inspection found no specific defect; it does not establish runtime correctness. Recommendations describe concrete cleanup opportunities and are lower priority than the ranked findings. Links open the reviewed source; finding IDs refer to the analysis above.


### config

| File | Review notes |
| --- | --- |
| [ApexConstants.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/config/ApexConstants.java:1) | No actionable issue. Small configuration seam; document whether returned constants are new instances or shared mutable objects. |

### controllers

| File | Review notes |
| --- | --- |
| [DriveController.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/controllers/DriveController.java:1) | Keep frame contracts explicit. Direction-dependent scaling, endpoint correction, and generic feedback composition deserve clear method names rather than stateful mode switches. |
| [FeedbackController.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/controllers/FeedbackController.java:1) | The fluent Calculation/EnumSet API is heavier than a scalar control equation needs. Preserve the explicit error-rate and clock support; profile allocations before changing it. |
| [PDSController.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/controllers/PDSController.java:1) | Compatibility facade adds another controller vocabulary. Identify active callers, migrate to one controller API, then deprecate/remove the duplicate surface. |
| [TurnController.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/controllers/TurnController.java:1) | Constructor/setter accept an ignored angular velocity gain; profiled methods accept an unused intendedDirection. Silent compatibility parameters should be removed or explicitly deprecated. |

### drivetrains

| File | Review notes |
| --- | --- |
| [BaseDrivetrain.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/drivetrains/BaseDrivetrain.java:1) | F03/F06. Separate body commands from joystick field-centric transforms. Validate finite output/configuration at the hardware boundary; central motor normalization is useful. |
| [BaseDrivetrainConstants.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/drivetrains/BaseDrivetrainConstants.java:1) | F02. Public fields bypass setters and validation. Validate the complete configuration during build; reject invalid tolerances and non-finite power. |
| [CoaxialSwerve.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/drivetrains/CoaxialSwerve.java:1) | F03. Update steering controllers every tick, not only when floating-point error changes: an unchanged error still needs derivative decay. Factor repeated module math into a small module helper; use Math.hypot. |
| [DualActuated.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/drivetrains/DualActuated.java:1) | Transition gating is useful. A fixed duration should be a documented mechanism assumption; update active mode/localizer/controller state together when a transition finishes. |
| [Kiwi.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/drivetrains/Kiwi.java:1) | No specific execution defect found. Three-motor mapping should have an explicit wheel orientation/sign diagram; reuse central normalization. |
| [Mecanum.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/drivetrains/Mecanum.java:1) | DirectionalLut caches configured limits; F20 applies to its consumers. Keep the directional limit model and command kinematics consistent and document zero-vector behavior. |
| [Motor.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/drivetrains/Motor.java:1) | Configuration and hardware binding share one mutable object. Prefer a fresh runtime binding per build so reused configuration cannot silently rebind an earlier drive. |
| [Tank.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/drivetrains/Tank.java:1) | No actionable issue in its simple mixing. Clarify ignored lateral input at the body-command API boundary. |

### feedforward

| File | Review notes |
| --- | --- |
| [CenteredBrakingFeedforward.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/feedforward/CenteredBrakingFeedforward.java:1) | Cycle-count pulse scheduling and hardcoded lead/duty thresholds obscure a physical model. Name the assumptions and use elapsed time if behavior must survive loop jitter. |
| [HolonomicFeedforward.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/feedforward/HolonomicFeedforward.java:1) | Motion-sign rules overlap MotionFeedforward. Share a small pure sign/feedforward function rather than duplicate corner cases. |
| [MecanumWheelModel.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/feedforward/MecanumWheelModel.java:1) | Long primitive constructor and indexed arrays are easy to misuse. Return an explicit out-of-domain result from bounded solving; solveBounded returning 1 for an invalid demand is a misleading success-looking contract. |
| [MotionCharacterization.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/feedforward/MotionCharacterization.java:1) | Validate array dimensions and null entries before indexing. Make direction/axis pairing explicit and keep calibrated domains separate from drivetrain-type guesses. |
| [MotorPowerModel.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/feedforward/MotorPowerModel.java:1) | Compact, validated model is a good direction. Make extrapolation outside calibrated speed/acceleration bounds explicit at callers. |

### follower

| File | Review notes |
| --- | --- |
| [Follower.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/follower/Follower.java:1) | F05–F07/F10/F11. Too many compatibility helpers and public diagnostic getters. Keep lifecycle, execution outcomes, frame conversion, and pure feedforward math separate. |
| [FollowerConstants.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/follower/FollowerConstants.java:1) | Global mutable singleton mixes persistence, profiles, derived gains, and runtime policy. Missing/malformed numeric values silently become zero. Load a validated snapshot and report corrupt/incomplete configuration explicitly. |
| [FollowerDiagnostics.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/follower/FollowerDiagnostics.java:1) | No specific defect found in the small hook interface. Document callback cost and exception behavior because hooks run during control updates. |
| [FollowerPathState.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/follower/FollowerPathState.java:1) | Reset does not clear every public diagnostic field. Define which fields are valid while idle and clear stale demand/progress/error values at traversal boundaries. |
| [FollowerStuckWatchdog.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/follower/FollowerStuckWatchdog.java:1) | Small helper is reasonable. F05 concerns its callers: a watchdog trip is a failure outcome, not endpoint success. |
| [HolonomicCommandAllocator.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/follower/HolonomicCommandAllocator.java:1) | Allocation policy is spread across this class, wheel-model special cases, and drivetrain normalization. Prefer one command/result contract with explicit frame and capacity semantics. |
| [HolonomicPathFollower.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/follower/HolonomicPathFollower.java:1) | F05/F09. Multiple feedforward, independent-reference, wheel-model, and pulse branches make behavior hard to trace. Extract model evaluation from lifecycle/control policy. |
| [MotionFeedforward.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/follower/MotionFeedforward.java:1) | Share sign/deadband conventions with HolonomicFeedforward. Keep this helper pure; avoid exposing unused legacy gain arguments through callers. |
| [MovementCallbackRunner.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/follower/MovementCallbackRunner.java:1) | F11. Put fired state in the execution context. Decide whether callback exceptions abort execution and whether reentrant callbacks are supported. |
| [PathCompletionPolicy.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/follower/PathCompletionPolicy.java:1) | F09. Hardcoded tolerance floors and velocity thresholds conflict with configuration. One small validated completion policy should cover tank and holonomic behavior. |
| [PathProgressEstimator.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/follower/PathProgressEstimator.java:1) | No specific defect found. Document how local projection behaves at self-intersections and with backward displacement. |
| [TankPathFollower.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/follower/TankPathFollower.java:1) | F05/F10. Scalar feedforward controllers can become direct equations. Keep Ramsete/capture blending and output budgeting readable and test their transition boundaries. |
| [TurnExecutor.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/follower/TurnExecutor.java:1) | F10. Share controller-enable semantics and completion outcomes with path execution. Inject the same clock into all nested controllers for deterministic timing. |
| [VelocityReference.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/follower/VelocityReference.java:1) | State conditioning and preview rules are nontrivial. Document the anti-windup invariant and the distinction between target velocity and allocated feasible velocity. |

### geometry

| File | Review notes |
| --- | --- |
| [Angle.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/geometry/Angle.java:1) | No specific arithmetic defect found. Define exact versus approximate equality and normalization semantics consistently with other value types. |
| [AngleUnit.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/geometry/AngleUnit.java:1) | No actionable issue. A small unit enum is appropriate. |
| [ArcPose.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/geometry/ArcPose.java:1) | F08. Inheriting Pose loses radius metadata in inherited copy/mirror operations. Preserve subtype semantics or represent arc waypoints by composition. |
| [BSpline.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/geometry/BSpline.java:1) | F21. Mutable last-t cache complicates an otherwise pure evaluator. Endpoint handling should select a valid segment without moving the requested point. |
| [CubicSpline1D.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/geometry/CubicSpline1D.java:1) | No specific defect found in the spline solver. Document knot-spacing/degenerate-input assumptions at construction. |
| [Dist.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/geometry/Dist.java:1) | F19. Object identity equality leaks into Vector. Dist.times(Dist) and div(Dist) preserve distance labels for squared/dimensionless results; use clearly named scalar math instead. |
| [DistUnit.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/geometry/DistUnit.java:1) | No actionable issue. Keep unit conversions centralized here. |
| [GeometryFactory.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/geometry/GeometryFactory.java:1) | F08. Follower constructor argument is ignored; remove/deprecate the misleading dependency. Validate null units/mirror values rather than silently treating null as another mirror mode. |
| [Matrix.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/geometry/Matrix.java:1) | Validate nonempty rectangular input before data[0] access. inverse needs a singular/near-singular failure contract; returning non-finite results is hard to diagnose downstream. |
| [ParametricSegment.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/geometry/ParametricSegment.java:1) | Sampled default bounds are approximate, not guaranteed containment. State that limitation; spatial grids need an explicit conservative-bound contract. |
| [PathPoint.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/geometry/PathPoint.java:1) | No specific defect found. Cache immutability matters because points are exposed through lookup tables. |
| [PathSegment.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/geometry/PathSegment.java:1) | getPointLUT exposes the internal array. A caller can replace samples and invalidate projection/bounds assumptions. Return a copy or read-only sampling access. |
| [Pose.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/geometry/Pose.java:1) | F08. Pose is also used as velocity/acceleration, so heading is sometimes radians and sometimes radians/sec. Name rate-bearing data clearly; copy need not duplicate immutable components. |
| [Scalar.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/geometry/Scalar.java:1) | Thin immutable double wrapper adds allocations without enforcing much. Consider removing it beneath Dist; avoid replacing it with a broader abstraction. |
| [Vector.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/geometry/Vector.java:1) | F19. dot/cross/magnitude-squared return Dist despite differing dimensions. Exact equality and approximate comparison should be separate, intentional APIs. |

### localizers

| File | Review notes |
| --- | --- |
| [BaseLocalizer.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/localizers/BaseLocalizer.java:1) | Filtering, derivative estimation, validity, pose state, and compatibility live together. Validate setFilterWindow consistently; define estimator reset behavior when switching filter mode. |
| [BaseLocalizerConstants.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/localizers/BaseLocalizerConstants.java:1) | Small interface is reasonable. Treat saved-geometry application as validated configuration input, not permission to bypass build invariants. |
| [CalibrationJson.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/localizers/CalibrationJson.java:1) | No specific defect found. Useful centralized validation; use it consistently in live setters/build methods as well as persistence. |
| [LocalizationConstants.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/localizers/LocalizationConstants.java:1) | Persistence and runtime application have different exception policies. Report invalid records and apply calibration transactionally; type-name compatibility alone does not identify a hardware setup. |
| [LocalizerSet.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/localizers/LocalizerSet.java:1) | F18. Build replacements before changing active state. Shared-localizer switches should not reset estimator history unless needed. |
| [MecanumDriveEncoders.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/localizers/MecanumDriveEncoders.java:1) | F15. Validate encoder scale and IMU initialization. Drive-wheel odometry assumptions/slip limits should be explicit; forward/strafe calibration currently shares one scale. |
| [OTOS.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/localizers/OTOS.java:1) | Scalar register rounding for negative adjustments is biased by adding .5 then truncating. Validate finite values, handle calibration timeout/interruption explicitly, and expose sensor readiness. |
| [Octoquad.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/localizers/Octoquad.java:1) | F04/F17. Large embedded driver obscures wrapper responsibilities. Separate protocol/status handling from pose integration and keep vendor attribution. |
| [OdometryPod.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/localizers/OdometryPod.java:1) | F15. Require a finite positive encoder scale; avoid accepting zero and generating non-finite deltas later. |
| [Pinpoint.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/localizers/Pinpoint.java:1) | F01/F16. Unit conversions and data validity belong at the driver boundary. Hardcoded corruption thresholds should not silently censor legitimate motion. |
| [TankDriveEncoders.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/localizers/TankDriveEncoders.java:1) | Its stronger geometry validation is a useful standard for the other encoders. Calibration should retain distinct left/right scales when measured hardware differs. |
| [ThreeWheel.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/localizers/ThreeWheel.java:1) | F15. Validate baseline/scale. Calibration must distinguish shared encoder scale from physically different pod resolutions. |
| [TwoWheel.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/localizers/TwoWheel.java:1) | F15. Validate scale/offsets and initialization result. Configured IMU identity should be available to calibration phases. |

### localizers/util

| File | Review notes |
| --- | --- |
| [AdaptiveKalmanFilter.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/localizers/util/AdaptiveKalmanFilter.java:1) | Specialized estimator is substantial; distinguish legacy derivative filtering from the joint pose estimator and migrate unused modes after measuring behavior. |
| [DataFilter.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/localizers/util/DataFilter.java:1) | No actionable issue. Small interface; specify timestamp units and invalid-sample behavior. |
| [FilterState.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/localizers/util/FilterState.java:1) | No actionable issue. Small state object; document whether values are filtered positions or derivative estimates. |
| [KalmanFilter.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/localizers/util/KalmanFilter.java:1) | F26. Small matrix math still allocates many temporary arrays; profile before optimizing and keep covariance failure diagnostics explicit. |
| [LowPassFilter.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/localizers/util/LowPassFilter.java:1) | This is a boxcar/moving-average filter, not a tunable frequency-domain low-pass model. Reject non-finite samples: NaN entering the rolling sum can poison later outputs. |
| [PoseEstimator.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/localizers/util/PoseEstimator.java:1) | Keep joint-state covariance and wrapped heading rules explicit. Share only genuinely identical matrix helpers with KalmanFilter; avoid parallel undocumented numerical conventions. |

### org/firstinspires/ftc/teamcode/apexpathing

| File | Review notes |
| --- | --- |
| [AutoTest.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/apexpathing/AutoTest.java:1) | 536+ lines mix route creation, test acceptance, CSV serialization, and saturation diagnostics. Reuse TuningCsvWriter, close in finally, and replace the Thread.yield deadline loop with a documented pacing helper. |
| [Constants.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/apexpathing/Constants.java:1) | Zero pod offsets are placeholders that hide F01. Require actual robot geometry for production examples and avoid implying this default is calibrated. |
| [ExampleAutoPath.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/apexpathing/ExampleAutoPath.java:1) | F08 breaks mirrored construction. Path definitions and callback status are all public mutable fields; keep example observability but encapsulate traversal state. |
| [FollowerTuner.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/apexpathing/FollowerTuner.java:1) | Completion predicates using != 0 accept negative/NaN values. Phase.tuned is mutable global enum state; keep status per run. Remove always-true phaseAvailable/menu lock remnants. |
| [LocalizationTuner.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/apexpathing/LocalizationTuner.java:1) | Mode/menu lifecycle is straightforward. Use finally for context.stop on exceptions and keep phase applicability/status logic in one place. |
| [SimpleDriveTest.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/apexpathing/SimpleDriveTest.java:1) | Explicitly set the authored start pose and check stop after waitForStart. Reset currentState for reused instances and use finally for follower.stop. |
| [TeleOpTest.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/apexpathing/TeleOpTest.java:1) | First loop-time sample uses epoch time because lastLoop starts at zero. Use monotonic nanos initialized at start; keep counters private/local and reset each run. |

### paths

| File | Review notes |
| --- | --- |
| [Callback.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/paths/Callback.java:1) | F11. Validate action/angle/progress rather than carrying null/NaN to runtime. Immutable trigger definition plus per-execution fired state is simpler. |

### paths/builders

| File | Review notes |
| --- | --- |
| [HolonomicPathBuilder.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/paths/builders/HolonomicPathBuilder.java:1) | F14. Arc expansion, heading resolution, constraint sampling, and build policy are crowded together. Repeated builds reuse one mutable Path and can repeat callback compilation. |
| [PathBuilder.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/paths/builders/PathBuilder.java:1) | rawPoses retains the caller's array; clone it. Make builders single-use or produce independent definitions on repeated builds. Reject invalid constraint progress instead of clamping it to an arbitrary .9. |
| [TankPathBuilder.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/paths/builders/TankPathBuilder.java:1) | Keep tangent-only validation explicit. Shares mutable/repeated-build risks with PathBuilder; avoid another copy of heading/constraint build policy. |
| [TurnBuilder.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/paths/builders/TurnBuilder.java:1) | Treat build as producing a new execution-independent turn. Define repeated build/callback semantics and reject incomplete input at one boundary. |

### paths/constraint

| File | Review notes |
| --- | --- |
| [AngularConstraint.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/paths/constraint/AngularConstraint.java:1) | Change notifications support live planning. Clarify ownership/lifetime of listeners so constraints reused across paths do not accumulate obsolete listeners. |
| [PathConstraint.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/paths/constraint/PathConstraint.java:1) | No specific defect found. Document when custom mutable constraints must notify and when live sampling is required. |
| [TranslationalConstraint.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/paths/constraint/TranslationalConstraint.java:1) | Same listener-lifetime concern as AngularConstraint. Mutable limits need coherent planner invalidation. |

### paths/heading

| File | Review notes |
| --- | --- |
| [HeadingInterpolator.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/paths/heading/HeadingInterpolator.java:1) | No actionable issue. Keep heading derivatives defined with respect to path parameter/distance, never implicitly time. |
| [HeadingNode.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/paths/heading/HeadingNode.java:1) | No actionable issue in the small value carrier. Validate node order/progress in the builder. |
| [HolonomicInterpolator.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/paths/heading/HolonomicInterpolator.java:1) | Terminal-error logic assumes a meaningful end heading even when tangent modes allow unspecified headings. Define terminal heading from the resolved interpolation and avoid NaN propagation. |
| [InterpolationStyle.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/paths/heading/InterpolationStyle.java:1) | No actionable issue. Public styles should document exactly which parameters are required. |
| [ReversedHeadingInterpolator.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/paths/heading/ReversedHeadingInterpolator.java:1) | No specific defect found. Derivative sign and endpoint mapping deserve explicit contracts; keep reversal algebra in this one adapter. |
| [TankInterpolator.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/paths/heading/TankInterpolator.java:1) | No specific defect found. Avoid duplicating tangent derivative conventions in builders. |

### paths/movements

| File | Review notes |
| --- | --- |
| [FollowerMovement.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/paths/movements/FollowerMovement.java:1) | Mutable started/ended flags mix reusable movement definitions with runtime state. Execution outcomes belong on the follower/traversal. |
| [Path.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/paths/movements/Path.java:1) | Stored fields retain their build-time settings, but public geometry/interpolator/constraint setters still mutate the definition. Freeze memory-built paths or rebuild derived data deliberately. Remove hidden singleton planning lookup. |
| [Turn.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/paths/movements/Turn.java:1) | Separate authored sweep/profile from runtime completion and callbacks. Keep profile selection explicit for dual-actuated drives. |

### storage

| File | Review notes |
| --- | --- |
| [ApexStorage.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/storage/ApexStorage.java:1) | Backup/replace logic is useful. Specify recovery for corrupt primary JSON, not only missing files, and keep failure reporting at callers. No additional definite defect found here. |

### trajectory

| File | Review notes |
| --- | --- |
| [OnlineTrajectoryPlanner.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/trajectory/OnlineTrajectoryPlanner.java:1) | Small facade is useful. Live settings and window reuse need one documented invalidation contract rather than per-field comparisons scattered underneath. |
| [StoppingHorizon.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/trajectory/StoppingHorizon.java:1) | Assumptions about braking authority must agree with the selected motor model. Use explicit units and conservative stopping guarantees rather than duplicated coefficient policy. |
| [TrajectoryField2D.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/trajectory/TrajectoryField2D.java:1) | Spatial-only storage cannot distinguish multiple traversals of a crossing. Document/reject ambiguous paths or add progress context. Bound memory in bytes for the robot, not just integer array overflow. |
| [TrajectorySample.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/trajectory/TrajectorySample.java:1) | Good direction toward a named sample contract. Keep units/frame and immutable vector components explicit. |
| [VelocityPursuit.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/trajectory/VelocityPursuit.java:1) | Preview/target rules should be documented as a specific policy. Avoid another independently tuned lookahead rule competing with endpoint capture. |

### trajectory/generators

| File | Review notes |
| --- | --- |
| [BaseProfileGenerator.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/trajectory/generators/BaseProfileGenerator.java:1) | Largest numerical maintenance hotspot: planning windows, caching, constraints, power feasibility, and diagnostics. Extract constraint sampling and share a single physical feasibility evaluator. |
| [ConstraintSchedule.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/trajectory/generators/ConstraintSchedule.java:1) | No specific defect found. Document duplicate progress-point precedence and preserve stable ordering. |
| [MecanumProfileGenerator.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/trajectory/generators/MecanumProfileGenerator.java:1) | F20. Derived directional limits must use the same immutable snapshot or revision as base-generator caches. |
| [SwerveProfileGenerator.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/trajectory/generators/SwerveProfileGenerator.java:1) | State assumptions about steering alignment/authority explicitly; translational envelope alone does not model module slew. Keep this a documented model limitation. |
| [TankProfileGenerator.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/trajectory/generators/TankProfileGenerator.java:1) | No specific defect found. Coordinate curvature and signed heading conventions with TankPathFollower and TurnProfileGenerator. |
| [TurnProfileGenerator.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/trajectory/generators/TurnProfileGenerator.java:1) | F13. Ensure measured braking and non-characterized profile power limits obey the same output budget as execution. |

### trajectory/profile

| File | Review notes |
| --- | --- |
| [CoupledMotionProfile.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/trajectory/profile/CoupledMotionProfile.java:1) | Documented monotone reachability contract is appropriate. Make endpoint clipping/failure semantics explicit; keep numerical validation separate from diagnostics. |
| [DynamicMotionProfile.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/trajectory/profile/DynamicMotionProfile.java:1) | F12/F13. Enforce positive-time traversal for positive distance and use the selected power model consistently in both reachability sweeps. |
| [FFLut.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/trajectory/profile/FFLut.java:1) | Validate row nullness, finiteness, ordering, and time monotonicity once. Multiple interpolation APIs need clear distinctions between linear-row interpolation and constant-acceleration kinematics. |
| [MotionParameters.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/trajectory/profile/MotionParameters.java:1) | Mutable mixed translation/angular/profile/power carrier makes accidental unit confusion easy. Prefer immutable profile samples and a separate actuator-demand result. |

### tuning

| File | Review notes |
| --- | --- |
| [TunerContext.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/tuning/TunerContext.java:1) | Useful common lifecycle/frame helpers. Make debug/header behavior consistent in specialized contexts and avoid exposing mutable configuration without validity status. |
| [TuningCsvWriter.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/tuning/TuningCsvWriter.java:1) | Implement AutoCloseable and use lifecycle/finally ownership. Buffered rows need a documented bound; avoid duplicate serialization/error handling in AutoTest. |

### tuning/follower

| File | Review notes |
| --- | --- |
| [TunerContext.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/tuning/follower/TunerContext.java:1) | Follower header does not call the shared debug-header helper. Keep UI composition consistent; validate follower binding before use. |
| [TuningMeasurements.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/tuning/follower/TuningMeasurements.java:1) | No actionable issue. Small pure projection helpers are preferable to re-deriving coordinate transforms in each phase. |
| [TuningPhase.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/tuning/follower/TuningPhase.java:1) | Finally/onExit cleanup is good. Mode selection, motion gating, and result acceptance should remain explicit; provide scoped controller-state restoration for derived phases. |

### tuning/follower/acceleration

| File | Review notes |
| --- | --- |
| [AccelerationFeedforwardPhase.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/tuning/follower/acceleration/AccelerationFeedforwardPhase.java:1) | Deprecated 598-line experimental workflow is no longer in the main sequence. Remove after caller migration or move to optional diagnostics; preserve its paired-direction/confirmation checks if retained. |
| [AccelerationSearch.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/tuning/follower/acceleration/AccelerationSearch.java:1) | Compact bounded search with measured best-candidate tracking is reasonable. Document its monotonic signed-error assumption. |
| [RampRegression.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/tuning/follower/acceleration/RampRegression.java:1) | Legacy regression carries several mode flags and duplicate elimination logic. Select model type explicitly before collecting samples; share a small scaled solver. |
| [package-info.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/tuning/follower/acceleration/package-info.java:1) | No actionable issue. Concise package documentation; keep it aligned with the classes actually retained. |

### tuning/follower/centripetal

| File | Review notes |
| --- | --- |
| [BinarySearch.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/tuning/follower/centripetal/BinarySearch.java:1) | Validate finite ordered bounds and positive threshold. Remove lastGuess if unused; define convergence in terms of bracket width. |
| [CentripetalPhase.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/tuning/follower/centripetal/CentripetalPhase.java:1) | Convergence can save the next narrowed candidate rather than a directly measured one. Confirm the final value with a run; separate geometric error from feedback masking. |
| [package-info.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/tuning/follower/centripetal/package-info.java:1) | No actionable issue. Concise package documentation; keep it aligned with the classes actually retained. |

### tuning/follower/characterization

| File | Review notes |
| --- | --- |
| [MotionCharacterizationPhase.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/tuning/follower/characterization/MotionCharacterizationPhase.java:1) | Finite readings alone do not establish fresh valid measurements. Check localizer validity and reset all trial state in init if phase reuse is supported. |
| [MotorAuthorityRegression.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/tuning/follower/characterization/MotorAuthorityRegression.java:1) | Normal equations and absolute pivot threshold are scale-sensitive. Normalize columns and use a shared small solver with explicit conditioning/failure results. |
| [package-info.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/tuning/follower/characterization/package-info.java:1) | No actionable issue. Concise package documentation; keep it aligned with the classes actually retained. |

### tuning/follower/feedback

| File | Review notes |
| --- | --- |
| [DrivePhase.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/tuning/follower/feedback/DrivePhase.java:1) | F22/F23. Restore temporary controller settings in onExit and use the same robot-forward projection for commands and metrics. |
| [HeadingPhase.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/tuning/follower/feedback/HeadingPhase.java:1) | F23. Close manual metrics on early exit and restore controller enable state. Keep angular wrapping/unwrapped trial distance consistent. |
| [ManualResponseMetrics.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/tuning/follower/feedback/ManualResponseMetrics.java:1) | F24. Report accepted settled duration rather than the first transient band entry; log against the active command limit. |
| [PDSRoutine.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/tuning/follower/feedback/PDSRoutine.java:1) | Optimization mixes test motion, metrics, timing, parameter search, and acceptance. Timed-out/poor trials need an explicit rejection policy, not merely a best available cost. |
| [package-info.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/tuning/follower/feedback/package-info.java:1) | No actionable issue. Concise package documentation; keep it aligned with the classes actually retained. |

### tuning/follower/limits

| File | Review notes |
| --- | --- |
| [LimitsPhase.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/tuning/follower/limits/LimitsPhase.java:1) | F25. Add fresh-sample/validity checks and distinguish timeout-limited measurements from a verified velocity plateau. Avoid sorting complete sample history every tick. |
| [package-info.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/tuning/follower/limits/package-info.java:1) | No actionable issue. Concise package documentation; keep it aligned with the classes actually retained. |

### tuning/follower

| File | Review notes |
| --- | --- |
| [package-info.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/tuning/follower/package-info.java:1) | No actionable issue. Concise package documentation; keep it aligned with the classes actually retained. |

### tuning/follower/sysid

| File | Review notes |
| --- | --- |
| [FeedforwardTuner.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/tuning/follower/sysid/FeedforwardTuner.java:1) | Check sample validity/freshness, not just finite velocities. Replace reflective simulator graph hooks with a diagnostic interface; document forward/reverse static-friction aggregation. |
| [SysIdRegression.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/tuning/follower/sysid/SysIdRegression.java:1) | Normalized regression and positive-parameter validation are useful. Share only common scaled-solver mechanics with other regression classes. |
| [package-info.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/tuning/follower/sysid/package-info.java:1) | No actionable issue. Concise package documentation; keep it aligned with the classes actually retained. |

### tuning/follower/tracking

| File | Review notes |
| --- | --- |
| [ProfiledTrackingOptimizer.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/tuning/follower/tracking/ProfiledTrackingOptimizer.java:1) | Unused velocityLimit/maxPower arguments in perturbation and legacy settings copying obscure the three actual parameters. Replace status-string interpretation with an explicit stage query. |
| [ProfiledTrackingPhase.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/tuning/follower/tracking/ProfiledTrackingPhase.java:1) | Quality checks and fixed reference are useful. Remove optimizer.status().startsWith("Validating") control flow; use an enum/query. Restore candidate settings on abort and separate CSV/summary from trial execution. |
| [package-info.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/tuning/follower/tracking/package-info.java:1) | No actionable issue. Concise package documentation; keep it aligned with the classes actually retained. |

### tuning/localizer

| File | Review notes |
| --- | --- |
| [LocalizationTunerContext.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/tuning/localizer/LocalizationTunerContext.java:1) | Calibration replacement/rollback should be transactional; rollback failure must not hide the original error. Snapshot enough runtime state to restore aborted calibration faithfully. |
| [LocalizerAdapter.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/tuning/localizer/LocalizerAdapter.java:1) | Interface is reasonable. Require a clear no-data/failure result for unsupported or invalid calibration trials. |
| [LocalizerAdapters.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/tuning/localizer/LocalizerAdapters.java:1) | Large nested adapter switchboard couples JSON and physical equations. Split into small typed adapters; validate trial list/counts and distinguish forward/strafe scale assumptions. |
| [TuningPhase.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/tuning/localizer/TuningPhase.java:1) | Cleanup lifecycle is useful. Acceptance should mean validated/saved calibration, and aborted live edits should restore prior settings. |

### tuning/localizer/calibration

| File | Review notes |
| --- | --- |
| [CalibrationAxis.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/tuning/localizer/calibration/CalibrationAxis.java:1) | No actionable issue. Small explicit enum is appropriate. |
| [CalibrationCandidate.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/tuning/localizer/calibration/CalibrationCandidate.java:1) | Copy JSONObject on input as well as output; retaining the caller's mutable object undermines snapshot semantics. |
| [CalibrationSnapshot.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/tuning/localizer/calibration/CalibrationSnapshot.java:1) | Public rawTicks array can mutate a supposed snapshot. Copy on construction and access, or expose indexed reads. |
| [DirectionTrial.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/tuning/localizer/calibration/DirectionTrial.java:1) | No specific defect found. Keep trial measurements immutable and validate before calibration computation. |
| [SpinTrial.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/tuning/localizer/calibration/SpinTrial.java:1) | No specific defect found. Independent angle and off-axis quality should participate in validation. |
| [package-info.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/tuning/localizer/calibration/package-info.java:1) | No actionable issue. Concise package documentation; keep it aligned with the classes actually retained. |

### tuning/localizer

| File | Review notes |
| --- | --- |
| [package-info.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/tuning/localizer/package-info.java:1) | No actionable issue. Concise package documentation; keep it aligned with the classes actually retained. |

### tuning/localizer/phases

| File | Review notes |
| --- | --- |
| [DirectionsPhase.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/tuning/localizer/phases/DirectionsPhase.java:1) | Define handling of no-response trials explicitly before accepting direction geometry. Short actuator pulses need fresh measurements and configured IMU identity. |
| [DistancePhase.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/tuning/localizer/phases/DistancePhase.java:1) | Validate trial heading stability and scale plausibility before acceptance. Avoid quietly applying one scale correction to both axes if hardware differs. |
| [FilterMetrics.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/tuning/localizer/phases/FilterMetrics.java:1) | Combining squared linear and angular acceleration without normalization mixes incompatible units. Report separate RMS measures or normalize by documented noise scales. |
| [FilterPhase.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/tuning/localizer/phases/FilterPhase.java:1) | Field-axis travel requirements can misclassify tank motion at nonzero heading. Use body-axis projection, restore settings on abort, and add measurable quality criteria beyond distance collected. |
| [RelativeImuAccumulator.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/tuning/localizer/phases/RelativeImuAccumulator.java:1) | Choosing the rotation axis from the first tiny delta is noise-sensitive. Estimate it from a sufficient motion window and verify off-axis consistency. |
| [SpinPhase.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/tuning/localizer/phases/SpinPhase.java:1) | Hardcoded hardware name "imu" ignores localizer configuration. Add a deadline/no-progress failure for automatic spinning and validate independent IMU samples. |
| [TestPhase.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/tuning/localizer/phases/TestPhase.java:1) | No specific defect found in the intentionally continuous test phase. Keep stop cleanup in the base lifecycle. |
| [package-info.java](C:/Users/joela/Documents/Apex/ApexPathing/TeamCode/src/main/java/tuning/localizer/phases/package-info.java:1) | No actionable issue. Concise package documentation; keep it aligned with the classes actually retained. |

## Verification and limits

- Read all 154 files in the requested tree, including current uncommitted changes. This is a source review, not a robot/hardware acceptance run.
- Executed [reproduce.ps1](C:/Users/joela/Documents/Apex/ApexPathing/reviews/java-review-2026-10-05/reproduce.ps1), compiling the **current original Java source** for the affected value/math classes. Minimal stubs replace only Android annotations and unrelated hardware/builder types; no reviewed method is stubbed.
- Observed: `setMaxPower(.3) = 1.0`; mirrored arc throws `ClassCastException`; equal-coordinate vectors and copies compare false; a 24-unit, two-sample rest-to-rest profile has duration 0; configured .5-inch/1-degree completion becomes 1.5 inches/2 degrees; unsettled completion returns true after 110 ms; characterized braking requires power 2 against budget 1; an indefinite covariance is accepted and later fails innovation-variance calculation.
- Attempted the relevant Gradle unit-test task. The wrapper needs Gradle 8.9 and failed to download it with `SocketException: Permission denied: connect`, before tests could run. Existing test coverage was not established by that attempt. No claim of a passing full build is made.
- Hardware behavior, loop timing, calibrated regression quality, and self-intersection grid behavior require targeted simulation/device checks. File notes marked as recommendations should not be treated as independently reproduced bugs.

## Suggested repair order

Fix F01–F07 first, then geometry/API/lifecycle defects F08–F25. Add focused regression coverage for the reproduced contracts and device register/status boundaries. Address covariance validation and configuration invariants. After those changes, consolidate settings ownership and physical-model evaluation; remove migrated compatibility code. Cosmetic cleanup comes last.
