# Repairs following the Java review

Implemented the 26 ranked defects from REVIEW.md in the existing working tree. Existing user changes were retained. Additional repairs cover configuration ownership, invalid inputs, lifecycle cleanup, calibration snapshots, and tuning rollback.

| Finding | Repair |
| --- | --- |
| F01 | Pinpoint offset registers receive millimeters; register-write regression uses nonzero offsets. |
| F02 | Maximum power preserves fractional values, clamps both ends, and rejects nonfinite values. |
| F03 | Swerve stop zeros steering servos and drive motors; per-module feedback uses each encoder's voltage range. |
| F04 | Octoquad initialization has a five-second deadline, terminal-fault rejection, and interrupt propagation. |
| F05 | Movement outcomes distinguish running, completed, cancelled, and failed. Stalls stop motors and retain a failure reason. |
| F06 | Holding rotates field translation into the current robot frame; tank holding uses the current heading. |
| F07 | Paused execution stops output even when holding was requested. |
| F08 | Arc mirroring and copying preserve ArcPose and radius. |
| F09 | Completion honors configured tolerances and requires a continuous settled interval. |
| F10 | Tank and turn execution respect controller enable flags independently of feedforward. |
| F11 | Fired callbacks are tracked per execution and fire again when the same movement is followed again. |
| F12 | Two-sample rest-to-rest requests gain an interior state; impossible positive-distance, zero-speed intervals fail explicitly. |
| F13 | Braking reachability enforces the supplied measured power model. |
| F14 | Missing facing-point input reaches the documented fallback instead of dereferencing null. |
| F15 | Odometry configuration rejects invalid resolution, geometry, and failed IMU initialization. |
| F16 | Pinpoint accepts complete, finite READY packets, preserves accepted-state timestamps on rejection, and stops hiding valid high velocity. |
| F17 | Octoquad uses its pose-validity predicate before updating estimates. |
| F18 | Localizer replacements are constructed before changing the active setup; construction failure preserves the usable localizer and pose. |
| F19 | Distance, scalar, angle, vector, pose, and arc pose have consistent value equality and hash codes. |
| F20 | Mecanum directional limits refresh when their four inputs change. |
| F21 | B-splines evaluate their exact endpoint. |
| F22 | Drive tuning commands and measures displacement along the starting robot heading. |
| F23 | Response logs close on exit; tuning phases restore prior controller enable flags. |
| F24 | Leaving the tolerance region invalidates previous settling; saturation uses the configured power ceiling. |
| F25 | Limits tuning changes power only during its lifecycle and restores the previous ceiling on exit. |
| F26 | Covariance validation rejects indefinite matrices while accepting singular positive-semidefinite matrices. |

Additional changes:

- Each follower owns detached settings. Factory-created paths and turns carry planning snapshots; turning no longer consults unrelated mutable settings after construction.
- Path changes invalidate stored motion data. Path builders reject reuse and invalid poses; feedforward tables reject invalid or unordered states. Trajectory grids have a 32 MiB allocation budget.
- Matrix operations reject invalid shapes and singular inversions. Feedforward model inputs are checked before indexing or solving.
- CSV writers implement AutoCloseable, stop after write errors, and bound buffered output.
- Localization calibration snapshots defensively copy tick arrays and candidate JSON. Configured IMU names are respected. Quaternion axis learning accumulates motion instead of accepting the first tiny noisy delta.
- Aborted filter and tracking trials restore previous settings. Filter metrics report linear and angular RMS separately and measure movement in the robot frame.
- Characterization resets trial state and rejects invalid localization. Centripetal search measures its final candidate before accepting it.
- Example OpModes and top-level tuners stop motors in finally blocks; autonomous examples inspect movement failure. TeleOp loop timing uses an initialized monotonic clock.

## Compatibility

Runtime tuning should use follower.getConstants(), not mutate FollowerConstants.getInstance() after constructing a follower. GeometryFactory(follower) binds planning settings to that follower. Explicit stored-path changes rebuild the cached trajectory at the next access.

A failed movement requires clearFailure() before an intentional retry. isBusy() alone is not a success check; use movementSucceeded() or getMovementOutcome(). Strict endpoint tolerances can expose calibration problems that previously appeared successful.

## Validation

The focused Gradle suite covers geometry, controllers, paths, trajectory, follower, localization, feedforward, tuning, storage, drivetrains, example OpMode logic, and hardware-backed review regressions. The final run passed 305 tests.

An additional 25-test simulation run failed seven tests. Three failures were corrected by migrating tests to the new per-follower settings and isolating the legacy centripetal model; those tests now pass in the focused suite. The example-auto tracking check has since been corrected as described below. Three broader checks remain unresolved:

- ApexSimulationTest.exportProfiledTurnVelocityResponse: final 10% exceeded its 0.4-second timing target.
- AutoTestSimulationTest.autoTestCompletesTwistingPath: host loop average was 24.8613 ms against a 23 ms fixture budget.
- FollowerTunerTelemetryTest.automaticWorkflowCompletesWithoutVelocityFeedbackTuner: motion-characterization validation failed in speed band 2.

### Example-auto tracking follow-up

The original tracking result came from an inconsistent simulation fixture: it reconstructed robot velocity from individual motors instead of advancing the full drivetrain mechanism, used wall time for control while stepping physics by a fixed 20 ms, and inherited mutable persisted calibration. The regression now uses the complete desktop-simulator physics, a shared virtual clock for control and sensor samples, and the existing versioned simulator calibration isolated from other tests. No braking algorithm, saved calibration, gains, or path constraints were changed by this follow-up.

Every path must now report COMPLETED and stay below 0.5 inches of peak cross-track error (previously 6 inches). The complete sequence passes: outbound 0.368255 in, reverse return 0.150714 in, strafe outbound 0.072840 in, strafe return 0.015872 in. These are corrected simulation measurements, not evidence of a physical robot improvement.

The last expanded regression run completed 308 tests: 307 passed; MecanumFrictionModelTest.brakingAccelerationJumpsAtZeroCommand failed its assertion that the zero-duty acceleration jump must exceed 100 in/s². The example-auto test passed alongside the original 305-test focused suite. This separate motor-response assertion remains unresolved; further experiments were stopped at the user's request.

Pedro Pathing's current [Foresight source](https://github.com/Pedro-Pathing/PedroPathing/blob/69094ade253249418c57ebd0a8df7cc9986d3898/core/src/main/java/com/pedropathing/algorithm/Foresight.java#L245) was reviewed at commit 69094ade253249418c57ebd0a8df7cc9986d3898. It predicts body-frame stopping displacement using fitted linear and quadratic velocity terms, then projects the stopping pose onto the path. It distinguishes coasting from active braking. Its [allocator](https://github.com/Pedro-Pathing/PedroPathing/blob/69094ade253249418c57ebd0a8df7cc9986d3898/core/src/main/java/com/pedropathing/algorithm/ForesightPowerAllocator.java#L47) explicitly prioritizes motion components according to deviation. The useful lesson is to validate the measured stopping response and allocation together. A negative motor command alone is not a general definition of deceleration in Apex's wheel model, which includes drag and regenerative braking. No Pedro implementation was copied or substituted.

The first unrestricted suite was interrupted after additional simulation failures; it is not reported as passing. No pre-change full simulation baseline was established, so these failures cannot be classified as pre-existing. Acceptance thresholds and saved simulator calibration were not weakened to make them pass. Physical robot verification remains outstanding for device startup and actuator behavior.

The original file-by-file review also contains optional architectural and style suggestions. These are not all implemented: vendor drivers remain embedded, public unit types and legacy APIs remain compatible, and unrelated numerical solvers have not been consolidated without evidence that their contracts match. This repair does not claim that every aesthetic suggestion has been applied.
