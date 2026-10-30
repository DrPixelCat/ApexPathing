# Setup and tuning workflows

Robot preparation code is grouped under `setup`: `setup.hardware` owns hardware configuration,
`setup.localization` owns localization calibration, and `setup.follower` owns follower tuning.
Shared interaction and telemetry classes live directly in `setup`; shared CSV writers live in
`setup.logging`. Runtime drivetrain and localizer implementations remain in their existing packages.

Workflows own their state directly. There is no shared state-machine engine,
builder, callback registration, or deferred transition queue. Each workflow uses
its own enum and named methods for the work appropriate to that procedure.

## Setup

`RobotSetup.handleSetupAction` shows the entire hardware setup progression in one
switch: choose saved settings or edit, advance each configuration field, validate
the complete draft, wait for a separate Save confirmation, then finish. Errors
and completion wait for STOP. One action is dispatched per frame, so a confirmation
cannot also accept the next screen.

`RobotSetupWizard` supplies the draft profile, field editor, screen views, clock,
validation, persistence, and hardware safeguards. Its private screen classes
describe the interface; they do not decide the next workflow step. `beginAction`
stops motor tests before navigation. `updateMotorTest` handles the separately held
test control, including the release required after navigation. `lines` only
describes the current screen. Encoder assignments, resolution, and directions
belong here; movement checks, geometry, and filtering belong to Localization Tuner.

`AdjustmentStep` shares bounded precision scaling, reset, and signed deltas between
hardware setup, follower tuning, distance entry, and manual rotation entry. Each
procedure supplies its existing starting step, bounds, and scaling factor; it
retains its own input priority, units, and validation of the edited value.

Only the final Save action writes the draft through `SetupStore`.
`close` stops output, releases motor tests and filter learning, and closes logs,
including when another cleanup operation fails. The OpMode owns the outer
hardware lifetime and calls cleanup in `finally`.

## Tuning

The two `TuningPhase` classes describe different workflows explicitly:

- Follower tuning selects a mode, initializes the procedure, runs measurements,
  shows results, and waits for confirmation to continue. The outer `run` owns
  final motor stop, procedure cleanup, and restoration of controller enablement.
- Localization tuning prepares, records, reviews, and accepts a calibration.
  `startRecording`, `collectMeasurement`, and `reviewMeasurement` identify the
  boundaries. `LocalizationTunerContext` owns applying and persisting accepted
  calibration, including rollback on save failure. Procedure cleanup releases
  collection resources and restores unaccepted filter settings.

Each phase loop samples confirmation once; `confirmPressed()` exposes that edge
to the active procedure. One dispatch handles only the step active at the start
of the update. Entering a step never updates that step with the same confirmation.
The existing loop pauses and measurement clocks retain the control cadence.

Distance and spin calibration own their drive/settle/manual-entry progressions
and timestamps. Filter calibration owns sample-count and movement thresholds,
learning policy, and CSV lifetime. Limits, feedforward acquisition, and the
acceleration diagnostic likewise own their trial enums. Their focused dispatches
call named measurement methods; calculations, units, coefficients, and sample
validation remain in the procedures. Already-readable optimizer and direction
trial flows keep their existing structures.

## Display and persistence

`TelemetryFrame` remains a useful shared display abstraction. Measurement updates
collect text and labelled values. `view` returns a read-only snapshot; `update`
publishes it without processing input, changing state, commanding motion, or
saving settings. Setup screen descriptions are also read-only. Methods that
handle input or measurements are distinct from methods that describe results.

The former `Behavior`, `Screen`, and `ScreenHost` adapters were unnecessary once
the owners handled their lifetimes directly. `WorkflowText` shares only the
operator instruction:

> Press STOP ⏹ to cancel or redo

There are no Back, Retry, Repeat, or Restart recovery controls. Errors require
STOP and a new OpMode run. Successful manual verification trials and required
opposite-direction trials are part of acquisition. Existing automatic rejection
of inadequate sample windows remains part of the measurement algorithms.

STOP ends the OpMode and triggers `finally` cleanup. Earlier accepted and saved
tuning phases remain saved; abandoning a later phase does not undo them. Setup
drafts remain unsaved until Save. The follower's internal lifecycle is outside
this workflow design and is unchanged by this cleanup.

The cleanup's test coverage and baseline simulation comparison are recorded in
[WorkflowCleanupValidation.md](WorkflowCleanupValidation.md).
