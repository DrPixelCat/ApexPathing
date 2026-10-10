# Trajectory tracking in Follower Tuner

Run `TRAJECTORY TRACKING` only after accepting drive and heading feedforward,
[motion characterization](MotionCharacterization.md), heading
and drive feedback, motion limits, and (for holonomic drives) centripetal tuning.
Drive and heading SysId acquisition requires identifiable, finite physical gains and
adequate excitation. JUnit checks pooled R squared and the 0.05 motor-power
per-run residual limit independently; fit quality does not gate the tuner.
In FTCodeSim, each SysId run is now staged with its full travel available;
the recorded bad drive runs included a reversal while power was still applied,
consistent with reaching the simulated field boundary. Windows moving against
the commanded direction, or accelerating against it in a dynamic run, are not
used in the fit.
The quasistatic launch in each direction also measures static breakaway power.
The smaller valid launch becomes the initial heading or drive feedback `kS`
before the position-feedback search. The moving-friction feedforward `kS`
remains a separate fitted term. A run without valid launches in both directions
cannot be accepted.
Movement limits keep 95% of measured top speed but use 80% of the measured
open-loop acceleration peak. The earlier 95% acceleration target repeatedly
exceeded what the follower could track on the corner course.

The phase uses one 36 by 36 inch course with a 7 inch radius corner. Holonomic
paths use `TANGENT_OPTIMAL` heading, as Apex Auto Test does. Tank paths use
`TANGENT_FORWARD`. Allow stopping room beyond the course. The first trial anchors
the course; each subsequent trial retraces that same path in the opposite direction,
preserving the headings so the robot returns backward without turning around.
Leave the robot at the previous endpoint. If a trial misses its endpoint, reposition
there before starting the return run. FTCodeSim stages only the first trial.
Use the OpMode Stop button to cancel a trial or discard unsaved
changes. X starts a trial only while idle.

Automatic mode measures the original settings twice, tries bounded changes to
one setting at a time, and saves the best measured settings without a confirmation run.
With a saved motion characterization, the active settings are velocity tracking gain
and corner speed scale. The measured power model replaces the arbitrary profile velocity
power adjustment; legacy profiles without characterization retain that adjustment.
Velocity gain applies during both
acceleration and deceleration. Pulse/coast settings remain in saved JSON for
compatibility but no longer affect trajectory following or appear in the editor.
Manual mode uses X to run the course, LB/RB to select a setting, dpad Left/Right
to change the step, and dpad Up/Down to edit. Every edit invalidates the prior
measurement. A saves after a complete, usable measurement.

Every complete trial needs sufficient acceleration, braking, and curved braking
samples for selection. The tuner rejects incomplete or unusable acquisition.
JUnit independently checks acceleration RMS against 18% and braking RMS against
16% of peak target speed (with a 4 in/s minimum allowance), cross-track peak
below 2 inches, heading peak below 12 degrees, and endpoint accuracy within
3 inches and 5 degrees. These quality checks do not block calibration or add
validation motion to the tuner. The saved completion marker belongs to the
active follower profile. The optional PD tuner X verification remains available.

The candidate score integrates speed error against the first trial's fixed
profile for the current direction, charging overspeed more heavily, then adds peak cross-track and heading
error. The fixed reference prevents a slower requested speed from winning just
by moving the target. The phase writes `profiled_tracking_*.csv` beside
`constants.json` with target and measured speed, acceleration, curvature,
heading and cross-track error, endpoint blend, and delivered front-wheel power.
The full tuner to Auto Test simulation also checks acceleration underspeed,
braking RMS, and moving-curve heading and cross-track error.

The phase now displays **Trajectory Tracking** and runs online trajectories. A separately
stored nominal reference supplies baseline comparisons; measured logging uses
`Follower.getTrajectoryTarget()`. Existing calibration keys and CSV names remain compatible.
See [trajectory planning](TrajectoryPlanning.md) for the build-mode API.
