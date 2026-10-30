package setup.follower;

import setup.AdjustmentStep;

import setup.WorkflowText;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import paths.geometry.Pose;

/**
 * Owns mode selection, measurement, and result confirmation for follower tuning.
 * Procedures own their measurement algorithms and evidence; this loop owns input sampling,
 * frame publication, motor stop, and restoration of controller enablement.
 *
 * @author Sohum Arora - 22985 Paraducks
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
@SuppressWarnings("ClassNamePrefixedWithPackageName")
public abstract class TuningPhase {
    enum TuningState {
        SELECT_MODE,
        TUNING,
        RESULTS,
        COMPLETE,
        ERROR
    }

    protected final TunerContext context;
    protected LinearOpMode opMode;
    protected boolean manualMode;
    protected final AdjustmentStep increment =
            new AdjustmentStep(.001, .00001, 1, 10);

    protected TuningPhase(TunerContext context) { this.context = context; }

    public boolean run(LinearOpMode opMode) {
        boolean driveEnabled = context.getFollower().isDriveControllerEnabled();
        boolean headingEnabled = context.getFollower().isHeadingControllerEnabled();
        try { return runPhase(opMode); }
        finally {
            try { context.getFollower().stop(); }
            finally {
                try { onExit(); }
                finally {
                    if (driveEnabled) { context.getFollower().enableDriveController(); }
                    else { context.getFollower().disableDriveController(); }
                    if (headingEnabled) { context.getFollower().enableHeadingController(); }
                    else { context.getFollower().disableHeadingController(); }
                }
            }
        }
    }

    protected void onExit() { }

    private TuningState state;
    private String error = "";
    private boolean confirmPressed;

    protected final boolean confirmPressed() { return confirmPressed; }

    private boolean runPhase(LinearOpMode opMode) {
        this.opMode = opMode;
        manualMode = manualTuneIsPossible() && !autoTuneIsPossible();
        increment.reset();
        error = "";
        state = TuningState.SELECT_MODE;
        confirmPressed = false;
        while (opMode.opModeIsActive() && !opMode.isStopRequested()) {
            context.beginFrame();
            boolean transitioning = context.getFollower().getDrivetrain() instanceof drivetrains.DualActuated
                    && ((drivetrains.DualActuated) context.getFollower().getDrivetrain()).isTransitioning();
            if (transitioning) {
                context.getFollower().update(false);
                context.getView().addData("Tuning", getTuningConstants());
            } else {
                boolean confirm = opMode.gamepad1.aWasPressed();
                confirmPressed = confirm;
                updatePhase(confirm);
            }
            context.getView().addLine(WorkflowText.STOP_HINT);
            boolean sticksEnabled = (state == TuningState.SELECT_MODE || state == TuningState.RESULTS
                    || state == TuningState.TUNING) && !routineMotionActive();
            context.publishFrame(!transitioning && sticksEnabled);
            if (state == TuningState.COMPLETE) {
                return opMode.opModeIsActive() && !opMode.isStopRequested();
            }
            // Keep the existing 50 Hz hardware and simulator sampling cadence.
            context.pauseLoop();
        }
        return false;
    }

    /** One dispatch per frame prevents confirmation from accepting a newly entered step. */
    private void updatePhase(boolean confirm) {
        switch (state) {
            case SELECT_MODE: updateModeSelection(confirm); break;
            case TUNING: updateMeasurement(); break;
            case RESULTS:
                context.getFollower().update();
                showResults();
                if (confirm) { state = TuningState.COMPLETE; }
                else { context.getFollower().manual(opMode.gamepad1); }
                break;
            case ERROR:
                context.getFollower().stop();
                context.getView().addData("Tuning stopped", error);
                break;
            case COMPLETE: break;
        }
    }

    private void updateModeSelection(boolean confirm) {
        context.getFollower().update();
        if (manualTuneIsPossible() && autoTuneIsPossible()
                && (opMode.gamepad1.dpadLeftWasPressed() || opMode.gamepad1.dpadRightWasPressed())) {
            manualMode = !manualMode;
        }
        showModeSelector();
        if (confirm) {
            state = TuningState.TUNING;
            try { init(); }
            catch (RuntimeException failure) { fail(failure); }
        } else { context.getFollower().manual(opMode.gamepad1); }
    }

    private void updateMeasurement() {
        try {
            context.getFollower().update(holdPointDuringRoutine());
            if (!manualMode) { context.getView().addData("Tuning", getTuningConstants()); }
            boolean complete = manualMode ? manualTuned() : autoTuned();
            if (complete) {
                context.getFollower().stop();
                state = TuningState.RESULTS;
            } else if (!routineMotionActive()) { context.getFollower().manual(opMode.gamepad1); }
        } catch (RuntimeException failure) { fail(failure); }
    }

    private void fail(RuntimeException failure) {
        context.getFollower().stop();
        error = failure.getMessage() == null ? failure.toString() : failure.getMessage();
        state = TuningState.ERROR;
    }

    private void showModeSelector() {
        context.getView().addLine("Next phase: " + getPhaseName());
        if (manualTuneIsPossible() && autoTuneIsPossible()) {
            context.getView().addLine(
                    "Dpad Left/Right: Automatic / Manual");
        }
        context.getView().addData("Mode", manualMode ? "Manual" : "Automatic");
        showPreRunInstructions();
        context.getView().addLine("Press A to run this phase.");
    }

    /** Adds phase-specific positioning or safety guidance before the operator starts motion. */
    protected void showPreRunInstructions() { }

    private void showResults() {
        context.getView().addLine(getPhaseName() + " constants:");
        reportResults();
        context.getView().addLine("Press A to save and continue.");
    }

    /**
     * Returns whether this phase currently owns drivetrain output. Idle screens return false so
     * the shared loop can pass the gamepad sticks through to the follower.
     */
    protected boolean routineMotionActive() { return context.getFollower().isBusy(); }

    /** Lets a phase use the follower's idle point-hold controller between direct-drive runs. */
    protected boolean holdPointDuringRoutine() { return false; }

    /** Displays a compact editable value list without spending a separate line on selection. */
    protected void addTunableValue(String label, double value, boolean selected) {
        context.getView().addLine((selected ? "-> " : "   ") + label + ": " +
                context.formatNumber(value));
    }

    protected String number(double value) { return context.formatNumber(value); }

    /**
     * Places a simulated movement test at its phase-specific staging pose. On hardware, resetting
     * odometry cannot move the physical robot, so positioning remains the operator's responsibility.
     */
    protected void positionRobotForSimulation(Pose pose) {
        context.positionRobotForSimulation(pose);
    }

    protected double manualChange() {
        if (opMode.gamepad1.dpadLeftWasPressed()) {
            increment.finer();
        } else if (opMode.gamepad1.dpadRightWasPressed()) {
            increment.coarser();
        } else if (opMode.gamepad1.dpadUpWasPressed()) {
            return increment.delta(1);
        } else if (opMode.gamepad1.dpadDownWasPressed()) {
            return increment.delta(-1);
        }
        return 0.0;
    }

    protected abstract String getPhaseName();

    protected String getTuningConstants() { return getPhaseName(); }

    protected abstract boolean manualTuneIsPossible();

    protected abstract boolean autoTuneIsPossible();

    protected abstract void init();

    protected abstract boolean manualTuned();

    protected abstract boolean autoTuned();

    protected abstract void reportResults();
}
