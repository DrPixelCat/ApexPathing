package tuning.follower;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import geometry.Pose;

/**
 * Base class for tuning phases for the follower tuner. Each phase is responsible for tuning a
 * specific aspect of the follower's behavior The class provides a framework for running the tuning
 * process, including selecting the tuning mode (manual or automatic), executing the tuning logic,
 * and displaying results.
 *
 * @author Sohum Arora - 22985 Paraducks
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
@SuppressWarnings("ClassNamePrefixedWithPackageName")
public abstract class TuningPhase {
    enum TuningState {
        SELECT_MODE,
        TUNING,
        RESULTS
    }

    protected final TunerContext context;
    protected LinearOpMode opMode;
    protected boolean manualMode;
    protected double increment;

    protected TuningPhase(TunerContext context) { this.context = context; }

    public boolean run(LinearOpMode opMode) {
        boolean driveEnabled = context.getFollower().isDriveControllerEnabled();
        boolean headingEnabled = context.getFollower().isHeadingControllerEnabled();
        try { return runPhase(opMode); }
        finally {
            context.getFollower().stop();
            try { onExit(); }
            finally {
                if (driveEnabled) { context.getFollower().enableDriveController(); }
                else { context.getFollower().disableDriveController(); }
                if (headingEnabled) { context.getFollower().enableHeadingController(); }
                else { context.getFollower().disableHeadingController(); }
            }
        }
    }

    protected void onExit() { }

    private boolean runPhase(LinearOpMode opMode) {
        this.opMode = opMode;
        manualMode = false;
        increment = 0.001;

        TuningState state = TuningState.SELECT_MODE;

        while (opMode.opModeIsActive()) {
            context.beginFrame();
            if (context.getFollower().getDrivetrain() instanceof drivetrains.DualActuated
                    && ((drivetrains.DualActuated) context.getFollower().getDrivetrain()).isTransitioning()) {
                context.getFollower().update(false);
                context.getTelemetry().addData("Tuning", getTuningConstants());
                context.publishFrame(false);
                context.pauseLoop();
                continue;
            }
            switch (state) {
                case SELECT_MODE:
                    context.getFollower().update();
                    showModeSelector();
                    if (manualTuneIsPossible() && autoTuneIsPossible() &&
                            (opMode.gamepad1.dpadLeftWasPressed() ||
                                    opMode.gamepad1.dpadRightWasPressed())) {
                        manualMode = !manualMode;
                    }
                    if (opMode.gamepad1.aWasPressed()) {
                        init();
                        state = TuningState.TUNING;
                    }
                    if (state == TuningState.SELECT_MODE) {
                        context.getFollower().manual(opMode.gamepad1);
                    }
                    break;
                case TUNING:
                    context.getFollower().update(holdPointDuringRoutine());
                    if (!manualMode) {
                        context.getTelemetry().addData("Tuning", getTuningConstants());
                    }
                    boolean complete;
                    if (manualMode) {
                        complete = manualTuned();
                    } else {
                        complete = autoTuned();
                    }
                    if (complete) {
                        context.getFollower().stop();
                        state = TuningState.RESULTS;
                    } else if (!routineMotionActive()) {
                        context.getFollower().manual(opMode.gamepad1);
                    }
                    break;
                case RESULTS:
                    context.getFollower().update();
                    showResults();
                    if (opMode.gamepad1.aWasPressed()) {
                        return opMode.opModeIsActive() && !opMode.isStopRequested();
                    }
                    context.getFollower().manual(opMode.gamepad1);
                    break;
            }
            context.publishFrame(state != TuningState.TUNING || !routineMotionActive());
            // A LinearOpMode can otherwise spin much faster than the hardware/localizer can
            // provide new samples. Besides wasting CPU, differentiating the same pose repeatedly
            // and then one discrete update creates enormous acceleration spikes (especially in
            // FTCodeSim, whose physics advances every 20 ms). Use a deterministic 50 Hz sampling
            // cadence for every tuner phase.
            context.pauseLoop();
        }

        return false;
    }

    private void showModeSelector() {
        context.getTelemetry().addLine("Next phase: " + getPhaseName());
        if (manualTuneIsPossible() && autoTuneIsPossible()) {
            context.getTelemetry().addLine(
                    "Dpad Left/Right: Automatic / Manual");
            context.getTelemetry().addData("Mode", manualMode ? "Manual" : "Automatic");
        } else {
            manualMode = manualTuneIsPossible();
            context.getTelemetry().addData("Mode", manualMode ? "Manual" : "Automatic");
        }
        showPreRunInstructions();
        context.getTelemetry().addLine("Press A to run this phase.");
    }

    /** Adds phase-specific positioning or safety guidance before the operator starts motion. */
    protected void showPreRunInstructions() { }

    private void showResults() {
        context.getTelemetry().addLine(getPhaseName() + " constants:");
        reportResults();
        context.getTelemetry().addLine("Press A to save and continue.");
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
        context.getTelemetry().addLine((selected ? "-> " : "   ") + label + ": " +
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
            increment = Math.max(increment / 10.0, 0.00001);
        } else if (opMode.gamepad1.dpadRightWasPressed()) {
            increment = Math.min(increment * 10.0, 1.0);
        } else if (opMode.gamepad1.dpadUpWasPressed()) {
            return increment;
        } else if (opMode.gamepad1.dpadDownWasPressed()) {
            return -increment;
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
