package setup.localization;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import setup.WorkflowText;

/** Owns collection, review and acceptance for localization calibration. */
public abstract class TuningPhase {
    private enum State { PREPARE, RECORD, REVIEW, COMPLETE, ERROR }
    protected final LocalizationTunerContext context;
    protected LinearOpMode opMode;
    private State state;
    private boolean confirmPressed;
    private String error;

    protected TuningPhase(LocalizationTunerContext context) { this.context = context; }
    protected final boolean confirmPressed() { return confirmPressed; }

    public final boolean run(LinearOpMode opMode) {
        this.opMode = opMode;
        state = State.PREPARE;
        error = "";
        confirmPressed = false;
        try {
            reset();
            while (opMode.opModeIsActive() && !opMode.isStopRequested()) {
                context.update();
                context.beginFrame();
                context.getView().addLine(getName());
                if (context.isDriveTransitioning()) {
                    context.stop();
                    context.getView().addLine("Waiting for drivetrain mode to settle...");
                } else {
                    confirmPressed = opMode.gamepad1.aWasPressed();
                    updatePhase();
                }
                context.getView().addLine(WorkflowText.STOP_HINT);
                context.getView().update();
                if (state == State.COMPLETE) {
                    return opMode.opModeIsActive() && !opMode.isStopRequested();
                }
                context.pauseLoop();
            }
            return false;
        } finally {
            try { context.stop(); }
            finally { cleanup(); }
        }
    }

    /** Dispatch once: an input frame belongs only to the step where it was sampled. */
    private void updatePhase() {
        switch (state) {
            case PREPARE:
                context.stop();
                updatePrepare();
                showPrepare();
                if (confirmPressed) { startRecording(); }
                break;
            case RECORD: collectMeasurement(); break;
            case REVIEW: reviewMeasurement(); break;
            case ERROR:
                context.stop();
                context.getView().addData("Could not record", error);
                break;
            case COMPLETE: break;
        }
    }

    private void startRecording() {
        state = State.RECORD;
        try { beginRecording(); }
        catch (RuntimeException failure) { recordFailure(failure); }
    }

    private void collectMeasurement() {
        try {
            if (record()) {
                context.stop();
                finishRecording();
                state = State.REVIEW;
            }
        } catch (RuntimeException failure) { recordFailure(failure); }
    }

    private void reviewMeasurement() {
        context.stop();
        showReview();
        context.getView().addLine("A: save");
        if (!confirmPressed) { return; }
        if (accept()) { state = State.COMPLETE; }
        else {
            error = context.getLastSaveError();
            if (error == null || error.isEmpty()) { error = "Could not save calibration."; }
            state = State.ERROR;
            cleanup();
        }
    }

    private void recordFailure(RuntimeException failure) {
        context.stop();
        error = failure.getMessage() == null ? failure.toString() : failure.getMessage();
        state = State.ERROR;
        cleanup();
    }

    protected abstract String getName();
    protected abstract void reset();
    protected void updatePrepare() { }
    protected abstract void showPrepare();
    protected abstract void beginRecording();
    protected abstract boolean record();
    protected void finishRecording() { }
    protected void cleanup() { }
    protected abstract void showReview();
    protected abstract boolean accept();
}
