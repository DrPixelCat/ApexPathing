package org.firstinspires.ftc.teamcode.apexpathing.tuning;

public abstract class TuningPhase {
    private final String name;
    private boolean awaitingStart = true;
    private boolean confirming;
    private boolean readyToRerun;
    private boolean manualMode;

    protected TuningPhase(String name) {
        this.name = name;
    }

    public final String name() {
        return name;
    }

    public final boolean isRunningAutomatic() {
        return !awaitingStart && !confirming;
    }

    public void onResume(TunerContext context) {
    }

    public final TuningPhase update(TunerContext context, TunerInput input) throws InterruptedException {
        if (awaitingStart) {
            updateAwaitingStart(context, input);
        } else if (confirming) {
            return updateConfirmation(context, input);
        } else if (updateAutomatic(context)) {
            context.updateFollowerConfig();
            readyToRerun = false;
            confirming = true;
        }

        return this;
    }

    private void updateAwaitingStart(TunerContext context, TunerInput input) throws InterruptedException {
        context.telemetry().addLine(name + " phase initialized");
        context.telemetry().addLine("A - Toggle mode");
        context.telemetry().addLine("B - Start tuning");
        context.telemetry().addData("Selected Mode", modeName());

        if (input.aPressed()) {
            toggleMode();
        } else if (input.bPressed()) {
            if (manualMode) {
                context.setManualGuess(currentManualValue(context));
                readyToRerun = false;
                awaitingStart = false;
                confirming = true;
            } else {
                startAutomatic(context);
            }
        }
    }

    private TuningPhase updateConfirmation(TunerContext context, TunerInput input) throws InterruptedException {
        context.telemetry().addData("Current Phase", name);
        context.telemetry().addData("Robot Pose", context.follower().getPose().toString());

        if (!readyToRerun) {
            context.telemetry().addLine(savePrompt());
            context.telemetry().addLine(rerunPrompt());

            if (manualMode) {
                context.telemetry().addLine("--- MANUAL TUNING ---");
                context.telemetry().addLine(manualInstructions());
                applyManualValue(context, context.manualGuess());
                context.updateFollowerConfig();
                context.telemetry().addData(manualTelemetryLabel(), currentManualValue(context));
            } else {
                reportAutomaticResult(context);
            }

            if (input.aPressed()) {
                onAccepted(context);
                return nextPhase(context);
            } else if (input.bPressed()) {
                readyToRerun = true;
            }
        } else {
            context.telemetry().addLine("A - Toggle mode");
            context.telemetry().addLine(rerunExecutionPrompt());
            context.telemetry().addData("Selected Mode", modeName());

            if (input.aPressed()) {
                toggleMode();
                if (manualMode) {
                    context.setManualGuess(currentManualValue(context));
                }
            } else if (input.bPressed()) {
                readyToRerun = false;
                if (manualMode) {
                    confirming = true;
                } else {
                    startAutomatic(context);
                }
            }
        }

        context.driveWithGamepad();
        return this;
    }

    private void startAutomatic(TunerContext context) throws InterruptedException {
        readyToRerun = false;
        awaitingStart = false;
        confirming = false;
        beginAutomatic(context);
    }

    private void toggleMode() {
        manualMode = !manualMode;
    }

    private String modeName() {
        return manualMode ? "MANUAL" : "AUTO";
    }

    protected String savePrompt() {
        return "A - Save and advance";
    }

    protected String rerunPrompt() {
        return "B - Rerun tuner";
    }

    protected String rerunExecutionPrompt() {
        return "B - Rerun tuner";
    }

    protected void onAccepted(TunerContext context) {
    }

    protected abstract void beginAutomatic(TunerContext context) throws InterruptedException;

    protected abstract boolean updateAutomatic(TunerContext context) throws InterruptedException;

    protected abstract double currentManualValue(TunerContext context);

    protected abstract void applyManualValue(TunerContext context, double value);

    protected abstract String manualInstructions();

    protected abstract String manualTelemetryLabel();

    protected abstract void reportAutomaticResult(TunerContext context);

    protected abstract TuningPhase nextPhase(TunerContext context);
}
