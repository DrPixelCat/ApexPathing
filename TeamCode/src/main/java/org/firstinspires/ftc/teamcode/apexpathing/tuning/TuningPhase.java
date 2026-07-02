package org.firstinspires.ftc.teamcode.apexpathing.tuning;

public abstract class TuningPhase {
    public enum Phase {
        HEADING,
        TRANSLATION,
        VELOCITY_FF,
        LATERAL_ACCEL
    }
    protected final TunerContext context;
    private final Phase phase;
    private boolean awaitingStart = true;
    private boolean confirming;
    private boolean readyToRerun;
    private boolean manualMode;

    protected TuningPhase(TunerContext context, Phase phase) {
        this.context = context;
        this.phase = phase;
    }

    public final Phase phase() {
        return phase;
    }

    public final boolean isRunningAutomatic() {
        return !awaitingStart && !confirming;
    }

    public void onResume() {
    }

    public final TuningPhase update(boolean aPressed, boolean bPressed) throws InterruptedException {
        if (awaitingStart) {
            updateAwaitingStart(aPressed, bPressed);
        } else if (confirming) {
            return updateConfirmation(aPressed, bPressed);
        } else if (updateAutomatic()) {
            context.updateFollowerConfig();
            readyToRerun = false;
            confirming = true;
        }

        return this;
    }

    private void updateAwaitingStart(boolean aPressed, boolean bPressed) throws InterruptedException {
        context.telemetry().addLine(phase + " phase initialized");
        context.telemetry().addLine("A - Toggle mode");
        context.telemetry().addLine("B - Start tuning");
        context.telemetry().addData("Selected Mode", modeName());

        if (aPressed) {
            toggleMode();
        } else if (bPressed) {
            if (manualMode) {
                context.setManualGuess(currentManualValue());
                readyToRerun = false;
                awaitingStart = false;
                confirming = true;
            } else {
                startAutomatic();
            }
        }
    }

    private TuningPhase updateConfirmation(boolean aPressed, boolean bPressed) throws InterruptedException {
        context.telemetry().addData("Current Phase", phase);
        context.telemetry().addData("Robot Pose", context.follower().getPose().toString());

        if (!readyToRerun) {
            context.telemetry().addLine(savePrompt());
            context.telemetry().addLine(rerunPrompt());

            if (manualMode) {
                context.telemetry().addLine("--- MANUAL TUNING ---");
                context.telemetry().addLine(manualInstructions());
                applyManualValue(context.manualGuess());
                context.updateFollowerConfig();
                context.telemetry().addData(manualTelemetryLabel(), currentManualValue());
            } else {
                reportAutomaticResult();
            }

            if (aPressed) {
                onAccepted();
                return nextPhase();
            } else if (bPressed) {
                readyToRerun = true;
            }
        } else {
            context.telemetry().addLine("A - Toggle mode");
            context.telemetry().addLine(rerunExecutionPrompt());
            context.telemetry().addData("Selected Mode", modeName());

            if (aPressed) {
                toggleMode();
                if (manualMode) {
                    context.setManualGuess(currentManualValue());
                }
            } else if (bPressed) {
                readyToRerun = false;
                if (manualMode) {
                    confirming = true;
                } else {
                    startAutomatic();
                }
            }
        }

        context.driveWithGamepad();
        return this;
    }

    private void startAutomatic() throws InterruptedException {
        readyToRerun = false;
        awaitingStart = false;
        confirming = false;
        beginAutomatic();
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

    protected void onAccepted() {
    }

    protected abstract void beginAutomatic() throws InterruptedException;

    protected abstract boolean updateAutomatic() throws InterruptedException;

    protected abstract double currentManualValue();

    protected abstract void applyManualValue(double value);

    protected abstract String manualInstructions();

    protected abstract String manualTelemetryLabel();

    protected abstract void reportAutomaticResult();

    protected abstract TuningPhase nextPhase();

    protected static class PDSRoutine {
        private final TunerContext context;
        private final boolean angular;
        private KsSearchRoutine ksSearch;
        private StepResponseRoutine stepResponse;

        protected PDSRoutine(TunerContext context, boolean angular) {
            this.context = context;
            this.angular = angular;
        }

        protected void begin() {
            ksSearch = new KsSearchRoutine(context, angular);
            stepResponse = null;
        }

        protected void resume() {
            if (stepResponse != null) {
                stepResponse.resume();
            }
        }

        protected boolean update() throws InterruptedException {
            if (ksSearch != null) {
                ksSearch.update();
                if (!ksSearch.isComplete()) {
                    return false;
                }

                onStaticFeedforwardFound(ksSearch.result());
                stepResponse = new StepResponseRoutine(context, angular);
                stepResponse.start();
                ksSearch = null;
                return false;
            }

            if (stepResponse == null || !stepResponse.update()) {
                return false;
            }

            applyStepResult(stepResponse.result());
            return true;
        }

        protected void onStaticFeedforwardFound(double value) {
        }

        protected void onStepResponseFound(double kP, double kD) {
        }

        private void applyStepResult(StepResult result) {
            double kP = result.kP > 0 ? result.kP : 0.01;
            double kD = result.kD > 0 ? result.kD : 0.001;

            if (!angular) {
                kP = Double.isFinite(result.kP) && result.kP > 0 ? result.kP : 0.01;
                kD = Double.isFinite(result.kD) && result.kD > 0 ? result.kD : 0.001;
            }

            onStepResponseFound(kP, kD);
        }
    }

    private static class KsSearchRoutine {
        private final TunerContext context;
        private final boolean angular;
        private double max = 0.2;
        private double min = 0.0;
        private double guess = 0.0;
        private double lastGuess = -1.0;
        private double maxDeviation;
        private boolean complete;

        KsSearchRoutine(TunerContext context, boolean angular) {
            this.context = context;
            this.angular = angular;
        }

        void update() throws InterruptedException {
            if (Math.abs(lastGuess - guess) <= 0.01) {
                complete = true;
                return;
            }

            context.resetPose();
            context.follower().update();
            guess = (max + min) / 2.0;
            maxDeviation = 0.0;
            context.timer.reset();

            while (context.isActive() && context.timer.time(java.util.concurrent.TimeUnit.MILLISECONDS) < 500) {
                context.follower().update();
                double position = angular
                        ? context.follower().getPose().getHeading().getRad()
                        : context.follower().getPose().getPos().getX().getIn();
                maxDeviation = Math.max(Math.abs(position), maxDeviation);

                if (angular) {
                    context.follower().teleOpDrive(0, 0, guess);
                } else {
                    context.follower().teleOpDrive(guess, 0, 0);
                }
            }

            if (maxDeviation > 0.025) {
                max = guess;
            } else {
                min = guess;
            }

            lastGuess = guess;
            context.stopDrive();
            context.sleep(500);
        }

        boolean isComplete() {
            return complete;
        }

        double result() {
            return guess;
        }
    }

    private static class StepResponseRoutine {
        private final TunerContext context;
        private final boolean angular;
        private double maxAccel;
        private double lastVel;
        private double lastTime;
        private double startTime;
        private double timeStamp;
        private double velAtTimeStamp;
        private StepResult result;

        StepResponseRoutine(TunerContext context, boolean angular) {
            this.context = context;
            this.angular = angular;
        }

        void start() {
            maxAccel = 0;
            lastVel = 0;
            timeStamp = 0;
            velAtTimeStamp = 0;
            lastTime = System.nanoTime();
            startTime = System.nanoTime();
            context.timer.reset();
            context.resetPose();
        }

        void resume() {
            lastTime = System.nanoTime();
            context.timer.reset();
        }

        boolean update() throws InterruptedException {
            if (context.timer.time(java.util.concurrent.TimeUnit.MILLISECONDS) >= 2000) {
                context.stopDrive();
                context.sleep(500);

                double responseDelay = timeStamp - (velAtTimeStamp / maxAccel);
                result = new StepResult(
                        1.2 / (responseDelay * maxAccel),
                        0.6 / maxAccel
                );
                return true;
            }

            context.follower().update();
            double currentVel = angular
                    ? context.follower().getVelocity().getHeading().getRad()
                    : context.follower().getVelocity().getPos().getX().getIn();

            double now = System.nanoTime();
            double deltaT = (now - lastTime) / 1e9;
            double deltaV = currentVel - lastVel;
            double accel = deltaT > 1e-6 ? deltaV / deltaT : 0.0;

            if (accel > maxAccel) {
                maxAccel = accel;
                timeStamp = (now - startTime) / 1e9;
                velAtTimeStamp = currentVel;
            }

            lastVel = currentVel;
            lastTime = now;

            if (angular) {
                context.follower().teleOpDrive(0, 0, 1.0);
            } else {
                context.follower().teleOpDrive(1.0, 0, 0);
            }

            return false;
        }

        StepResult result() {
            return result;
        }
    }

    private static class StepResult {
        final double kP;
        final double kD;

        StepResult(double kP, double kD) {
            this.kP = kP;
            this.kD = kD;
        }
    }
}
