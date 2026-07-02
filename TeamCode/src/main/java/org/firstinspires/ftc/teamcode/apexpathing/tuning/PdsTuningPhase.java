package org.firstinspires.ftc.teamcode.apexpathing.tuning;

public abstract class PdsTuningPhase extends TuningPhase {
    private final boolean angular;
    private KsSearchRoutine ksSearch;
    private StepResponseRoutine stepResponse;

    protected PdsTuningPhase(String name, boolean angular) {
        super(name);
        this.angular = angular;
    }

    @Override
    public void onResume(TunerContext context) {
        if (stepResponse != null) {
            stepResponse.resume(context);
        }
    }

    @Override
    protected void beginAutomatic(TunerContext context) {
        ksSearch = new KsSearchRoutine(angular);
        stepResponse = null;
    }

    @Override
    protected boolean updateAutomatic(TunerContext context) throws InterruptedException {
        if (ksSearch != null) {
            ksSearch.update(context);
            if (!ksSearch.isComplete()) {
                return false;
            }

            setS(context, ksSearch.result());
            stepResponse = new StepResponseRoutine(angular);
            stepResponse.start(context);
            ksSearch = null;
            return false;
        }

        if (stepResponse == null || !stepResponse.update(context)) {
            return false;
        }

        applyStepResult(context, stepResponse.result());
        return true;
    }

    @Override
    protected double currentManualValue(TunerContext context) {
        return getS(context);
    }

    @Override
    protected void applyManualValue(TunerContext context, double value) {
        setS(context, value);
    }

    @Override
    protected String manualInstructions() {
        return "Tune kSGuess via Config Panels. Drive to test.";
    }

    @Override
    protected String manualTelemetryLabel() {
        return "Current kSGuess";
    }

    @Override
    protected void reportAutomaticResult(TunerContext context) {
        context.telemetry().addData(name() + " P", getP(context));
        context.telemetry().addData(name() + " D", getD(context));
        context.telemetry().addData(name() + " S", getS(context));
    }

    private void applyStepResult(TunerContext context, StepResult result) {
        double kP = result.kP > 0 ? result.kP : 0.01;
        double kD = result.kD > 0 ? result.kD : 0.001;

        if (!angular) {
            kP = Double.isFinite(result.kP) && result.kP > 0 ? result.kP : 0.01;
            kD = Double.isFinite(result.kD) && result.kD > 0 ? result.kD : 0.001;
        }

        setP(context, kP);
        setD(context, kD);
    }

    protected abstract double getP(TunerContext context);

    protected abstract void setP(TunerContext context, double value);

    protected abstract double getD(TunerContext context);

    protected abstract void setD(TunerContext context, double value);

    protected abstract double getS(TunerContext context);

    protected abstract void setS(TunerContext context, double value);

    private static class KsSearchRoutine {
        private final boolean angular;
        private double max = 0.2;
        private double min = 0.0;
        private double guess = 0.0;
        private double lastGuess = -1.0;
        private double maxDeviation;
        private boolean complete;

        KsSearchRoutine(boolean angular) {
            this.angular = angular;
        }

        void update(TunerContext context) throws InterruptedException {
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
        private final boolean angular;
        private double maxAccel;
        private double lastVel;
        private double lastTime;
        private double startTime;
        private double timeStamp;
        private double velAtTimeStamp;
        private StepResult result;

        StepResponseRoutine(boolean angular) {
            this.angular = angular;
        }

        void start(TunerContext context) {
            maxAccel = 0;
            lastVel = 0;
            timeStamp = 0;
            velAtTimeStamp = 0;
            lastTime = System.nanoTime();
            startTime = System.nanoTime();
            context.timer.reset();
            context.resetPose();
        }

        void resume(TunerContext context) {
            lastTime = System.nanoTime();
            context.timer.reset();
        }

        boolean update(TunerContext context) throws InterruptedException {
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
