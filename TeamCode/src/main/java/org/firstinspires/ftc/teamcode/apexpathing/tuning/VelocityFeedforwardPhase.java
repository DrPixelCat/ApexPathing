package org.firstinspires.ftc.teamcode.apexpathing.tuning;

public class VelocityFeedforwardPhase extends TuningPhase {
    public VelocityFeedforwardPhase() {
        super("VELOCITY_FF");
    }

    @Override
    protected void beginAutomatic(TunerContext context) {
    }

    @Override
    protected boolean updateAutomatic(TunerContext context) throws InterruptedException {
        context.follower().teleOpDrive(0, 1.0, 0);
        context.sleep(1500);

        double maxVel = Math.abs(context.follower().getVelocity().getPos().getX().getIn());
        context.velocityFF = 1.0 / maxVel;

        context.stopDrive();
        context.sleep(500);
        return true;
    }

    @Override
    protected double currentManualValue(TunerContext context) {
        return context.velocityFF;
    }

    @Override
    protected void applyManualValue(TunerContext context, double value) {
        context.velocityFF = value;
    }

    @Override
    protected String manualInstructions() {
        return "Tune kSGuess (kV) via Config Panels. Drive to test.";
    }

    @Override
    protected String manualTelemetryLabel() {
        return "Current kV";
    }

    @Override
    protected void reportAutomaticResult(TunerContext context) {
        context.telemetry().addData("Velocity FF (kV)", context.velocityFF);
    }

    @Override
    protected String rerunExecutionPrompt() {
        return "B - Execute rerun";
    }

    @Override
    protected void onAccepted(TunerContext context) {
        context.maxLateralAccel = 50.0;
    }

    @Override
    protected TuningPhase nextPhase(TunerContext context) {
        return new LateralAccelerationPhase();
    }
}
