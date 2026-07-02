package org.firstinspires.ftc.teamcode.apexpathing.tuning;

public class HeadingPhase extends TuningPhase {
    private final PDSRoutine pdsRoutine;

    public HeadingPhase(TunerContext context) {
        super(context, Phase.HEADING);
        pdsRoutine = new PDSRoutine(context, true) {
            @Override
            protected void onStaticFeedforwardFound(double value) {
                context.headingS = value;
            }

            @Override
            protected void onStepResponseFound(double kP, double kD) {
                context.headingP = kP;
                context.headingD = kD;
            }
        };
    }

    @Override
    public void onResume() {
        pdsRoutine.resume();
    }

    @Override
    protected void beginAutomatic() {
        pdsRoutine.begin();
    }

    @Override
    protected boolean updateAutomatic() throws InterruptedException {
        return pdsRoutine.update();
    }

    @Override
    protected double currentManualValue() {
        return context.headingS;
    }

    @Override
    protected void applyManualValue(double value) {
        context.headingS = value;
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
    protected void reportAutomaticResult() {
        context.telemetry().addData("Heading P", context.headingP);
        context.telemetry().addData("Heading D", context.headingD);
        context.telemetry().addData("Heading S", context.headingS);
    }

    @Override
    protected TuningPhase nextPhase() {
        return new TranslationPhase(context);
    }
}
