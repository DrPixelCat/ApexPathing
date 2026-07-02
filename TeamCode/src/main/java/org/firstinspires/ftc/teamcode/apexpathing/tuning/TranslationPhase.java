package org.firstinspires.ftc.teamcode.apexpathing.tuning;

public class TranslationPhase extends TuningPhase {
    private final PDSRoutine pdsRoutine;

    public TranslationPhase(TunerContext context) {
        super(context, Phase.TRANSLATION);
        pdsRoutine = new PDSRoutine(context, false) {
            @Override
            protected void onStaticFeedforwardFound(double value) {
                context.translationS = value;
            }

            @Override
            protected void onStepResponseFound(double kP, double kD) {
                context.translationP = kP;
                context.translationD = kD;
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
        return context.translationS;
    }

    @Override
    protected void applyManualValue(double value) {
        context.translationS = value;
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
        context.telemetry().addData("Translation P", context.translationP);
        context.telemetry().addData("Translation D", context.translationD);
        context.telemetry().addData("Translation S", context.translationS);
    }

    @Override
    protected TuningPhase nextPhase() {
        return new VelocityFeedforwardPhase(context);
    }
}
