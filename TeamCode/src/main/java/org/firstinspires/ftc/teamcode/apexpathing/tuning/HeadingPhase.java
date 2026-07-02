package org.firstinspires.ftc.teamcode.apexpathing.tuning;

public class HeadingPhase extends PdsTuningPhase {
    public HeadingPhase() {
        super("HEADING", true);
    }

    @Override
    protected double getP(TunerContext context) {
        return context.headingP;
    }

    @Override
    protected void setP(TunerContext context, double value) {
        context.headingP = value;
    }

    @Override
    protected double getD(TunerContext context) {
        return context.headingD;
    }

    @Override
    protected void setD(TunerContext context, double value) {
        context.headingD = value;
    }

    @Override
    protected double getS(TunerContext context) {
        return context.headingS;
    }

    @Override
    protected void setS(TunerContext context, double value) {
        context.headingS = value;
    }

    @Override
    protected TuningPhase nextPhase(TunerContext context) {
        return null;
    }
}
