package tuning;

import controllers.PDSController.PDSCoefficients;

public class ForwardPhase extends DrivePhase {
    public ForwardPhase(TuneContext context) {
        super(context, TuneAxis.FORWARD);
    }

    @Override
    protected void saveResult(FeedforwardCalc.Result result, double safeVelocity,
                               double safeAcceleration) {
        context.constants.translationalKV = result.kV;
        context.constants.translationalKA = result.kA;
        context.constants.translationalCoeffs = makeGains(result);
        context.constants.velocityFeedbackGain = result.velocityGain(0.15);
        context.constants.forwardVelLimitIn = safeVelocity;
        context.constants.forwardAccelLimitIn = safeAcceleration;
        context.constants.Kcentripetal = result.kA;
    }

    @Override
    protected TuneValue[] values() {
        PDSCoefficients pds = context.constants.translationalCoeffs;
        return new TuneValue[]{
                new TuneValue("Position kP", () -> pds.kP, value -> pds.kP = value,
                        0.001, 0.0, 2.0),
                new TuneValue("Position kD", () -> pds.kD, value -> pds.kD = value,
                        0.0005, 0.0, 2.0),
                new TuneValue("Forward kS", () -> pds.kS, value -> pds.kS = value,
                        0.002, 0.0, 0.5),
                new TuneValue("Forward kV", () -> context.constants.translationalKV,
                        value -> context.constants.translationalKV = value,
                        0.0005, 0.0, 1.0),
                new TuneValue("Forward kA", () -> context.constants.translationalKA,
                        value -> context.constants.translationalKA = value,
                        0.0005, 0.0, 1.0),
                new TuneValue("Velocity feedback", () -> context.constants.velocityFeedbackGain,
                        value -> context.constants.velocityFeedbackGain = value,
                        0.001, 0.0, 2.0),
                new TuneValue("Forward velocity limit", () -> context.constants.forwardVelLimitIn,
                        value -> context.constants.forwardVelLimitIn = value,
                        1.0, 1.0, 150.0),
                new TuneValue("Forward acceleration limit", () -> context.constants.forwardAccelLimitIn,
                        value -> context.constants.forwardAccelLimitIn = value,
                        2.0, 1.0, 400.0),
                new TuneValue("Centripetal kA", () -> context.constants.Kcentripetal,
                        value -> context.constants.Kcentripetal = value,
                        0.0005, 0.0, 1.0)
        };
    }

    @Override protected PDSCoefficients manualGains() {
        return context.constants.translationalCoeffs;
    }
    @Override protected double manualKV() { return context.constants.translationalKV; }
    @Override protected double velocityGain() {
        return context.constants.velocityFeedbackGain;
    }
    @Override protected double speedLimit() {
        return context.constants.forwardVelLimitIn;
    }
}
