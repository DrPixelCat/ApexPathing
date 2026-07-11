package tuning;

import controllers.PDSController.PDSCoefficients;

public class TurnPhase extends DrivePhase {
    public TurnPhase(TuneContext context) {
        super(context, TuneAxis.ANGULAR);
    }

    @Override
    protected void saveResult(FeedforwardCalc.Result result, double safeVelocity, double safeAcceleration) {
        context.constants.angularKV = result.kV;
        context.constants.angularKA = result.kA;
        context.constants.headingCoeffs = makeGains(result);
        context.constants.angularVelocityFeedbackGain = result.velocityGain(0.12);
        context.constants.angularVelLimitRad = safeVelocity;
        context.constants.angularAccelLimitRad = safeAcceleration;
    }

    @Override
    protected TuneValue[] values() {
        PDSCoefficients pds = context.constants.headingCoeffs;
        return new TuneValue[]{
                new TuneValue("Heading kP", () -> pds.kP, value -> pds.kP = value,
                        0.005, 0.0, 5.0),
                new TuneValue("Heading kD", () -> pds.kD, value -> pds.kD = value,
                        0.001, 0.0, 5.0),
                new TuneValue("Heading kS", () -> pds.kS, value -> pds.kS = value,
                        0.002, 0.0, 0.5),
                new TuneValue("Angular kV", () -> context.constants.angularKV,
                        value -> context.constants.angularKV = value,
                        0.002, 0.0, 2.0),
                new TuneValue("Angular kA", () -> context.constants.angularKA,
                        value -> context.constants.angularKA = value,
                        0.001, 0.0, 2.0),
                new TuneValue("Angular velocity feedback",
                        () -> context.constants.angularVelocityFeedbackGain,
                        value -> context.constants.angularVelocityFeedbackGain = value,
                        0.002, 0.0, 5.0),
                new TuneValue("Angular velocity limit",
                        () -> context.constants.angularVelLimitRad,
                        value -> context.constants.angularVelLimitRad = value,
                        0.1, 0.1, 30.0),
                new TuneValue("Angular acceleration limit",
                        () -> context.constants.angularAccelLimitRad,
                        value -> context.constants.angularAccelLimitRad = value,
                        0.2, 0.1, 100.0)
        };
    }

    @Override protected PDSCoefficients manualGains() {
        return context.constants.headingCoeffs;
    }
    @Override protected double manualKV() { return context.constants.angularKV; }
    @Override protected double velocityGain() {
        return context.constants.angularVelocityFeedbackGain;
    }
    @Override protected double speedLimit() {
        return context.constants.angularVelLimitRad;
    }
}
