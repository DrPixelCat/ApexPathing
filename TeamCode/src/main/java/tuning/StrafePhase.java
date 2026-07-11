package tuning;

import controllers.PDSController.PDSCoefficients;

public class StrafePhase extends DrivePhase {
    public StrafePhase(TuneContext context) {
        super(context, TuneAxis.STRAFE);
    }

    @Override
    protected void saveResult(FeedforwardCalc.Result result, double safeVelocity,
                               double safeAcceleration) {
        context.constants.lateralCoeffs = makeGains(result);
        context.constants.strafeVelLimitIn = safeVelocity;
        context.constants.strafeAccelLimitIn = safeAcceleration;
    }

    @Override
    protected TuneValue[] values() {
        PDSCoefficients pds = context.constants.lateralCoeffs;
        return new TuneValue[]{
                new TuneValue("Lateral kP", () -> pds.kP, value -> pds.kP = value,
                        0.001, 0.0, 2.0),
                new TuneValue("Lateral kD", () -> pds.kD, value -> pds.kD = value,
                        0.0005, 0.0, 2.0),
                new TuneValue("Lateral kS", () -> pds.kS, value -> pds.kS = value,
                        0.002, 0.0, 0.5),
                new TuneValue("Strafe velocity limit", () -> context.constants.strafeVelLimitIn,
                        value -> context.constants.strafeVelLimitIn = value,
                        1.0, 1.0, 150.0),
                new TuneValue("Strafe acceleration limit", () -> context.constants.strafeAccelLimitIn,
                        value -> context.constants.strafeAccelLimitIn = value,
                        2.0, 1.0, 400.0)
        };
    }

    @Override protected PDSCoefficients manualGains() {
        return context.constants.lateralCoeffs;
    }

    @Override
    protected double manualKV() {
        if (context.constants.strafeVelLimitIn > 0.0 &&
                context.constants.forwardVelLimitIn > 0.0) {
            return context.constants.translationalKV *
                    (context.constants.forwardVelLimitIn / context.constants.strafeVelLimitIn);
        }
        return context.constants.translationalKV;
    }

    @Override protected double velocityGain() {
        return context.constants.velocityFeedbackGain;
    }
    @Override protected double speedLimit() {
        return context.constants.strafeVelLimitIn;
    }
}
