package feedforward.tank;

import core.FollowerConstants;
import feedforward.BaseProfileGenerator;
import geometry.PathPoint;
import geometry.Vector;
import paths.movements.Path;

/**
 * Generates path profiles for differential/tonk drives. We love tonk drives.
 */
public class TankProfileGenerator extends BaseProfileGenerator {
    private static final double EPSILON = 1e-6;
    private static final int VELOCITY_SEARCH_ITERATIONS = 10;

    private final FollowerConstants config;

    public TankProfileGenerator(FollowerConstants config, Path path) {
        super.path = path;
        this.config = config;
    }

    @Override
    protected double calculateMaxTangentialVelocity(PathPoint point,
                                                    Path path, double maxAngVel,
                                                    double maxAngAccel) {
        double s = point.getDistanceToEnd_in();
        double kappa = point.getSignedCurvature();
        double dKappa = point.getCurvatureDerivative();
        Vector finalTangent = path.getParametricPath().getFirstDerivative(1.0);

        double fPrime = path.getInterpolator().getHeadingFirstDerivative(s, kappa, finalTangent);
        double fDoublePrime = path.getInterpolator().getHeadingSecondDerivative(s, dKappa,
                finalTangent);

        double maxPhysicalVel = config.forwardVelocityLimit.getIn();

        double effectiveAngVelLimit = Math.min(config.angularVelocityLimit.getRad(), maxAngVel);
        double effectiveAngAccelLimit = Math.min(config.angularAccelerationLimit.getRad(),
                maxAngAccel);

        if (Math.abs(fPrime) > EPSILON) {
            double maxVelFromOmega = effectiveAngVelLimit / Math.abs(fPrime);
            maxPhysicalVel = Math.min(maxPhysicalVel, maxVelFromOmega);
        }

        if (Math.abs(fDoublePrime) > EPSILON) {
            double maxVelFromAlpha = Math.sqrt(effectiveAngAccelLimit / Math.abs(fDoublePrime));
            maxPhysicalVel = Math.min(maxPhysicalVel, maxVelFromAlpha);
        }

        double min_v = 0.0;
        double max_v = maxPhysicalVel;

        for (int i = 0; i < VELOCITY_SEARCH_ITERATIONS; i++) {
            double mid_v = (min_v + max_v) / 2.0;

            if (evaluatePower(mid_v, 0.0, fPrime, fDoublePrime) > 1.0) {
                max_v = mid_v;
            } else {
                min_v = mid_v;
            }
        }

        return Math.min(min_v, maxPhysicalVel);
    }

    private double evaluatePower(double v, double a, double fPrime, double fDoublePrime) {
        double transPower = Math.abs(
                (v * config.translationalKV)
                        + (a * config.translationalKA)
                        + signedStatic(v, a, config.translationalCoeffs.kS)
        );

        double omega = fPrime * v;
        double alpha = (fDoublePrime * (v * v)) + (fPrime * a);
        double headingKs = signedStatic(omega, alpha, config.headingCoeffs.kS);

        double rotPower =
                Math.abs((omega * config.angularKV) + (alpha * config.angularKA) + headingKs);

        return transPower + rotPower;
    }

    @Override
    protected void evaluatePoint(Path path, PathPoint prev, PathPoint current, double v_prev,
                                 double v, double a_t, EvaluationResult outResult) {
        double s = current.getDistanceToEnd_in();
        double kappa = current.getSignedCurvature();
        double dKappa = current.getCurvatureDerivative();
        Vector finalTangent = path.getParametricPath().getFirstDerivative(1.0);

        double fPrime = path.getInterpolator().getHeadingFirstDerivative(s, kappa, finalTangent);
        double fDoublePrime = path.getInterpolator().getHeadingSecondDerivative(s, dKappa,
                finalTangent);

        double omega = fPrime * v;
        double alpha = (fDoublePrime * (v * v)) + (fPrime * a_t);

        double pForward =
                (v * config.translationalKV)
                        + (a_t * config.translationalKA)
                        + signedStatic(v, a_t, config.translationalCoeffs.kS);

        double headingKs = signedStatic(omega, alpha, config.headingCoeffs.kS);
        double pHeading = (omega * config.angularKV) + (alpha * config.angularKA) + headingKs;

        outResult.pForward = Math.abs(pForward);
        outResult.pLateral = 0.0;
        outResult.pHeading = Math.abs(pHeading);

        outResult.totalPower = outResult.pForward + outResult.pHeading;
        outResult.maxUtilization = outResult.totalPower;
    }

    @Override
    protected double getMaxTangentialAccel(double currentVel, PathPoint point, Path path,
                                           double maxAngAccel) {
        return calculateAngularLimitedTangentialAccel(currentVel, point, path, maxAngAccel, false);
    }

    @Override
    protected double calculateDynamicMaxAccel(double currentVel, PathPoint point, Path path,
                                              double maxAngAccel) {
        return calculateAngularLimitedTangentialAccel(currentVel, point, path, maxAngAccel, true);
    }

    private double calculateAngularLimitedTangentialAccel(double currentVel, PathPoint point,
                                                          Path path, double maxAngAccel,
                                                          boolean positiveAccel) {
        double s = point.getDistanceToEnd_in();
        double kappa = point.getSignedCurvature();
        double dKappa = point.getCurvatureDerivative();
        Vector finalTangent = path.getParametricPath().getFirstDerivative(1.0);

        double fPrime = path.getInterpolator().getHeadingFirstDerivative(s, kappa, finalTangent);
        double fDoublePrime = path.getInterpolator().getHeadingSecondDerivative(s, dKappa,
                finalTangent);

        double maxPhysicalAccel = config.forwardAccelerationLimit.getIn();
        double effectiveAngAccelLimit = Math.min(config.angularAccelerationLimit.getRad(),
                maxAngAccel);
        if (effectiveAngAccelLimit < EPSILON) {
            return 0.0;
        }

        double alphaBase = fDoublePrime * currentVel * currentVel;
        if (Math.abs(fPrime) < EPSILON) {
            return Math.abs(alphaBase) <= effectiveAngAccelLimit + EPSILON
                    ? maxPhysicalAccel : 0.0;
        }

        double boundA = (-effectiveAngAccelLimit - alphaBase) / fPrime;
        double boundB = (effectiveAngAccelLimit - alphaBase) / fPrime;
        double minAccel = Math.min(boundA, boundB);
        double maxAccel = Math.max(boundA, boundB);

        double angularLimitedAccel = positiveAccel ? maxAccel : -minAccel;
        return Math.min(maxPhysicalAccel, Math.max(0.0, angularLimitedAccel));
    }

    private double signedStatic(double velocity, double accel, double kS) {
        if (Math.abs(velocity) > EPSILON) {
            return Math.signum(velocity) * kS;
        }
        if (Math.abs(accel) > EPSILON) {
            return Math.signum(accel) * kS;
        }
        return 0.0;
    }
}
