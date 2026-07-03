package feedforward.holonomic.mecanum;

import core.FollowerConstants;
import drivetrains.Mecanum;
import drivetrains.Mecanum.MecanumDirectionalLut.DirectionalKinematics;
import feedforward.BaseProfileGenerator;
import geometry.Angle;
import geometry.PathPoint;
import geometry.Vector;
import paths.movements.Path;
import util.DistUnit;

/**
 * Generates holonomic profiles for mecanum drives, including direction-specific limits.
 */
public class MecanumProfileGenerator extends BaseProfileGenerator {
    private static final double EPSILON = 1e-6;
    private static final int VELOCITY_SEARCH_ITERATIONS = 10;

    private final FollowerConstants config;
    private final Mecanum.MecanumDirectionalLut limitCalculator;

    public MecanumProfileGenerator(FollowerConstants config, Path path) {
        super.path = path;
        this.config = config;
        this.limitCalculator = new Mecanum.MecanumDirectionalLut(
                config.forwardVelocityLimit.getIn(),
                config.forwardAccelerationLimit.getIn(),
                config.strafeVelocityLimit.getIn(),
                config.strafeAccelerationLimit.getIn()
        );
    }

    @Override
    protected double calculateMaxTangentialVelocity(PathPoint point,
                                                    Path path, double maxAngVel,
                                                    double maxAngAccel) {
        double s = point.getDistanceToEnd_in();
        Vector tangent = point.getFirstDerivative();
        double kappa = point.getSignedCurvature();
        double dKappa = point.getCurvatureDerivative();
        Vector finalTangent = path.getParametricPath().getFirstDerivative(1.0);

        Angle headingAtPoint = path.getInterpolator().getHeadingTarg(s, tangent, finalTangent);
        double fPrime = path.getInterpolator().getHeadingFirstDerivative(s, kappa, finalTangent);
        double fDoublePrime = path.getInterpolator().getHeadingSecondDerivative(s, dKappa,
                finalTangent);

        DirectionalKinematics dirK = limitCalculator.getKinematics(tangent, headingAtPoint);
        DirectionalKinematics normalK = limitCalculator.getKinematics(getNormalVector(point),
                headingAtPoint);
        double maxPhysicalVel = dirK.maxVel;

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

            if (evaluatePower(mid_v, 0.0, kappa, fPrime, fDoublePrime, dirK, normalK) > 1.0) {
                max_v = mid_v;
            } else {
                min_v = mid_v;
            }
        }

        return Math.min(min_v, maxPhysicalVel);
    }

    private double evaluatePower(double v, double a, double kappa, double fPrime,
                                 double fDoublePrime, DirectionalKinematics tangentKinematics,
                                 DirectionalKinematics normalKinematics) {
        double boostedKV = config.translationalKV * tangentKinematics.velMultiplier;
        double boostedKA = config.translationalKA * tangentKinematics.accelMultiplier;
        double transPower = (v * boostedKV)
                + (a * boostedKA)
                + signedStatic(v, a, config.translationalCoeffs.kS);

        double latPower = Math.abs(
                (v * v * kappa) * config.Kcentripetal * normalKinematics.accelMultiplier
        );

        double omega = fPrime * v;
        double alpha = (fDoublePrime * (v * v)) + (fPrime * a);
        double headingKs = signedStatic(omega, alpha, config.headingCoeffs.kS);
        double rotPower =
                Math.abs((omega * config.angularKV) + (alpha * config.angularKA) + headingKs);

        return transPower + latPower + rotPower;
    }

    @Override
    protected void evaluatePoint(Path path, PathPoint prev, PathPoint current, double v_prev,
                                 double v, double a_t, EvaluationResult outResult) {
        double s = current.getDistanceToEnd_in();
        double kappa = current.getSignedCurvature();
        double dKappa = current.getCurvatureDerivative();
        Vector finalTangent = path.getParametricPath().getFirstDerivative(1.0);
        Angle robotHeading = path.getInterpolator().getHeadingTarg(
                s, current.getFirstDerivative(), finalTangent
        );

        DirectionalKinematics dirK = limitCalculator.getKinematics(current.getFirstDerivative(),
                robotHeading);
        DirectionalKinematics normalK = limitCalculator.getKinematics(getNormalVector(current),
                robotHeading);

        double fPrime = path.getInterpolator().getHeadingFirstDerivative(s, kappa, finalTangent);
        double fDoublePrime = path.getInterpolator().getHeadingSecondDerivative(s, dKappa,
                finalTangent);

        double omega = fPrime * v;
        double alpha = (fDoublePrime * (v * v)) + (fPrime * a_t);

        double pForward = (v * config.translationalKV * dirK.velMultiplier)
                + (a_t * config.translationalKA * dirK.accelMultiplier)
                + signedStatic(v, a_t, config.translationalCoeffs.kS);

        // Centripetal correction is a sideways force, so mecanum inefficiency applies here too.
        double pLateral = (v * v * kappa) * config.Kcentripetal * normalK.accelMultiplier;

        double headingKs = signedStatic(omega, alpha, config.headingCoeffs.kS);
        double pHeading = (omega * config.angularKV) + (alpha * config.angularKA) + headingKs;

        outResult.pForward = Math.abs(pForward);
        outResult.pLateral = Math.abs(pLateral);
        outResult.pHeading = Math.abs(pHeading);

        outResult.totalPower = outResult.pForward + outResult.pLateral + outResult.pHeading;
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
        Vector tangent = point.getFirstDerivative();
        double kappa = point.getSignedCurvature();
        double dKappa = point.getCurvatureDerivative();
        Vector finalTangent = path.getParametricPath().getFirstDerivative(1.0);

        Angle robotHeading = path.getInterpolator().getHeadingTarg(s, tangent, finalTangent);
        DirectionalKinematics dirK = limitCalculator.getKinematics(tangent, robotHeading);

        double fPrime = path.getInterpolator().getHeadingFirstDerivative(s, kappa, finalTangent);
        double fDoublePrime = path.getInterpolator().getHeadingSecondDerivative(s, dKappa,
                finalTangent);

        double maxPhysicalAccel = dirK.maxAccel;
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

    private Vector getNormalVector(PathPoint point) {
        double kappa = point.getSignedCurvature();
        if (Math.abs(kappa) < EPSILON) {
            return Vector.zero();
        }

        double vx = point.getFirstDerivative().getX().getIn();
        double vy = point.getFirstDerivative().getY().getIn();
        if (kappa < 0.0) {
            return Vector.of(vy, -vx, DistUnit.IN);
        }
        return Vector.of(-vy, vx, DistUnit.IN);
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
