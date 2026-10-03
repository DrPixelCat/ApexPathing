package feedforward.generators;

import core.FollowerConstants;
import core.MotionCharacterization.Axis;
import geometry.PathPoint;
import geometry.Vector;
import paths.movements.Path;

/** Differential-drive reference powers with physical traction and heading limits. */
public class TankProfileGenerator extends BaseProfileGenerator {
    public TankProfileGenerator(FollowerConstants constants, Path path) { super(constants, path); }
    public TankProfileGenerator(FollowerConstants constants, Path path, double powerLimit) { super(constants, path, powerLimit); }
    static double tractionLimitedVelocity(double velocity, double curvature, double acceleration) {
        return !Double.isFinite(acceleration) || acceleration <= 0 || Math.abs(curvature) <= EPSILON
                ? velocity : Math.min(velocity, Math.sqrt(acceleration / Math.abs(curvature)));
    }
    @Override protected double calculateMaxTangentialVelocity(PathPoint point, Path path, double maxAngVel, double maxAngAccel) {
        double ceiling = cornerVelocityLimit(calibratedVelocityLimit(Axis.FORWARD, constants.forwardVelLimitIn), point.getSignedCurvature());
        ceiling = tractionLimitedVelocity(ceiling, point.getSignedCurvature(), constants.maxCentripetalAccelIn);
        double prime = path.getInterpolator().getHeadingFirstDerivative(point.getDistanceToEndIn(), point.getSignedCurvature(), path.getParametricPath().getFirstDerivative(1));
        if (Math.abs(prime) > 1e-12) {
            ceiling = Math.min(ceiling, Math.min(maxAngVel, calibratedVelocityLimit(Axis.ANGULAR, constants.angularVelLimitRad)) / Math.abs(prime));
        }
        return ceiling;
    }
    @Override protected double getPhysicalTangentialAccel(PathPoint point, Path path) { return constants.forwardAccelLimitIn; }
    @Override protected PowerEvaluator powerEvaluator(Path path, PathPoint point) {
        return new PowerEvaluator() {
            private final double[] coefficients = new double[6];
            private final AccelerationInterval interval = new AccelerationInterval();
            @Override public double utilization(double v, double a, double omega, double alpha, double normalVelocity) {
            double forward = v * profileKV() + a * constants.translationalKA + signedStatic(v, a, constants.translationalFeedforwardKS);
            forward = constants.characterizedAxisDemand(Axis.FORWARD, v, a, forward);
            double turn = constants.characterizedPower(Axis.ANGULAR, omega, alpha, omega * constants.angularKV + alpha * constants.angularKA);
            return Math.abs(forward) + Math.abs(turn);
            }
            @Override public double maximumAcceleration(double v, double prime, double second, double min, double max, double limit) {
                if (core.HolonomicFeedforward.motionSign(v, min) != core.HolonomicFeedforward.motionSign(v, max)) {
                    return Double.NEGATIVE_INFINITY;
                }
                double forward = v * profileKV() + signedStatic(v, min, constants.translationalFeedforwardKS);
                double omega = prime * v, baseAlpha = second * v * v;
                double turn = omega * constants.angularKV + baseAlpha * constants.angularKA;
                if (!linearAxisPower(Axis.FORWARD, v, 0, 1, forward, constants.translationalKA, min, max, true, coefficients, 0)
                        || !linearAxisPower(Axis.FORWARD, -v, 0, -1, -forward, -constants.translationalKA, min, max, true, coefficients, 2)
                        || !linearAxisPower(Axis.ANGULAR, omega, baseAlpha, prime, turn, prime * constants.angularKA,
                        min, max, true, coefficients, 4)) { return Double.NEGATIVE_INFINITY; }
                interval.reset(min, max);
                interval.restrictL1(coefficients[0], coefficients[1], coefficients[4], coefficients[5], 0, 0, limit);
                interval.restrictL1(coefficients[2], coefficients[3], coefficients[4], coefficients[5], 0, 0, limit);
                return interval.maximum();
            }
        };
    }
    @Override protected void evaluatePoint(Path path, PathPoint prev, PathPoint current, double vPrev,
                                            double velocity, double acceleration, EvaluationResult result) {
        Vector end = path.getParametricPath().getFirstDerivative(1);
        double prime = path.getInterpolator().getHeadingFirstDerivative(current.getDistanceToEndIn(), current.getSignedCurvature(), end);
        double second = path.getInterpolator().getHeadingSecondDerivative(current.getDistanceToEndIn(), current.getCurvatureDerivative(), end);
        double forward = velocity * profileKV() + acceleration * constants.getTranslationalKA(velocity, acceleration)
                + signedStatic(velocity, acceleration, constants.translationalFeedforwardKS);
        forward = constants.characterizedAxisDemand(Axis.FORWARD, velocity, acceleration, forward);
        double omega = prime * velocity, alpha = second * velocity * velocity + prime * acceleration;
        double heading = constants.characterizedPower(Axis.ANGULAR, omega, alpha, omega * constants.angularKV + alpha * constants.angularKA);
        result.pForward = Math.abs(forward); result.pLateral = 0; result.pHeading = Math.abs(heading);
        result.totalPower = result.maxUtilization = Math.abs(forward) + Math.abs(heading);
    }
}
