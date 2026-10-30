package drivetrains.models;

import follower.FollowerConstants;
import feedforward.MotionPowerCalculator;
import feedforward.MotionCharacterization.Axis;
import paths.geometry.Vector;

/** Differential traction limits and signed forward/turn feedforward. */
public final class TankDriveModel extends DriveModel {
    public TankDriveModel(FollowerConstants constants) { super(constants); }
    @Override public DriveModel withConstants(FollowerConstants constants) { return new TankDriveModel(constants); }
    @Override public boolean cacheSolverSamples() { return true; }
    @Override public boolean isHolonomic() { return false; }
    @Override public double translationDemand(double x, double y) { return Math.abs(x); }
    public static double tractionLimitedVelocity(double velocity, double curvature, double acceleration) {
        return !Double.isFinite(acceleration) || acceleration <= 0 || Math.abs(curvature) <= EPSILON
                ? velocity : Math.min(velocity, Math.sqrt(acceleration / Math.abs(curvature)));
    }
    @Override public double maximumVelocity(Vector bodyDirection, double curvature, double prime, double maxAngVel) {
        double ceiling = cornerVelocityLimit(calibratedVelocityLimit(Axis.FORWARD, constants.forwardVelLimitIn), curvature);
        ceiling = tractionLimitedVelocity(ceiling, curvature, constants.maxCentripetalAccelIn);
        if (Math.abs(prime) > 1e-12) {
            ceiling = Math.min(ceiling, Math.min(maxAngVel, calibratedVelocityLimit(Axis.ANGULAR, constants.angularVelLimitRad)) / Math.abs(prime));
        }
        return ceiling;
    }
    @Override public double maximumTangentialAcceleration(Vector bodyDirection) { return constants.forwardAccelLimitIn; }
    @Override public PowerEvaluator powerEvaluator(Vector bodyDirection, double curvature) {
        return new PowerEvaluator(bodyDirection, curvature) {
            private final double[] coefficients = new double[6];
            private final AccelerationInterval interval = new AccelerationInterval();
            @Override public double maximumAcceleration(double v, double prime, double second, double min, double max, double limit) {
                if (MotionPowerCalculator.motionSign(v, min) != MotionPowerCalculator.motionSign(v, max)) {
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
}
