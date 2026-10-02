package feedforward.generators;

import core.FollowerConstants;
import core.HolonomicFeedforward;
import core.MotionCharacterization.Axis;
import geometry.Angle;
import geometry.PathPoint;
import geometry.Vector;
import paths.movements.Path;

/** Isotropic translation with coupled heading and signed acceleration constraints. */
public class SwerveProfileGenerator extends BaseProfileGenerator {
    public SwerveProfileGenerator(FollowerConstants constants, Path path) { super(constants, path); }
    public SwerveProfileGenerator(FollowerConstants constants, Path path, double powerLimit) { super(constants, path, powerLimit); }
    @Override protected double calculateMaxTangentialVelocity(PathPoint point, Path path, double maxAngVel, double maxAngAccel) {
        double ceiling = cornerVelocityLimit(calibratedVelocityLimit(Axis.FORWARD, constants.forwardVelLimitIn), point.getSignedCurvature());
        if (constants.motionCharacterization != null) { ceiling = Math.min(ceiling, calibratedVelocityLimit(Axis.STRAFE, Double.MAX_VALUE)); }
        double prime = path.getInterpolator().getHeadingFirstDerivative(point.getDistanceToEndIn(), point.getSignedCurvature(), path.getParametricPath().getFirstDerivative(1));
        if (Math.abs(prime) > 1e-12) {
            ceiling = Math.min(ceiling, Math.min(maxAngVel, calibratedVelocityLimit(Axis.ANGULAR, constants.angularVelLimitRad)) / Math.abs(prime));
        }
        double normalCost = Math.abs(point.getSignedCurvature() * constants.kCentripetal);
        if (normalCost > 1e-12) { ceiling = Math.min(ceiling, Math.sqrt(HolonomicFeedforward.MAX_CENTRIPETAL_POWER / normalCost)); }
        return ceiling;
    }
    @Override protected double getPhysicalTangentialAccel(PathPoint point, Path path) { return constants.forwardAccelLimitIn; }
    @Override protected void evaluatePoint(Path path, PathPoint prev, PathPoint current, double vPrev,
                                            double velocity, double acceleration, EvaluationResult result) {
        Vector end = path.getParametricPath().getFirstDerivative(1);
        Angle heading = path.getInterpolator().getHeadingTarg(current.getDistanceToEndIn(), current.getFirstDerivative(), end);
        Vector body = current.getFirstDerivative().normalize().rotate(heading.times(-1));
        double prime = path.getInterpolator().getHeadingFirstDerivative(current.getDistanceToEndIn(), current.getSignedCurvature(), end);
        double second = path.getInterpolator().getHeadingSecondDerivative(current.getDistanceToEndIn(), current.getCurvatureDerivative(), end);
        Vector tangent = HolonomicFeedforward.translation(constants, body, velocity, acceleration, profileKV());
        Vector normal = HolonomicFeedforward.centripetal(body, velocity, current.getSignedCurvature(), constants.kCentripetal);
        double turn = HolonomicFeedforward.heading(constants, prime * velocity, second * velocity * velocity + prime * acceleration);
        result.pForward = tangent.getMag().getIn(); result.pLateral = normal.getMag().getIn(); result.pHeading = Math.abs(turn);
        result.totalPower = result.maxUtilization = tangent.plus(normal).getMag().getIn() + Math.abs(turn);
    }
}
