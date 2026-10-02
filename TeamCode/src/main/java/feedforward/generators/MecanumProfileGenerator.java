package feedforward.generators;

import core.FollowerConstants;
import core.HolonomicFeedforward;
import core.MotionCharacterization.Axis;
import drivetrains.Mecanum;
import drivetrains.Mecanum.DirectionalLut.DirectionalKinematics;
import geometry.Angle;
import geometry.PathPoint;
import geometry.Vector;
import paths.movements.Path;

/** Mecanum physical limits with the same signed body-axis powers as the follower. */
public class MecanumProfileGenerator extends BaseProfileGenerator {
    private final Mecanum.DirectionalLut limitCalculator;
    public MecanumProfileGenerator(FollowerConstants constants, Path path) {
        super(constants, path);
        limitCalculator = limits(constants);
    }
    public MecanumProfileGenerator(FollowerConstants constants, Path path, double powerLimit) {
        super(constants, path, powerLimit);
        limitCalculator = limits(constants);
    }
    private static Mecanum.DirectionalLut limits(FollowerConstants c) {
        return new Mecanum.DirectionalLut(c.forwardVelLimitIn, c.forwardAccelLimitIn, c.strafeVelLimitIn, c.strafeAccelLimitIn);
    }
    private Angle heading(PathPoint point, Path path) {
        return path.getInterpolator().getHeadingTarg(point.getDistanceToEndIn(), point.getFirstDerivative(),
                path.getParametricPath().getFirstDerivative(1));
    }
    @Override protected double calculateMaxTangentialVelocity(PathPoint point, Path path, double maxAngVel, double maxAngAccel) {
        Angle heading = heading(point, path);
        DirectionalKinematics directional = limitCalculator.getKinematics(point.getFirstDerivative(), heading);
        double ceiling = cornerVelocityLimit(directional.maxVel, point.getSignedCurvature());
        Vector body = point.getFirstDerivative().normalize().rotate(heading.times(-1));
        if (Math.abs(body.getX().getIn()) > 1e-12) {
            ceiling = Math.min(ceiling, calibratedVelocityLimit(Axis.FORWARD, Double.MAX_VALUE) / Math.abs(body.getX().getIn()));
        }
        if (Math.abs(body.getY().getIn()) > 1e-12) {
            ceiling = Math.min(ceiling, calibratedVelocityLimit(Axis.STRAFE, Double.MAX_VALUE) / Math.abs(body.getY().getIn()));
        }
        double prime = path.getInterpolator().getHeadingFirstDerivative(point.getDistanceToEndIn(), point.getSignedCurvature(),
                path.getParametricPath().getFirstDerivative(1));
        if (Math.abs(prime) > 1e-12) {
            ceiling = Math.min(ceiling, Math.min(maxAngVel, calibratedVelocityLimit(Axis.ANGULAR, constants.angularVelLimitRad)) / Math.abs(prime));
        }
        double normalCost = Math.abs(point.getSignedCurvature() * constants.kCentripetal);
        if (normalCost > 1e-12) { ceiling = Math.min(ceiling, Math.sqrt(HolonomicFeedforward.MAX_CENTRIPETAL_POWER / normalCost)); }
        return ceiling;
    }
    @Override protected double getPhysicalTangentialAccel(PathPoint point, Path path) {
        return limitCalculator.getKinematics(point.getFirstDerivative(), heading(point, path)).maxAccel;
    }
    @Override protected PowerEvaluator powerEvaluator(Path path, PathPoint point) {
        Vector end = path.getParametricPath().getFirstDerivative(1);
        Angle targetHeading = path.getInterpolator().getHeadingTarg(point.getDistanceToEndIn(), point.getFirstDerivative(), end);
        Vector body = point.getFirstDerivative().normalize().rotate(targetHeading.times(-1));
        double bx = body.getX().getIn(), by = body.getY().getIn();
        double normal = point.getSignedCurvature() * constants.kCentripetal;
        return (v, a, omega, alpha, normalVelocity) -> {
            double x, y;
            if (constants.motionCharacterization == null) {
                double tangent = v * profileKV() + a * constants.translationalKA + signedStatic(v, a, constants.translationalFeedforwardKS);
                x = bx * tangent - by * normal * normalVelocity * normalVelocity;
                y = by * tangent + bx * normal * normalVelocity * normalVelocity;
            } else {
                Vector tangent = HolonomicFeedforward.translation(constants, body, v, a, profileKV());
                x = tangent.getX().getIn() - by * normal * normalVelocity * normalVelocity;
                y = tangent.getY().getIn() + bx * normal * normalVelocity * normalVelocity;
            }
            double turn = HolonomicFeedforward.heading(constants, omega, alpha);
            return Math.abs(x) + Math.abs(y) + Math.abs(turn);
        };
    }
    @Override protected void evaluatePoint(Path path, PathPoint prev, PathPoint current, double vPrev,
                                            double velocity, double acceleration, EvaluationResult result) {
        Vector end = path.getParametricPath().getFirstDerivative(1);
        Vector body = current.getFirstDerivative().normalize().rotate(heading(current, path).times(-1));
        double prime = path.getInterpolator().getHeadingFirstDerivative(current.getDistanceToEndIn(), current.getSignedCurvature(), end);
        double second = path.getInterpolator().getHeadingSecondDerivative(current.getDistanceToEndIn(), current.getCurvatureDerivative(), end);
        Vector tangentPower = HolonomicFeedforward.translation(constants, body, velocity, acceleration, profileKV());
        Vector normalPower = HolonomicFeedforward.centripetal(body, velocity, current.getSignedCurvature(), constants.kCentripetal);
        double turn = HolonomicFeedforward.heading(constants, prime * velocity, second * velocity * velocity + prime * acceleration);
        result.pForward = HolonomicFeedforward.mecanumDemand(tangentPower, 0);
        result.pLateral = HolonomicFeedforward.mecanumDemand(normalPower, 0);
        result.pHeading = Math.abs(turn);
        result.totalPower = result.maxUtilization = HolonomicFeedforward.mecanumDemand(tangentPower.plus(normalPower), turn);
    }
}
