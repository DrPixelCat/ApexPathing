package trajectory.generators;

import follower.FollowerConstants;
import feedforward.HolonomicFeedforward;
import feedforward.MecanumWheelModel;
import feedforward.MotionCharacterization.Axis;
import drivetrains.Mecanum;
import drivetrains.Mecanum.DirectionalLut.DirectionalKinematics;
import geometry.Angle;
import geometry.PathPoint;
import geometry.Vector;
import paths.movements.Path;

/** Mecanum physical limits with the same signed body-axis powers as the follower. */
public class MecanumProfileGenerator extends BaseProfileGenerator {
    private Mecanum.DirectionalLut limitCalculator;
    private double cachedForwardVelocity, cachedForwardAcceleration, cachedStrafeVelocity, cachedStrafeAcceleration;

    private Mecanum.DirectionalLut limits() {
        if (limitCalculator == null || cachedForwardVelocity != constants.forwardVelLimitIn
                || cachedForwardAcceleration != constants.forwardAccelLimitIn
                || cachedStrafeVelocity != constants.strafeVelLimitIn
                || cachedStrafeAcceleration != constants.strafeAccelLimitIn) {
            cachedForwardVelocity = constants.forwardVelLimitIn;
            cachedForwardAcceleration = constants.forwardAccelLimitIn;
            cachedStrafeVelocity = constants.strafeVelLimitIn;
            cachedStrafeAcceleration = constants.strafeAccelLimitIn;
            limitCalculator = new Mecanum.DirectionalLut(cachedForwardVelocity, cachedForwardAcceleration,
                    cachedStrafeVelocity, cachedStrafeAcceleration);
        }
        return limitCalculator;
    }
    private final boolean straightGeometry;
    private final MecanumWheelModel.Demand wheelDemand = new MecanumWheelModel.Demand();
    private MecanumWheelModel wheelModel() {
        return constants.motionCharacterization == null ? null : constants.motionCharacterization.mecanumWheelModel;
    }
    // The velocity/geometry box is conservative for a straight path. Rapidly changing
    // curvature requires adaptive interval geometry rather than crossed endpoint states.
    @Override protected boolean enforceIntervalPowerBounds() { return wheelModel()!=null && straightGeometry; }
    private static boolean isStraight(Path path) {
        for(PathPoint point:path.getGeneratedPoints()) if(Math.abs(point.getSignedCurvature())>1e-9) return false;
        return true;
    }
    public MecanumProfileGenerator(FollowerConstants constants, Path path) {
        super(constants, path);
        straightGeometry = isStraight(path);
    }
    public MecanumProfileGenerator(FollowerConstants constants, Path path, double powerLimit) {
        super(constants, path, powerLimit);
        straightGeometry = isStraight(path);
    }
    private Angle heading(PathPoint point, Path path) {
        return path.getInterpolator().getHeadingTarg(point.getDistanceToEndIn(), point.getFirstDerivative(),
                path.getParametricPath().getFirstDerivative(1));
    }
    @Override protected double calculateMaxTangentialVelocity(PathPoint point, Path path, double maxAngVel, double maxAngAccel) {
        Angle heading = heading(point, path);
        DirectionalKinematics directional = limits().getKinematics(point.getFirstDerivative(), heading);
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
        if (wheelModel() != null) {
            double factor = Math.abs(body.getX().getIn()) + Math.abs(body.getY().getIn())
                    + Math.abs(wheelModel().effectiveRadius * prime);
            if (factor > 1e-12) ceiling = Math.min(ceiling, wheelModel().maximumWheelSpeed / factor);
            return ceiling;
        }
        double normalCost = Math.abs(point.getSignedCurvature() * constants.kCentripetal);
        if (normalCost > 1e-12) { ceiling = Math.min(ceiling, Math.sqrt(HolonomicFeedforward.MAX_CENTRIPETAL_POWER / normalCost)); }
        return ceiling;
    }
    @Override protected double getPhysicalTangentialAccel(PathPoint point, Path path) {
        return limits().getKinematics(point.getFirstDerivative(), heading(point, path)).maxAccel;
    }
    @Override protected PowerEvaluator powerEvaluator(Path path, PathPoint point) {
        Vector end = path.getParametricPath().getFirstDerivative(1);
        Angle targetHeading = path.getInterpolator().getHeadingTarg(point.getDistanceToEndIn(), point.getFirstDerivative(), end);
        Vector body = point.getFirstDerivative().normalize().rotate(targetHeading.times(-1));
        double bx = body.getX().getIn(), by = body.getY().getIn();
        double normal = point.getSignedCurvature() * constants.kCentripetal;
        return new PowerEvaluator() {
            private final double[] coefficients = new double[6];
            private final AccelerationInterval interval = new AccelerationInterval();
            private final MecanumWheelModel.Demand wheels = new MecanumWheelModel.Demand();
            @Override public double utilization(double v, double a, double omega, double alpha, double normalVelocity) {
            if (wheelModel() != null) {
                double an = point.getSignedCurvature() * normalVelocity * normalVelocity;
                wheelModel().solve(bx*v, by*v, omega, bx*a-by*an, by*a+bx*an, alpha, wheels);
                return wheels.peak;
            }
            double x, y;
            if (constants.motionCharacterization == null) {
                double tangent = v * profileKV() + a * constants.translationalKA + signedStatic(v, a, constants.translationalFeedforwardKS);
                x = bx * tangent - by * normal * normalVelocity * normalVelocity;
                y = by * tangent + bx * normal * normalVelocity * normalVelocity;
            } else {
                double tangent = v * profileKV() + a * constants.getTranslationalKA(v, a) + signedStatic(v, a, constants.translationalFeedforwardKS);
                x = bx * tangent; y = by * tangent;
                if (v * a >= -1e-6) {
                    x = constants.characterizedPower(Axis.FORWARD, v * bx, a * bx, x);
                    y = constants.characterizedPower(Axis.STRAFE, v * by, a * by, y);
                }
                x -= by * normal * normalVelocity * normalVelocity;
                y += bx * normal * normalVelocity * normalVelocity;
            }
            double turn = HolonomicFeedforward.heading(constants, omega, alpha);
            return Math.abs(x) + Math.abs(y) + Math.abs(turn);
            }
            @Override public double maximumAcceleration(double v, double prime, double second, double min, double max, double limit) {
                if (wheelModel() != null) return wheelModel().maximumAcceleration(bx,by,point.getSignedCurvature(),v,
                        prime,second,min,max,limit);
                double omega = prime * v, baseAlpha = second * v * v;
                double firstAlpha = baseAlpha + prime * min, lastAlpha = baseAlpha + prime * max;
                if (feedforward.HolonomicFeedforward.motionSign(v, min) != feedforward.HolonomicFeedforward.motionSign(v, max)
                        || feedforward.HolonomicFeedforward.motionSign(omega, firstAlpha) != feedforward.HolonomicFeedforward.motionSign(omega, lastAlpha)
                        || (constants.motionCharacterization != null
                            && (v * min < -1e-6) != (v * max < -1e-6))) { return Double.NEGATIVE_INFINITY; }
                double tangent = v * profileKV() + signedStatic(v, min, constants.translationalFeedforwardKS);
                double turn = omega * constants.angularKV + baseAlpha * constants.angularKA
                        + signedStatic(omega, firstAlpha, constants.angularFeedforwardKS);
                boolean characterizedTranslation = v * min >= -1e-6;
                if (!linearAxisPower(Axis.FORWARD, bx * v, 0, bx, bx * tangent, bx * constants.translationalKA,
                        min, max, characterizedTranslation, coefficients, 0)
                        || !linearAxisPower(Axis.STRAFE, by * v, 0, by, by * tangent, by * constants.translationalKA,
                        min, max, characterizedTranslation, coefficients, 2)
                        || !linearAxisPower(Axis.ANGULAR, omega, baseAlpha, prime, turn, prime * constants.angularKA,
                        min, max, true, coefficients, 4)) { return Double.NEGATIVE_INFINITY; }
                interval.reset(min, max);
                interval.restrictL1(coefficients[0] - by * normal * v * v, coefficients[1],
                        coefficients[2] + bx * normal * v * v, coefficients[3], coefficients[4], coefficients[5], limit);
                return interval.maximum();
            }
        };
    }
    @Override protected void evaluatePoint(Path path, PathPoint prev, PathPoint current, double vPrev,
                                            double velocity, double acceleration, EvaluationResult result) {
        Vector end = path.getParametricPath().getFirstDerivative(1);
        Vector body = current.getFirstDerivative().normalize().rotate(heading(current, path).times(-1));
        double prime = path.getInterpolator().getHeadingFirstDerivative(current.getDistanceToEndIn(), current.getSignedCurvature(), end);
        double second = path.getInterpolator().getHeadingSecondDerivative(current.getDistanceToEndIn(), current.getCurvatureDerivative(), end);
        if (wheelModel() != null) {
            double bx=body.getX().getIn(), by=body.getY().getIn(), an=current.getSignedCurvature()*velocity*velocity;
            wheelModel().solve(bx*velocity, by*velocity, prime*velocity, bx*acceleration-by*an,
                    by*acceleration+bx*an, second*velocity*velocity+prime*acceleration, wheelDemand);
            result.pForward = Math.abs((wheelDemand.mean(0)+wheelDemand.mean(1)+wheelDemand.mean(2)+wheelDemand.mean(3))/4);
            result.pLateral = Math.abs((-wheelDemand.mean(0)+wheelDemand.mean(1)+wheelDemand.mean(2)-wheelDemand.mean(3))/4);
            result.pHeading = Math.abs((-wheelDemand.mean(0)+wheelDemand.mean(1)-wheelDemand.mean(2)+wheelDemand.mean(3))/4);
            result.totalPower = result.maxUtilization = wheelDemand.peak;
            return;
        }
        Vector tangentPower = HolonomicFeedforward.translation(constants, body, velocity, acceleration, profileKV());
        Vector normalPower = HolonomicFeedforward.centripetal(body, velocity, current.getSignedCurvature(), constants.kCentripetal);
        double turn = HolonomicFeedforward.heading(constants, prime * velocity, second * velocity * velocity + prime * acceleration);
        result.pForward = HolonomicFeedforward.mecanumDemand(tangentPower, 0);
        result.pLateral = HolonomicFeedforward.mecanumDemand(normalPower, 0);
        result.pHeading = Math.abs(turn);
        result.totalPower = result.maxUtilization = HolonomicFeedforward.mecanumDemand(tangentPower.plus(normalPower), turn);
    }
}
