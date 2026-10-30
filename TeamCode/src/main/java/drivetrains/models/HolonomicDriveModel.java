package drivetrains.models;

import follower.FollowerConstants;
import feedforward.MotionPowerCalculator;
import feedforward.MecanumWheelModel;
import feedforward.MotionCharacterization.Axis;
import paths.geometry.Angle;
import paths.geometry.Vector;

/** Body-axis feedforward shared by mecanum, swerve, and circular omni drives. */
public final class HolonomicDriveModel extends DriveModel {
    public enum Metric { MECANUM, ISOTROPIC, OMNI }
    private final Metric metric;
    private final WheelProjectionModel projections;
    private final boolean allowWheelModel;
    private final java.util.function.Function<FollowerConstants, DirectionalKinematics> kinematicsFactory;
    private final DirectionalKinematics kinematics;

    public HolonomicDriveModel(FollowerConstants constants, Metric metric) { this(constants,metric,null,null); }
    public HolonomicDriveModel(FollowerConstants constants, Metric metric, double[] wheelX, double[] wheelY) {
        this(constants,metric,wheelX,wheelY,true);
    }
    HolonomicDriveModel(FollowerConstants constants, Metric metric, double[] wheelX, double[] wheelY, boolean allowWheelModel) {
        this(constants,metric,projections(wheelX,wheelY),allowWheelModel,null);
    }
    public HolonomicDriveModel(FollowerConstants constants, WheelProjectionModel projections) {
        this(constants,Metric.OMNI,java.util.Objects.requireNonNull(projections),false,null);
    }
    private HolonomicDriveModel(FollowerConstants constants, Metric metric, WheelProjectionModel projections,
                                boolean allowWheelModel,
                                java.util.function.Function<FollowerConstants, DirectionalKinematics> factory) {
        super(constants);
        this.metric = java.util.Objects.requireNonNull(metric);
        this.projections = projections; this.allowWheelModel = allowWheelModel;
        if (factory != null) kinematicsFactory = factory;
        else if (metric == Metric.MECANUM) kinematicsFactory = DirectionalKinematics::mecanum;
        else if (projections != null) kinematicsFactory = settings -> DirectionalKinematics.projected(settings,projections);
        else if (metric == Metric.OMNI) kinematicsFactory = DirectionalKinematics::omni;
        else kinematicsFactory = DirectionalKinematics::isotropic;
        kinematics = java.util.Objects.requireNonNull(kinematicsFactory.apply(constants));
    }
    private static WheelProjectionModel projections(double[] x,double[] y) {
        if (x == null && y == null) return null;
        return new WheelProjectionModel(x,y,1);
    }
    /** The factory must use its argument, rather than capture live settings, to preserve planning snapshots. */
    public HolonomicDriveModel withDirectionalKinematics(
            java.util.function.Function<FollowerConstants, DirectionalKinematics> factory) {
        return new HolonomicDriveModel(constants,metric,projections,allowWheelModel,java.util.Objects.requireNonNull(factory));
    }
    public Metric getMetric() { return metric; }
    @Override public DriveModel withConstants(FollowerConstants constants) {
        return new HolonomicDriveModel(constants,metric,projections,allowWheelModel,kinematicsFactory);
    }
    @Override public boolean cacheSolverSamples() { return kinematics.cacheSolverSamples(); }
    @Override public boolean isHolonomic() { return true; }
    @Override public Vector positionHoldCommand(Vector bodyCorrection) {
        if (metric != Metric.MECANUM) return bodyCorrection;
        double forward = constants.forwardVelLimitIn, strafe = constants.strafeVelLimitIn;
        double ratio = Double.isFinite(forward) && forward > 0 && Double.isFinite(strafe) && strafe > 0
                ? forward / strafe : 1;
        return trajectory.TrajectorySample.vector(bodyCorrection.getX().getIn(), bodyCorrection.getY().getIn() * ratio);
    }
    @Override public DirectionalLimits directionalLimits(Angle direction) { return kinematics.limitsAt(direction); }
    @Override public double translationDemand(double x, double y) {
        return projections != null ? projections.demand(x,y,0)
                : metric == Metric.MECANUM ? Math.abs(x)+Math.abs(y) : Math.hypot(x,y);
    }
    @Override public double commandDemand(double x,double y,double turn) {
        return projections != null ? projections.demand(x,y,turn) : super.commandDemand(x,y,turn);
    }
    @Override public MecanumWheelModel wheelModel() {
        return !allowWheelModel || metric != Metric.MECANUM || constants.motionCharacterization == null ? null : constants.motionCharacterization.mecanumWheelModel;
    }
    @Override public boolean enforceIntervalPowerBounds(boolean straightGeometry) { return wheelModel()!=null && straightGeometry; }
    @Override public double maximumVelocity(Vector body, double curvature, double prime, double maxAngVel) {
        double ceiling = directionalLimits(body.getTheta()).maxVelocity;
        ceiling = cornerVelocityLimit(ceiling, curvature);
        if (Math.abs(body.getX().getIn()) > 1e-12) {
            ceiling = Math.min(ceiling, calibratedVelocityLimit(Axis.FORWARD, Double.MAX_VALUE) / Math.abs(body.getX().getIn()));
        }
        if (Math.abs(body.getY().getIn()) > 1e-12) {
            ceiling = Math.min(ceiling, calibratedVelocityLimit(Axis.STRAFE, Double.MAX_VALUE) / Math.abs(body.getY().getIn()));
        }
        if (Math.abs(prime) > 1e-12) {
            ceiling = Math.min(ceiling, Math.min(maxAngVel, calibratedVelocityLimit(Axis.ANGULAR, constants.angularVelLimitRad)) / Math.abs(prime));
        }
        if (wheelModel() != null) {
            double factor = Math.abs(body.getX().getIn()) + Math.abs(body.getY().getIn())
                    + Math.abs(wheelModel().effectiveRadius * prime);
            if (factor > 1e-12) ceiling = Math.min(ceiling, wheelModel().maximumWheelSpeed / factor);
            return ceiling;
        }
        double normalCost = Math.abs(curvature * constants.kCentripetal);
        if (normalCost > 1e-12) { ceiling = Math.min(ceiling, Math.sqrt(MotionPowerCalculator.MAX_CENTRIPETAL_POWER / normalCost)); }
        return ceiling;
    }
    @Override public double maximumTangentialAcceleration(Vector body) {
        return directionalLimits(body.getTheta()).maxAcceleration;
    }
    @Override public PowerEvaluator powerEvaluator(Vector body, double curvature) {
        double bx = body.getX().getIn(), by = body.getY().getIn();
        double normal = curvature * constants.kCentripetal;
        return new PowerEvaluator(body, curvature) {
            private final double[] coefficients = new double[6];
            private final AccelerationInterval interval = new AccelerationInterval();
            @Override public double maximumAcceleration(double v, double prime, double second, double min, double max, double limit) {
                if (wheelModel() != null) return wheelModel().maximumAcceleration(bx,by,curvature,v,
                        prime,second,min,max,limit);
                double omega = prime * v, baseAlpha = second * v * v;
                double firstAlpha = baseAlpha + prime * min, lastAlpha = baseAlpha + prime * max;
                if (MotionPowerCalculator.motionSign(v, min) != MotionPowerCalculator.motionSign(v, max)
                        || MotionPowerCalculator.motionSign(omega, firstAlpha) != MotionPowerCalculator.motionSign(omega, lastAlpha)
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
                coefficients[0] -= by * normal * v * v;
                coefficients[2] += bx * normal * v * v;
                if (projections != null) return projections.maximumAcceleration(coefficients,min,max,limit);
                if (metric != Metric.MECANUM) return maximumNormPowerAcceleration(coefficients,min,max,limit);
                interval.reset(min, max);
                interval.restrictL1(coefficients[0],coefficients[1],coefficients[2],coefficients[3],coefficients[4],coefficients[5],limit);
                return interval.maximum();
            }
        };
    }
}
