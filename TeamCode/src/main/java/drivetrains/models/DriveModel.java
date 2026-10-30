package drivetrains.models;

import feedforward.MotionPowerCalculator;
import feedforward.MotionCharacterization.Axis;
import follower.FollowerConstants;
import paths.geometry.Vector;
import paths.geometry.Angle;
import paths.geometry.DistUnit;

/** Hardware-independent physical limits and reference power. Feedback and actuator state stay in followers.
 * Models bind to one settings object; use withConstants to take an independent planning snapshot. */
public abstract class DriveModel {
    protected static final double EPSILON = 1e-6;
    protected final FollowerConstants constants;
    protected DriveModel(FollowerConstants constants) { this.constants = java.util.Objects.requireNonNull(constants); }
    public FollowerConstants getConstants() { return constants.copy(); }
    public abstract DriveModel withConstants(FollowerConstants constants);
    public abstract boolean isHolonomic();
    public feedforward.MecanumWheelModel wheelModel() { return null; }
    /** Convert a body-space position correction to motor-command units before budgeting it. */
    public Vector positionHoldCommand(Vector bodyCorrection) { return bodyCorrection; }
    public abstract double maximumVelocity(Vector bodyDirection, double curvature, double headingPrime, double maxAngularVelocity);
    public abstract double maximumTangentialAcceleration(Vector bodyDirection);
    /** Optional direct capability query for runtime tools and custom followers. */
    public DirectionalLimits directionalLimits(Angle direction) {
        Vector body = Vector.of(Math.cos(direction.getRad()),Math.sin(direction.getRad()), DistUnit.IN);
        return new DirectionalLimits(maximumVelocity(body,0,0,constants.angularVelLimitRad),maximumTangentialAcceleration(body));
    }
    public abstract PowerEvaluator powerEvaluator(Vector bodyDirection, double curvature);
    /** Opt in only when geometry-dependent values are immutable or covered by solver input invalidation. */
    public boolean cacheSolverSamples() { return false; }
    public boolean enforceIntervalPowerBounds(boolean straightGeometry) { return false; }
    /** Motor demand for translation alone; commandDemand also accounts for signed rotation. */
    public abstract double translationDemand(double x, double y);
    public final double commandDemand(Vector bodyPower, double headingPower) {
        return commandDemand(bodyPower.getX().getIn(),bodyPower.getY().getIn(),headingPower);
    }
    /** Coupled motor command demand; override when wheel projections allow translation/rotation cancellation. */
    public double commandDemand(double x,double y,double headingPower) {
        return translationDemand(x,y) + Math.abs(headingPower);
    }
    private MotionPowerCalculator powerCalculator;
    public final MotionPowerCalculator powerCalculator() {
        if(powerCalculator==null) powerCalculator=new MotionPowerCalculator(constants,isHolonomic(),this::commandDemand,this::planningCommandDemand);
        return powerCalculator;
    }
    protected double planningCommandDemand(double x,double y,double turn) { return commandDemand(x,y,turn); }
    /** Bound geometry once, then reuse the same pure calculation for hypothetical solver states. */
    public class PowerEvaluator {
        private final double bx,by,curvature;
        private final MotionPowerCalculator.Output output=new MotionPowerCalculator.Output();
        private final feedforward.MecanumWheelModel.Demand wheels=new feedforward.MecanumWheelModel.Demand();
        protected PowerEvaluator(Vector body,double curvature) {
            bx=body.getX().getIn(); by=body.getY().getIn(); this.curvature=curvature;
        }
        public double utilization(double v,double a,double omega,double alpha,double normalVelocity) {
            double normal=curvature*normalVelocity*normalVelocity;
            if(wheelModel()!=null) {
                wheelModel().solve(bx*v,by*v,omega,bx*a-by*normal,by*a+bx*normal,alpha,wheels);
                return wheels.peak;
            }
            powerCalculator().calculate(bx,by,v,a,omega,alpha,normal,true,MotionPowerCalculator.motionSign(v,a),output);
            return output.demand;
        }
        /** Negative infinity requests generic convex search; NaN means infeasible. */
        public double maximumAcceleration(double v,double prime,double second,double min,double max,double limit) {
            return Double.NEGATIVE_INFINITY;
        }
    }
    protected double profileKV() {
        return Double.isFinite(constants.profileVelocityPowerCoefficient) && constants.profileVelocityPowerCoefficient > 0
                ? constants.profileVelocityPowerCoefficient : constants.translationalKV;
    }
    /** Measured speed range is a hard bound; holding power is checked with signed acceleration. */
    protected double calibratedVelocityLimit(Axis axis, double configured) {
        return constants.motionCharacterization == null ? configured
                : Math.min(configured, constants.motionCharacterization.maximumMeasuredVelocity(axis));
    }
    protected double cornerVelocityLimit(double limit, double curvature) {
        double scale = constants.profileCornerSpeedScale;
        return !Double.isFinite(scale) || scale <= 0 || scale >= 1 ? limit
                : limit * (1 - (1 - scale) * Math.min(1, Math.abs(curvature) / .08));
    }
    protected double signedStatic(double velocity, double acceleration, double kS) {
        return MotionPowerCalculator.motionSign(velocity, acceleration) * kS;
    }
    /**
     * Intersects all sign combinations of {@code |o1+s1*a|+|o2+s2*a|+|o3+s3*a| <= limit}.
     */
    protected static final class AccelerationInterval {
        double low, high;
        void reset(double min, double max) { low = min; high = max; }
        void restrictL1(double o1, double s1, double o2, double s2, double o3, double s3, double limit) {
            for (int signs = 0; signs < 8 && low <= high; signs++) {
                double a = (signs & 1) == 0 ? -1 : 1, b = (signs & 2) == 0 ? -1 : 1, c = (signs & 4) == 0 ? -1 : 1;
                double offset = a * o1 + b * o2 + c * o3, slope = a * s1 + b * s2 + c * s3;
                if (slope > 0) { high = Math.min(high, (limit - offset) / slope); }
                else if (slope < 0) { low = Math.max(low, (limit - offset) / slope); }
                else if (offset > limit) { low = Double.POSITIVE_INFINITY; }
            }
        }
        double maximum() { return low <= high ? high : Double.NaN; }
    }

    /** Boundary roots of ||offset + slope*a|| + |turnOffset + turnSlope*a| <= limit.
     * The signed turn branches yield quadratics. Every candidate is checked against
     * the unsquared inequality; ambiguous floating-point cases use the generic search. */
    public static double maximumNormPowerAcceleration(double[] coefficients, double min, double max, double limit) {
        double ox = coefficients[0], sx = coefficients[1], oy = coefficients[2], sy = coefficients[3];
        double ot = coefficients[4], st = coefficients[5];
        if (Math.hypot(ox + sx * max, oy + sy * max) + Math.abs(ot + st * max) <= limit) { return max; }
        double best = Double.NEGATIVE_INFINITY;
        boolean ambiguous = false;
        double margin = 1e-10 * Math.max(1, Math.max(Math.abs(min), Math.abs(max)));
        for (int sign = -1; sign <= 1; sign += 2) {
            double r = limit - sign * ot, rs = -sign * st;
            double a = sx * sx + sy * sy - rs * rs;
            double b = 2 * (ox * sx + oy * sy - r * rs);
            double c = ox * ox + oy * oy - r * r;
            double scale = Math.max(Math.abs(a), Math.max(Math.abs(b), Math.abs(c)));
            if (!Double.isFinite(scale) || scale == 0) { ambiguous = true; continue; }
            a /= scale; b /= scale; c /= scale;
            double first, second;
            if (a == 0) {
                if (b == 0) { if (c == 0) { ambiguous = true; } continue; }
                first = second = -c / b;
            } else {
                double discriminant = b * b - 4 * a * c;
                if (discriminant < 0) { if (discriminant > -1e-14) { ambiguous = true; } continue; }
                double q = -.5 * (b + Math.copySign(Math.sqrt(discriminant), b));
                first = q == 0 ? 0 : q / a;
                second = q == 0 ? 0 : c / q;
            }
            for (int root = 0; root < 2; root++) {
                double candidate = root == 0 ? first : second;
                if (!Double.isFinite(candidate)) { ambiguous = true; continue; }
                if (candidate < min || candidate > max) {
                    if (candidate >= min - margin && candidate <= max + margin) { ambiguous = true; }
                    continue;
                }
                double turn = ot + st * candidate;
                // Squaring also produces roots on the wrong signed turn or negative RHS.
                if (sign * turn < -1e-12 || limit - sign * turn < -1e-12) { continue; }
                // Move one ulp into the branch before verifying the original physical demand.
                candidate = Math.nextAfter(candidate, min);
                double demand = Math.hypot(ox + sx * candidate, oy + sy * candidate) + Math.abs(ot + st * candidate);
                if (demand <= limit + 1e-12) { best = Math.max(best, candidate); }
                else { ambiguous = true; }
            }
        }
        if (Double.isFinite(best)) { return best; }
        // With an infeasible max, any nonempty convex feasible interval must have an
        // upper boundary root in range. Only certify emptiness when no root was ambiguous.
        return ambiguous ? Double.NEGATIVE_INFINITY : Double.NaN;
    }

    /** Resolve one calibrated axis into offset+slope*a only when the entire interval stays
     * in a single model branch. Threshold crossings retain the generic solver. */
    protected boolean linearAxisPower(Axis axis, double velocity, double base, double slope,
                                      double legacyOffset, double legacySlope, double min, double max,
                                      boolean useCharacterization, double[] result, int index) {
        double first = base + slope * min, last = base + slope * max;
        feedforward.MotionCharacterization characterization = useCharacterization ? constants.motionCharacterization : null;
        if (characterization == null) { result[index] = legacyOffset; result[index + 1] = legacySlope; return true; }
        boolean firstZero = Math.abs(velocity) < 1e-6 && Math.abs(first) < 1e-6;
        boolean lastZero = Math.abs(velocity) < 1e-6 && Math.abs(last) < 1e-6;
        if (firstZero != lastZero) { return false; }
        if (firstZero) { result[index] = result[index + 1] = 0; return true; }
        feedforward.MotorPowerModel firstModel = characterization.model(axis, Math.abs(velocity) > 1e-6 ? velocity : first);
        feedforward.MotorPowerModel lastModel = characterization.model(axis, Math.abs(velocity) > 1e-6 ? velocity : last);
        boolean firstLegacy = firstModel == null || velocity * first < -1e-6 || Math.abs(velocity) > firstModel.maxVelocity + 1e-6;
        boolean lastLegacy = lastModel == null || velocity * last < -1e-6 || Math.abs(velocity) > lastModel.maxVelocity + 1e-6;
        if (firstLegacy != lastLegacy) { return false; }
        if (firstLegacy) { result[index] = legacyOffset; result[index + 1] = legacySlope; return true; }
        if (firstModel != lastModel || first * last < 0) { return false; }
        double direction = Math.copySign(1, Math.abs(velocity) > 1e-6 ? velocity : first + last);
        double accelSign = Math.signum(first + last);
        double kA = firstModel.kA(Math.abs(velocity));
        result[index] = direction * (firstModel.holdingPower(Math.abs(velocity)) + kA * accelSign * base);
        result[index + 1] = direction * kA * accelSign * slope;
        return true;
    }
}
