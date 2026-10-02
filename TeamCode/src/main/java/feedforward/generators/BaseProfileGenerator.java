package feedforward.generators;

import androidx.annotation.NonNull;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;
import core.FollowerConstants;
import core.MotionCharacterization.Axis;
import feedforward.CoupledMotionProfile;
import feedforward.DynamicMotionProfile;
import feedforward.FFLut;
import feedforward.MotionParameters;
import geometry.PathPoint;
import geometry.Vector;
import paths.constraint.AngularConstraint;
import paths.constraint.PathConstraint;
import paths.constraint.PathConstraint.Type;
import paths.constraint.TranslationalConstraint;
import paths.movements.FollowerMovement;
import paths.movements.Path;

/** Shared sampled path solver with signed, coupled power and acceleration constraints. */
public abstract class BaseProfileGenerator {
    protected static final double EPSILON = 1e-6;
    private static final double UTILIZATION_TOLERANCE = 1e-6;
    private static final int SEARCH_ITERATIONS = 32;
    private final Double requestedPowerLimit;
    protected final FollowerConstants constants;
    protected final FollowerMovement path;
    private DebugReport lastReport;

    protected BaseProfileGenerator(FollowerConstants constants, FollowerMovement path) {
        this(constants, path, path instanceof Path ? ((Path) path).getProfilePowerLimit() : null);
    }
    protected BaseProfileGenerator(FollowerConstants constants, FollowerMovement path, double powerLimit) {
        this(constants, path, Double.valueOf(powerLimit));
    }
    private BaseProfileGenerator(FollowerConstants constants, FollowerMovement path, Double powerLimit) {
        if (powerLimit != null && (!Double.isFinite(powerLimit) || powerLimit <= 0 || powerLimit > 1)) {
            throw new IllegalArgumentException("Profile power limit must be finite and in (0, 1].");
        }
        this.constants = constants; this.path = path; this.requestedPowerLimit = powerLimit;
    }
    public double getProfilePowerLimit() {
        if (requestedPowerLimit == null) { return constants.profilePowerLimit(); }
        return constants.motionCharacterization == null ? requestedPowerLimit
                : Math.min(requestedPowerLimit, constants.motionCharacterization.availableMotorPower);
    }
    protected double utilizationLimit() { return getProfilePowerLimit(); }
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
        return core.HolonomicFeedforward.motionSign(velocity, acceleration) * kS;
    }
    public DebugReport getLastDebugReport() { return lastReport; }
    /** Physical/authored speed ceiling, without assuming acceleration is zero. */
    protected abstract double calculateMaxTangentialVelocity(PathPoint point, Path path, double maxAngVel, double maxAngAccel);
    protected abstract double getPhysicalTangentialAccel(PathPoint point, Path path);
    protected abstract void evaluatePoint(Path path, PathPoint prev, PathPoint current, double vPrev,
                                          double velocity, double acceleration, EvaluationResult result);

    public MotionParameters[] generate() {
        if (!(path instanceof Path)) { throw new IllegalArgumentException("Path generator requires a Path."); }
        final Path actualPath = (Path) path;
        final PathPoint[] points = actualPath.getGeneratedPoints();
        if (points == null || points.length < 2) { throw new IllegalStateException("At least two path points are required."); }
        final Sample[] samples = samples(actualPath, points);
        double[] positions = new double[points.length], ceilings = new double[points.length];
        for (int i = 0; i < points.length; i++) {
            positions[i] = actualPath.getParametricPath().getLengthIn() - points[i].getDistanceToEndIn();
            ceilings[i] = samples[i].maxVelocity;
        }
        double endVelocity = actualPath.isAccelBoosted()
                ? maximumSteadyVelocity(actualPath, points[points.length - 1], samples[samples.length - 1]) : 0;
        if (actualPath.isAccelBoosted()) {
            ceilings[0] = maximumSteadyVelocity(actualPath, points[0], samples[0]);
        }
        double[] velocities = CoupledMotionProfile.solve(positions, ceilings, actualPath.isAccelBoosted(), endVelocity,
                (i, squared, nextCeiling) -> maximumNextSquared(actualPath, points[i], samples[i], squared,
                        nextCeiling, positions[i + 1] - positions[i]));
        lastReport = new DebugReport();
        lastReport.powerLimit = utilizationLimit(); lastReport.iterationsRun = 1;
        lastReport.powerFeasible = true; lastReport.kinematicsFeasible = true;
        MotionParameters[] output = new MotionParameters[points.length];
        double time = 0;
        EvaluationResult power = new EvaluationResult();
        for (int i = 0; i < points.length; i++) {
            double ds = i + 1 < points.length ? positions[i + 1] - positions[i] : 0;
            double v = velocities[i];
            double acceleration = ds > 1e-9 ? (velocities[i + 1] * velocities[i + 1] - v * v) / (2 * ds) : 0;
            Sample sample = samples[i];
            double omega = sample.headingPrime * v;
            double alpha = sample.headingSecond * v * v + sample.headingPrime * acceleration;
            evaluatePoint(actualPath, points[i], points[i], v, v, acceleration, power);
            MotionParameters state = new MotionParameters(v, acceleration, omega, alpha, positions[i]).setTimeSeconds(time);
            state.setMotorPower(power.totalPower); output[i] = state;
            lastReport.finalMaxUtilization = Math.max(lastReport.finalMaxUtilization, power.maxUtilization);
            double constraintRatio = Math.max(ratio(v, sample.maxVelocity), Math.max(ratio(Math.abs(acceleration), sample.maxAccel),
                    Math.max(ratio(Math.abs(omega), sample.maxOmega), ratio(Math.abs(alpha), sample.maxAlpha))));
            lastReport.finalMaxConstraintRatio = Math.max(lastReport.finalMaxConstraintRatio, constraintRatio);
            lastReport.powerFeasible &= Double.isFinite(power.maxUtilization) && power.maxUtilization <= utilizationLimit() + UTILIZATION_TOLERANCE;
            lastReport.kinematicsFeasible &= Double.isFinite(constraintRatio) && constraintRatio <= 1 + 1e-6;
            if (ds > 1e-9) {
                double sum = v + velocities[i + 1];
                time = sum > 1e-9 ? time + 2 * ds / sum : Double.POSITIVE_INFINITY;
            }
        }
        lastReport.durationSeconds = time;
        lastReport.converged = lastReport.powerFeasible && lastReport.kinematicsFeasible;
        if (!lastReport.converged) { throw new IllegalStateException("Generated path failed feasibility validation: " + lastReport); }
        return output;
    }

    private Sample[] samples(Path actualPath, PathPoint[] points) {
        Sample[] result = new Sample[points.length];
        Vector finalTangent = actualPath.getParametricPath().getFirstDerivative(1);
        double length = Math.max(actualPath.getParametricPath().getLengthIn(), EPSILON);
        PathConstraint[] constraints = actualPath.getConstraints();
        Arrays.sort(constraints, Comparator.comparingDouble(PathConstraint::getS));
        for (int i = 0; i < points.length; i++) {
            PathPoint point = points[i]; Sample sample = new Sample();
            double physicalAccel = nonNegative(getPhysicalTangentialAccel(point, actualPath));
            double physicalAlpha = nonNegative(constants.angularAccelLimitRad);
            double physicalOmega = nonNegative(calibratedVelocityLimit(Axis.ANGULAR, constants.angularVelLimitRad));
            sample.maxAccel = physicalAccel; sample.maxAlpha = physicalAlpha; sample.maxOmega = physicalOmega;
            double maxVelocity = Double.MAX_VALUE;
            double progress = actualPath.constraintProgress(1 - point.getDistanceToEndIn() / length);
            for (PathConstraint constraint : constraints) {
                if (constraint.getS() <= progress + 1e-12) {
                    if (constraint instanceof TranslationalConstraint) {
                        double value = nonNegative(((TranslationalConstraint) constraint).getValueIn());
                        if (constraint.getType() == Type.VELOCITY) { maxVelocity = value; }
                        else { sample.maxAccel = Math.min(physicalAccel, value); }
                    } else if (constraint instanceof AngularConstraint) {
                        double value = nonNegative(((AngularConstraint) constraint).getValueRad());
                        if (constraint.getType() == Type.VELOCITY) { sample.maxOmega = Math.min(physicalOmega, value); }
                        else { sample.maxAlpha = Math.min(physicalAlpha, value); }
                    }
                }
            }
            sample.maxVelocity = nonNegative(Math.min(maxVelocity, calculateMaxTangentialVelocity(point, actualPath, sample.maxOmega, sample.maxAlpha)));
            sample.headingPrime = actualPath.getInterpolator().getHeadingFirstDerivative(point.getDistanceToEndIn(), point.getSignedCurvature(), finalTangent);
            sample.headingSecond = actualPath.getInterpolator().getHeadingSecondDerivative(point.getDistanceToEndIn(), point.getCurvatureDerivative(), finalTangent);
            result[i] = sample;
        }
        return result;
    }

    private double maximumNextSquared(Path actualPath, PathPoint point, Sample sample, double squared, double nextCeiling, double ds) {
        double min = Math.max(-sample.maxAccel, -squared / (2 * ds));
        double max = Math.min(sample.maxAccel, (nextCeiling - squared) / (2 * ds));
        double baseAlpha = sample.headingSecond * squared;
        if (Math.abs(sample.headingPrime) > 1e-12) {
            double a = (-sample.maxAlpha - baseAlpha) / sample.headingPrime;
            double b = (sample.maxAlpha - baseAlpha) / sample.headingPrime;
            min = Math.max(min, Math.min(a, b)); max = Math.min(max, Math.max(a, b));
        } else if (Math.abs(baseAlpha) > sample.maxAlpha + 1e-9) { return Double.NaN; }
        if (min > max + 1e-9) { return Double.NaN; }
        if (min > max) { min = max; }
        double v = Math.sqrt(Math.max(0, squared));
        // Characterized response switches at translation/heading acceleration zero.
        double headingZero = Math.abs(sample.headingPrime) > 1e-12 ? -baseAlpha / sample.headingPrime : 0;
        double[] breaks = {min, max, Math.max(min, Math.min(max, 0)), Math.max(min, Math.min(max, headingZero))};
        Arrays.sort(breaks);
        double best = Double.NaN;
        for (int j = breaks.length - 2; j >= 0; j--) {
            double candidate = maximumPowerFeasibleAcceleration(actualPath, point, v, breaks[j], breaks[j + 1]);
            if (Double.isFinite(candidate)) { best = candidate; break; }
        }
        return Double.isFinite(best) ? Math.max(0, Math.min(nextCeiling, squared + 2 * ds * best)) : Double.NaN;
    }

    /** Each signed model branch is convex in acceleration for a fixed velocity. */
    private double maximumPowerFeasibleAcceleration(Path actualPath, PathPoint point, double v, double min, double max) {
        if (power(actualPath, point, v, max) <= utilizationLimit()) { return max; }
        double minimum = min;
        if (power(actualPath, point, v, min) > utilizationLimit()) {
            double left = min, right = max;
            for (int i = 0; i < SEARCH_ITERATIONS; i++) {
                double a = left + (right - left) / 3, b = right - (right - left) / 3;
                if (power(actualPath, point, v, a) <= power(actualPath, point, v, b)) { right = b; }
                else { left = a; }
            }
            minimum = (left + right) / 2;
            if (power(actualPath, point, v, minimum) > utilizationLimit()) { return Double.NaN; }
        }
        double low = minimum, high = max;
        for (int i = 0; i < SEARCH_ITERATIONS; i++) {
            double mid = (low + high) / 2;
            if (power(actualPath, point, v, mid) <= utilizationLimit()) { low = mid; }
            else { high = mid; }
        }
        return low;
    }
    private double maximumSteadyVelocity(Path actualPath, PathPoint point, Sample sample) {
        double low = 0, high = sample.maxVelocity;
        for (int i = 0; i < SEARCH_ITERATIONS; i++) {
            double mid = (low + high) / 2;
            if (Math.abs(sample.headingSecond * mid * mid) <= sample.maxAlpha && power(actualPath, point, mid, 0) <= utilizationLimit()) { low = mid; }
            else { high = mid; }
        }
        return low;
    }
    private double power(Path actualPath, PathPoint point, double velocity, double acceleration) {
        EvaluationResult result = new EvaluationResult();
        evaluatePoint(actualPath, point, point, velocity, velocity, acceleration, result);
        return Double.isFinite(result.maxUtilization) ? result.maxUtilization : Double.POSITIVE_INFINITY;
    }
    private static double nonNegative(double value) { return Double.isFinite(value) ? Math.max(0, value) : 0; }
    private static double ratio(double value, double bound) { return bound > 0 ? value / bound : value <= 1e-9 ? 0 : Double.POSITIVE_INFINITY; }
    private static final class Sample { double maxVelocity, maxAccel, maxOmega, maxAlpha, headingPrime, headingSecond; }

    /** Legacy tank quick profiles intentionally omit coupled heading constraints. */
    public FFLut generateQuick(FollowerConstants config) {
        if (config.motionCharacterization != null) { return new FFLut(generate()); }
        Path actualPath = (Path) path;
        PathPoint[] points = actualPath.getGeneratedPoints();
        double[] positions = new double[points.length], ceilings = new double[points.length];
        DynamicMotionProfile.PowerModel model = new DynamicMotionProfile.PowerModel(config.forwardVelLimitIn, config.forwardAccelLimitIn,
                profileKV(), config.translationalKA, config.translationalFeedforwardKS, utilizationLimit());
        for (int i = 0; i < points.length; i++) {
            positions[i] = actualPath.getParametricPath().getLengthIn() - points[i].getDistanceToEndIn(); ceilings[i] = model.getVelocityLimit();
        }
        double boundary = actualPath.isAccelBoosted() ? model.getVelocityLimit() : 0;
        DynamicMotionProfile.State[] states = DynamicMotionProfile.fromVelocityEnvelope(positions, ceilings, boundary, boundary,
                model.forwardReachability(), model.backwardReachability()).getStates();
        MotionParameters[] output = new MotionParameters[states.length];
        Vector end = actualPath.getParametricPath().getFirstDerivative(1);
        for (int i = 0; i < states.length; i++) {
            double prime = actualPath.getInterpolator().getHeadingFirstDerivative(points[i].getDistanceToEndIn(), points[i].getSignedCurvature(), end);
            double second = actualPath.getInterpolator().getHeadingSecondDerivative(points[i].getDistanceToEndIn(), points[i].getCurvatureDerivative(), end);
            output[i] = new MotionParameters(states[i].velocity, states[i].acceleration, prime * states[i].velocity,
                    second * states[i].velocity * states[i].velocity + prime * states[i].acceleration, positions[i]).setTimeSeconds(states[i].timeSeconds);
        }
        return new FFLut(output);
    }
    public static class EvaluationResult {
        public double totalPower, maxUtilization, pForward, pLateral, pHeading;
        public void copyFrom(EvaluationResult other) {
            totalPower = other.totalPower; maxUtilization = other.maxUtilization;
            pForward = other.pForward; pLateral = other.pLateral; pHeading = other.pHeading;
        }
    }
    public static class DebugReport {
        public int iterationsRun;
        public boolean converged, powerFeasible, kinematicsFeasible;
        public double finalMaxUtilization, finalMaxConstraintRatio, durationSeconds, powerLimit;
        public List<IterationLog> logs = new ArrayList<>();
        @Override @NonNull public String toString() {
            return String.format(Locale.ENGLISH, "Converged: %b | Power feasible: %b | Kinematics feasible: %b | Max util: %.6f / %.3f | Max constraint ratio: %.6f | Duration: %.3fs",
                    converged, powerFeasible, kinematicsFeasible, finalMaxUtilization, powerLimit, finalMaxConstraintRatio, durationSeconds);
        }
    }
    /** Compatibility with clients that inspected the former pinning logs. */
    public static class IterationLog {
        public int iteration, pinnedIndex;
        public double previousVelocity, newVelocity, maxUtilization, totalPower, pForward, pLateral, pHeading;
    }
}
