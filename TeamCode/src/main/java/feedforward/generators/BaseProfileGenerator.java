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
    private int lastWindowSampleCount;
    public int getLastWindowSampleCount() { return lastWindowSampleCount; }

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

    /** Geometry-dependent terms are computed once per solver node, outside numerical searches. */
    protected interface PowerEvaluator { double utilization(double velocity, double acceleration, double omega, double alpha, double normalVelocity); }
    protected abstract PowerEvaluator powerEvaluator(Path path, PathPoint point);
    private double power(Sample sample, double v, double a) {
        return sample.power.utilization(v, a, sample.headingPrime * v, sample.headingSecond * v * v + sample.headingPrime * a, v);
    }

    public MotionParameters[] generate() { return generate(((Path) path).isAccelBoosted(), false); }
    public MotionParameters[] generateRestToRest() { return generate(false, true); }

    private MotionParameters[] generate(boolean boosted, boolean exactTransitions) {
        if (!(path instanceof Path)) { throw new IllegalArgumentException("Path generator requires a Path."); }
        final Path actualPath = (Path) path;
        PathPoint[] original = actualPath.getGeneratedPoints();
        PathPoint[] points;
        double pathLength = actualPath.getParametricPath().getLengthIn();
        if (exactTransitions && pathLength <= 1e-9) { return new MotionParameters[]{new MotionParameters()}; }
        if (exactTransitions) {
            java.util.TreeSet<Double> distances = new java.util.TreeSet<>();
            for (PathPoint point : original) { distances.add(pathLength - point.getDistanceToEndIn()); }
            if (original.length < 3 && pathLength > 1e-6) { distances.add(pathLength / 2); }
            for (PathConstraint constraint : actualPath.getConstraints()) { distances.add(actualPath.constraintProgress(constraint.getS()) * pathLength); }
            points = new PathPoint[distances.size()]; int index = 0;
            for (double distance : distances) { points[index++] = actualPath.getParametricPath().getPointAtDistance(distance); }
        } else { points = original; }
        final PathPoint[] solverPoints = points;
        if (points == null || points.length < 2) { throw new IllegalStateException("At least two path points are required."); }
        final Sample[] samples = samples(actualPath, points);
        double[] positions = new double[points.length], ceilings = new double[points.length];
        for (int i = 0; i < points.length; i++) {
            positions[i] = actualPath.getParametricPath().getLengthIn() - points[i].getDistanceToEndIn();
            ceilings[i] = samples[i].maxVelocity;
        }
        double endVelocity = boosted
                ? maximumSteadyVelocity(actualPath, points[points.length - 1], samples[samples.length - 1]) : 0;
        if (boosted) {
            ceilings[0] = maximumSteadyVelocity(actualPath, points[0], samples[0]);
        }
        double[] velocities = CoupledMotionProfile.solve(positions, ceilings, boosted, endVelocity,
                (i, squared, nextCeiling) -> maximumNextSquared(actualPath, solverPoints[i], samples[i], squared,
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

    /** A bounded receding-horizon solve; the first boundary is the measured speed, not rest. */
    public WindowTarget generateWindow(double startIn, double horizonIn, double measuredVelocity, double dt) {
        return generateWindow(startIn, horizonIn, measuredVelocity, Double.NaN, dt);
    }

    public WindowTarget generateWindow(double startIn, double horizonIn, double measuredVelocity, double measuredOmega, double dt) {
        Path actualPath = (Path) path;
        geometry.PathSegment segment = actualPath.getParametricPath();
        double start = Math.max(0, Math.min(segment.getLengthIn(), startIn));
        double end = Math.min(segment.getLengthIn(), start + horizonIn);
        if (!Double.isFinite(measuredVelocity) || !Double.isFinite(horizonIn) || horizonIn <= 0) {
            throw new IllegalArgumentException("Finite velocity and positive lookahead required.");
        }
        if (end - start < 1e-6) {
            lastWindowSampleCount = 1;
            if (Math.abs(measuredVelocity) < 1e-6 && (!Double.isFinite(measuredOmega) || Math.abs(measuredOmega) < 1e-6)) {
                return new WindowTarget(new MotionParameters(0, 0, 0, 0, start), false);
            }
            Sample boundary = samples(actualPath, new PathPoint[]{segment.getPointAtDistance(start)})[0];
            MotionParameters recovery = recoveryMotion(boundary, measuredVelocity, measuredOmega);
            recovery.setDistAlongCurve(start);
            return new WindowTarget(recovery, true, 0);
        }
        java.util.TreeSet<Double> distances = new java.util.TreeSet<>();
        distances.add(start); distances.add(end);
        distances.add((start + end) / 2);
        PathPoint[] geometry = segment.getPointLUT();
        int low = 0, high = geometry.length;
        while (low < high) {
            int mid = (low + high) >>> 1;
            if (segment.getLengthIn() - geometry[mid].getDistanceToEndIn() <= start) { low = mid + 1; }
            else { high = mid; }
        }
        for (int i = low; i < geometry.length; i++) {
            double s = segment.getLengthIn() - geometry[i].getDistanceToEndIn();
            if (s >= end) { break; }
            if (s - start > 1e-6 && end - s > 1e-6) { distances.add(s); }
        }
        for (PathConstraint constraint : actualPath.getConstraints()) {
            double s = actualPath.constraintProgress(constraint.getS()) * segment.getLengthIn();
            if (s - start > 1e-6 && end - s > 1e-6) { distances.add(s); }
        }
        PathPoint[] points = new PathPoint[distances.size()];
        double[] positions = new double[points.length], ceilings = new double[points.length];
        int index = 0;
        for (double distance : distances) { positions[index] = distance; points[index++] = segment.getPointAtDistance(distance); }
        Sample[] limits = samples(actualPath, points);
        for (int i = 0; i < points.length; i++) { ceilings[i] = limits[i].maxVelocity; }
        lastWindowSampleCount = points.length;
        double acceleration;
        double recoveryOmega = 0, recoveryAlpha = 0, normalVelocity = Double.NaN;
        boolean overspeed = measuredVelocity < 0;
        try {
            if (measuredVelocity < 0 || (Double.isFinite(measuredOmega)
                    && Math.abs(measuredOmega) > limits[0].maxOmega + 1e-6)) {
                throw new CoupledMotionProfile.OverspeedException(0);
            }
            double[] speeds = CoupledMotionProfile.solve(positions, ceilings, measuredVelocity, 0,
                    (i, squared, nextCeiling) -> maximumNextSquared(actualPath, points[i], limits[i], squared,
                            nextCeiling, positions[i + 1] - positions[i]));
            acceleration = (speeds[1] * speeds[1] - measuredVelocity * measuredVelocity)
                    / (2 * (positions[1] - positions[0]));
        } catch (CoupledMotionProfile.OverspeedException exception) {
            overspeed = true;
            MotionParameters recovery = recoveryMotion(limits[0], measuredVelocity, measuredOmega);
            acceleration = recovery.getTangentialAccel();
            recoveryOmega = recovery.getAngularVel(); recoveryAlpha = recovery.getAngularAccel();
            normalVelocity = Math.min(Math.abs(recovery.getTangentialVel()), limits[0].maxVelocity);
        }
        // A short reference preview provides velocity feedback with a reachable target while
        // the solved boundary remains the measured state. It also avoids a zero-rate heading
        // reference cancelling launch acceleration at a curved start.
        double step = .05;
        double solvedAcceleration = acceleration;
        // The published state is one control interval ahead. Validate the same acceleration
        // there too; checking only the measured state can overrun power/alpha during launch.
        if (!overspeed && acceleration > 0 && step > 0) {
            double lowAccel = 0, highAccel = acceleration;
            for (int i = 0; i < SEARCH_ITERATIONS; i++) {
                double candidate = (lowAccel + highAccel) / 2;
                double predicted = measuredVelocity + candidate * step;
                double alpha = limits[0].headingSecond * predicted * predicted + limits[0].headingPrime * candidate;
                if (predicted <= limits[0].maxVelocity + 1e-6
                        && Math.abs(alpha) <= limits[0].maxAlpha + 1e-6
                        && power(limits[0], predicted, candidate) <= utilizationLimit() + 1e-6) { lowAccel = candidate; }
                else { highAccel = candidate; }
            }
            acceleration = lowAccel;
        }
        Sample initial = limits[0];
        if (!overspeed && !validPreview(initial, measuredVelocity, acceleration)) { acceleration = solvedAcceleration; }
        double commandedVelocity = measuredVelocity + acceleration * step;
        if (!overspeed) {
            for (int i = 0; i < 32 && !validPreview(initial, Math.max(0, commandedVelocity), acceleration); i++) {
                step *= .5;
                commandedVelocity = measuredVelocity + acceleration * step;
            }
            if (!validPreview(initial, Math.max(0, commandedVelocity), acceleration)) {
                commandedVelocity = measuredVelocity; // the validated initial boundary, with its feasible braking
            }
        }
        if (measuredVelocity >= 0 && commandedVelocity < 0) { commandedVelocity = 0; }
        return new WindowTarget(new MotionParameters(commandedVelocity, acceleration,
                overspeed ? recoveryOmega : initial.headingPrime * commandedVelocity,
                overspeed ? recoveryAlpha : initial.headingSecond * commandedVelocity * commandedVelocity + initial.headingPrime * acceleration, start), overspeed,
                Double.isFinite(normalVelocity) ? normalVelocity : commandedVelocity);
    }

    private boolean validPreview(Sample sample, double velocity, double acceleration) {
        return velocity <= sample.maxVelocity + 1e-6
                && Math.abs(sample.headingSecond * velocity * velocity + sample.headingPrime * acceleration) <= sample.maxAlpha + 1e-6
                && power(sample, velocity, acceleration) <= utilizationLimit() + 1e-6;
    }

    private MotionParameters recoveryMotion(Sample sample, double velocity, double measuredOmega) {
        double sign = velocity < 0 ? 1 : -1;
        double omega = Double.isFinite(measuredOmega) ? measuredOmega : sample.headingPrime * velocity;
        // An overspeed state need not satisfy omega = headingPrime*v. Brake translation
        // from its actual speed and slew the angular reference from the measured rate.
        for (int i = 128; i >= 0; i--) {
            double a = sign * Math.min(sample.maxAccel, Math.abs(velocity) / .05) * i / 128.0;
            double predicted = velocity + a * .05;
            if (velocity * predicted < 0) { predicted = 0; }
            double desiredOmega = Math.max(-sample.maxOmega, Math.min(sample.maxOmega, sample.headingPrime * predicted));
            double alpha = Math.max(-sample.maxAlpha, Math.min(sample.maxAlpha, (desiredOmega - omega) / .05));
            double normalVelocity = Math.min(Math.abs(predicted), sample.maxVelocity);
            for (int j = 32; j >= 0; j--) {
                double feasibleAlpha = alpha * j / 32.0;
                double targetOmega = omega + feasibleAlpha * .05;
                if (sample.power.utilization(velocity, a, omega, feasibleAlpha, normalVelocity) <= utilizationLimit() + 1e-6
                        && sample.power.utilization(predicted, a, targetOmega, feasibleAlpha, normalVelocity) <= utilizationLimit() + 1e-6) {
                    return new MotionParameters(predicted, a, targetOmega, feasibleAlpha, 0);
                }
            }
        }
        throw new IllegalStateException("No feasible braking command for measured overspeed " + velocity);
    }

    public static final class WindowTarget {
        public final MotionParameters motion;
        public final boolean overspeed;
        public final double normalVelocity;
        public WindowTarget(MotionParameters motion, boolean overspeed) { this(motion, overspeed, motion.getTangentialVel()); }
        public WindowTarget(MotionParameters motion, boolean overspeed, double normalVelocity) {
            this.motion = motion; this.overspeed = overspeed; this.normalVelocity = normalVelocity;
        }
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
            sample.power = powerEvaluator(actualPath, point);
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
            double candidate = maximumPowerFeasibleAcceleration(sample, v, breaks[j], breaks[j + 1]);
            if (Double.isFinite(candidate)) { best = candidate; break; }
        }
        return Double.isFinite(best) ? Math.max(0, Math.min(nextCeiling, squared + 2 * ds * best)) : Double.NaN;
    }

    /** Each signed model branch is convex in acceleration for a fixed velocity. */
    private double maximumPowerFeasibleAcceleration(Sample sample, double v, double min, double max) {
        if (power(sample, v, max) <= utilizationLimit()) { return max; }
        double minimum = min;
        if (power(sample, v, min) > utilizationLimit()) {
            double left = min, right = max;
            for (int i = 0; i < SEARCH_ITERATIONS; i++) {
                double a = left + (right - left) / 3, b = right - (right - left) / 3;
                if (power(sample, v, a) <= power(sample, v, b)) { right = b; }
                else { left = a; }
            }
            minimum = (left + right) / 2;
            if (power(sample, v, minimum) > utilizationLimit()) { return Double.NaN; }
        }
        double low = minimum, high = max;
        for (int i = 0; i < SEARCH_ITERATIONS; i++) {
            double mid = (low + high) / 2;
            if (power(sample, v, mid) <= utilizationLimit()) { low = mid; }
            else { high = mid; }
        }
        return low;
    }
    private double maximumSteadyVelocity(Path actualPath, PathPoint point, Sample sample) {
        double low = 0, high = sample.maxVelocity;
        for (int i = 0; i < SEARCH_ITERATIONS; i++) {
            double mid = (low + high) / 2;
            if (Math.abs(sample.headingSecond * mid * mid) <= sample.maxAlpha && power(sample, mid, 0) <= utilizationLimit()) { low = mid; }
            else { high = mid; }
        }
        return low;
    }
    private static double nonNegative(double value) { return Double.isFinite(value) ? Math.max(0, value) : 0; }
    private static double ratio(double value, double bound) { return bound > 0 ? value / bound : value <= 1e-9 ? 0 : Double.POSITIVE_INFINITY; }
    private static final class Sample { PowerEvaluator power; double maxVelocity, maxAccel, maxOmega, maxAlpha, headingPrime, headingSecond; }

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
