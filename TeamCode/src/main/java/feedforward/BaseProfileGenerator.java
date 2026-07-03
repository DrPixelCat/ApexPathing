package feedforward;

import java.util.ArrayList;
import java.util.List;

import core.FollowerConstants;
import geometry.PathPoint;
import geometry.Vector;
import paths.constraint.AngularConstraint;
import paths.constraint.ConstraintType;
import paths.constraint.PathConstraint;
import paths.constraint.TranslationalConstraint;
import paths.movements.FollowerMovement;
import paths.movements.Path;

/**
 * Shared path-parameterized profile generator.
 * <p>
 * Subclasses provide the drivetrain-specific power model through {@link #evaluatePoint}.
 */
public abstract class BaseProfileGenerator {
    private static final double UTILIZATION_LIMIT = 1.0;
    private static final double UTILIZATION_TOLERANCE = 1e-3;
    private static final double EPSILON = 1e-6;
    private static final int PIN_SEARCH_ITERATIONS = 24;

    protected FollowerMovement path;
    private DebugReport lastReport;

    public DebugReport getLastDebugReport() {
        return lastReport;
    }

    // region Abstract Methods
    protected abstract double calculateMaxTangentialVelocity(PathPoint point, Path path,
                                                             double maxAngVel, double maxAngAccel);

    /**
     * Evaluates normalized drivetrain utilization for the segment ending at {@code current}.
     */
    protected abstract void evaluatePoint(
            Path path, PathPoint prev, PathPoint current,
            double v_prev, double v, double a_t,
            EvaluationResult outResult
    );

    protected abstract double getMaxTangentialAccel(double currentVel, PathPoint point, Path path
            , double maxAngAccel);

    protected abstract double calculateDynamicMaxAccel(double currentVel, PathPoint point,
                                                       Path path, double maxAngAccel);

    // region Master Loop

    /**
     * Builds the first pass profile before the iterative pinning loop.
     * Useful for comparing the raw constraint sweep against {@link #generate()}.
     */
    public MotionParameters[] generateInitialProfile() {
        Path path = (Path) this.path;
        PathPoint[] points = path.getGeneratedPoints();

        MotionParameters[] outputParams = generateBasePass(points, path);
        runBackwardPass(outputParams, points, path);
        runForwardPass(outputParams, points, path);

        populateKinematicsAndPower(outputParams, points, path);
        return outputParams;
    }

    /**
     * Generates a velocity profile and tightens any point that exceeds drivetrain utilization.
     */
    public MotionParameters[] generate() {
        if (!(path instanceof Path)) {
            throw new IllegalArgumentException("BaseFeedforwardGen only handles Path movements.");
        }

        Path path = (Path) this.path;
        PathPoint[] points = path.getGeneratedPoints();
        if (points == null || points.length == 0) {
            throw new IllegalStateException("Points must be set before generating.");
        }

        lastReport = new DebugReport();

        MotionParameters[] outputParams = generateBasePass(points, path);
        runBackwardPass(outputParams, points, path);
        runForwardPass(outputParams, points, path);

        ProfileEvaluation profileEval = populateKinematicsAndPower(outputParams, points, path);
        int maxIterations = Math.max(25, points.length * 2);
        int iterations = 0;

        while (profileEval.maxUtilization > UTILIZATION_LIMIT + UTILIZATION_TOLERANCE
                && iterations < maxIterations) {
            int checkIndex = Math.max(1, profileEval.worstIndex);
            int pinIndex = choosePinIndex(outputParams, checkIndex);

            double previousVelocity = outputParams[pinIndex].getTangentialVel();
            double pinnedVelocity = findPinnedVelocity(
                    outputParams, points, path, pinIndex, checkIndex, previousVelocity
            );

            if (previousVelocity - pinnedVelocity < EPSILON) {
                pinnedVelocity = previousVelocity * 0.98;
            }

            outputParams[pinIndex].setTangentialVel(Math.max(0.0, pinnedVelocity));

            IterationLog log = new IterationLog();
            log.iteration = iterations + 1;
            log.pinnedIndex = pinIndex;
            log.previousVelocity = previousVelocity;
            log.newVelocity = outputParams[pinIndex].getTangentialVel();
            log.maxUtilization = profileEval.maxUtilization;
            log.totalPower = profileEval.totalPower;
            log.pForward = profileEval.pForward;
            log.pLateral = profileEval.pLateral;
            log.pHeading = profileEval.pHeading;
            lastReport.logs.add(log);

            runBackwardPass(outputParams, points, path);
            runForwardPass(outputParams, points, path);
            profileEval = populateKinematicsAndPower(outputParams, points, path);
            iterations++;
        }

        lastReport.iterationsRun = Math.max(1, iterations + 1);
        lastReport.finalMaxUtilization = profileEval.maxUtilization;
        lastReport.converged =
                profileEval.maxUtilization <= UTILIZATION_LIMIT + UTILIZATION_TOLERANCE;

        return outputParams;
    }

    /**
     * Calculates and populates the remaining kinematic variables (acceleration, angular velocity, power)
     * based on the final velocity sweep.
     */
    private ProfileEvaluation populateKinematicsAndPower(MotionParameters[] lut, PathPoint[] points,
                                                         Path path) {
        EvaluationResult currentEval = new EvaluationResult();
        ProfileEvaluation profileEval = new ProfileEvaluation();
        Vector finalTangent = path.getParametricPath().getFirstDerivative(1.0);
        lut[0].setTangentialAccel(0.0);
        lut[0].setAngularVel(0.0);
        lut[0].setAngularAccel(0.0);

        for (int i = 1; i < points.length; i++) {
            double ds = Math.abs(points[i].getDistanceToEnd_in() - points[i - 1].getDistanceToEnd_in());
            double v = lut[i].getTangentialVel();
            double v_prev = lut[i - 1].getTangentialVel();
            double a_t = (ds < EPSILON) ? 0.0 : ((v * v) - (v_prev * v_prev)) / (2.0 * ds);

            lut[i].setTangentialAccel(a_t);

            double s = points[i].getDistanceToEnd_in();
            double kappa = points[i].getSignedCurvature();
            double dKappa = points[i].getCurvatureDerivative();

            double fPrime = path.getInterpolator().getHeadingFirstDerivative(s, kappa, finalTangent);
            double fDoublePrime = path.getInterpolator().getHeadingSecondDerivative(s, dKappa, finalTangent);

            lut[i].setAngularVel(fPrime * v);
            lut[i].setAngularAccel((fDoublePrime * (v * v)) + (fPrime * a_t));

            evaluatePoint(path, points[i - 1], points[i], v_prev, v, a_t, currentEval);
            lut[i].setMotorPower(currentEval.totalPower);

            if (currentEval.maxUtilization > profileEval.maxUtilization) {
                profileEval.worstIndex = i;
                profileEval.maxUtilization = currentEval.maxUtilization;
                profileEval.totalPower = currentEval.totalPower;
                profileEval.pForward = currentEval.pForward;
                profileEval.pLateral = currentEval.pLateral;
                profileEval.pHeading = currentEval.pHeading;
            }
        }

        lut[0].setMotorPower(lut.length > 1 ? lut[1].getMotorPower() : 0.0);
        return profileEval;
    }

    public FeedforwardLut generateQuick(FollowerConstants config) {
        Path path = (Path) this.path;
        PathPoint[] points = path.getGeneratedPoints();
        MotionParameters[] lut = new MotionParameters[points.length];

        // Base pass: Max velocity everywhere
        for (int i = 0; i < points.length; i++) {
            lut[i] = new MotionParameters();
            lut[i].setTangentialVel(config.forwardVelocityLimit.getIn());
        }

        // Backward pass: Naive deceleration
        // If boosted, relax the boundary condition so the path ends at cruising speed
        if (!path.isAccelBoosted()) {
            lut[points.length - 1].setTangentialVel(0.0);
        }

        for (int i = points.length - 2; i >= 0; i--) {
            double ds =
                    Math.abs(points[i + 1].getDistanceToEnd_in() - points[i].getDistanceToEnd_in());
            double nextVel = lut[i + 1].getTangentialVel();
            double maxReachableVel =
                    Math.sqrt((nextVel * nextVel) + (2.0 * config.forwardAccelerationLimit.getIn() * ds));
            lut[i].setTangentialVel(Math.min(lut[i].getTangentialVel(), maxReachableVel));
        }

        // Forward pass: Naive acceleration and populate angular targets
        // If boosted, relax the boundary condition so the path begins at cruising speed
        if (!path.isAccelBoosted()) {
            lut[0].setTangentialVel(0.0);
        }
        Vector finalTangent = path.getParametricPath().getFirstDerivative(1.0);

        for (int i = 1; i < points.length; i++) {
            double ds =
                    Math.abs(points[i].getDistanceToEnd_in() - points[i - 1].getDistanceToEnd_in());
            double prevVel = lut[i - 1].getTangentialVel();
            double maxReachableVel =
                    Math.sqrt((prevVel * prevVel) + (2.0 * config.forwardAccelerationLimit.getIn() * ds));

            double v = Math.min(lut[i].getTangentialVel(), maxReachableVel);
            lut[i].setTangentialVel(v);

            double a_t = (ds < EPSILON) ? 0.0 : ((v * v) - (prevVel * prevVel)) / (2.0 * ds);
            lut[i].setTangentialAccel(a_t);

            double s = points[i].getDistanceToEnd_in();
            double kappa = points[i].getSignedCurvature();
            double dKappa = points[i].getCurvatureDerivative();

            double fPrime = path.getInterpolator().getHeadingFirstDerivative(s, kappa,
                    finalTangent);
            double fDoublePrime = path.getInterpolator().getHeadingSecondDerivative(s, dKappa,
                    finalTangent);

            lut[i].setAngularVel(fPrime * v);
            lut[i].setAngularAccel((fDoublePrime * (v * v)) + (fPrime * a_t));
        }

        return new FeedforwardLut(lut);
    }

    // region Fwd/Bkwd Passes

    private MotionParameters[] generateBasePass(PathPoint[] points, Path path) {
        MotionParameters[] lut = new MotionParameters[points.length];
        double pathLength_in = Math.max(path.getParametricPath().getLengthIn(), EPSILON);
        PathConstraint[] constraints = path.getConstraints();

        for (int i = 0; i < points.length; i++) {
            double pctCompleted = 1.0 - (points[i].getDistanceToEnd_in() / pathLength_in);
            double currentMaxVel = Double.MAX_VALUE;
            double currentMaxAngVel = Double.MAX_VALUE;
            double currentMaxAngAccel = Double.MAX_VALUE;
            for (PathConstraint constraint : constraints) {
                if (constraint.getS() <= pctCompleted) {
                    if (constraint instanceof TranslationalConstraint && constraint.getType() == ConstraintType.VELOCITY)  {
                        currentMaxVel = ((TranslationalConstraint) constraint).getValueIn();
                    } else if (constraint instanceof AngularConstraint && constraint.getType() == ConstraintType.VELOCITY) {
                        currentMaxAngVel = ((AngularConstraint) constraint).getValueRad();
                    } else if (constraint instanceof AngularConstraint && constraint.getType() == ConstraintType.ACCELERATION) {
                        currentMaxAngAccel = ((AngularConstraint) constraint).getValueRad();
                    }
                }
            }

            double maxVel = calculateMaxTangentialVelocity(points[i], path, currentMaxAngVel, currentMaxAngAccel);
            if (currentMaxVel != Double.MAX_VALUE && currentMaxVel > 0.0) {
                maxVel = Math.min(maxVel, currentMaxVel);
            }

            lut[i] = new MotionParameters().setTangentialVel(maxVel);
        }
        return lut;
    }

    private void runBackwardPass(MotionParameters[] lut, PathPoint[] points, Path path) {
        if (!path.isAccelBoosted()) {
            lut[points.length - 1].setTangentialVel(0.0);
        }

        double pathLength_in = Math.max(path.getParametricPath().getLengthIn(), EPSILON);
        PathConstraint[] constraints = path.getConstraints();

        for (int i = points.length - 2; i >= 0; i--) {
            double ds =
                    Math.abs(points[i + 1].getDistanceToEnd_in() - points[i].getDistanceToEnd_in());
            double nextVel = lut[i + 1].getTangentialVel();
            if (ds <= EPSILON) {
                lut[i].setTangentialVel(Math.min(lut[i].getTangentialVel(), nextVel));
                continue;
            }

            double pctCompleted = 1.0 - (points[i + 1].getDistanceToEnd_in() / pathLength_in);
            double currentMaxAccel = Double.MAX_VALUE;
            double currentMaxAngAccel = Double.MAX_VALUE;

            for (PathConstraint constraint : constraints) {
                if (constraint.getS() <= pctCompleted) {
                    if (constraint instanceof TranslationalConstraint &&
                            constraint.getType() == ConstraintType.ACCELERATION) {
                        currentMaxAccel = ((TranslationalConstraint) constraint).getValueIn();
                    } else if (constraint instanceof AngularConstraint &&
                            constraint.getType() == ConstraintType.ACCELERATION) {
                        currentMaxAngAccel = ((AngularConstraint) constraint).getValueRad();
                    }
                }
            }

            double maxAccel = getMaxTangentialAccel(nextVel, points[i + 1], path,
                    currentMaxAngAccel);
            if (currentMaxAccel != Double.MAX_VALUE && currentMaxAccel > 0.0) {
                maxAccel = Math.min(maxAccel, currentMaxAccel);
            }
            maxAccel = sanitizeNonNegative(maxAccel);

            double maxReachableVel = safeReachableVelocity(nextVel, maxAccel, ds);
            double cappedVel = Math.min(lut[i].getTangentialVel(), maxReachableVel);
            if (ds > EPSILON && cappedVel > nextVel + EPSILON) {
                cappedVel = findMaxPreviousVelocity(
                        path, points[i], points[i + 1], nextVel, cappedVel, maxAccel, ds
                );
            }
            lut[i].setTangentialVel(cappedVel);
        }
    }

    private void runForwardPass(MotionParameters[] lut, PathPoint[] points, Path path) {
        /* If boosted, relax the boundary condition so the path begins at cruising speed so the
        feedback controller can provide maximum +/- power */
        if (!path.isAccelBoosted()) {
            lut[0].setTangentialVel(0.0);
        }

        double pathLength_in = Math.max(path.getParametricPath().getLengthIn(), EPSILON);
        PathConstraint[] constraints = path.getConstraints();

        for (int i = 1; i < points.length; i++) {
            double ds =
                    Math.abs(points[i].getDistanceToEnd_in() - points[i - 1].getDistanceToEnd_in());
            double prevVel = lut[i - 1].getTangentialVel();
            if (ds <= EPSILON) {
                lut[i].setTangentialVel(Math.min(lut[i].getTangentialVel(), prevVel));
                continue;
            }

            double pctCompleted = 1.0 - (points[i].getDistanceToEnd_in() / pathLength_in);
            double currentMaxAccel = Double.MAX_VALUE;
            double currentMaxAngAccel = Double.MAX_VALUE;

            for (PathConstraint constraint : constraints) {
                if (constraint.getS() <= pctCompleted) {
                    if (constraint instanceof TranslationalConstraint && constraint.getType() == ConstraintType.ACCELERATION) {
                        currentMaxAccel = ((TranslationalConstraint) constraint).getValueIn();
                    } else if (constraint instanceof AngularConstraint && constraint.getType() == ConstraintType.ACCELERATION) {
                        currentMaxAngAccel = ((AngularConstraint) constraint).getValueRad();
                    }
                }
            }

            double dynamicAccel = calculateDynamicMaxAccel(prevVel, points[i], path,
                    currentMaxAngAccel);
            if (currentMaxAccel != Double.MAX_VALUE && currentMaxAccel > 0.0) {
                dynamicAccel = Math.min(dynamicAccel, currentMaxAccel);
            }
            dynamicAccel = sanitizeNonNegative(dynamicAccel);

            double maxReachableVel = safeReachableVelocity(prevVel, dynamicAccel, ds);
            double cappedVel = Math.min(lut[i].getTangentialVel(), maxReachableVel);
            if (ds > EPSILON && cappedVel > prevVel + EPSILON) {
                cappedVel = findMaxNextVelocity(
                        path, points[i - 1], points[i], prevVel, cappedVel, dynamicAccel, ds
                );
            }
            lut[i].setTangentialVel(cappedVel);
        }
    }

    private int choosePinIndex(MotionParameters[] lut, int worstIndex) {
        int preferredIndex = worstIndex;
        if (worstIndex > 0 && lut[worstIndex].getTangentialAccel() < -EPSILON) {
            preferredIndex = worstIndex - 1;
        } else if (lut[worstIndex].getTangentialVel() <= EPSILON && worstIndex > 0) {
            preferredIndex = worstIndex - 1;
        }

        if (lut[preferredIndex].getTangentialVel() > EPSILON) {
            return preferredIndex;
        }

        for (int i = worstIndex - 1; i >= 0; i--) {
            if (lut[i].getTangentialVel() > EPSILON) {
                return i;
            }
        }

        for (int i = worstIndex + 1; i < lut.length; i++) {
            if (lut[i].getTangentialVel() > EPSILON) {
                return i;
            }
        }

        return preferredIndex;
    }

    private double findPinnedVelocity(MotionParameters[] currentProfile, PathPoint[] points,
                                      Path path, int pinIndex, int checkIndex,
                                      double currentVelocity) {
        if (currentVelocity <= EPSILON) {
            return 0.0;
        }

        double low = 0.0;
        double high = currentVelocity;
        double bestVelocity = 0.0;
        int boundedCheckIndex = Math.max(1, Math.min(checkIndex, currentProfile.length - 1));

        for (int i = 0; i < PIN_SEARCH_ITERATIONS; i++) {
            double candidate = (low + high) / 2.0;
            MotionParameters[] trialProfile = copyProfile(currentProfile);
            trialProfile[pinIndex].setTangentialVel(candidate);

            runBackwardPass(trialProfile, points, path);
            runForwardPass(trialProfile, points, path);
            populateKinematicsAndPower(trialProfile, points, path);

            if (trialProfile[boundedCheckIndex].getMotorPower()
                    <= UTILIZATION_LIMIT + UTILIZATION_TOLERANCE) {
                bestVelocity = trialProfile[pinIndex].getTangentialVel();
                low = candidate;
            } else {
                high = candidate;
            }
        }

        return Math.min(bestVelocity, currentVelocity);
    }

    private MotionParameters[] copyProfile(MotionParameters[] profile) {
        MotionParameters[] copy = new MotionParameters[profile.length];
        for (int i = 0; i < profile.length; i++) {
            copy[i] = new MotionParameters()
                    .setTangentialVel(profile[i].getTangentialVel())
                    .setTangentialAccel(profile[i].getTangentialAccel())
                    .setAngularVel(profile[i].getAngularVel())
                    .setAngularAccel(profile[i].getAngularAccel());
            copy[i].setMotorPower(profile[i].getMotorPower());
            copy[i].setDistAlongCurve(profile[i].getDistAlongCurve());
        }
        return copy;
    }

    private double findMaxPreviousVelocity(Path path, PathPoint prevPoint,
                                           PathPoint currentPoint, double nextVel,
                                           double upperVel, double maxDecel, double ds) {
        if (upperVel <= nextVel + EPSILON) {
            return upperVel;
        }

        double low = Math.min(nextVel, upperVel);
        double high = upperVel;
        if (isBackwardTransitionFeasible(path, prevPoint, currentPoint, nextVel, high,
                maxDecel, ds)) {
            return high;
        }

        for (int i = 0; i < PIN_SEARCH_ITERATIONS; i++) {
            double candidate = (low + high) / 2.0;
            if (isBackwardTransitionFeasible(path, prevPoint, currentPoint, nextVel, candidate,
                    maxDecel, ds)) {
                low = candidate;
            } else {
                high = candidate;
            }
        }

        return low;
    }

    private double findMaxNextVelocity(Path path, PathPoint prevPoint, PathPoint currentPoint,
                                       double prevVel, double upperVel, double maxAccel,
                                       double ds) {
        if (upperVel <= prevVel + EPSILON) {
            return upperVel;
        }

        double low = Math.min(prevVel, upperVel);
        double high = upperVel;
        if (isForwardTransitionFeasible(path, prevPoint, currentPoint, prevVel, high,
                maxAccel, ds)) {
            return high;
        }

        for (int i = 0; i < PIN_SEARCH_ITERATIONS; i++) {
            double candidate = (low + high) / 2.0;
            if (isForwardTransitionFeasible(path, prevPoint, currentPoint, prevVel, candidate,
                    maxAccel, ds)) {
                low = candidate;
            } else {
                high = candidate;
            }
        }

        return low;
    }

    private boolean isBackwardTransitionFeasible(Path path, PathPoint prevPoint,
                                                 PathPoint currentPoint, double nextVel,
                                                 double prevVel, double maxDecel, double ds) {
        double accel = calculateTangentialAccel(prevVel, nextVel, ds);
        if (accel < -maxDecel - EPSILON) {
            return false;
        }
        return isPowerFeasible(path, prevPoint, currentPoint, prevVel, nextVel, accel);
    }

    private boolean isForwardTransitionFeasible(Path path, PathPoint prevPoint,
                                                PathPoint currentPoint, double prevVel,
                                                double nextVel, double maxAccel, double ds) {
        double accel = calculateTangentialAccel(prevVel, nextVel, ds);
        if (accel > maxAccel + EPSILON) {
            return false;
        }
        return isPowerFeasible(path, prevPoint, currentPoint, prevVel, nextVel, accel);
    }

    private boolean isPowerFeasible(Path path, PathPoint prevPoint, PathPoint currentPoint,
                                    double prevVel, double nextVel, double accel) {
        EvaluationResult result = new EvaluationResult();
        evaluatePoint(path, prevPoint, currentPoint, prevVel, nextVel, accel, result);
        return result.maxUtilization <= UTILIZATION_LIMIT + UTILIZATION_TOLERANCE;
    }

    private double calculateTangentialAccel(double prevVel, double nextVel, double ds) {
        if (ds <= EPSILON) {
            return 0.0;
        }
        return ((nextVel * nextVel) - (prevVel * prevVel)) / (2.0 * ds);
    }

    private double safeReachableVelocity(double startVelocity, double accel, double ds) {
        if (ds <= EPSILON) {
            return startVelocity;
        }

        double velocitySq = (startVelocity * startVelocity) + (2.0 * accel * ds);
        return Math.sqrt(Math.max(0.0, velocitySq));
    }

    private double sanitizeNonNegative(double value) {
        if (!Double.isFinite(value) || value < 0.0) {
            return 0.0;
        }
        return value;
    }

    // region Logging and CSV Export

    public static class EvaluationResult {
        public double totalPower = 0.0;
        public double maxUtilization = 0.0;
        public double pForward = 0.0;
        public double pLateral = 0.0;
        public double pHeading = 0.0;

        public void copyFrom(EvaluationResult other) {
            this.totalPower = other.totalPower;
            this.maxUtilization = other.maxUtilization;
            this.pForward = other.pForward;
            this.pLateral = other.pLateral;
            this.pHeading = other.pHeading;
        }
    }

    public static class DebugReport {
        public int iterationsRun = 0;
        public boolean converged = false;
        public double finalMaxUtilization = 0.0;
        public List<IterationLog> logs = new ArrayList<>();

        public String getSummary() {
            return String.format("Converged: %b | Iterations: %d | Final Max Util: %.3f",
                    converged, iterationsRun, finalMaxUtilization);
        }
    }

    public static class IterationLog {
        public int iteration;
        public int pinnedIndex;
        public double previousVelocity, newVelocity;
        public double maxUtilization, totalPower;
        public double pForward, pLateral, pHeading;
    }

    private static class ProfileEvaluation {
        int worstIndex = 0;
        double maxUtilization = 0.0;
        double totalPower = 0.0;
        double pForward = 0.0;
        double pLateral = 0.0;
        double pHeading = 0.0;
    }
}
