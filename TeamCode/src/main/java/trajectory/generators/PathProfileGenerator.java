package trajectory.generators;

import androidx.annotation.NonNull;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;
import follower.FollowerConstants;
import drivetrains.models.DriveModel;
import drivetrains.models.DriveModel.PowerEvaluator;
import feedforward.MotionCharacterization.Axis;
import trajectory.profile.CoupledMotionProfile;
import trajectory.profile.MotionParameters;
import trajectory.profile.FFLut;
import trajectory.VelocityPursuit;
import geometry.PathPoint;
import geometry.Vector;
import paths.constraint.PathConstraint;
import paths.movements.Path;

/**
 * Shared sampled path solver with signed, coupled power and acceleration constraints.
 *
 * @author Joel H
 */
public class PathProfileGenerator {
    /** Optional per-path construction hook shared by online and stored planning. */
    @FunctionalInterface
    public interface Factory {
        PathProfileGenerator create(Path path, FollowerConstants settings, DriveModel model);
    }
    protected static final double EPSILON = 1e-6;
    private static final double UTILIZATION_TOLERANCE = 1e-6;
    private static final int SEARCH_ITERATIONS = 32;
    private final Double requestedPowerLimit;
    protected final FollowerConstants constants;
    protected final Path path;
    private DebugReport lastReport;
    private int lastWindowSampleCount;
    private ConstraintSchedule constraintSchedule;
    private long constraintRevision = Long.MIN_VALUE;
    private geometry.PathSegment scheduledSegment;
    private Sample[] cachedSamples;
    private final java.util.IdentityHashMap<PathPoint, Sample> cachedTransitionSamples = new java.util.IdentityHashMap<>();
    private geometry.PathSegment cachedSegment;
    private paths.heading.HeadingInterpolator cachedInterpolator;
    private ConstraintSchedule cachedSchedule;
    private long cachedHeadingRevision;
    private feedforward.MotionCharacterization cachedCharacterization;
    private final double[] cachedPlanningValues = new double[9];
    private double[] windowPositions = new double[16], windowCeilings = new double[16];
    private PathPoint[] windowGeometry = new PathPoint[16];
    private Sample[] windowSamples = new Sample[16];
    private int windowSize;
    private final double[] accelerationBreaks = new double[4];
    private final CoupledMotionProfile.Workspace windowSolver = new CoupledMotionProfile.Workspace();
    private final CoupledMotionProfile.Transitions windowTransitions = transitionsForWindow();
    private CoupledMotionProfile.Transitions transitionsForWindow() {
        return new CoupledMotionProfile.Transitions() {
            @Override public double maximumNextSquared(int i, double squared, double nextCeiling) {
                return PathProfileGenerator.this.maximumNextSquared(windowSamples[i], windowSamples[i + 1],
                        squared, nextCeiling, windowPositions[i + 1] - windowPositions[i]);
            }
            @Override public double previousSquaredUpperBound(int i, double nextCeiling) {
                return predecessorBound(nextCeiling, windowSamples[i], windowPositions[i + 1] - windowPositions[i]);
            }
        };
    }
    private static double predecessorBound(double nextCeiling, Sample sample, double distance) {
        // qNext = q + 2*a*ds and a >= -maxAccel, regardless of path shape or power model.
        double upper = nextCeiling + 2 * distance * sample.maxAccel;
        // |headingSecond*q + headingPrime*a| <= maxAlpha also gives a necessary
        // bound without assuming that braking and heading acceleration have the same sign.
        if (Math.abs(sample.headingSecond) > 1e-12) {
            upper = Math.min(upper, (sample.maxAlpha + Math.abs(sample.headingPrime) * sample.maxAccel)
                    / Math.abs(sample.headingSecond));
        }
        double b = sample.headingPrime / (2 * distance), a = sample.headingSecond - b;
        if (Math.abs(a) > 1e-12) {
            // alpha = a*q + b*qNext, with 0 <= qNext <= nextCeiling. Choose
            // the most favorable next-speed endpoint to obtain an outer feasibility bound.
            upper = Math.min(upper, (sample.maxAlpha + Math.max(0, -Math.signum(a) * b * nextCeiling)) / Math.abs(a));
        }
        return Math.nextUp(upper);
    }
    public int getLastWindowSampleCount() { return lastWindowSampleCount; }
    private final VelocityPursuit velocityPursuit = new VelocityPursuit();
    private FFLut pursuitReference;
    private double[] windowGoalSquared = new double[16];
    public void resetVelocityPursuit() { velocityPursuit.reset(); }
    /** Stored paths retain their precomputed speed goals; feasibility is checked at actual speed. */
    public void setVelocityPursuitReference(FFLut reference) { pursuitReference = reference; }

    private ConstraintSchedule constraints(Path actualPath) {
        long revision = actualPath.getConstraintRevision();
        if (constraintSchedule == null || revision < 0 || revision != constraintRevision
                || scheduledSegment != actualPath.getParametricPath()) {
            constraintSchedule = new ConstraintSchedule(actualPath);
            constraintRevision = revision; scheduledSegment = actualPath.getParametricPath();
        }
        return constraintSchedule;
    }

    private final DriveModel driveModel;
    private geometry.PathSegment straightSegment;
    private boolean straightGeometry;
    public PathProfileGenerator(Path path) { this(path.getPlanningConstants(),path); }
    public PathProfileGenerator(Path path,DriveModel model) { this(model.getConstants(),path,model); }
    public PathProfileGenerator(Path path,DriveModel model,double powerLimit) { this(model.getConstants(),path,model,Double.valueOf(powerLimit)); }
    public PathProfileGenerator(FollowerConstants constants,Path path) { this(constants,path,path.getDriveModel(constants)); }
    public PathProfileGenerator(FollowerConstants constants,Path path,double powerLimit) { this(constants,path,path.getDriveModel(constants),Double.valueOf(powerLimit)); }
    public PathProfileGenerator(FollowerConstants constants,Path path,DriveModel model) { this(constants,path,model,path.getProfilePowerLimit()); }
    public PathProfileGenerator(FollowerConstants constants,Path path,DriveModel model,double powerLimit) { this(constants,path,model,Double.valueOf(powerLimit)); }
    private PathProfileGenerator(FollowerConstants constants,Path path,DriveModel model,Double powerLimit) {
        if(powerLimit!=null && (!Double.isFinite(powerLimit) || powerLimit<=0 || powerLimit>1))
            throw new IllegalArgumentException("Profile power limit must be finite and in (0, 1].");
        if(model.isHolonomic() != (path.getPathType()==Path.PathType.HOLONOMIC))
            throw new IllegalArgumentException("Drive model does not match path mode");
        this.constants=constants; this.path=path; requestedPowerLimit=powerLimit;
        driveModel=model.withConstants(constants);
    }
    public DriveModel getDriveModel() { return driveModel; }
    private Vector bodyDirection(Path actual,PathPoint point) {
        geometry.Angle heading=actual.getInterpolator().getHeadingTarg(point.getDistanceToEndIn(),point.getFirstDerivative(),
                actual.getParametricPath().getFirstDerivative(1));
        return point.getFirstDerivative().normalize().rotate(heading.times(-1));
    }
    private double calculateMaxTangentialVelocity(PathPoint point,Path actual,double maxAngVel) {
        double prime=actual.getInterpolator().getHeadingFirstDerivative(point.getDistanceToEndIn(),point.getSignedCurvature(),
                actual.getParametricPath().getFirstDerivative(1));
        return driveModel.maximumVelocity(bodyDirection(actual,point),point.getSignedCurvature(),prime,maxAngVel);
    }
    private double getPhysicalTangentialAccel(PathPoint point,Path actual) {
        return driveModel.maximumTangentialAcceleration(bodyDirection(actual,point));
    }
    private boolean enforceIntervalPowerBounds() {
        Path actual=path;
        if(straightSegment!=actual.getParametricPath()) {
            straightSegment=actual.getParametricPath(); straightGeometry=true;
            for(PathPoint point:actual.getGeneratedPoints()) if(Math.abs(point.getSignedCurvature())>1e-9) { straightGeometry=false;break; }
        }
        return driveModel.enforceIntervalPowerBounds(straightGeometry);
    }
    private boolean cacheSolverSamples() { return driveModel.cacheSolverSamples(); }
    private PowerEvaluator powerEvaluator(Path actual,PathPoint point) {
        return driveModel.powerEvaluator(bodyDirection(actual,point),point.getSignedCurvature());
    }
    public double getProfilePowerLimit() {
        if (requestedPowerLimit == null) { return constants.profilePowerLimit(); }
        return constants.motionCharacterization == null ? requestedPowerLimit
                : Math.min(requestedPowerLimit, constants.motionCharacterization.availableMotorPower);
    }
    protected double utilizationLimit() { return getProfilePowerLimit(); }
    public DebugReport getLastDebugReport() { return lastReport; }
    protected boolean useAnalyticPowerBounds() { return true; }
    private double power(Sample sample, double v, double a) {
        return sample.power.utilization(v, a, sample.headingPrime * v, sample.headingSecond * v * v + sample.headingPrime * a, v);
    }

    public MotionParameters[] generate() { return generate(path.isAccelBoosted(), false); }
    public MotionParameters[] generateRestToRest() { return generate(false, true); }

    private MotionParameters[] generate(boolean boosted, boolean exactTransitions) {
        final Path actualPath = path;
        if (exactTransitions && actualPath.getParametricPath().getLengthIn() <= 1e-9)
            return new MotionParameters[]{new MotionParameters()};
        PathPoint[] points = profilePoints(actualPath, exactTransitions);
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
                new CoupledMotionProfile.Transitions() {
                    @Override public double maximumNextSquared(int i, double squared, double nextCeiling) {
                        return PathProfileGenerator.this.maximumNextSquared(samples[i], samples[i + 1], squared,
                                nextCeiling, positions[i + 1] - positions[i]);
                    }
                    @Override public double previousSquaredUpperBound(int i, double nextCeiling) {
                        return predecessorBound(nextCeiling, samples[i], positions[i + 1] - positions[i]);
                    }
                });
        return buildValidatedProfile(samples, positions, velocities);
    }

    private PathPoint[] profilePoints(Path actualPath, boolean exactTransitions) {
        PathPoint[] original = actualPath.getGeneratedPoints();
        PathPoint[] points;
        double pathLength = actualPath.getParametricPath().getLengthIn();
        if (exactTransitions) {
            java.util.TreeSet<Double> distances = new java.util.TreeSet<>();
            for (PathPoint point : original) { distances.add(pathLength - point.getDistanceToEndIn()); }
            if (original.length < 3 && pathLength > 1e-6) { distances.add(pathLength / 2); }
            for (PathConstraint constraint : actualPath.getConstraints()) { distances.add(actualPath.constraintProgress(constraint.getS()) * pathLength); }
            points = new PathPoint[distances.size()]; int index = 0;
            for (double distance : distances) { points[index++] = actualPath.getParametricPath().getPointAtDistance(distance); }
        } else { points = original; }
        if (points == null || points.length < 2) { throw new IllegalStateException("At least two path points are required."); }
        return points;
    }

    private MotionParameters[] buildValidatedProfile(Sample[] samples, double[] positions, double[] velocities) {
        lastReport = new DebugReport();
        lastReport.powerLimit = utilizationLimit(); lastReport.iterationsRun = 1;
        lastReport.powerFeasible = true; lastReport.kinematicsFeasible = true;
        MotionParameters[] output = new MotionParameters[positions.length];
        double time = 0;
        for (int i = 0; i < positions.length; i++) {
            double ds = i + 1 < positions.length ? positions[i + 1] - positions[i] : 0;
            double v = velocities[i];
            double acceleration = ds > 1e-9 ? (velocities[i + 1] * velocities[i + 1] - v * v) / (2 * ds) : 0;
            Sample sample = samples[i];
            double omega = sample.headingPrime * v;
            double alpha = sample.headingSecond * v * v + sample.headingPrime * acceleration;
            double demand=power(sample,v,acceleration);
            MotionParameters state = new MotionParameters(v, acceleration, omega, alpha, positions[i]).setTimeSeconds(time);
            state.setMotorPower(demand); output[i] = state;
            lastReport.finalMaxUtilization = Math.max(lastReport.finalMaxUtilization, demand);
            double constraintRatio = Math.max(ratio(v, sample.maxVelocity), Math.max(ratio(Math.abs(acceleration), sample.maxAccel),
                    Math.max(ratio(Math.abs(omega), sample.maxOmega), ratio(Math.abs(alpha), sample.maxAlpha))));
            lastReport.finalMaxConstraintRatio = Math.max(lastReport.finalMaxConstraintRatio, constraintRatio);
            lastReport.powerFeasible &= Double.isFinite(demand) && demand <= utilizationLimit() + UTILIZATION_TOLERANCE;
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

    /** Legacy preview overload. OnlineTrajectoryPlanner explicitly chooses zero for holonomic paths. */
    public WindowTarget generateWindow(double startIn, double horizonIn, double measuredVelocity, double measuredOmega, double dt) {
        return generateWindow(startIn, horizonIn, measuredVelocity, measuredOmega, dt, .05);
    }

    /** A zero preview publishes the solved state at the measurement time; acceleration remains the command. */
    public WindowTarget generateWindow(double startIn, double horizonIn, double measuredVelocity, double measuredOmega,
                                       double dt, double previewSeconds) {
        if(!Double.isFinite(previewSeconds) || previewSeconds<0 || previewSeconds>.05)
            throw new IllegalArgumentException("Reference preview must be in [0, .05] seconds");
        return generateWindowInternal(startIn, horizonIn, measuredVelocity, measuredOmega, dt, previewSeconds, 0);
    }

    /** Converge to the independently solved stopping envelope ahead, without an integrated speed reference. */
    public WindowTarget generatePursuitWindow(double startIn,double horizonIn,double measuredVelocity,
                                             double measuredOmega,double dt,double convergenceSeconds) {
        if (!Double.isFinite(convergenceSeconds) || convergenceSeconds<=0 || convergenceSeconds>1)
            throw new IllegalArgumentException("Velocity convergence time must be in (0, 1] seconds");
        return generateWindowInternal(startIn,horizonIn,measuredVelocity,measuredOmega,dt,0,convergenceSeconds);
    }

    private WindowTarget generateWindowInternal(double startIn, double horizonIn, double measuredVelocity, double measuredOmega, double dt,
                                                double previewSeconds, double pursuitSeconds) {
        Path actualPath = path;
        geometry.PathSegment segment = actualPath.getParametricPath();
        ConstraintSchedule schedule = constraints(actualPath);
        double start = Math.max(0, Math.min(segment.getLengthIn(), startIn));
        double end = Math.min(segment.getLengthIn(), start + horizonIn);
        if (!Double.isFinite(measuredVelocity) || !Double.isFinite(horizonIn) || horizonIn <= 0) {
            throw new IllegalArgumentException("Finite velocity and positive lookahead required.");
        }
        if (end - start < 1e-6) return endpointRecovery(actualPath, segment, start, measuredVelocity,
                measuredOmega, previewSeconds);
        prepareWindow(actualPath, segment, schedule, start, end);
        Sample[] limits = windowSamples;
        double acceleration;
        double recoveryOmega = 0, recoveryAlpha = 0, normalVelocity = Double.NaN;
        boolean overspeed = measuredVelocity < 0;
        try {
            if (measuredVelocity < 0 || (Double.isFinite(measuredOmega)
                    && Math.abs(measuredOmega) > limits[0].maxOmega + 1e-6)) {
                throw new CoupledMotionProfile.OverspeedException(0);
            }
            double nextSquared = CoupledMotionProfile.solveNextSquared(windowPositions, windowCeilings, windowSize,
                    measuredVelocity, 0, windowTransitions, windowSolver);
            acceleration = (nextSquared - measuredVelocity * measuredVelocity)
                    / (2 * (windowPositions[1] - windowPositions[0]));
        } catch (CoupledMotionProfile.OverspeedException exception) {
            overspeed = true;
            MotionParameters recovery = recoveryMotion(limits[0], measuredVelocity, measuredOmega);
            acceleration = recovery.getTangentialAccel();
            recoveryOmega = recovery.getAngularVel(); recoveryAlpha = recovery.getAngularAccel();
            normalVelocity = Math.min(Math.abs(recovery.getTangentialVel()), limits[0].maxVelocity);
        }
        double goalDistance = Double.NaN, goalVelocity = Double.NaN, currentCeiling = Double.NaN;
        if (pursuitSeconds > 0 && !overspeed) {
            for (int i = 0; i < windowSize; i++) {
                windowGoalSquared[i] = windowSolver.controllableSquaredAt(i);
                if (pursuitReference != null) {
                    windowGoalSquared[i] = Math.min(windowGoalSquared[i],
                            pursuitReference.getSquaredTangentialVelocity(windowPositions[i]));
                }
            }
            VelocityPursuit.Goal goal = velocityPursuit.goal(start, end, measuredVelocity,
                    pursuitSeconds, windowPositions, windowGoalSquared, windowSize);
            goalDistance = goal.distanceIn; goalVelocity = goal.velocity;
            currentCeiling = Math.sqrt(Math.max(0, windowSolver.controllableSquaredAt(0)));
            acceleration = constrainPursuitAcceleration(limits[0], measuredVelocity,
                    velocityPursuit.damp(goal, start, measuredVelocity, pursuitSeconds, dt), acceleration,
                    windowPositions[1] - start);
        }
        double step = previewSeconds;
        Sample initial = limits[0];
        if (!overspeed) acceleration = previewAcceleration(initial, measuredVelocity, acceleration, step);
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
        if (pursuitSeconds > 0) velocityPursuit.applied(acceleration);
        double commandedOmega=overspeed && Double.isFinite(measuredOmega)
                ? measuredOmega+recoveryAlpha*step : overspeed?recoveryOmega:initial.headingPrime*commandedVelocity;
        MotionParameters motion = new MotionParameters(commandedVelocity, acceleration,
                commandedOmega,
                overspeed ? recoveryAlpha : initial.headingSecond * commandedVelocity * commandedVelocity + initial.headingPrime * acceleration, start);
        double commandedNormalVelocity = Double.isFinite(normalVelocity) ? normalVelocity : commandedVelocity;
        motion.setMotorPower(initial.power.utilization(commandedVelocity, acceleration,
                motion.getAngularVel(), motion.getAngularAccel(), commandedNormalVelocity));
        return new WindowTarget(motion, overspeed, commandedNormalVelocity,goalDistance,goalVelocity,currentCeiling,
                windowGeometry[0]);
    }

    /** Handle the degenerate window at the endpoint without entering the spatial solver. */
    private WindowTarget endpointRecovery(Path actualPath, geometry.PathSegment segment, double start,
            double measuredVelocity, double measuredOmega, double previewSeconds) {
        lastWindowSampleCount = 1;
        if (Math.abs(measuredVelocity) < 1e-6 && (!Double.isFinite(measuredOmega) || Math.abs(measuredOmega) < 1e-6)) {
            return new WindowTarget(new MotionParameters(0, 0, 0, 0, start), false);
        }
        Sample boundary = samples(actualPath, new PathPoint[]{segment.getPointAtDistance(start)})[0];
        MotionParameters recovery = recoveryMotion(boundary, measuredVelocity, measuredOmega);
        if(previewSeconds==0) recovery.setTangentialVel(measuredVelocity)
                .setAngularVel(Double.isFinite(measuredOmega)?measuredOmega:recovery.getAngularVel());
        recovery.setDistAlongCurve(start);
        recovery.setMotorPower(boundary.power.utilization(recovery.getTangentialVel(),
                recovery.getTangentialAccel(), recovery.getAngularVel(), recovery.getAngularAccel(), 0));
        return new WindowTarget(recovery, true, 0);
    }

    /** Merge geometry, constraint transitions, and boundary samples into reusable window storage. */
    private void prepareWindow(Path actualPath, geometry.PathSegment segment, ConstraintSchedule schedule,
            double start, double end) {
        PathPoint[] geometry = segment.getPointLUT();
        int low = 0, high = geometry.length;
        while (low < high) {
            int mid = (low + high) >>> 1;
            if (segment.getLengthIn() - geometry[mid].getDistanceToEndIn() <= start) { low = mid + 1; }
            else { high = mid; }
        }
        int g = low;
        int c = ConstraintSchedule.upperBound(schedule.transitions, start + 1e-6);
        int boundary = 0;
        windowSize = 0;
        // Merge three sorted streams directly into reusable arrays, without boxed tree nodes.
        while (boundary < 3) {
            double gs = g < geometry.length ? segment.getLengthIn() - geometry[g].getDistanceToEndIn() : Double.POSITIVE_INFINITY;
            if (gs - start <= 1e-6) { g++; continue; }
            if (end - gs <= 1e-6) { gs = Double.POSITIVE_INFINITY; }
            double cs = c < schedule.transitions.length ? schedule.transitions[c] : Double.POSITIVE_INFINITY;
            if (!(end - cs > 1e-6)) { cs = Double.POSITIVE_INFINITY; }
            double bs = boundary == 0 ? start : boundary == 1 ? (start + end) / 2 : end;
            double distance = Math.min(bs, Math.min(gs, cs));
            PathPoint point = distance == gs ? geometry[g] : distance == cs ? schedule.pointAt(c, segment) : null;
            if (distance == gs) { g++; }
            if (distance == cs) { c++; }
            if (distance == bs) { boundary++; }
            appendWindow(distance, point != null ? point : segment.getPointAtDistance(distance));
        }
        fillSamples(actualPath, windowGeometry, windowSamples, windowSize);
        Sample[] limits = windowSamples;
        for (int i = 0; i < windowSize; i++) { windowCeilings[i] = limits[i].maxVelocity; }
        lastWindowSampleCount = windowSize;
    }

    /** Limit acceleration at the requested preview time, retaining feasible braking at the boundary. */
    private double previewAcceleration(Sample initial, double measuredVelocity, double acceleration, double step) {
        double solvedAcceleration = acceleration;
        // When preview is requested, validate the same acceleration at that future state;
        // checking only the measured state can overrun power/alpha during launch.
        if (acceleration > 0 && step > 0) {
            double lowAccel = 0, highAccel = acceleration;
            for (int i = 0; i < SEARCH_ITERATIONS; i++) {
                double candidate = (lowAccel + highAccel) / 2;
                double predicted = measuredVelocity + candidate * step;
                double alpha = initial.headingSecond * predicted * predicted + initial.headingPrime * candidate;
                if (predicted <= initial.maxVelocity + 1e-6
                        && Math.abs(alpha) <= initial.maxAlpha + 1e-6
                        && power(initial, predicted, candidate) <= utilizationLimit() + 1e-6) { lowAccel = candidate; }
                else { highAccel = candidate; }
            }
            acceleration = lowAccel;
        }
        if (!validPreview(initial, measuredVelocity, acceleration)) { acceleration = solvedAcceleration; }
        return acceleration;
    }

    /** Mandatory braking wins over damping; extra braking must fit current alpha and wheel authority. */
    private double constrainPursuitAcceleration(Sample sample, double velocity, double desired,
                                               double feasibleMaximum, double intervalDistance) {
        double minimum = Math.max(-sample.maxAccel, -velocity * velocity / (2 * intervalDistance));
        if (Math.abs(sample.headingPrime) > 1e-12) {
            double baseAlpha = sample.headingSecond * velocity * velocity;
            double a = (-sample.maxAlpha - baseAlpha) / sample.headingPrime;
            double b = (sample.maxAlpha - baseAlpha) / sample.headingPrime;
            minimum = Math.max(minimum, Math.min(a, b));
        }
        desired = Math.min(feasibleMaximum, Math.max(minimum, desired));
        if (power(sample, velocity, desired) <= utilizationLimit() + 1e-6) return desired;
        // Search toward the solver's known feasible command, within the same kinematic bounds.
        double infeasible = desired, feasible = feasibleMaximum;
        for (int i = 0; i < 24; i++) {
            double candidate = (infeasible + feasible) / 2;
            if (power(sample, velocity, candidate) <= utilizationLimit() + 1e-6) feasible = candidate;
            else infeasible = candidate;
        }
        return feasible;
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
        public final double goalDistanceIn,goalVelocity,currentSpeedCeiling;
        /** Geometry already evaluated at the measured progress; absent for endpoint recovery. */
        public final PathPoint point;
        public WindowTarget(MotionParameters motion, boolean overspeed) { this(motion, overspeed, motion.getTangentialVel()); }
        public WindowTarget(MotionParameters motion, boolean overspeed, double normalVelocity) {
            this(motion,overspeed,normalVelocity,Double.NaN,Double.NaN,Double.NaN);
        }
        public WindowTarget(MotionParameters motion, boolean overspeed, double normalVelocity,
                            double goalDistanceIn,double goalVelocity,double currentSpeedCeiling) {
            this(motion, overspeed, normalVelocity, goalDistanceIn, goalVelocity, currentSpeedCeiling, null);
        }
        private WindowTarget(MotionParameters motion, boolean overspeed, double normalVelocity,
                             double goalDistanceIn,double goalVelocity,double currentSpeedCeiling, PathPoint point) {
            this.motion = motion; this.overspeed = overspeed; this.normalVelocity = normalVelocity;
            this.goalDistanceIn=goalDistanceIn;this.goalVelocity=goalVelocity;this.currentSpeedCeiling=currentSpeedCeiling;
            this.point = point;
        }
    }

    private Sample[] samples(Path actualPath, PathPoint[] points) {
        Sample[] result = new Sample[points.length];
        fillSamples(actualPath, points, result, points.length);
        return result;
    }

    private void appendWindow(double distance, PathPoint point) {
        if (windowSize > 0 && distance == windowPositions[windowSize - 1]) { return; }
        if (windowSize == windowPositions.length) {
            int size = windowSize * 2;
            windowPositions = Arrays.copyOf(windowPositions, size); windowCeilings = Arrays.copyOf(windowCeilings, size);
            windowGeometry = Arrays.copyOf(windowGeometry, size); windowSamples = Arrays.copyOf(windowSamples, size);
            windowGoalSquared = Arrays.copyOf(windowGoalSquared, size);
        }
        windowPositions[windowSize] = distance; windowGeometry[windowSize++] = point;
    }

    private boolean planningChanged(int index, double value) {
        boolean changed = Double.compare(cachedPlanningValues[index], value) != 0;
        cachedPlanningValues[index] = value; return changed;
    }

    private void fillSamples(Path actualPath, PathPoint[] points, Sample[] result, int count) {
        Vector finalTangent = actualPath.getParametricPath().getFirstDerivative(1);
        double length = Math.max(actualPath.getParametricPath().getLengthIn(), EPSILON);
        ConstraintSchedule schedule = constraints(actualPath);
        geometry.PathSegment segment = actualPath.getParametricPath();
        paths.heading.HeadingInterpolator interpolator = actualPath.getInterpolator();
        long headingRevision = interpolator.getConfigurationRevision();
        boolean cacheable = headingRevision >= 0 && actualPath.getConstraintRevision() >= 0 && cacheSolverSamples();
        boolean changed = planningChanged(0, constants.forwardVelLimitIn) | planningChanged(1, constants.strafeVelLimitIn)
                | planningChanged(2, constants.forwardAccelLimitIn) | planningChanged(3, constants.strafeAccelLimitIn)
                | planningChanged(4, constants.angularVelLimitRad) | planningChanged(5, constants.angularAccelLimitRad)
                | planningChanged(6, constants.profileCornerSpeedScale) | planningChanged(7, constants.kCentripetal)
                | planningChanged(8, constants.maxCentripetalAccelIn);
        if (cachedSamples == null || changed || cachedSegment != segment || cachedInterpolator != interpolator
                || cachedHeadingRevision != headingRevision || cachedSchedule != schedule
                || cachedCharacterization != constants.motionCharacterization || !cacheable) {
            cachedSamples = cacheable ? new Sample[segment.getPointLUT().length] : null; cachedSegment = segment;
            cachedTransitionSamples.clear();
            cachedInterpolator = interpolator; cachedHeadingRevision = headingRevision; cachedSchedule = schedule;
            cachedCharacterization = constants.motionCharacterization;
        }
        PathPoint[] lut = segment.getPointLUT();
        double physicalAlpha = nonNegative(constants.angularAccelLimitRad);
        double physicalOmega = nonNegative(constants.motionCharacterization == null ? constants.angularVelLimitRad
                : Math.min(constants.angularVelLimitRad, constants.motionCharacterization.maximumMeasuredVelocity(Axis.ANGULAR)));
        for (int i = 0; i < count; i++) {
            PathPoint point = points[i];
            int lutIndex = Math.max(0, Math.min(lut.length - 1, (int) Math.round(point.getT() * (lut.length - 1))));
            boolean retained = lut[lutIndex] == point && cacheable;
            if (retained && cachedSamples[lutIndex] != null) { result[i] = cachedSamples[lutIndex]; continue; }
            boolean transition = cacheable && schedule.retains(point);
            if (transition && cachedTransitionSamples.containsKey(point)) { result[i] = cachedTransitionSamples.get(point); continue; }
            Sample sample = new Sample();
            double physicalAccel = nonNegative(getPhysicalTangentialAccel(point, actualPath));
            sample.maxAccel = physicalAccel; sample.maxAlpha = physicalAlpha; sample.maxOmega = physicalOmega;
            double maxVelocity = Double.MAX_VALUE;
            double progress = actualPath.constraintProgress(1 - point.getDistanceToEndIn() / length);
            double[] authored = schedule.at(progress);
            if (authored != null) {
                maxVelocity = authored[0]; sample.maxAccel = Math.min(physicalAccel, authored[1]);
                sample.maxOmega = Math.min(physicalOmega, authored[2]); sample.maxAlpha = Math.min(physicalAlpha, authored[3]);
            }
            sample.maxVelocity = nonNegative(Math.min(maxVelocity, calculateMaxTangentialVelocity(point, actualPath, sample.maxOmega)));
            sample.headingPrime = actualPath.getInterpolator().getHeadingFirstDerivative(point.getDistanceToEndIn(), point.getSignedCurvature(), finalTangent);
            sample.headingSecond = actualPath.getInterpolator().getHeadingSecondDerivative(point.getDistanceToEndIn(), point.getCurvatureDerivative(), finalTangent);
            sample.power = powerEvaluator(actualPath, point);
            result[i] = sample;
            if (retained) { cachedSamples[lutIndex] = sample; }
            if (transition) { cachedTransitionSamples.put(point, sample); }
        }
    }

    private double maximumNextSquared(Sample sample, Sample nextSample, double squared, double nextCeiling, double ds) {
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
        double best = max;
        if (!(power(sample, v, max) <= utilizationLimit())) {
            // Only split model branches when the greatest kinematically allowed acceleration
            // fails power feasibility. This shortcut applies to every drivetrain and curvature.
            double headingZero = Math.abs(sample.headingPrime) > 1e-12 ? -baseAlpha / sample.headingPrime : 0;
            double[] breaks = accelerationBreaks;
            breaks[0] = min; breaks[1] = max; breaks[2] = Math.max(min, Math.min(max, 0));
            breaks[3] = Math.max(min, Math.min(max, headingZero));
            Arrays.sort(breaks);
            best = Double.NaN;
            for (int j = breaks.length - 2; j >= 0; j--) {
                double candidate = maximumPowerFeasibleAcceleration(sample, v, breaks[j], breaks[j + 1]);
                if (Double.isFinite(candidate)) { best = candidate; break; }
            }
        }
        if(Double.isFinite(best) && enforceIntervalPowerBounds()) {
            // A held acceleration must also fit the wheel authority later in the
            // interval. Checking only its starting speed permits a launch overrun.
            double endVelocity=Math.sqrt(Math.max(0,squared+2*ds*best));
            double lowVelocity=Math.max(1e-6,Math.min(v,endVelocity)),highVelocity=Math.max(1e-6,Math.max(v,endVelocity));
            double a1=sample.power.maximumAcceleration(lowVelocity,sample.headingPrime,sample.headingSecond,min,best,utilizationLimit());
            double a2=sample.power.maximumAcceleration(highVelocity,sample.headingPrime,sample.headingSecond,min,best,utilizationLimit());
            double a3=nextSample.power.maximumAcceleration(lowVelocity,nextSample.headingPrime,nextSample.headingSecond,min,best,utilizationLimit());
            double a4=nextSample.power.maximumAcceleration(highVelocity,nextSample.headingPrime,nextSample.headingSecond,min,best,utilizationLimit());
            if(!Double.isFinite(a1) || !Double.isFinite(a2) || !Double.isFinite(a3) || !Double.isFinite(a4)) return Double.NaN;
            best=Math.min(best,Math.min(Math.min(a1,a2),Math.min(a3,a4)));
            if(power(sample,lowVelocity,best)>utilizationLimit()+1e-6 || power(sample,highVelocity,best)>utilizationLimit()+1e-6
                    || power(nextSample,lowVelocity,best)>utilizationLimit()+1e-6 || power(nextSample,highVelocity,best)>utilizationLimit()+1e-6) return Double.NaN;
        }
        return Double.isFinite(best) ? Math.max(0, Math.min(nextCeiling, squared + 2 * ds * best)) : Double.NaN;
    }

    /** Each signed model branch is convex in acceleration for a fixed velocity. */
    private double maximumPowerFeasibleAcceleration(Sample sample, double v, double min, double max) {
        if (power(sample, v, max) <= utilizationLimit()) { return max; }
        if (useAnalyticPowerBounds()) {
            double analytic = sample.power.maximumAcceleration(v, sample.headingPrime, sample.headingSecond, min, max, utilizationLimit());
            if (analytic != Double.NEGATIVE_INFINITY) {
                if (Double.isNaN(analytic)) { return Double.NaN; }
                // Verify against the actual signed evaluator, including floating-point boundaries.
                if (power(sample, v, analytic) <= utilizationLimit() + 1e-12) { return analytic; }
            }
        }
        double minimum = min;
        if (power(sample, v, min) > utilizationLimit()) {
            double left = min, right = max;
            for (int i = 0; i < SEARCH_ITERATIONS; i++) {
                if (right - left <= 1e-9) { break; }
                double a = left + (right - left) / 3, b = right - (right - left) / 3;
                if (power(sample, v, a) <= power(sample, v, b)) { right = b; }
                else { left = a; }
            }
            minimum = (left + right) / 2;
            if (power(sample, v, minimum) > utilizationLimit()) { return Double.NaN; }
        }
        double low = minimum, high = max;
        for (int i = 0; i < SEARCH_ITERATIONS; i++) {
            if (high - low <= 1e-9) { break; }
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
