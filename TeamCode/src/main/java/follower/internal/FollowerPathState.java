package follower.internal;

import follower.Follower;
import follower.FollowerConstants;

import paths.geometry.PathPoint;
import util.DeltaTime;

import paths.geometry.Vector;

/** Mutable path-execution state shared by the path executors and Follower diagnostics. */
final class FollowerPathState implements follower.FollowerTelemetry {
    trajectory.TrajectorySample trajectorySample;
    trajectory.OnlineTrajectoryPlanner onlinePlanner;
    private final DeltaTime timing = new DeltaTime(.02);
    private trajectory.profile.FFLut independentReference;
    private double referenceElapsed;
    boolean independentReferenceFinished() {
        return independentReference != null && referenceElapsed >= independentReference.getDurationSeconds();
    }
    private FollowerConstants planningConstants;
    private drivetrains.models.DriveModel planningDriveModel;
    void prepare(paths.movements.Path path, FollowerConstants constants) {
        prepare(path,constants,path.getDriveModel(constants));
    }
    void prepare(paths.movements.Path path, FollowerConstants constants, drivetrains.models.DriveModel model) {
        planningConstants = constants.forProfile(path.getPathType() == paths.movements.Path.PathType.TANK
                ? FollowerConstants.Profile.TANK : FollowerConstants.Profile.HOLONOMIC);
        planningDriveModel = model.withConstants(planningConstants);
        velocityReference.reset();
        onlinePlanner = path.getBuildMode() == paths.movements.Path.BuildMode.ONLINE
                ? new trajectory.OnlineTrajectoryPlanner(path, planningConstants, planningDriveModel) : null;
        timing.reset();
        independentReference = null; referenceElapsed = 0;
    }
    trajectory.TrajectorySample target(paths.movements.Path path, Vector position, Vector velocity, double omega, long now) {
        return target(path,position,velocity,omega,now,false);
    }
    trajectory.TrajectorySample target(paths.movements.Path path, Vector position, Vector velocity, double omega, long now,
                                       boolean independent) {

        boolean first = !timing.isInitialized();
        double dt = Math.max(0, timing.update(now));
        if (independent) {
            if (independentReference == null) independentReference = path.getFeedforwardLut() != null
                    ? path.getFeedforwardLut() : new trajectory.profile.FFLut(path.createGenerator(planningConstants,planningDriveModel).generateRestToRest());
            referenceElapsed += first ? 0 : dt;
            trajectorySample=independentSample(path,position,referenceElapsed);
            return trajectorySample;
        }
        trajectorySample = onlinePlanner != null ? onlinePlanner.sample(position, velocity, omega, dt)
                : path.getTrajectoryField().sample(position);
        return trajectorySample;
    }
    trajectory.TrajectorySample previewReference(paths.movements.Path path,Vector position,double previewSeconds) {
        return independentSample(path,position,referenceElapsed+previewSeconds);
    }
    private trajectory.TrajectorySample independentSample(paths.movements.Path path,Vector position,double seconds) {
            trajectory.profile.MotionParameters motion=independentReference.getKinematicParamsByTime(seconds);
            PathPoint point=path.getParametricPath().getPointAtDistance(motion.getProgression());
            Vector end=path.getParametricPath().getFirstDerivative(1);
            double prime=path.getInterpolator().getHeadingFirstDerivative(point.getDistanceToEndIn(),point.getSignedCurvature(),end);
            double second=path.getInterpolator().getHeadingSecondDerivative(point.getDistanceToEndIn(),point.getCurvatureDerivative(),end);
            double v=motion.getTangentialVel();
            motion.setAngularVel(prime*v).setAngularAccel(prime*motion.getTangentialAccel()+second*v*v);
            return trajectory.TrajectorySample.onPath(path,motion.getProgression(),position,motion,false);
    }
    final PathCompletionPolicy completion = new PathCompletionPolicy();
    final PathProgressEstimator progressEstimator = new PathProgressEstimator();
    final FollowerStuckWatchdog stuckWatchdog = new FollowerStuckWatchdog();
    final VelocityReference velocityReference = new VelocityReference();

    double t;
    double crossTrackError;
    double centripetalError;
    Vector closestPathPoint = Vector.zero();
    Vector crossTrackNormal = Vector.zero();
    Vector pathNormal = Vector.zero();
    Vector crossTrackCorrection = Vector.zero();
    Vector centripetalCorrection = Vector.zero();
    Follower.CommandDemand lastCommandDemand = Follower.CommandDemand.ZERO;
    double trackingVelocityTarget;
    double trackingMeasuredVelocity;
    double trackingProgressVelocity;
    double trackingFeedforward;
    double trackingHeadingFeedforward;
    boolean wheelFeedforwardActive;
    double trackingVelocityFeedback;
    double trackingAngularVelocityTarget;
    double trackingEndpointBlend;
    boolean velocitySampleAvailable;
    boolean centeredBrakingActive;
    double centeredBrakingMeanPower;

    Follower.MovementOutcome completionOutcome(double endpointDistance, double headingError,
            Vector velocity, double omega, double distanceTolerance, double headingTolerance, long now) {
        boolean inside = PathCompletionPolicy.isInsideTolerance(endpointDistance, headingError,
                distanceTolerance, headingTolerance);
        if (completion.isComplete(inside, PathCompletionPolicy.isVelocitySettled(velocity.getMagSq().getIn(), omega), now))
            return Follower.MovementOutcome.COMPLETED;
        if (stuckWatchdog.update(!inside && FollowerMath.stuckForSpeed(velocity.getMag().getIn()), now))
            return Follower.MovementOutcome.FAILED;
        return Follower.MovementOutcome.RUNNING;
    }

    void reset() {
        t = crossTrackError = centripetalError = 0;
        closestPathPoint = crossTrackNormal = pathNormal = crossTrackCorrection = centripetalCorrection = Vector.zero();
        lastCommandDemand = Follower.CommandDemand.ZERO;
        trackingVelocityTarget = trackingMeasuredVelocity = trackingProgressVelocity = 0;
        trackingFeedforward = trackingVelocityFeedback = trackingAngularVelocityTarget = trackingEndpointBlend = 0;
        trajectorySample = null; onlinePlanner = null; timing.reset();
        independentReference=null; referenceElapsed=0;
        completion.reset();
        progressEstimator.reset();
        stuckWatchdog.reset();
        velocityReference.reset();
        velocitySampleAvailable = false;
        centeredBrakingActive = false; centeredBrakingMeanPower = 0;
        wheelFeedforwardActive = false; trackingHeadingFeedforward = 0;
    }

    void pause() {
        if(onlinePlanner!=null) onlinePlanner.resetVelocityPursuit();
        trajectorySample = null; timing.reset(); progressEstimator.reset(); completion.reset();
        velocitySampleAvailable = false;
        centeredBrakingActive = false; centeredBrakingMeanPower = 0;
        wheelFeedforwardActive = false; trackingHeadingFeedforward = 0;
        stuckWatchdog.reset();
        velocityReference.reset();
    }

    @Override public double getTrackingVelocityTarget() { return trackingVelocityTarget; }
    /** Future speed goal and its path location; NaN when pursuit is inactive or recovering overspeed. */
    @Override public double getVelocityGoalDistanceIn() { return onlinePlanner == null ? Double.NaN
            : onlinePlanner.getVelocityGoalDistanceIn(); }
    @Override public double getVelocityGoal() { return onlinePlanner == null ? Double.NaN
            : onlinePlanner.getVelocityGoal(); }
    @Override public double getCurrentSpeedCeiling() { return onlinePlanner == null ? Double.NaN
            : onlinePlanner.getCurrentSpeedCeiling(); }
    @Override public double getTrackingMeasuredVelocity() { return trackingMeasuredVelocity; }
    @Override public double getTrackingProgressVelocity() { return trackingProgressVelocity; }
    @Override public double getTrackingFeedforward() { return trackingFeedforward; }
    @Override public double getTrackingHeadingFeedforward() { return trackingHeadingFeedforward; }
    @Override public boolean isWheelFeedforwardActive() { return wheelFeedforwardActive; }
    @Override public double getTrackingVelocityFeedback() { return trackingVelocityFeedback; }
    @Override public double getTrackingAngularVelocityTarget() {
        return trackingAngularVelocityTarget;
    }
    @Override public double getTrackingEndpointBlend() { return trackingEndpointBlend; }
    @Override public double getBestT() { return t; }
    @Override public double getCrossTrackErrorIn() { return crossTrackError; }
    @Override public double getCentripetalErrorIn() { return centripetalError; }
    @Override public Follower.CommandDemand getLastCommandDemand() { return lastCommandDemand; }
    @Override public Vector getClosestPathPoint() { return closestPathPoint; }
    @Override public Vector getCrossTrackNormal() { return crossTrackNormal; }
    @Override public Vector getPathNormal() { return pathNormal; }
    @Override public Vector getCrossTrackCorrection() { return crossTrackCorrection; }
    @Override public Vector getCentripetalCorrection() { return centripetalCorrection; }

    @Override public boolean isCenteredBrakingActive() { return centeredBrakingActive; }
    @Override public double getCenteredBrakingMeanPower() { return centeredBrakingMeanPower; }
    @Override public boolean isVelocitySampleAvailable() { return velocitySampleAvailable; }
}
