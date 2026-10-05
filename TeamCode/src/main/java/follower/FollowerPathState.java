package follower;

import geometry.Vector;

/** Mutable path-execution state shared by the path executors and Follower diagnostics. */
final class FollowerPathState {
    trajectory.TrajectorySample trajectorySample;
    trajectory.OnlineTrajectoryPlanner onlinePlanner;
    private long targetSampleNanos;
    private trajectory.profile.FFLut independentReference;
    private double referenceElapsed;
    boolean independentReferenceFinished() {
        return independentReference != null && referenceElapsed >= independentReference.getDurationSeconds();
    }
    private FollowerConstants planningConstants;
    void prepare(paths.movements.Path path, FollowerConstants constants) {
        planningConstants = constants.forProfile(path.getPathType() == paths.movements.Path.PathType.TANK
                ? FollowerConstants.Profile.TANK : FollowerConstants.Profile.HOLONOMIC);
        velocityReference.reset();
        onlinePlanner = path.getBuildMode() == paths.movements.Path.BuildMode.ONLINE
                ? new trajectory.OnlineTrajectoryPlanner(path, planningConstants) : null;
        targetSampleNanos = 0;
        independentReference = null; referenceElapsed = 0;
    }
    trajectory.TrajectorySample target(paths.movements.Path path, Vector position, Vector velocity, double omega, long now) {
        return target(path,position,velocity,omega,now,false);
    }
    trajectory.TrajectorySample target(paths.movements.Path path, Vector position, Vector velocity, double omega, long now,
                                       boolean independent) {
        double dt = targetSampleNanos == 0 ? .02 : Math.max(0, (now - targetSampleNanos) * 1e-9);
        boolean first=targetSampleNanos==0;
        targetSampleNanos = now;
        if (independent) {
            if (independentReference == null) independentReference = path.getFeedforwardLut() != null
                    ? path.getFeedforwardLut() : new trajectory.profile.FFLut(path.createGenerator(planningConstants).generateRestToRest());
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
            geometry.PathPoint point=path.getParametricPath().getPointAtDistance(motion.getProgression());
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

    void reset() {
        t = crossTrackError = centripetalError = 0;
        closestPathPoint = crossTrackNormal = pathNormal = crossTrackCorrection = centripetalCorrection = Vector.zero();
        lastCommandDemand = Follower.CommandDemand.ZERO;
        trackingVelocityTarget = trackingMeasuredVelocity = trackingProgressVelocity = 0;
        trackingFeedforward = trackingVelocityFeedback = trackingAngularVelocityTarget = trackingEndpointBlend = 0;
        trajectorySample = null; onlinePlanner = null; targetSampleNanos = 0;
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
        trajectorySample = null; targetSampleNanos = 0; progressEstimator.reset(); completion.reset();
        velocitySampleAvailable = false;
        centeredBrakingActive = false; centeredBrakingMeanPower = 0;
        wheelFeedforwardActive = false; trackingHeadingFeedforward = 0;
        stuckWatchdog.reset();
        velocityReference.reset();
    }
}
