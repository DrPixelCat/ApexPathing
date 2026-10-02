package core;

import geometry.Vector;

/** Mutable path-execution state shared by the path executors and Follower diagnostics. */
final class FollowerPathState {
    trajectory.TrajectorySample trajectorySample;
    trajectory.OnlineTrajectoryPlanner onlinePlanner;
    private long targetSampleNanos;
    void prepare(paths.movements.Path path) {
        onlinePlanner = path.getBuildMode() == paths.movements.Path.BuildMode.ONLINE
                ? new trajectory.OnlineTrajectoryPlanner(path, path.getPlanningConstants()) : null;
        targetSampleNanos = 0;
    }
    trajectory.TrajectorySample target(paths.movements.Path path, Vector position, Vector velocity, double omega, long now) {
        double dt = targetSampleNanos == 0 ? .02 : Math.max(0, (now - targetSampleNanos) * 1e-9);
        targetSampleNanos = now;
        trajectorySample = onlinePlanner != null ? onlinePlanner.sample(position, velocity, omega, dt)
                : path.getTrajectoryField().sample(position);
        return trajectorySample;
    }
    final PathCompletionPolicy completion = new PathCompletionPolicy();
    final PathProgressEstimator progressEstimator = new PathProgressEstimator();
    final FollowerStuckWatchdog stuckWatchdog = new FollowerStuckWatchdog();

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
    double trackingVelocityFeedback;
    double trackingAngularVelocityTarget;
    double trackingEndpointBlend;
    boolean velocitySampleAvailable;

    void reset() {
        trajectorySample = null; onlinePlanner = null; targetSampleNanos = 0;
        completion.reset();
        progressEstimator.reset();
        stuckWatchdog.reset();
        velocitySampleAvailable = false;
    }

    void pause() {
        trajectorySample = null; targetSampleNanos = 0; progressEstimator.reset(); completion.reset();
        velocitySampleAvailable = false;
        stuckWatchdog.reset();
    }
}
