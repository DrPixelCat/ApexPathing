package trajectory;

import follower.FollowerConstants;
import trajectory.generators.PathProfileGenerator;
import geometry.PathSegment;
import geometry.Vector;
import paths.movements.Path;

/**
 * Per-traversal bounded feasibility planner, optionally consuming stored future-speed goals.
 *
 * @author Joel H
 */
public final class OnlineTrajectoryPlanner {
    private final Path path;
    private final PathProfileGenerator generator;
    private final double horizonIn;
    private final double previewSeconds;
    private final double pursuitSeconds;
    private double goalDistanceIn = Double.NaN, goalVelocity = Double.NaN, currentSpeedCeiling = Double.NaN;
    private final PathSegment.Projection projection = new PathSegment.Projection();

    public OnlineTrajectoryPlanner(Path path, FollowerConstants constants) {
        this(path, constants, path.getPathType() == Path.PathType.HOLONOMIC ? 0 : .05, 0);
    }

    public OnlineTrajectoryPlanner(Path path,FollowerConstants constants,drivetrains.models.DriveModel model) {
        this(path,constants,path.getPathType() == Path.PathType.HOLONOMIC ? 0 : .05,0,model);
    }

    /** Explicit legacy preview is available for controlled comparisons. */
    public OnlineTrajectoryPlanner(Path path,FollowerConstants constants,double previewSeconds) {
        this(path,constants,previewSeconds,0);
    }

    /** Explicit pursuit is retained for experimental comparisons; normal following uses current-error P control. */
    public OnlineTrajectoryPlanner(Path path,FollowerConstants constants,double previewSeconds,double pursuitSeconds) {
        this(path,constants,previewSeconds,pursuitSeconds,path.getDriveModel(constants));
    }
    private OnlineTrajectoryPlanner(Path path,FollowerConstants constants,double previewSeconds,double pursuitSeconds,
                                    drivetrains.models.DriveModel model) {
        if (!Double.isFinite(previewSeconds) || previewSeconds < 0 || previewSeconds > .05
                || !Double.isFinite(pursuitSeconds) || pursuitSeconds < 0 || pursuitSeconds > 1)
            throw new IllegalArgumentException("Preview must be in [0, .05] and convergence time in [0, 1]");
        this.path = path;
        this.previewSeconds=previewSeconds;
        this.pursuitSeconds=pursuitSeconds;
        generator = path.createGenerator(constants,model);
        if (path.getBuildMode() == Path.BuildMode.MEMORY && pursuitSeconds > 0)
            generator.setVelocityPursuitReference(path.getFeedforwardLut());
        horizonIn = path.getLookaheadDistanceIn() != null ? path.getLookaheadDistanceIn()
                : StoppingHorizon.distance(constants, path, generator.getProfilePowerLimit(),model);
    }

    public double getHorizonIn() { return horizonIn; }
    public int getLastSampleCount() { return generator.getLastWindowSampleCount(); }
    public int getLastProjectionCandidateCount() { return projection.getLastCandidateCount(); }
    public boolean isVelocityPursuitEnabled() { return pursuitSeconds>0; }
    public double getVelocityGoalDistanceIn() { return goalDistanceIn; }
    public double getVelocityGoal() { return goalVelocity; }
    public double getCurrentSpeedCeiling() { return currentSpeedCeiling; }
    public void resetVelocityPursuit() {
        generator.resetVelocityPursuit();
        goalDistanceIn = goalVelocity = currentSpeedCeiling = Double.NaN;
    }

    public TrajectorySample sample(Vector position, Vector velocity, double dt) {
        return sample(position, velocity, Double.NaN, dt);
    }

    public TrajectorySample sample(Vector position, Vector velocity, double angularVelocity, double dt) {
        PathSegment segment = path.getParametricPath();
        double t = segment.getBestT(position, projection);
        double distance = segment.getLengthIn() - segment.getDistanceToEndIn(segment.getPosition(t), t);
        // Use the same distance-interpolated tangent that TrajectorySample publishes.
        // A second tangent from projection can create a small artificial velocity error.
        double speed = velocity.dot(segment.getFirstDerivative(segment.getTAtDistance(distance)).normalize()).getIn();
        PathProfileGenerator.WindowTarget target = pursuitSeconds>0
                ? generator.generatePursuitWindow(distance,horizonIn,speed,angularVelocity,dt,pursuitSeconds)
                : generator.generateWindow(distance, horizonIn, speed, angularVelocity, dt, previewSeconds);
        goalDistanceIn=target.goalDistanceIn;goalVelocity=target.goalVelocity;currentSpeedCeiling=target.currentSpeedCeiling;
        return target.point != null
                ? TrajectorySample.onPath(path, target.point, position, target.motion, target.overspeed, target.normalVelocity)
                : TrajectorySample.onPath(path, distance, position, target.motion, target.overspeed, target.normalVelocity);
    }
}
