package trajectory;

import core.FollowerConstants;
import feedforward.generators.BaseProfileGenerator;
import geometry.PathSegment;
import geometry.Vector;
import paths.movements.Path;

/** Per-traversal rolling planner. No full-path trajectory is generated here. */
public final class OnlineTrajectoryPlanner {
    private final Path path;
    private final BaseProfileGenerator generator;
    private final double horizonIn;
    private final PathSegment.Projection projection = new PathSegment.Projection();

    public OnlineTrajectoryPlanner(Path path, FollowerConstants constants) {
        this.path = path;
        generator = path.createGenerator(constants);
        horizonIn = path.getLookaheadDistanceIn() != null ? path.getLookaheadDistanceIn()
                : StoppingHorizon.distance(constants, path, generator.getProfilePowerLimit());
    }

    public double getHorizonIn() { return horizonIn; }
    public int getLastSampleCount() { return generator.getLastWindowSampleCount(); }
    public int getLastProjectionCandidateCount() { return projection.getLastCandidateCount(); }

    public TrajectorySample sample(Vector position, Vector velocity, double dt) {
        return sample(position, velocity, Double.NaN, dt);
    }

    public TrajectorySample sample(Vector position, Vector velocity, double angularVelocity, double dt) {
        PathSegment segment = path.getParametricPath();
        double t = segment.getBestT(position, projection);
        double distance = segment.getLengthIn() - segment.getDistanceToEndIn(segment.getPosition(t), t);
        double speed = velocity.dot(segment.getFirstDerivative(t).normalize()).getIn();
        BaseProfileGenerator.WindowTarget target = generator.generateWindow(distance, horizonIn, speed, angularVelocity, dt);
        return TrajectorySample.onPath(path, distance, position, target.motion, target.overspeed, target.normalVelocity);
    }
}
