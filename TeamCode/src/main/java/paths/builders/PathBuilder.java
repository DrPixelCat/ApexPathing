package paths.builders;

import java.util.ArrayList;
import java.util.List;

import geometry.ArcPose;
import geometry.Pose;
import paths.callbacks.Callback;
import paths.constraint.PathConstraint;
import paths.heading.InterpolationStyle;
import paths.movements.Path;

public abstract class PathBuilder {
    Path path;
    final Pose[] rawPoses;
    InterpolationStyle style = InterpolationStyle.TANGENT_FORWARD;
    final List<Runnable> buildTasks = new ArrayList<>();

    /**
     * Creates a new PathBuilder using the provided poses.
     *
     * @param type The type of the path to be built.
     * @param poses A sequence of Pose objects defining the path. Must contain at least two poses.
     *              Endpoints cannot be ArcPoses.
     */
    public PathBuilder(Path.PathType type, Pose... poses) {
        this.rawPoses = poses;
        this.path = new Path(type);
        if (poses.length < 2) {
            throw new IllegalArgumentException("A B-Spline must be created with > 1 points!");
        }
        if (poses[0] instanceof ArcPose || poses[poses.length - 1] instanceof ArcPose) {
            throw new IllegalArgumentException("Endpoints can't be arcs!");
        }
    }

    protected void addConstraintInternal(PathConstraint constraint) {
        if (constraint.getS() >= 1.0 || constraint.getS() < 0.0) {
            constraint.setS(Math.min(Math.max(constraint.getS(), 0.0), 0.9));
            path.addWarning("s must be within [0, 1) bounds! Normalized to " + constraint.getS() +
                    " for safety.");
        }
        path.addConstraint(constraint);
    }

    protected void addDistanceCallbackInternal(double s, Runnable action) {
        this.buildTasks.add(() -> path.addCallback(new Callback(s, action)));
    }

    public abstract Path profiledBuild();
    public abstract Path quickBuild();
}