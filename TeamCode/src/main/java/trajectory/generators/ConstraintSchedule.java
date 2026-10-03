package trajectory.generators;

import java.util.Arrays;
import java.util.Comparator;
import paths.constraint.AngularConstraint;
import paths.constraint.PathConstraint;
import paths.constraint.TranslationalConstraint;
import paths.movements.Path;
import geometry.PathPoint;
import geometry.PathSegment;

/** Stable authored ordering and prefix limits, rebuilt only after a constraint mutation. */
final class ConstraintSchedule {
    final double[] transitions;
    private final double[] progress;
    private final double[][] limits;
    private final PathPoint[] transitionPoints;
    private final java.util.IdentityHashMap<PathPoint, Boolean> retainedPoints = new java.util.IdentityHashMap<>();

    ConstraintSchedule(Path path) {
        PathConstraint[] constraints = path.getConstraints();
        Arrays.sort(constraints, Comparator.comparingDouble(PathConstraint::getS));
        progress = new double[constraints.length];
        transitions = new double[constraints.length];
        transitionPoints = new PathPoint[constraints.length];
        limits = new double[constraints.length][4];
        double velocity = Double.MAX_VALUE, acceleration = Double.MAX_VALUE;
        double omega = Double.MAX_VALUE, alpha = Double.MAX_VALUE;
        for (int i = 0; i < constraints.length; i++) {
            PathConstraint constraint = constraints[i];
            progress[i] = constraint.getS();
            transitions[i] = path.constraintProgress(progress[i]) * path.getParametricPath().getLengthIn();
            if (constraint instanceof TranslationalConstraint) {
                double value = nonNegative(((TranslationalConstraint) constraint).getValueIn());
                if (constraint.getType() == PathConstraint.Type.VELOCITY) { velocity = value; }
                else { acceleration = value; }
            } else if (constraint instanceof AngularConstraint) {
                double value = nonNegative(((AngularConstraint) constraint).getValueRad());
                if (constraint.getType() == PathConstraint.Type.VELOCITY) { omega = value; }
                else { alpha = value; }
            }
            limits[i][0] = velocity; limits[i][1] = acceleration;
            limits[i][2] = omega; limits[i][3] = alpha;
        }
        Arrays.sort(transitions);
    }

    double[] at(double value) {
        int index = upperBound(progress, value + 1e-12) - 1;
        return index >= 0 ? limits[index] : null;
    }

    PathPoint pointAt(int index, PathSegment segment) {
        if (transitionPoints[index] == null) {
            transitionPoints[index] = segment.getPointAtDistance(transitions[index]);
            retainedPoints.put(transitionPoints[index], Boolean.TRUE);
        }
        return transitionPoints[index];
    }

    boolean retains(PathPoint point) { return retainedPoints.containsKey(point); }

    static int upperBound(double[] values, double query) {
        int low = 0, high = values.length;
        while (low < high) {
            int mid = (low + high) >>> 1;
            if (values[mid] <= query) { low = mid + 1; } else { high = mid; }
        }
        return low;
    }

    private static double nonNegative(double value) { return Double.isFinite(value) ? Math.max(0, value) : 0; }
}
