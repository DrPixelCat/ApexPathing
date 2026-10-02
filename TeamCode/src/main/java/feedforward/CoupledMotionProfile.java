package feedforward;

/** Backward controllability and forward reachability over squared path speeds. */
public final class CoupledMotionProfile {
    private static final int SEARCH_ITERATIONS = 32;

    public interface Transitions {
        /** Greatest feasible successor in [0, ceilingSquared], or NaN if none exists. */
        double maximumNextSquared(int interval, double velocitySquared, double ceilingSquared);
    }

    private CoupledMotionProfile() { }

    /**
     * Keeps the original physical ceilings. A backward pass finds states that can still stop;
     * the forward pass takes the greatest feasible acceleration into that controllable set.
     * Constraints may require acceleration or braking even below a velocity ceiling.
     * This interval representation assumes controllable speeds form [0, maximum] and
     * transition feasibility is monotone across that interval. It is not a general
     * optimizer for disconnected feasible sets or arbitrary nonconvex motor models.
     */
    public static double[] solve(double[] positions, double[] ceilings, boolean boosted,
                                 double endVelocity, Transitions transitions) {
        if (positions == null || ceilings == null || positions.length != ceilings.length
                || positions.length < 2 || transitions == null
                || !Double.isFinite(endVelocity) || endVelocity < 0) {
            throw new IllegalArgumentException("Matching position/ceiling arrays, transitions, and a nonnegative end speed are required.");
        }
        for (int i = 0; i < positions.length; i++) {
            if (!Double.isFinite(positions[i]) || (i > 0 && positions[i] < positions[i - 1])
                    || !Double.isFinite(ceilings[i]) || ceilings[i] < 0) {
                throw new IllegalArgumentException("Finite nondecreasing positions and nonnegative speed ceilings are required.");
            }
        }
        int last = positions.length - 1;
        double[] controllable = new double[ceilings.length];
        controllable[last] = Math.min(ceilings[last] * ceilings[last], endVelocity * endVelocity);
        for (int i = last - 1; i >= 0; i--) {
            double upper = ceilings[i] * ceilings[i];
            if (positions[i + 1] - positions[i] <= 1e-9) {
                controllable[i] = Math.min(upper, controllable[i + 1]);
            } else if (Double.isFinite(transitions.maximumNextSquared(i, upper, controllable[i + 1]))) {
                controllable[i] = upper;
            } else {
                double low = 0, high = upper;
                if (!Double.isFinite(transitions.maximumNextSquared(i, 0, controllable[i + 1]))) {
                    throw new IllegalStateException("No controllable state at path interval " + i);
                }
                for (int search = 0; search < SEARCH_ITERATIONS; search++) {
                    double mid = (low + high) / 2;
                    if (Double.isFinite(transitions.maximumNextSquared(i, mid, controllable[i + 1]))) {
                        low = mid;
                    } else { high = mid; }
                }
                controllable[i] = low;
            }
        }

        double[] squared = new double[ceilings.length];
        squared[0] = boosted ? controllable[0] : 0;
        for (int i = 0; i < last; i++) {
            squared[i + 1] = positions[i + 1] - positions[i] <= 1e-9
                    ? squared[i] : transitions.maximumNextSquared(i, squared[i], controllable[i + 1]);
            if (!Double.isFinite(squared[i + 1]) || squared[i + 1] < -1e-7) {
                throw new IllegalStateException("No reachable state at path interval " + i);
            }
        }
        double[] velocities = new double[squared.length];
        for (int i = 0; i < velocities.length; i++) { velocities[i] = Math.sqrt(Math.max(0, squared[i])); }
        return velocities;
    }
}
