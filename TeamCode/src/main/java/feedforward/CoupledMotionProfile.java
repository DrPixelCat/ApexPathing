package feedforward;

/** Backward controllability and forward reachability over squared path speeds. */
public final class CoupledMotionProfile {
    private static final int SEARCH_ITERATIONS = 32;
    private static final java.util.concurrent.atomic.AtomicLong solveCount = new java.util.concurrent.atomic.AtomicLong();
    /** Global solve count for performance diagnostics; includes precomputation. */
    public static long getSolveCount() { return solveCount.get(); }

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
        return solveInternal(positions, ceilings, boosted ? Double.NaN : 0, endVelocity, transitions);
    }

    /** Uses the actual initial speed. An uncontrollable initial state is never silently clipped. */
    public static double[] solve(double[] positions, double[] ceilings, double startVelocity,
                                 double endVelocity, Transitions transitions) {
        if (!Double.isFinite(startVelocity) || startVelocity < 0) {
            throw new IllegalArgumentException("A finite nonnegative initial speed is required.");
        }
        return solveInternal(positions, ceilings, startVelocity, endVelocity, transitions);
    }

    public static final class OverspeedException extends IllegalStateException {
        public final double maximumStartVelocity;
        public OverspeedException(double maximumStartVelocity) {
            super("Initial speed exceeds stopping envelope: " + maximumStartVelocity);
            this.maximumStartVelocity = maximumStartVelocity;
        }
    }

    /** Reusable controllability storage for an online traversal. */
    public static final class Workspace {
        private double[] controllable = new double[16];
        private double[] storage(int count) {
            if (controllable.length < count) { controllable = new double[Math.max(count, controllable.length * 2)]; }
            return controllable;
        }
    }

    /** Returns the first successor without allocating full speed arrays. Future transitions
     * are still validated so coarse-window fallback retains the same failure detection. */
    public static double solveNextSquared(double[] positions, double[] ceilings, int count, double startVelocity,
                                          double endVelocity, Transitions transitions, Workspace workspace) {
        if (!Double.isFinite(startVelocity) || startVelocity < 0 || workspace == null || positions == null
                || ceilings == null || count < 2 || count > positions.length || count > ceilings.length) {
            throw new IllegalArgumentException("A finite nonnegative initial speed and workspace are required.");
        }
        solveCount.incrementAndGet();
        double[] controllable = workspace.storage(count);
        backward(positions, ceilings, count, endVelocity, transitions, controllable);
        double squared = startVelocity * startVelocity;
        if (squared > controllable[0] + 1e-6) { throw new OverspeedException(Math.sqrt(controllable[0])); }
        double first = squared;
        for (int i = 0; i < count - 1; i++) {
            squared = positions[i + 1] - positions[i] <= 1e-9
                    ? squared : transitions.maximumNextSquared(i, squared, controllable[i + 1]);
            if (!Double.isFinite(squared) || squared < -1e-7) {
                throw new IllegalStateException("No reachable state at path interval " + i);
            }
            if (i == 0) { first = squared; }
        }
        return Math.max(0, first);
    }

    private static double[] solveInternal(double[] positions, double[] ceilings, double startVelocity,
                                         double endVelocity, Transitions transitions) {
        solveCount.incrementAndGet();
        if (positions == null || ceilings == null || positions.length != ceilings.length) {
            throw new IllegalArgumentException("Matching position/ceiling arrays are required.");
        }
        int count = positions.length;
        double[] controllable = new double[count];
        backward(positions, ceilings, count, endVelocity, transitions, controllable);
        int last = count - 1;
        double[] squared = new double[count];
        squared[0] = Double.isNaN(startVelocity) ? controllable[0] : startVelocity * startVelocity;
        if (squared[0] > controllable[0] + 1e-6) {
            throw new OverspeedException(Math.sqrt(controllable[0]));
        }
        for (int i = 0; i < last; i++) {
            squared[i + 1] = positions[i + 1] - positions[i] <= 1e-9
                    ? squared[i] : transitions.maximumNextSquared(i, squared[i], controllable[i + 1]);
            if (!Double.isFinite(squared[i + 1]) || squared[i + 1] < -1e-7) {
                throw new IllegalStateException("No reachable state at path interval " + i);
            }
        }
        double[] velocities = new double[count];
        for (int i = 0; i < count; i++) { velocities[i] = Math.sqrt(Math.max(0, squared[i])); }
        return velocities;
    }

    private static void backward(double[] positions, double[] ceilings, int count,
                                 double endVelocity, Transitions transitions, double[] controllable) {
        if (positions == null || ceilings == null || count > positions.length || count > ceilings.length
                || count < 2 || transitions == null
                || !Double.isFinite(endVelocity) || endVelocity < 0) {
            throw new IllegalArgumentException("Matching position/ceiling arrays, transitions, and a nonnegative end speed are required.");
        }
        for (int i = 0; i < count; i++) {
            if (!Double.isFinite(positions[i]) || (i > 0 && positions[i] < positions[i - 1])
                    || !Double.isFinite(ceilings[i]) || ceilings[i] < 0) {
                throw new IllegalArgumentException("Finite nondecreasing positions and nonnegative speed ceilings are required.");
            }
        }
        int last = count - 1;
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
                    if (high - low <= 1e-8) { break; }
                    double mid = (low + high) / 2;
                    if (Double.isFinite(transitions.maximumNextSquared(i, mid, controllable[i + 1]))) {
                        low = mid;
                    } else { high = mid; }
                }
                controllable[i] = low;
            }
        }

    }
}
