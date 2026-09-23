package feedforward;

/**
 * A reusable one-dimensional velocity envelope with dynamic acceleration limits.
 *
 * <p>Callers provide per-position velocity ceilings plus forward and backward reachability rules.
 * The supplied {@link PowerModel} implements {@code power = kA*a + kV*v + kS} while retaining
 * configured velocity and acceleration limits as hard safety caps. Positions and velocities are
 * unsigned; callers apply movement direction when mapping the scalar states to an axis.
 */
public final class DynamicMotionProfile {
    private static final double EPSILON = 1e-9;

    /** Calculates the greatest velocity reachable across one distance interval. */
    public interface Reachability {
        double maxReachableVelocity(double boundaryVelocity, double velocityCeiling,
                                    double distance, double boundaryPosition, int intervalIndex);
    }

    /** One scalar state. Acceleration applies to the segment beginning at this state. */
    public static final class State {
        public final double position;
        public final double velocity;
        public final double acceleration;
        public final double timeSeconds;

        private State(double position, double velocity, double acceleration, double timeSeconds) {
            this.position = position;
            this.velocity = velocity;
            this.acceleration = acceleration;
            this.timeSeconds = timeSeconds;
        }
    }

    /** Reachability rules derived from a linear velocity/acceleration feedforward model. */
    public static final class PowerModel {
        private final double velocityLimit;
        private final double forwardAccelerationLimit;
        private final double brakingAccelerationLimit;
        private final double kV;
        private final double kA;
        private final double kS;
        private final double powerLimit;

        public PowerModel(double velocityLimit, double accelerationLimit,
                          double kV, double kA, double kS) {
            this(velocityLimit, accelerationLimit, kV, kA, kS, 1.0);
        }

        public PowerModel(double velocityLimit, double accelerationLimit,
                          double kV, double kA, double kS, double powerLimit) {
            this(velocityLimit, accelerationLimit, accelerationLimit,
                    kV, kA, kS, powerLimit);
        }

        /** Creates a model with distinct positive-acceleration and braking safety caps. */
        public PowerModel(double velocityLimit, double forwardAccelerationLimit,
                          double brakingAccelerationLimit, double kV, double kA,
                          double kS, double powerLimit) {
            requirePositiveFinite("Velocity limit", velocityLimit);
            requirePositiveFinite("Forward acceleration limit", forwardAccelerationLimit);
            requirePositiveFinite("Braking acceleration limit", brakingAccelerationLimit);
            requireNonnegativeFinite("kV", kV);
            requireNonnegativeFinite("kA", kA);
            requireNonnegativeFinite("kS", kS);
            requirePositiveFinite("Power limit", powerLimit);
            if (kS >= powerLimit) {
                throw new IllegalArgumentException("kS must be smaller than the power limit.");
            }
            this.velocityLimit = velocityLimit;
            this.forwardAccelerationLimit = forwardAccelerationLimit;
            this.brakingAccelerationLimit = brakingAccelerationLimit;
            this.kV = kV;
            this.kA = kA;
            this.kS = kS;
            this.powerLimit = powerLimit;
        }

        /** Maximum cruising speed allowed by both configuration and back-EMF power. */
        public double getVelocityLimit() {
            if (kV <= EPSILON) { return velocityLimit; }
            return Math.min(velocityLimit, Math.max(0.0, (powerLimit - kS) / kV));
        }

        /** Forward reachability with acceleration reduced as velocity consumes motor power. */
        public Reachability forwardReachability() {
            return new Reachability() {
                @Override public double maxReachableVelocity(
                        double velocity, double ceiling, double distance,
                        double boundaryPosition, int intervalIndex) {
                    double acceleration = forwardAccelerationLimit;
                    if (kA > EPSILON) {
                        acceleration = Math.min(acceleration,
                                Math.max(0.0, (powerLimit - kS - kV * velocity) / kA));
                    }
                    return Math.min(ceiling,
                            Math.sqrt(velocity * velocity + 2.0 * acceleration * distance));
                }
            };
        }

        /** Backward reachability constrained by both braking acceleration and reverse power. */
        public Reachability backwardReachability() {
            return new Reachability() {
                @Override public double maxReachableVelocity(
                        double nextVelocity, double ceiling, double distance,
                        double boundaryPosition, int intervalIndex) {
                    double kinematic = Math.sqrt(nextVelocity * nextVelocity
                            + 2.0 * brakingAccelerationLimit * distance);
                    double reachable = Math.min(ceiling, kinematic);
                    if (kA <= EPSILON) { return reachable; }

                    double distanceKV = distance * kV;
                    double radicand = distanceKV * distanceKV
                            + kA * kA * nextVelocity * nextVelocity
                            + 2.0 * kA * distance * (powerLimit + kS);
                    double powerLimited = (distanceKV
                            + Math.sqrt(Math.max(0.0, radicand))) / kA;
                    return Math.min(reachable, powerLimited);
                }
            };
        }
    }

    private final State[] states;

    private DynamicMotionProfile(State[] states) { this.states = states; }

    /** Creates an evenly sampled rest-to-rest profile using a linear motor-power model. */
    public static DynamicMotionProfile forPowerLimitedDistance(
            double distance, int samples, PowerModel model) {
        requirePositiveFinite("Distance", distance);
        if (samples < 2 || model == null) {
            throw new IllegalArgumentException("At least two samples and a power model are required.");
        }
        double[] positions = new double[samples];
        double[] velocityLimits = new double[samples];
        double velocityLimit = model.getVelocityLimit();
        for (int i = 0; i < samples; i++) {
            positions[i] = distance * i / (samples - 1.0);
            velocityLimits[i] = velocityLimit;
        }
        return fromVelocityEnvelope(positions, velocityLimits, 0.0, 0.0,
                model.forwardReachability(), model.backwardReachability());
    }

    /** Runs a braking sweep followed by an acceleration sweep over sampled velocity ceilings. */
    public static DynamicMotionProfile fromVelocityEnvelope(
            double[] positions, double[] velocityLimits, double startVelocity, double endVelocity,
            Reachability forwardReachability, Reachability backwardReachability) {
        if (positions == null || velocityLimits == null || positions.length != velocityLimits.length
                || positions.length < 2) {
            throw new IllegalArgumentException(
                    "Positions and velocity limits must have the same length of at least two.");
        }
        if (forwardReachability == null || backwardReachability == null) {
            throw new IllegalArgumentException("Forward and backward reachability are required.");
        }
        requireNonnegativeFinite("Start velocity", startVelocity);
        requireNonnegativeFinite("End velocity", endVelocity);

        double[] velocities = velocityLimits.clone();
        for (int i = 0; i < positions.length; i++) {
            if (!Double.isFinite(positions[i]) || (i > 0 && positions[i] < positions[i - 1])) {
                throw new IllegalArgumentException(
                        "Profile positions must be finite and nondecreasing.");
            }
            requireNonnegativeFinite("Velocity limit", velocities[i]);
        }

        int last = positions.length - 1;
        velocities[last] = Math.min(velocities[last], endVelocity);
        for (int i = last - 1; i >= 0; i--) {
            double ds = positions[i + 1] - positions[i];
            double reachable = ds <= EPSILON ? velocities[i + 1]
                    : backwardReachability.maxReachableVelocity(
                    velocities[i + 1], velocities[i], ds, positions[i + 1], i);
            requireNonnegativeFinite("Backward reachable velocity", reachable);
            velocities[i] = Math.min(velocities[i], reachable);
        }

        velocities[0] = Math.min(velocities[0], startVelocity);
        for (int i = 1; i <= last; i++) {
            double ds = positions[i] - positions[i - 1];
            double reachable = ds <= EPSILON ? velocities[i - 1]
                    : forwardReachability.maxReachableVelocity(
                    velocities[i - 1], velocities[i], ds, positions[i - 1], i - 1);
            requireNonnegativeFinite("Forward reachable velocity", reachable);
            velocities[i] = Math.min(velocities[i], reachable);
        }

        State[] states = new State[positions.length];
        double elapsed = 0.0;
        for (int i = 0; i < last; i++) {
            double ds = positions[i + 1] - positions[i];
            double velocitySum = velocities[i] + velocities[i + 1];
            double acceleration = ds <= EPSILON ? 0.0
                    : (velocities[i + 1] * velocities[i + 1]
                    - velocities[i] * velocities[i]) / (2.0 * ds);
            states[i] = new State(positions[i], velocities[i], acceleration, elapsed);
            if (velocitySum > EPSILON) { elapsed += 2.0 * ds / velocitySum; }
        }
        states[last] = new State(positions[last], velocities[last], 0.0, elapsed);
        return new DynamicMotionProfile(states);
    }

    public State[] getStates() { return states.clone(); }

    public double getDurationSeconds() { return states[states.length - 1].timeSeconds; }

    public double getPeakVelocity() {
        double peak = 0.0;
        for (State state : states) { peak = Math.max(peak, state.velocity); }
        return peak;
    }

    public double getPeakAcceleration() {
        double peak = 0.0;
        for (State state : states) { peak = Math.max(peak, Math.abs(state.acceleration)); }
        return peak;
    }

    /** Evaluates the sampled profile with constant acceleration within each interval. */
    public State atTime(double timeSeconds) {
        if (!Double.isFinite(timeSeconds)) {
            throw new IllegalArgumentException("Time must be finite.");
        }
        if (timeSeconds <= 0.0) { return states[0]; }
        int last = states.length - 1;
        if (timeSeconds >= states[last].timeSeconds) { return states[last]; }
        for (int i = 0; i < last; i++) {
            State start = states[i];
            if (timeSeconds < states[i + 1].timeSeconds) {
                double dt = timeSeconds - start.timeSeconds;
                return new State(start.position + start.velocity * dt
                                + 0.5 * start.acceleration * dt * dt,
                        Math.max(0.0, start.velocity + start.acceleration * dt),
                        start.acceleration, timeSeconds);
            }
        }
        return states[last];
    }

    private static void requirePositiveFinite(String name, double value) {
        if (!Double.isFinite(value) || value <= 0.0) {
            throw new IllegalArgumentException(name + " must be positive and finite.");
        }
    }

    private static void requireNonnegativeFinite(String name, double value) {
        if (!Double.isFinite(value) || value < 0.0) {
            throw new IllegalArgumentException(name + " must be nonnegative and finite.");
        }
    }
}
