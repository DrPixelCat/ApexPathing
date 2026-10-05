package trajectory;

/** Spatial speed convergence. The caller supplies an independent feasible speed envelope. */
public final class VelocityPursuit {
    public static final double DEFAULT_TIME_SECONDS = .35;
    public static final double MIN_DISTANCE_IN = 2;
    private double appliedAcceleration;
    private boolean initialized;

    public void reset() { initialized = false; appliedAcceleration = 0; }

    public static final class Goal {
        public final double distanceIn, velocity, acceleration;
        Goal(double distanceIn, double squaredVelocity, double acceleration) {
            this.distanceIn = distanceIn;
            this.velocity = Math.sqrt(Math.max(0, squaredVelocity));
            this.acceleration = acceleration;
        }
    }

    /** Interpolate squared speed, retaining any more restrictive intervening speed minimum. */
    public Goal goal(double start, double end, double velocity, double seconds,
                     double[] distances, double[] squaredSpeeds, int count) {
        if (!Double.isFinite(seconds) || seconds <= 0 || seconds > 1)
            throw new IllegalArgumentException("Velocity convergence time must be in (0, 1] seconds");
        double lookahead = Math.min(end - start, Math.max(MIN_DISTANCE_IN, Math.max(0, velocity) * seconds));
        if (!(lookahead > 0) || count < 2)
            throw new IllegalArgumentException("A positive convergence interval is required");
        double distance = start + lookahead;
        int index = 0;
        while (index + 1 < count - 1 && distances[index + 1] < distance) index++;
        double fraction = (distance - distances[index]) / (distances[index + 1] - distances[index]);
        double squared = squaredSpeeds[index] * (1 - fraction) + squaredSpeeds[index + 1] * fraction;
        double acceleration = (squared - velocity * velocity) / (2 * lookahead);
        for (int i = 1; i <= index; i++) {
            double candidate = (squaredSpeeds[i] - velocity * velocity) / (2 * (distances[i] - start));
            if (squaredSpeeds[i] <= squaredSpeeds[i - 1] && squaredSpeeds[i] <= squaredSpeeds[i + 1]
                    && candidate < acceleration) {
                distance = distances[i]; squared = squaredSpeeds[i]; acceleration = candidate;
            }
        }
        return new Goal(distance, squared, acceleration);
    }

    /** Near cruise, Ta = L/(4v) critically damps the ideal local acceleration/velocity system. */
    public double damp(Goal goal, double start, double velocity, double seconds, double dt) {
        if (!initialized || !Double.isFinite(dt) || dt > .25 || dt < 0) reset();
        double time = .25 * Math.min(seconds, (goal.distanceIn - start) / Math.max(1, velocity));
        double step = Double.isFinite(dt) && dt >= 0 && dt <= .25 ? dt : .02;
        double weight = 1 - Math.exp(-step / Math.max(.005, time));
        return appliedAcceleration + weight * (goal.acceleration - appliedAcceleration);
    }

    /** Remember the feasible command, so smoothing cannot accumulate unapplied acceleration. */
    public void applied(double acceleration) { appliedAcceleration = acceleration; initialized = true; }
}
