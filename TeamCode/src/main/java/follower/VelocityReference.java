package follower;

/**
 * Current-time velocity reference obtained by integrating held planned acceleration.
 * Sensor-time queries never include future motion. Allocation conditions the reference
 * to prevent unattainable velocity error accumulating under saturation.
 */
public final class VelocityReference {
    private static final int CAPACITY = 32;
    private static final double MAX_GAP_SECONDS = .25;
    private final long[] times = new long[CAPACITY];
    private final double[] velocities = new double[CAPACITY];
    private final double[] accelerations = new double[CAPACITY];
    private int size, newest = -1;
    private double intervalSeconds, ageSeconds, error;
    private boolean usable;

    public void reset() {
        size = 0;
        newest = -1;
        usable = false;
        error = 0;
        intervalSeconds = ageSeconds = 0;
    }

    public double update(long commandTime, long measurementTime,
                         double measuredVelocity, double plannedAcceleration) {
        usable = false;
        error = 0;
        if (commandTime < 0 || measurementTime < 0 || measurementTime > commandTime
                || !Double.isFinite(measuredVelocity) || !Double.isFinite(plannedAcceleration)) {
            reset();
            return Double.isFinite(measuredVelocity) ? measuredVelocity : 0;
        }
        ageSeconds = (commandTime - measurementTime) * 1e-9;
        if (ageSeconds > MAX_GAP_SECONDS) {
            reset();
            return measuredVelocity;
        }
        intervalSeconds = size == 0 ? .02 : (commandTime - times[newest]) * 1e-9;
        if (size == 0 || intervalSeconds < 0 || intervalSeconds > MAX_GAP_SECONDS) {
            reset();
            intervalSeconds = .02;
            ageSeconds = (commandTime - measurementTime) * 1e-9;
            append(measurementTime, Math.max(0, measuredVelocity), 0);
            if (commandTime > measurementTime) {
                append(commandTime, Math.max(0, measuredVelocity), plannedAcceleration);
            } else accelerations[newest] = plannedAcceleration;
        } else if (commandTime > times[newest]) {
            append(commandTime, advance(velocities[newest], accelerations[newest], intervalSeconds),
                    plannedAcceleration);
        } else accelerations[newest] = plannedAcceleration;

        double measuredReference = at(measurementTime);
        if (!Double.isFinite(measuredReference)) {
            reset();
            return measuredVelocity;
        }
        error = measuredReference - measuredVelocity;
        usable = true;
        return measuredReference;
    }

    /** Limit the discrete correction bandwidth: 0 <= rate*(interval+known age) <= 1. */
    public double powerGain(double configuredGain, double accelerationPerPower) {
        if (!usable || !Double.isFinite(configuredGain) || configuredGain <= 0) return 0;
        if (accelerationPerPower <= 0 || !Double.isFinite(accelerationPerPower)) return configuredGain;
        return Math.min(configuredGain,
                1 / (accelerationPerPower * Math.max(.001, intervalSeconds + ageSeconds)));
    }

    /** Back-condition only the unapplied velocity correction, preserving earlier sensor-time history. */
    public void condition(double appliedFraction) {
        if (!usable || !Double.isFinite(appliedFraction)) return;
        double fraction = Math.max(0, Math.min(1, appliedFraction));
        velocities[newest] = Math.max(0, velocities[newest] - (1 - fraction) * error);
    }

    public boolean isUsable() { return usable; }
    public double getAgeSeconds() { return ageSeconds; }
    public double getIntervalSeconds() { return intervalSeconds; }

    private void append(long time, double velocity, double acceleration) {
        newest = (newest + 1) % CAPACITY;
        times[newest] = time;
        velocities[newest] = velocity;
        accelerations[newest] = acceleration;
        size = Math.min(CAPACITY, size + 1);
    }

    private double at(long time) {
        for (int i = 0; i < size; i++) {
            int index = (newest - i + CAPACITY) % CAPACITY;
            if (times[index] <= time) {
                return advance(velocities[index], accelerations[index], (time - times[index]) * 1e-9);
            }
        }
        return Double.NaN;
    }

    private static double advance(double velocity, double acceleration, double seconds) {
        return Math.max(0, velocity + acceleration * seconds);
    }
}
