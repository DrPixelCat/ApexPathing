package util;

/**
 * Tracks nanosecond timestamps and handles invalid or out-of-range intervals.
 *
 * @author Joel H
 */
public final class DeltaTime {
    public static final double MINIMUM_SECONDS = 1e-6;
    public static final double MAXIMUM_SECONDS = 0.1;
    private final double initialSeconds;
    private double lastValidSeconds;
    private long previousNanos;
    private boolean initialized;
    private boolean timeAnomalyDetected;

    public DeltaTime() { this(MINIMUM_SECONDS); }

    /** Sets the first interval and the fallback until a valid interval is observed. */
    public DeltaTime(double initialSeconds) {
        double initial = Double.isFinite(initialSeconds) ? initialSeconds : MINIMUM_SECONDS;
        this.initialSeconds = Math.max(MINIMUM_SECONDS, Math.min(MAXIMUM_SECONDS, initial));
        lastValidSeconds = this.initialSeconds;
    }

    /** Records a timestamp. Duplicate/backward timestamps do not advance the clock. */
    public double update(long nowNanos) {
        double dt = intervalSeconds(nowNanos);
        timeAnomalyDetected = isOutOfRange(dt);
        if (isAfterPrevious(nowNanos)) {
            previousNanos = nowNanos;
            initialized = true;
        }
        if (!timeAnomalyDetected) { lastValidSeconds = dt; }
        return lastValidSeconds;
    }

    /** Previews a safe interval without changing timing or anomaly state. */
    public double elapsed(long nowNanos) {
        double dt = intervalSeconds(nowNanos);
        return isOutOfRange(dt) ? lastValidSeconds : dt;
    }

    /** Detects out-of-range intervals without changing state. */
    public boolean isAnomaly(long nowNanos) {
        return isOutOfRange(intervalSeconds(nowNanos));
    }

    /** Whether a sample requires recovery after a gap beyond the fixed maximum. */
    public boolean isLongGap(long nowNanos) {
        return intervalSeconds(nowNanos) > MAXIMUM_SECONDS;
    }

    public boolean isTimeAnomalyDetected() { return timeAnomalyDetected; }

    public boolean isAfterPrevious(long nowNanos) {
        return !initialized || nowNanos - previousNanos > 0;
    }

    /** Rejects non-increasing samples without modifying the interval. */
    public void requireIncreasing(long nowNanos) {
        if (!isAfterPrevious(nowNanos)) {
            throw new IllegalArgumentException("Measurement timestamps must increase");
        }
    }

    public boolean isInitialized() { return initialized; }

    public void reset() {
        initialized = false;
        timeAnomalyDetected = false;
        lastValidSeconds = initialSeconds;
    }

    /** Starts a new interval and clears previous anomaly history. */
    public void reset(long nowNanos) {
        reset();
        previousNanos = nowNanos;
        initialized = true;
    }

    private double intervalSeconds(long nowNanos) {
        return initialized ? (nowNanos - previousNanos) / 1_000_000_000.0 : initialSeconds;
    }

    private static boolean isOutOfRange(double seconds) {
        return seconds < MINIMUM_SECONDS || seconds > MAXIMUM_SECONDS;
    }

    /** Converts nanoseconds to seconds, with a minimum interval of one microsecond. */
    public static double seconds(long elapsedNanos) {
        return Math.max(MINIMUM_SECONDS, elapsedNanos / 1_000_000_000.0);
    }
}
