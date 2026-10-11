package follower.internal;


import util.DeltaTime;

/**
 * Owns endpoint tolerances and the completion dwell timer for path movements.
 *
 * @author Joel H
 */
final class PathCompletionPolicy {
    private static final double SETTLED_LINEAR_VELOCITY_SQ = 64.0;
    private static final double SETTLED_ANGULAR_VELOCITY = 0.25;
    private static final double COMPLETION_DWELL_SECONDS = 0.10;

    private long toleranceEnteredNanos;

    boolean isComplete(boolean insideTolerance, boolean velocitySettled, long nowNanos) {
        if (!insideTolerance || !velocitySettled) {
            toleranceEnteredNanos = 0L;
            return false;
        }
        if (toleranceEnteredNanos == 0L) {
            toleranceEnteredNanos = nowNanos;
            return false;
        }
        return DeltaTime.seconds(nowNanos - toleranceEnteredNanos) >= COMPLETION_DWELL_SECONDS;
    }

    void reset() {
        toleranceEnteredNanos = 0L;
    }

    static boolean isInsideTolerance(double endpointDistance, double endpointHeadingError,
                                     double configuredDistanceTolerance,
                                     double configuredHeadingTolerance) {
        return Double.isFinite(endpointDistance) && Double.isFinite(endpointHeadingError)
                && endpointDistance <= distanceTolerance(configuredDistanceTolerance)
                && Math.abs(endpointHeadingError) <= headingTolerance(configuredHeadingTolerance);
    }

    static boolean isVelocitySettled(double linearVelocitySq, double angularVelocity) {
        return Double.isFinite(linearVelocitySq) && Double.isFinite(angularVelocity)
                && linearVelocitySq <= SETTLED_LINEAR_VELOCITY_SQ
                && Math.abs(angularVelocity) <= SETTLED_ANGULAR_VELOCITY;
    }

    static double distanceTolerance(double configuredTolerance) {
        return configuredTolerance;
    }

    static double headingTolerance(double configuredTolerance) {
        return configuredTolerance;
    }
}
