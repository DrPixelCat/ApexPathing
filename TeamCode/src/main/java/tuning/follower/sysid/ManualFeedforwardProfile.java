package tuning.follower.sysid;

/** Fixed time-based trapezoid and time-weighted measured-minus-target velocity errors. */
final class ManualFeedforwardProfile {
    static final double RAMP_SECONDS = 1.0;
    static final double CRUISE_END = 2.0;
    static final double END_SECONDS = 3.0;
    final double peak;
    private double previousTime, previousVelocity;
    private double accelerationError, accelerationTime, cruiseError, cruiseTime;
    private boolean cruiseReached;

    ManualFeedforwardProfile(double peak) { this.peak = peak; }

    double velocity(double time) {
        if (time <= 0 || time >= END_SECONDS) { return 0; }
        return peak * (time < RAMP_SECONDS ? time / RAMP_SECONDS
                : time < CRUISE_END ? 1 : (END_SECONDS - time) / RAMP_SECONDS);
    }

    double acceleration(double time) {
        return time < 0 || time >= END_SECONDS ? 0
                : time < RAMP_SECONDS ? peak / RAMP_SECONDS
                : time < CRUISE_END ? 0 : -peak / RAMP_SECONDS;
    }

    void begin(double measured) { previousVelocity = measured; }

    void sample(double time, double measured) {
        if (!Double.isFinite(time) || !Double.isFinite(measured) || time <= previousTime) { return; }
        double start = previousTime, end = Math.min(time, RAMP_SECONDS);
        if (end > start) {
            accelerationError += integrateError(start, end, time, measured);
            accelerationTime += end - start;
        }
        start = Math.max(previousTime, RAMP_SECONDS);
        end = Math.min(time, CRUISE_END);
        if (end > start) {
            double direction = Math.signum(peak);
            double v0 = interpolate(start, time, measured) * direction;
            double v1 = interpolate(end, time, measured) * direction;
            double target = Math.abs(peak);
            if (!cruiseReached) {
                if (v0 >= target) { cruiseReached = true; }
                else if (v1 >= target) {
                    start += (end - start) * (target - v0) / (v1 - v0);
                    cruiseReached = true;
                }
            }
            if (cruiseReached && end > start) {
                cruiseError += integrateError(start, end, time, measured);
                cruiseTime += end - start;
            }
        }
        previousTime = time;
        previousVelocity = measured;
    }

    private double interpolate(double time, double end, double measured) {
        return previousVelocity + (measured - previousVelocity)
                * (time - previousTime) / (end - previousTime);
    }

    private double integrateError(double start, double end, double time, double measured) {
        return .5 * (interpolate(start, time, measured) - velocity(start)
                + interpolate(end, time, measured) - velocity(end)) * (end - start);
    }

    double accelerationMeanError() {
        return accelerationTime > 0 ? accelerationError / accelerationTime : Double.NaN;
    }

    double cruiseMeanError() { return cruiseTime > 0 ? cruiseError / cruiseTime : Double.NaN; }
}
