package setup.follower.feedback;

import util.DeltaTime;

import setup.logging.TuningCsvWriter;

import java.util.function.LongSupplier;

/** Compact scalar-response recorder shared by the manual position-controller tests. */
final class ManualResponseMetrics {
    private static final double SETTLE_HOLD_SECONDS = 0.25;

    private final LongSupplier clock;
    private long started;

    ManualResponseMetrics() { this(System::nanoTime); }
    ManualResponseMetrics(LongSupplier clock) { this.clock = java.util.Objects.requireNonNull(clock); }
    private TuningCsvWriter csv;
    private String csvPath = "Not started";
    private boolean active;
    private double start;
    private double target;
    private double positionTolerance;
    private double velocityTolerance;
    private double lastTime;
    private final DeltaTime timing = new DeltaTime();
    private double squaredErrorIntegral;
    private double timeWeightedSquaredError;
    private double peakVelocity;
    private double overshoot;
    private double finalError = Double.NaN;
    private double settledSince = -1.0;
    private double settlingTime = Double.NaN;
    private double powerLimit = 1;
    private int samples;
    private int saturatedSamples;

    void begin(String prefix, double start, double target,
               double positionTolerance, double velocityTolerance) {
        begin(prefix, start, target, positionTolerance, velocityTolerance, 1);
    }

    void begin(String prefix, double start, double target, double positionTolerance,
               double velocityTolerance, double powerLimit) {
        if (!Double.isFinite(powerLimit) || powerLimit <= 0 || powerLimit > 1) {
            throw new IllegalArgumentException("Response power limit must be in (0, 1]");
        }
        this.powerLimit = powerLimit;
        close();
        csv = TuningCsvWriter.open(prefix,
                "time_s", "target", "position", "error", "velocity", "max_motor_power");
        csvPath = csv.getPath();
        this.start = start;
        this.target = target;
        this.positionTolerance = positionTolerance;
        this.velocityTolerance = velocityTolerance;
        lastTime = 0.0;
        timing.reset(0L);
        squaredErrorIntegral = 0.0;
        timeWeightedSquaredError = 0.0;
        peakVelocity = 0.0;
        overshoot = 0.0;
        finalError = target - start;
        settledSince = -1.0;
        settlingTime = Double.NaN;
        samples = 0;
        saturatedSamples = 0;
        active = true;
        started = clock.getAsLong();
    }

    void sample(double position, double velocity, double maxMotorPower) {
        if (!active || !Double.isFinite(position) || !Double.isFinite(velocity)) { return; }
        long elapsedNanos = Math.max(0L, clock.getAsLong() - started);
        double elapsed = DeltaTime.seconds(elapsedNanos);
        double dt = Math.max(0.0, timing.update(elapsedNanos));

        double error = target - position;
        squaredErrorIntegral += error * error * dt;
        timeWeightedSquaredError += elapsed * error * error * dt;
        peakVelocity = Math.max(peakVelocity, Math.abs(velocity));
        double direction = Math.signum(target - start);
        overshoot = Math.max(overshoot, Math.max(0.0, direction * (position - target)));
        finalError = error;
        samples++;
        if (Double.isFinite(maxMotorPower) && maxMotorPower >= 0.99 * powerLimit) { saturatedSamples++; }

        boolean withinTolerance = Math.abs(error) <= positionTolerance &&
                Math.abs(velocity) <= velocityTolerance;
        if (withinTolerance) {
            if (settledSince < 0.0) { settledSince = elapsed; }
            if (!Double.isFinite(settlingTime) &&
                    elapsed - settledSince >= SETTLE_HOLD_SECONDS) {
                settlingTime = elapsed;
            }
        } else {
            settledSince = -1.0;
            settlingTime = Double.NaN;
        }
        if (csv != null) {
            csv.writeRow(elapsed, target, position, error, velocity, maxMotorPower);
        }
        lastTime = elapsed;
    }

    void finish() {
        active = false;
        close();
    }

    void close() {
        if (csv != null) {
            csv.close();
            csv = null;
        }
    }

    boolean isActive() { return active; }
    double getFinalError() { return finalError; }
    double getOvershoot() { return overshoot; }
    double getSettlingTime() { return settlingTime; }
    double getRmsError() {
        return lastTime > 0.0 ? Math.sqrt(squaredErrorIntegral / lastTime) : Double.NaN;
    }
    double getTimeWeightedSquaredError() { return timeWeightedSquaredError; }
    double getPeakVelocity() { return peakVelocity; }
    double getSaturationFraction() {
        return samples == 0 ? 0.0 : (double) saturatedSamples / samples;
    }
    String getCsvPath() { return csvPath; }

}
