package localizers.util;

import util.DeltaTime;

/**
 * A boxcar moving average of measurements and timestamped first differences.
 */
public class LowPassFilter implements DataFilter {
    private final int DEFAULT_SAMPLE_SIZE = 5;
    private double[] vals;
    private double[] rateVals;
    private boolean isWindowFull = false;
    private int size;
    private int iterator = 0;

    private double sum = 0;
    private double rateSum = 0;

    private final DeltaTime timing = new DeltaTime();
    private double lastRawValue = 0;

    public LowPassFilter() {
        this.size = DEFAULT_SAMPLE_SIZE;
        vals = new double[this.size];
        rateVals = new double[this.size];
    }

    public void reset() {
        vals = new double[size];
        rateVals = new double[size];
        iterator = 0;
        sum = 0;
        rateSum = 0;
        isWindowFull = false;
        timing.reset();
        lastRawValue = 0;
    }

    public void setSampleSize(int size) {
        if (size < 1) { throw new IllegalArgumentException("Sample size must be positive"); }
        this.size = size;
        reset();
    }

    public boolean isWindowFull() {
        return isWindowFull;
    }

    @Override
    public FilterState update(double measurement) {
        return update(measurement, System.nanoTime());
    }

    @Override
    public FilterState update(double measurement, long currentNanos) {
        if (!Double.isFinite(measurement)) { throw new IllegalArgumentException("Measurement must be finite"); }
        timing.requireIncreasing(currentNanos);
        boolean first = !timing.isInitialized();
        double dt = timing.update(currentNanos);
        double rawRate = first ? 0.0 : (measurement - lastRawValue) / dt;
        lastRawValue = measurement;

        // Advance the circular buffer
        iterator++;
        if (iterator > size - 1) {
            isWindowFull = true;
            iterator = 0;
        }

        // Update the value moving average
        sum -= vals[iterator];
        vals[iterator] = measurement;
        sum += measurement;

        // Update the rate moving average
        rateSum -= rateVals[iterator];
        rateVals[iterator] = rawRate;
        rateSum += rawRate;

        double smoothedValue;
        double smoothedRate;

        if (isWindowFull) {
            smoothedValue = sum / size;
            smoothedRate = rateSum / size;
        } else {
            smoothedValue = sum / iterator;
            smoothedRate = rateSum / iterator;
        }

        return new FilterState(smoothedValue, smoothedRate);
    }
}
