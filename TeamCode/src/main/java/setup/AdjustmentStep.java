package setup;

/** Mutable numeric edit step with bounded precision, independent of input devices and units. */
public final class AdjustmentStep {
    private final double initial;
    private final double minimum;
    private final double maximum;
    private final double factor;
    private double value;

    public AdjustmentStep(double initial, double minimum, double maximum, double factor) {
        if (!Double.isFinite(initial) || !Double.isFinite(minimum)
                || !Double.isFinite(maximum) || !Double.isFinite(factor)
                || minimum <= 0 || minimum > initial || initial > maximum || factor <= 1) {
            throw new IllegalArgumentException("Invalid adjustment step bounds or scale");
        }
        this.initial = initial;
        this.minimum = minimum;
        this.maximum = maximum;
        this.factor = factor;
        reset();
    }

    public double get() { return value; }
    public void set(double value) {
        if (!Double.isFinite(value) || value < minimum || value > maximum) {
            throw new IllegalArgumentException("Adjustment step outside bounds");
        }
        this.value = value;
    }
    public void reset() { value = initial; }
    public void finer() { value = Math.max(minimum, value / factor); }
    public void coarser() { value = Math.min(maximum, value * factor); }

    /** Returns a signed edit delta; the caller owns validation and value limits. */
    public double delta(int direction) { return direction * value; }
}
