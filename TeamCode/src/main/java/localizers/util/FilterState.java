package localizers.util;


/**
 * A simple class to store the current state of a filter.
 */
public class FilterState {
    private final double value;
    private final double rate;

    public FilterState(double value, double rate) {
        this.value = value;
        this.rate = rate;
    }

    public double value() {
        return value;
    }

    public double rate() {
        return rate;
    }
}