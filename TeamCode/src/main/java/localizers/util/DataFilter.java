package localizers.util;

/**
 * An interface for all 1 dimensional data filters to use
 */
public interface DataFilter {
    public FilterState update(double measurement);
    /** Accepts a shared sensor timestamp; custom filters may retain their own clock. */
    default FilterState update(double measurement, long timeNanos) { return update(measurement); }
    public void reset();
}
