package localizers.util;

/**
 * An interface for all 1 dimensional data filters to use
 *
 * @author Joel H
 */
public interface DataFilter {
    /**
     * Updates the state of the filter
     * @param measurement The current 1D value to be filtered
     * @return The processed {@link FilterState}
     */
    public FilterState update(double measurement);
    /**
     * Updates the filter using the supplied measurement timestamp.
     * @param measurement The current 1D value to be filtered
     * @param timeNanos The measurement timestamp in nanoseconds
     * @return The processed {@link FilterState}
     */
    default FilterState update(double measurement, long timeNanos) { return update(measurement); }
    public void reset();
}
