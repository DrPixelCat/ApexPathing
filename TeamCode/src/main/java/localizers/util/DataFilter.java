package localizers.util;

/**
 * An interface for all 1 dimensional data filters to use
 */
public interface DataFilter {
    FilterState update(double measurement);
    void reset();
}
