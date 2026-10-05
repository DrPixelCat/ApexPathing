package tuning.follower.centripetal;


/**
 * A binary search algorithm that finds a value within a specified range. The search continues until
 * the difference between the last guess and the current guess is less than the specified threshold.
 *
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public class BinarySearch {
    private final double threshold;
    private double minimum, maximum, guess;
    private boolean converged;
    public enum SearchDirection { HIGHER, LOWER }

    BinarySearch(double minimum, double maximum, double threshold) {
        if (!Double.isFinite(minimum) || !Double.isFinite(maximum) || !Double.isFinite(threshold)
                || maximum <= minimum || threshold <= 0) {
            throw new IllegalArgumentException("Search requires finite ordered bounds and positive threshold");
        }
        this.maximum = maximum;
        this.minimum = minimum;
        this.threshold = threshold;
        guess = (maximum + minimum) / 2.0;
    }
    public void advance(SearchDirection direction) {
        java.util.Objects.requireNonNull(direction, "Search direction");
        if (direction == SearchDirection.HIGHER) {
            minimum = guess;
        } else {
            maximum = guess;
        }

        double nextGuess = (minimum + maximum) / 2.0;
        converged = Math.abs(nextGuess - guess) <= threshold;
        guess = nextGuess;
    }

    public double current() { return guess; }
    public boolean hasConverged() { return converged; }
}
