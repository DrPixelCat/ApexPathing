package tuning;

public class BinarySearch {
    private double maximum;
    private double minimum;
    private final double threshold;
    private double guess;

    BinarySearch(double minimum, double maximum, double threshold) {
        this.maximum = maximum;
        this.minimum = minimum;
        this.threshold = threshold;
        guess = (maximum + minimum) / 2.0;
    }

    boolean updateGuess(boolean increase) {
        double lastGuess = guess;
        if (increase) {
            minimum = guess;
        } else {
            maximum = guess;
        }
        guess = (maximum + minimum) / 2.0;
        return Math.abs(lastGuess - guess) > threshold;
    }

    double getGuess() {
        return guess;
    }
}
