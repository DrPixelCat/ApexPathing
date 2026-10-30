package paths.geometry;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

/**
 * An immutable, unitless scalar value.
 *
 * @author Joel H
 */
public class Scalar {
    private final double value;

    // region Constructors and factory methods

    private Scalar(double value) { this.value = value; }

    /** Creates a Scalar with the given value. */
    public static Scalar of(double value) { return new Scalar(value); }

    /** Creates a Scalar with a value of zero. */
    public static Scalar zero() { return new Scalar(0); }

    // endregion
    // region Getters

    /** @return the value of this Scalar. */
    public double get() { return value; }

    // endregion
    // region Arithmetic operations

    /** @return a new Scalar that is the sum of this Scalar and another one. */
    public Scalar plus(Scalar other) { return new Scalar(value + other.value); }

    /** @return a new Scalar that is the sum of this Scalar and a number. */
    public Scalar plus(double other) { return new Scalar(value + other); }

    /** @return a new Scalar that is the difference of this Scalar and another one. */
    public Scalar minus(Scalar other) { return new Scalar(value - other.value); }

    /** @return a new Scalar that is the difference of this Scalar and a number. */
    public Scalar minus(double other) { return new Scalar(value - other); }

    /** @return a new Scalar that is this Scalar multiplied by a number. */
    public Scalar times(double multiplier) { return new Scalar(value * multiplier); }

    /** @return a new Scalar that is this Scalar multiplied by another one. */
    public Scalar times(Scalar other) { return new Scalar(value * other.value); }

    /** @return a new Scalar that is this Scalar divided by a number. */
    public Scalar div(double divisor) { return new Scalar(value / divisor); }

    /** @return a new Scalar that is this Scalar divided by another one. */
    public Scalar div(Scalar other) { return new Scalar(value / other.value); }

    /** @return a new Scalar that is the absolute value of this Scalar. */
    public Scalar abs() { return new Scalar(Math.abs(value)); }

    /** @return a new Scalar that is the hypotenuse of this Scalar and another one. */
    public Scalar hypot(Scalar other) { return new Scalar(Math.hypot(value, other.value)); }

    // endregion
    // region Other operations and methods

    /** @return a new Scalar that is the mirror of this Scalar across the origin. */
    public Scalar mirror() { return new Scalar(-value); }

    /** @return a copy of this Scalar. */
    public Scalar copy() { return new Scalar(value); }

    @SuppressLint("DefaultLocale")
    @Override
    @NonNull
    public String toString() { return String.format("%.3f", value); }

    @Override public boolean equals(Object other) {
        return other != null && getClass() == other.getClass()
                && Double.compare(value, ((Scalar) other).value) == 0;
    }

    @Override public int hashCode() { return Double.hashCode(value); }
}
