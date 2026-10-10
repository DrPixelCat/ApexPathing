package geometry;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

/**
 * A class representing a distance that can be easily converted between {@link DistUnit}s.
 *
 * @author Dylan B. - 18597 RoboClovers - Delta
 * @author Joel H
 */
public class Dist {
    private final Scalar inches;

    // region Constructors and factory methods

    private Dist(Scalar inches) { this.inches = inches; }

    /** Creates a Distance from a specific unit. */
    public static Dist of(double value, DistUnit unit) {
        return new Dist(Scalar.of(unit.toInches(value)));
    }

    /** @return the distance value in the requested unit. */
    public double get(DistUnit unit) { return unit.fromInches(this.inches.get()); }

    /** Creates a Distance from inches. */
    public static Dist fromIn(double inches) { return of(inches, DistUnit.IN); }

    /** Creates a Distance from feet. */
    public static Dist fromFt(double feet) { return of(feet, DistUnit.FT); }

    /** Creates a Distance from millimeters. */
    public static Dist fromMm(double millimeters) { return of(millimeters, DistUnit.MM); }

    /** Creates a Distance from centimeters. */
    public static Dist fromCm(double centimeters) {return of(centimeters, DistUnit.CM);}

    /** Creates a Distance from meters. */
    public static Dist fromM(double meters) { return of(meters, DistUnit.M); }

    /** Creates a Distance with a value of 0 inches. */
    public static Dist zero() { return new Dist(Scalar.zero()); }

    // endregion
    // region Getters

    /** @return the distance in inches. */
    public double getIn() { return this.inches.get(); }

    /** @return the distance in feet. */
    public double getFt() { return get(DistUnit.FT); }

    /** @return the distance in millimeters. */
    public double getMm() { return get(DistUnit.MM); }

    /** @return the distance in centimeters. */
    public double getCm() { return get(DistUnit.CM); }

    /** @return the distance in meters. */
    public double getM() { return get(DistUnit.M); }

    // endregion
    // region Arithmetic operations

    /** @return a new Distance that is the sum of this Distance and another one. */
    public Dist plus(Dist other) { return new Dist(this.inches.plus(other.inches)); }

    /** @return a new Distance that is the difference of this Distance and another one. */
    public Dist minus(Dist other) { return new Dist(this.inches.minus(other.inches)); }

    /** @return a new Distance that is this Distance multiplied by a scalar. */
    public Dist times(double scalar) { return new Dist(this.inches.times(scalar)); }

    /** @return a new Distance that is this Distance multiplied by a Scalar. */
    public Dist times(Scalar scalar) { return new Dist(this.inches.times(scalar)); }

    /** @return a new Distance that is this Distance multiplied by another Distance. */
    public Dist times(Dist other) { return new Dist(this.inches.times(other.inches)); }

    /** @return a new Distance that is this Distance divided by a scalar. */
    public Dist div(double scalar) { return new Dist(this.inches.div(scalar)); }

    /** @return a new Distance that is this Distance divided by a Scalar. */
    public Dist div(Scalar scalar) { return new Dist(this.inches.div(scalar)); }

    /** @return a new Distance that is this Distance divided by another Distance. */
    public Dist div(Dist other) { return new Dist(this.inches.div(other.inches)); }

    /** @return a new Distance that is the absolute value of this Distance. */
    public Dist abs() { return new Dist(this.inches.abs()); }

    /** @return a new Distance that is the hypotenuse of this Distance and another one. */
    public Dist hypot(Dist other) { return new Dist(this.inches.hypot(other.inches)); }

    // endregion
    // region Other operations and methods

    /** @return a new Distance that is the mirror of this Distance across the origin. */
    public Dist mirror() { return new Dist(this.inches.mirror()); }

    /** @return a copy of this Distance */
    public Dist copy() { return new Dist(this.inches.copy()); }

    @SuppressLint("DefaultLocale")
    @Override
    @NonNull
    public String toString() { return String.format("%s in", this.inches); }

    @Override public boolean equals(Object other) {
        return other != null && getClass() == other.getClass()
                && Double.compare(getIn(), ((Dist) other).getIn()) == 0;
    }

    @Override public int hashCode() { return Double.hashCode(getIn()); }
}
