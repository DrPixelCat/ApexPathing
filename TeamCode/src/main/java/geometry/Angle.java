package geometry;

import android.annotation.SuppressLint;
import androidx.annotation.NonNull;

import util.AngleUnit;

/**
 * A class representing an angle that can be easily converted between {@link AngleUnit}s.
 *
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public final class Angle {
    private final double radians;

    // region Constructors and factory methods
    // Private constructor to force use of the factory method
    private Angle(double radians) { this.radians = radians; }

    /** Creates an Angle from a specific unit. */
    public static Angle of(double value, AngleUnit unit) { return new Angle(unit.toRadians(value)); }

    /** Creates an Angle from degrees. */
    public static Angle fromDeg(double degrees) { return of(degrees, AngleUnit.DEG); }

    /** Creates an Angle from radians. */
    public static Angle fromRad(double radians) { return of(radians, AngleUnit.RAD); }
    // endregion

    // region Getters
    /** @return the angle value in the requested unit. */
    public double get(AngleUnit unit) { return unit.fromRadians(this.radians); }

    /** @return the angle in degrees. */
    public double getDeg() { return get(AngleUnit.DEG); }

    /** @return the angle in radians. */
    public double getRad() { return this.radians; }
    // endregion

    // region Arithmetic operations
    /** @return a new Angle that is the sum of this Angle and another one. */
    public Angle plus(Angle other) { return new Angle(this.radians + other.radians); }

    /** @return a new Angle that is the difference between this Angle and another one. */
    public Angle minus(Angle other) { return new Angle(this.radians - other.radians); }
    // endregion

    // region Other operations and methods
    /** @return a new Angle that is the mirror of this Angle across the X-axis. */
    public Angle mirrorX() { return new Angle(-this.radians); }

    /** @return a new Angle that is the mirror of this Angle across the Y-axis. */
    public Angle mirrorY() { return new Angle(Math.PI - this.radians); }

    /** Normalizes an angle in radians to the range [0, 2π]. */
    public static double normalize(double radians) {
        return (radians % (2 * Math.PI) + (2 * Math.PI)) % (2 * Math.PI);
    }

    /** @return a new Angle that is normalized to [0, 2π] radians */
    public Angle normalized() { return new Angle(normalize(this.radians)); }

    /** @return a copy of this Angle */
    public Angle copy() { return new Angle(this.radians); }

    @SuppressLint("DefaultLocale")
    @Override
    @NonNull
    public String toString() { return String.format("%.3f rad", this.radians); }
    // endregion
}