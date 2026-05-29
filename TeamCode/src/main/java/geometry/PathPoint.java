package geometry;

/**
 * Represents a precalculated, discrete point along a parametric path segment.
 * <p>
 * This class is used to populate a Look-Up Table (LUT), allowing the robot to
 * quickly map a parametric 't' value to physical coordinates and estimate the
 * remaining arc-length distance to the end of the curve in O(1) time.
 * <p>
 * Author: DrPixelCat
 */
public class PathPoint {
    private final double t;
    private final double distanceToEnd;
    private final Vector location;

    /**
     * Constructs a precalculated path point.
     *
     * @param t             The parametric value [0.0, 1.0] representing this point's location on the curve.
     * @param distanceToEnd The calculated arc length from this point to the end of the segment.
     * @param location      The physical 2D coordinate of the curve at this 't' value.
     */
    public PathPoint(double t, double distanceToEnd, Vector location) {
        this.t = t;
        this.distanceToEnd = distanceToEnd;
        this.location = location;
    }

    /**
     * Retrieves the physical 2D coordinate of this point.
     *
     * @return The location as a Vector.
     */
    public Vector getLocation() {
        return location;
    }

    /**
     * Retrieves the parametric progression of this point along the curve.
     *
     * @return The 't' value, typically between [0.0, 1.0].
     */
    public double getT() {
        return t;
    }

    /**
     * Retrieves the precalculated arc length from this point to the end of the segment.
     *
     * @return The remaining distance in inches.
     */
    public double getDistanceToEnd_in() {
        return distanceToEnd;
    }
}