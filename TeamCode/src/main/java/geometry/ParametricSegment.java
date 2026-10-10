package geometry;

/**
 * Represents a purely mathematical, stateless 2D parametric curve.
 * Evaluated strictly using the parameter 't' in the domain [0.0, 1.0].
 */
public interface ParametricSegment {
    /** @return The (x, y) coordinate at parameter t */
    Vector getPosition(double t);

    /** @return The first derivative (velocity vector) at parameter t */
    Vector getFirstDerivative(double t);

    /** @return The second derivative (acceleration vector) at parameter t */
    Vector getSecondDerivative(double t);
    /** Bounds in inches, ordered minX, minY, maxX, maxY. Custom curves can override
     * this sampled default; built-in splines compute polynomial extrema exactly. */
    default double[] getBoundsIn() {
        double[] bounds = {Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY,
                Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY};
        for (int i = 0; i <= 1024; i++) {
            Vector point = getPosition(i / 1024.0);
            double x = point.getX().getIn(), y = point.getY().getIn();
            bounds[0] = Math.min(bounds[0], x); bounds[1] = Math.min(bounds[1], y);
            bounds[2] = Math.max(bounds[2], x); bounds[3] = Math.max(bounds[3], y);
        }
        return bounds;
    }

}