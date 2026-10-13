package geometry;

/**
 * Represents a Uniform Cubic B-Spline.
 *
 * <p>B-Splines guarantee C2 continuity (smooth position, velocity, and acceleration) across the
 * entire path. Because they are evaluated using a sliding 4-point window, calculating a point on
 * the curve runs in O(1) constant time, regardless of how many control points are in the path.
 *
 * @author Sohum Arora - 22895 Paraducks
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public class BSpline implements ParametricSegment {
    @SuppressWarnings({"ArrayCreationWithoutNewKeyword", "ConstantExpression"})
    private static final Matrix<Double> BLEND_MATRIX = Matrix.of(new double[][]{
            {-1.0 / 6.0, 3.0 / 6.0, -3.0 / 6.0, 1.0 / 6.0},
            {3.0 / 6.0, -6.0 / 6.0, 3.0 / 6.0, 0.0},
            {-3.0 / 6.0, 0.0, 3.0 / 6.0, 0.0},
            {1.0 / 6.0, 4.0 / 6.0, 1.0 / 6.0, 0.0}
    });

    /**
     * Cached polynomial coefficients for each segment. cx[segmentIndex] returns the
     * [c3, c2, c1, c0] array for that segment
     */
    private final double[][] cx;
    private final double[][] cy;
    private final int numSegments;

    private SegmentData cachedSegment;

    /** Bundles the normalized parameter, local segment parameter, and precomputed coefficients. */
    private static class SegmentData {
        final double t;
        final double localT;
        final double[] cX;
        final double[] cY;

        SegmentData(double t, double localT, double[] cX, double[] cY) {
            this.t = t;
            this.localT = localT;
            this.cX = cX;
            this.cY = cY;
        }
    }

    /**
     * Constructs a continuous B-Spline from an array of waypoints.
     * Automatically generates "ghost points" at the start and end to guarantee
     * the curve properly anchors to the first and last input points.
     *
     * @param inputPoints An array of Vector waypoints the spline is built around.
     * @throws IllegalArgumentException if there are 1 or fewer points provided.
     */
    public BSpline(Vector[] inputPoints) throws IllegalArgumentException {
        if (inputPoints.length < 2) {
            throw new IllegalArgumentException("You can't make a B-Spline curve with < 2 points!");
        }

        // Create ghost points
        Vector[] paddedPoints = new Vector[inputPoints.length + 2];
        paddedPoints[0] = inputPoints[1].reflect(inputPoints[0]);
        paddedPoints[paddedPoints.length - 1] = inputPoints[inputPoints.length - 2]
                .reflect(inputPoints[inputPoints.length - 1]);
        System.arraycopy(inputPoints, 0, paddedPoints, 1, inputPoints.length);

        // Precompute and cache coefficients for all segments
        this.numSegments = paddedPoints.length - 3;
        this.cx = new double[numSegments][4];
        this.cy = new double[numSegments][4];

        for (int i = 0; i < numSegments; i++) {
            Vector p0 = paddedPoints[i];
            Vector p1 = paddedPoints[i + 1];
            Vector p2 = paddedPoints[i + 2];
            Vector p3 = paddedPoints[i + 3];

            double[] xWindow = new double[]{p0.getX().getIn(), p1.getX().getIn(), p2.getX().getIn(),
                    p3.getX().getIn()};
            double[] yWindow = new double[]{p0.getY().getIn(), p1.getY().getIn(), p2.getY().getIn(),
                    p3.getY().getIn()};

            this.cx[i] = BLEND_MATRIX.multiply(Matrix.of(xWindow).transpose()).transpose().toDoubleArray()[0];
            this.cy[i] = BLEND_MATRIX.multiply(Matrix.of(yWindow).transpose()).transpose().toDoubleArray()[0];
        }
    }

    private SegmentData getSegmentData(double t) {
        if (cachedSegment != null && cachedSegment.t == t) {
            return cachedSegment;
        }

        double newT = t;
        if (!Double.isFinite(newT)) { throw new IllegalArgumentException("Spline parameter must be finite"); }
        if (newT >= 1.0) { newT = 1.0; }
        if (newT < 0.0) { newT = 0.0; }

        double continuousIndex = newT * numSegments;
        int segment = Math.min((int) continuousIndex, numSegments - 1);
        double localT = continuousIndex - segment;

        cachedSegment = new SegmentData(t, localT, cx[segment], cy[segment]);
        return cachedSegment;
    }

    /**
     * Calculates the physical (x, y) position on the curve at a given percentage.
     *
     * @param t The global path parameter [0.0, 1.0].
     * @return A Vector representing the coordinate location.
     */
    @Override
    public Vector getPosition(double t) {
        SegmentData seg = getSegmentData(t);

        return Vector.of(
                ((seg.cX[0] * seg.localT + seg.cX[1]) * seg.localT + seg.cX[2]) *
                        seg.localT + seg.cX[3],
                ((seg.cY[0] * seg.localT + seg.cY[1]) * seg.localT + seg.cY[2]) *
                        seg.localT + seg.cY[3],
                DistUnit.IN
        );
    }

    /**
     * Calculates the first derivative (velocity vector) of the curve at a given percentage.
     *
     * @param t The global path parameter [0.0, 1.0].
     * @return A Vector representing the parametric velocity.
     */
    @Override
    public Vector getFirstDerivative(double t) {
        SegmentData segment = getSegmentData(t);

        // Velocity: 3*c3*t^2 + 2*c2*t + c1
        return Vector.of(
                (3.0 * segment.cX[0] * segment.localT + 2.0 * segment.cX[1]) *
                        segment.localT + segment.cX[2],
                (3.0 * segment.cY[0] * segment.localT + 2.0 * segment.cY[1]) *
                        segment.localT + segment.cY[2],
                DistUnit.IN
        ).times(numSegments); // Chain rule scaling
    }

    /**
     * Calculates the second derivative (acceleration vector) of the curve at a given percentage.
     *
     * @param t The global path parameter [0.0, 1.0].
     * @return A Vector representing the parametric acceleration.
     */
    @Override
    public Vector getSecondDerivative(double t) {
        SegmentData segment = getSegmentData(t);

        // Acceleration: 6*c3*t + 2*c2
        return Vector.of(
                6.0 * segment.cX[0] * segment.localT + 2.0 * segment.cX[1],
                6.0 * segment.cY[0] * segment.localT + 2.0 * segment.cY[1],
                DistUnit.IN
        ).times(numSegments * numSegments); // Chain rule scaling
    }
    @Override public double[] getBoundsIn() {
        double[] bounds = {Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY,
                Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY};
        for (int i = 0; i < numSegments; i++) {
            double[] x = polynomialRange(cx[i]), y = polynomialRange(cy[i]);
            bounds[0] = Math.min(bounds[0], x[0]); bounds[1] = Math.min(bounds[1], y[0]);
            bounds[2] = Math.max(bounds[2], x[1]); bounds[3] = Math.max(bounds[3], y[1]);
        }
        return bounds;
    }
    private static double polynomial(double[] c, double t) { return ((c[0] * t + c[1]) * t + c[2]) * t + c[3]; }
    private static double[] polynomialRange(double[] c) {
        double start = polynomial(c, 0), end = polynomial(c, 1);
        double[] range = {Math.min(start, end), Math.max(start, end)};
        double a = 3 * c[0], b = 2 * c[1], d = c[2];
        if (Math.abs(a) < 1e-12) {
            if (Math.abs(b) > 1e-12) { includeRoot(range, c, -d / b); }
        } else {
            double discriminant = b * b - 4 * a * d;
            if (discriminant >= 0) {
                double q = -.5 * (b + Math.copySign(Math.sqrt(discriminant), b));
                if (Math.abs(q) < 1e-15) { includeRoot(range, c, -b / (2 * a)); }
                else { includeRoot(range, c, q / a); includeRoot(range, c, d / q); }
            }
        }
        return range;
    }
    private static void includeRoot(double[] range, double[] c, double t) {
        if (t > 0 && t < 1) {
            double value = polynomial(c, t);
            range[0] = Math.min(range[0], value); range[1] = Math.max(range[1], value);
        }
    }

}
