package geometry;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;
import paths.builders.HolonomicPathBuilder;

public class PathSegmentTest {
    @Test public void projectionHierarchyMatchesOriginalScanAfterJumpsAndNearbyRouteBranches() {
        PathSegment segment = new PathSegment(new BSpline(new Vector[]{
                Vector.of(0, 0, DistUnit.IN), Vector.of(100, 0, DistUnit.IN), Vector.of(100, 5, DistUnit.IN),
                Vector.of(0, 5, DistUnit.IN), Vector.of(0, 10, DistUnit.IN), Vector.of(100, 10, DistUnit.IN)}));
        PathSegment.Projection projection = new PathSegment.Projection();
        java.util.Random random = new java.util.Random(913);
        for (int i = 0; i < 300; i++) {
            Vector query = Vector.of(random.nextDouble() * 150 - 25, random.nextDouble() * 60 - 25, DistUnit.IN);
            assertEquals(scanProjection(segment, query), segment.getBestT(query, projection), 1e-10);
        }
        PathSegment line = new PathSegment(new BSpline(new Vector[]{Vector.zero(), Vector.of(1000, 0, DistUnit.IN)}));
        for (double x : new double[]{0, 10, 900, 400, 999, -100}) {
            assertEquals(scanProjection(line, Vector.of(x, 2, DistUnit.IN)), line.getBestT(Vector.of(x, 2, DistUnit.IN), projection), 1e-10);
            assertTrue(projection.getLastCandidateCount() < 30);
        }
    }

    // Independent differential reference retained from the pre-optimization projection.
    private static double scanProjection(PathSegment segment, Vector location) {
        double t = 0, best = Double.MAX_VALUE;
        for (PathPoint point : segment.getPointLUT()) {
            double squared = point.getLocation().minus(location).getMagSq().getIn();
            if (squared < best) { best = squared; t = point.getT(); }
        }
        for (int i = 0; i < 5; i++) {
            Vector difference = segment.getPosition(t).minus(location), first = segment.getFirstDerivative(t);
            double numerator = difference.dot(first).getIn();
            if (Math.abs(numerator) < 1e-6 && t > 0 && t < 1) { return t; }
            double denominator = first.dot(first).plus(difference.dot(segment.getSecondDerivative(t))).getIn();
            if (denominator <= 0) { return t; }
            double previous = t;
            t = Math.max(0, Math.min(1, t - numerator / denominator));
            if (Math.abs(t - previous) < 1e-6) { return t; }
        }
        return t;
    }
    @Test
    public void cachedCurvatureDerivativeMatchesRuntimeGeometry() {
        PathSegment segment = new PathSegment(new Parabola(1.0));
        assertCachedDerivativesMatch(segment);
    }

    @Test
    public void autoTestCornersUseTheSameCurvatureDerivativeInPlannerAndRuntime() {
        GeometryFactory factory = new GeometryFactory();
        PathSegment segment = new HolonomicPathBuilder(Pose.zero(),
                factory.arcPose(30, 0, 7), factory.arcPose(30, -30, 7),
                factory.arcPose(-30, -30, 7), factory.arcPose(-30, 30, 7),
                factory.pose(30, 30, -90)).quickBuild().getParametricPath();
        assertCachedDerivativesMatch(segment);
    }

    private static void assertCachedDerivativesMatch(PathSegment segment) {
        for (PathPoint point : segment.getPointLUT()) {
            assertEquals("Curvature derivative at t=" + point.getT(),
                    segment.getCurvatureDerivative(point.getT()),
                    point.getCurvatureDerivative(), 1e-12);
        }
    }

    @Test
    public void closestPointProjectionFindsKnownNormalIntersection() {
        PathSegment segment = new PathSegment(new Parabola(1.0));
        double expectedT = 0.4;
        Vector point = segment.getPosition(expectedT);
        Vector normal = PathSegment.calculateArcNormal(
                segment.getFirstDerivative(expectedT),
                segment.getSecondDerivative(expectedT)
        );
        Vector query = point.plus(normal.times(3.0));

        assertEquals(expectedT, segment.getBestT(query), 1e-5);
    }

    @Test
    public void closestPointProjectionWorksForOppositeCurvature() {
        PathSegment segment = new PathSegment(new Parabola(-1.0));
        double expectedT = 0.65;
        Vector point = segment.getPosition(expectedT);
        Vector normal = PathSegment.calculateArcNormal(
                segment.getFirstDerivative(expectedT),
                segment.getSecondDerivative(expectedT)
        );
        Vector query = point.plus(normal.times(-2.0));

        assertEquals(expectedT, segment.getBestT(query), 1e-5);
    }

    private static final class Parabola implements ParametricSegment {
        private final double direction;

        private Parabola(double direction) { this.direction = direction; }

        @Override
        public Vector getPosition(double t) {
            return Vector.of(20.0 * t, direction * 10.0 * t * t, DistUnit.IN);
        }

        @Override
        public Vector getFirstDerivative(double t) {
            return Vector.of(20.0, direction * 20.0 * t, DistUnit.IN);
        }

        @Override
        public Vector getSecondDerivative(double t) {
            return Vector.of(0.0, direction * 20.0, DistUnit.IN);
        }
    }
}
