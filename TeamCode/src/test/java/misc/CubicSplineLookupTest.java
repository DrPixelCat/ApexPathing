package geometry;

import org.junit.Test;
import static org.junit.Assert.*;

import paths.geometry.CubicSpline1D;

public class CubicSplineLookupTest {
    @Test public void sharedSplineQueriesUseTheCursorOnlyAsAHint() throws Exception {
        double[] x = new double[200], y = new double[200];
        for (int i = 0; i < x.length; i++) { x[i] = i * .25; y[i] = Math.sin(x[i]); }
        CubicSpline1D spline = new CubicSpline1D(x, y);
        double[][] expected = new double[199][3];
        for (int i = 0; i < expected.length; i++) {
            double query = x[i] + .1;
            expected[i][0] = spline.evaluate(query); expected[i][1] = spline.getFirstDerivative(query);
            expected[i][2] = spline.getSecondDerivative(query);
        }
        java.util.concurrent.ExecutorService workers = java.util.concurrent.Executors.newFixedThreadPool(2);
        try {
            java.util.List<java.util.concurrent.Future<?>> results = new java.util.ArrayList<>();
            for (int worker = 0; worker < 2; worker++) {
                final int offset = worker;
                results.add(workers.submit(() -> {
                    for (int i = 0; i < 10000; i++) {
                        int index = (i * 43 + offset * 19) % expected.length;
                        double query = x[index] + .1;
                        assertEquals(expected[index][0], spline.evaluate(query), 1e-12);
                        assertEquals(expected[index][1], spline.getFirstDerivative(query), 1e-12);
                        assertEquals(expected[index][2], spline.getSecondDerivative(query), 1e-12);
                    }
                }));
            }
            for (java.util.concurrent.Future<?> result : results) { result.get(); }
        } finally { workers.shutdownNow(); }
    }
    @Test public void cachedLookupPreservesKnotsDerivativesAndBackwardQueries() {
        double[] x = new double[200], y = new double[200];
        for (int i = 0; i < x.length; i++) { x[i] = i * .25; y[i] = Math.sin(x[i]); }
        CubicSpline1D spline = new CubicSpline1D(x, y);
        java.util.Random random = new java.util.Random(28);
        for (int i = 0; i < 400; i++) {
            double query = i < 200 ? x[i] : random.nextDouble() * 55 - 2;
            CubicSpline1D reference = new CubicSpline1D(x, y);
            assertEquals(reference.evaluate(query), spline.evaluate(query), 1e-12);
            assertEquals(reference.getFirstDerivative(query), spline.getFirstDerivative(query), 1e-12);
            assertEquals(reference.getSecondDerivative(query), spline.getSecondDerivative(query), 1e-12);
            if (i < 200) { assertEquals(y[i], spline.evaluate(query), 1e-12); }
        }
    }
}
