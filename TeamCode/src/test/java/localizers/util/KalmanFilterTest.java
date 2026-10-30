package localizers.util;

import org.junit.Test;
import static org.junit.Assert.*;

import paths.geometry.Matrix;

public class KalmanFilterTest {

    @Test public void covarianceValidationHandlesThreeStatePivotsAndRankDeficiency() {
        KalmanFilter filter = new KalmanFilter(3);
        double[][] rankOne = {{1, 2, 3}, {2, 4, 6}, {3, 6, 9}};
        filter.reset(new double[] {1, 2, 3}, rankOne);
        assertEquals(0.5, filter.innovationVariance(new double[] {2, -1, 0}, 0.5), 0);
        assertThrows(IllegalArgumentException.class,
                () -> filter.setCovariance(new double[][] {{1, 0, 2}, {0, 4, 0}, {2, 0, 1}}));
        for (int row = 0; row < 3; row++) {
            assertArrayEquals(rankOne[row], filter.getCovariance()[row], 0);
        }
        assertArrayEquals(new double[] {1, 2, 3}, filter.getState(), 0);
    }

    @Test public void gainDivisionSupportsSubnormalInnovationVariance() {
        KalmanFilter filter = new KalmanFilter(1);
        filter.reset(new double[] {0}, new double[][] {{1e-320}});
        filter.correctComponent(2, 0, 1e-320);
        assertEquals(1, filter.getState(0), 0);
        assertEquals(5e-321, filter.getCovariance(0, 0), 0);
    }

    @Test public void exactObservationRemovesUncertaintyWithoutInventingNoise() {
        KalmanFilter filter = new KalmanFilter(1);
        filter.reset(new double[] {0}, new double[][] {{4}});
        filter.correctComponent(2, 0, 0);
        assertEquals(2, filter.getState(0), 0);
        assertEquals(0, filter.getCovariance(0, 0), 0);
        assertEquals(0, filter.innovationVariance(new double[] {1}, 0), 0);
        assertThrows(IllegalArgumentException.class, () -> filter.correctComponent(3, 0, 0));
        assertEquals(2, filter.getState(0), 0);
        assertEquals(0, filter.getCovariance(0, 0), 0);
    }

    @Test public void noiselessPredictionPreservesKnownStateAndTinyUncertainty() {
        KalmanFilter known = new KalmanFilter(1);
        known.reset(new double[] {2}, new double[][] {{0}});
        known.predict(new double[][] {{1}}, new double[][] {{0}});
        known.correctComponent(10, 0, 1);
        assertEquals(2, known.getState(0), 0);
        assertEquals(0, known.getCovariance(0, 0), 0);

        KalmanFilter tiny = new KalmanFilter(1);
        tiny.reset(new double[] {0}, new double[][] {{1e-30}});
        tiny.predict(new double[][] {{1}}, new double[][] {{0}});
        assertEquals(1e-30, tiny.getCovariance(0, 0), 0);
        tiny.correctComponent(10, 0, 1e-30);
        assertEquals(5, tiny.getState(0), 0);
        assertEquals(5e-31, tiny.getCovariance(0, 0), 1e-45);

        tiny.reset(new double[] {0}, new double[][] {{Double.MIN_VALUE}});
        tiny.predict(new double[][] {{1}}, new double[][] {{0}});
        assertEquals(Double.MIN_VALUE, tiny.getCovariance(0, 0), 0);
    }

    @Test public void acceptsRoundoffAsymmetryFromExternalCovariancePropagation() {
        Matrix<Double> p = Matrix.of(new double[][] {
                {1.7275636800328682, 0.06832234717598455},
                {0.06832234717598455, 1.3087194553326598}});
        Matrix<Double> f = Matrix.of(new double[][] {
                {1, 0.27707849007413665}, {-0.246745923929066, 1}});
        double[][] propagated = f.multiply(p).multiply(f.transpose()).toDoubleArray();
        KalmanFilter filter = new KalmanFilter(2);
        filter.reset(new double[] {0, 0}, propagated);
        assertEquals(propagated[0][0], filter.getCovariance(0, 0), 0);
        assertEquals(propagated[1][1], filter.getCovariance(1, 1), 0);
        assertEquals(filter.getCovariance(0, 1), filter.getCovariance(1, 0), 0);
        assertEquals(0, filter.getCovariance(0, 1), 1e-16);
    }

    @Test public void covarianceValidationWorksAcrossStateUnitsAndMagnitude() {
        KalmanFilter filter = new KalmanFilter(2);
        for (double scale : new double[] {1e200, 1e-200}) {
            filter.reset(new double[] {0, 0}, new double[][] {{scale, scale / 2}, {scale / 2, scale}});
            assertEquals(scale, filter.getCovariance(0, 0), 0);
            filter.predict(new double[][] {{1, 0}, {0, 1}}, new double[][] {{0, 0}, {0, 0}});
            assertEquals(scale, filter.getCovariance(0, 0), 0);
            assertThrows(IllegalArgumentException.class,
                    () -> filter.setCovariance(new double[][] {{scale, 2 * scale}, {2 * scale, scale}}));
        }
        filter.setCovariance(new double[][] {{1e200, 0.5}, {0.5, 1e-200}});
        assertEquals(0.5, filter.getCovariance(0, 1), 0);
        assertThrows(IllegalArgumentException.class,
                () -> filter.setCovariance(new double[][] {{1e200, 2}, {2, 1e-200}}));
        assertThrows(IllegalArgumentException.class,
                () -> filter.setCovariance(new double[][] {{0, 1e-30}, {1e-30, 1}}));
    }

    @Test public void covarianceSymmetrizationDoesNotOverflowLargeFiniteDiagonalEntries() {
        KalmanFilter filter = new KalmanFilter(1);
        filter.reset(new double[] {3}, new double[][] {{1e308}});
        filter.predict(new double[][] {{1}}, new double[][] {{0}});
        assertEquals(3, filter.getState(0), 0);
        assertEquals(1e308, filter.getCovariance(0, 0), 0);
    }

    @Test public void coupledThreeStatePredictionAndCorrectionMatchKnownResults() {
        KalmanFilter filter = new KalmanFilter(3);
        filter.reset(new double[] {1, 2, 3}, new double[][] {{2, 0, 0}, {0, 3, 0}, {0, 0, 4}});
        filter.predict(new double[][] {{1, 1, 0}, {0, 1, 1}, {0, 0, 1}},
                new double[][] {{1, 0, 0}, {0, 2, 0}, {0, 0, 3}});
        assertArrayEquals(new double[] {3, 5, 3}, filter.getState(), 0);
        assertArrayEquals(new double[] {6, 3, 0}, filter.getCovariance()[0], 0);
        assertArrayEquals(new double[] {3, 9, 4}, filter.getCovariance()[1], 0);
        assertArrayEquals(new double[] {0, 4, 7}, filter.getCovariance()[2], 0);

        filter.correct(10, new double[] {1, 0, 1}, 2);
        assertArrayEquals(new double[] {4.6, 103.0 / 15, 73.0 / 15}, filter.getState(), 1e-12);
        assertArrayEquals(new double[] {3.6, 0.2, -2.8}, filter.getCovariance()[0], 1e-12);
        assertArrayEquals(new double[] {0.2, 86.0 / 15, 11.0 / 15}, filter.getCovariance()[1], 1e-12);
        assertArrayEquals(new double[] {-2.8, 11.0 / 15, 56.0 / 15}, filter.getCovariance()[2], 1e-12);
    }

    @Test public void overflowingMatrixPredictionLeavesThePreviousEstimateIntact() {
        KalmanFilter filter = new KalmanFilter(1);
        filter.reset(new double[] {1}, new double[][] {{2}});
        assertThrows(IllegalArgumentException.class,
                () -> filter.predict(new double[][] {{Double.MAX_VALUE}}, new double[][] {{0}}));
        assertEquals(1, filter.getState(0), 0);
        assertEquals(2, filter.getCovariance(0, 0), 0);
    }

    @Test(expected = IllegalArgumentException.class)
    public void rejectsIndefiniteCovarianceImmediately() {
        new KalmanFilter(2).reset(new double[]{0, 0}, new double[][]{{1, 2}, {2, 1}});
    }
    @Test public void acceptsSingularPositiveSemidefiniteCovariance() {
        KalmanFilter filter = new KalmanFilter(2);
        filter.reset(new double[]{0, 0}, new double[][]{{1, 1}, {1, 1}});
        assertEquals(.1, filter.innovationVariance(new double[]{1, -1}, .1), 1e-12);
    }

    @Test public void matchesHandComputedScalarPredictionAndCorrection() {
        KalmanFilter filter = new KalmanFilter(1);
        filter.reset(new double[] {0}, new double[][] {{4}});
        filter.predict(new double[][] {{1}}, new double[][] {{1}});
        assertEquals(5, filter.getCovariance(0, 0), 0);
        filter.correctComponent(2, 0, 5);
        assertEquals(1, filter.getState(0), 1e-12);
        assertEquals(2.5, filter.getCovariance(0, 0), 1e-12);
    }

    @Test public void supportsLinearObservationsThatMixStateComponents() {
        KalmanFilter filter = new KalmanFilter(2);
        filter.reset(new double[] {0, 1}, new double[][] {{1, 0}, {0, 1}});
        filter.predict(new double[][] {{1, 1}, {0, 1}}, new double[][] {{0, 0}, {0, 0}});
        assertEquals(2, filter.innovation(4, new double[] {1, 1}), 0);
        assertEquals(6, filter.innovationVariance(new double[] {1, 1}, 1), 0);
        filter.correct(4, new double[] {1, 1}, 1);
        assertEquals(2, filter.getState(0), 1e-12);
        assertEquals(5.0 / 3, filter.getState(1), 1e-12);
        assertEquals(0.5, filter.getCovariance(0, 0), 1e-12);
        assertEquals(0, filter.getCovariance(0, 1), 1e-12);
        assertEquals(1.0 / 3, filter.getCovariance(1, 1), 1e-12);
    }

    @Test public void stateDimensionsAndInstancesAreIndependentAndSnapshotsAreDefensive() {
        KalmanFilter filter = new KalmanFilter(3), other = new KalmanFilter(3);
        double[] initial = {1, 2, 3};
        double[][] covariance = {{1, 0, 0}, {0, 2, 0}, {0, 0, 3}};
        filter.reset(initial, covariance);
        initial[0] = 99; covariance[0][0] = 99;
        filter.correctComponent(5, 1, 2);
        assertEquals(1, filter.getState(0), 0);
        assertEquals(3.5, filter.getState(1), 1e-12);
        assertEquals(3, filter.getState(2), 0);
        assertEquals(0, other.getState(1), 0);
        filter.getState()[0] = 99;
        filter.getCovariance()[0][0] = 99;
        assertEquals(1, filter.getState(0), 0);
        assertEquals(1, filter.getCovariance(0, 0), 0);
    }

    @Test public void invalidInputsLeaveStateAndCovarianceUnchanged() {
        KalmanFilter filter = new KalmanFilter(2);
        filter.reset(new double[] {1, 2}, new double[][] {{1, 0}, {0, 1}});
        assertThrows(IllegalArgumentException.class, () -> filter.correctComponent(Double.NaN, 0, 1));
        assertThrows(IllegalArgumentException.class, () -> filter.correctComponent(2, 0, -1));
        assertThrows(IllegalArgumentException.class, () -> filter.predict(new double[][] {{1}}, new double[][] {{1}}));
        assertThrows(IllegalArgumentException.class, () -> filter.setCovariance(new double[][] {{1, 2}, {0, 1}}));
        assertThrows(IllegalArgumentException.class, () -> filter.reset(new double[] {Double.NaN, 0},
                new double[][] {{1, 0}, {0, 1}}));
        assertEquals(1, filter.getState(0), 0);
        assertEquals(2, filter.getState(1), 0);
        assertEquals(1, filter.getCovariance(0, 0), 0);
    }
}
