package localizers.util;

import org.junit.Test;
import static org.junit.Assert.*;

public class KalmanFilterTest {

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
        assertThrows(IllegalArgumentException.class, () -> filter.correctComponent(2, 0, 0));
        assertThrows(IllegalArgumentException.class, () -> filter.predict(new double[][] {{1}}, new double[][] {{1}}));
        assertThrows(IllegalArgumentException.class, () -> filter.setCovariance(new double[][] {{1, 2}, {0, 1}}));
        assertThrows(IllegalArgumentException.class, () -> filter.reset(new double[] {Double.NaN, 0},
                new double[][] {{1, 0}, {0, 1}}));
        assertEquals(1, filter.getState(0), 0);
        assertEquals(2, filter.getState(1), 0);
        assertEquals(1, filter.getCovariance(0, 0), 0);
    }
}
