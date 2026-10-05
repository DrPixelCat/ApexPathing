package localizers.util;

import geometry.Angle;
import geometry.DistUnit;
import geometry.Pose;
import geometry.Vector;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.*;

public class PoseEstimatorTest {
    private static final long STEP = 20_000_000L;

    @Test public void estimatesVelocityAndAccelerationWithoutDifferencingFilteredPosition() {
        for (long period : new long[] {10_000_000L, 20_000_000L, 40_000_000L}) {
            PoseEstimator filter = new PoseEstimator();
            for (int i = 0; i < 300; i++) {
                double t = i * period / 1e9;
                filter.updateOdometry(pose(1.5 * t * t, -t * t, 0.1 * t * t), i * period);
                if (i > 100) {
                    assertEquals(3 * t, filter.getVelocity().getX().getIn(), 0.01);
                    assertEquals(-2 * t, filter.getVelocity().getY().getIn(), 0.01);
                    assertEquals(0.2 * t, filter.getVelocity().getHeading().getRad(), 0.002);
                    assertEquals(3, filter.getAcceleration().getX().getIn(), 0.02);
                    assertEquals(-2, filter.getAcceleration().getY().getIn(), 0.02);
                    assertEquals(0.2, filter.getAcceleration().getHeading().getRad(), 0.005);
                }
            }
        }
    }

    @Test public void jointStateReducesVelocityAndAccelerationErrorComparedWithSeparateDerivativeFilter() {
        PoseEstimator filter = new PoseEstimator();
        AdaptiveKalmanFilter separate = new AdaptiveKalmanFilter();
        separate.setTuning(new AdaptiveKalmanFilter.KalmanTuning(0.25, 625));
        Random random = new Random(9274);
        double previous = 0, jointVelocityError = 0, separateVelocityError = 0;
        double jointAccelerationError = 0, separateAccelerationError = 0;
        int samples = 0;
        for (int i = 0; i < 1800; i++) {
            double t = i * 0.02;
            double position = 8 * Math.sin(0.7 * t);
            double speed = 5.6 * Math.cos(0.7 * t);
            double acceleration = -3.92 * Math.sin(0.7 * t);
            double reading = position + random.nextGaussian() * 0.05;
            filter.updateOdometry(pose(reading, 0, 0), i * STEP);
            FilterState legacy = separate.update(i == 0 ? 0 : (reading - previous) / 0.02, i * STEP);
            if (i >= 200) {
                jointVelocityError += square(filter.getVelocity().getX().getIn() - speed);
                separateVelocityError += square(legacy.value() - speed);
                jointAccelerationError += square(filter.getAcceleration().getX().getIn() - acceleration);
                separateAccelerationError += square(legacy.rate() - acceleration);
                samples++;
            }
            previous = reading;
        }
        System.out.printf(java.util.Locale.US,
                "Joint-state comparison: velocity RMS %.5f vs separate %.5f in/s; acceleration RMS %.5f vs separate %.5f in/s^2%n",
                Math.sqrt(jointVelocityError / samples), Math.sqrt(separateVelocityError / samples),
                Math.sqrt(jointAccelerationError / samples), Math.sqrt(separateAccelerationError / samples));
        assertTrue(jointVelocityError < separateVelocityError);
        assertTrue(jointAccelerationError < separateAccelerationError);
    }

    @Test public void usesCorrelatedNativeVelocityInTheSameStateAndKeepsCovariancePositive() {
        PoseEstimator positionOnly = new PoseEstimator();
        PoseEstimator withVelocity = new PoseEstimator();
        Random random = new Random(1597);
        double inferredError = 0, observedError = 0;
        for (int i = 0; i < 1500; i++) {
            double t = i * 0.02;
            double position = 8 * Math.sin(0.7 * t), speed = 5.6 * Math.cos(0.7 * t);
            double commonNoise = random.nextGaussian();
            Pose measured = pose(position + commonNoise * 0.05, 0, 0);
            positionOnly.updateOdometry(measured, i * STEP);
            withVelocity.updateOdometry(measured, pose(speed + commonNoise * 0.3, 0, 0), i * STEP);
            if (i >= 200) {
                inferredError += square(positionOnly.getVelocity().getX().getIn() - speed);
                observedError += square(withVelocity.getVelocity().getX().getIn() - speed);
            }
            assertPositiveCovariance(withVelocity.getStateCovariance());
        }
        assertTrue("Native velocity must improve tracking despite correlated noise", observedError < inferredError);
        assertTrue(Math.abs(withVelocity.getStateCovariance()[0][1]) > 0);
        double[][] covariance = withVelocity.getStateCovariance();
        covariance[0][1] = Double.NaN;
        assertTrue(Double.isFinite(withVelocity.getStateCovariance()[0][1]));
    }

    @Test public void invalidNativeVelocityDoesNotPartiallyUpdateAndPauseClearsStaleMotion() {
        PoseEstimator filter = new PoseEstimator();
        filter.updateOdometry(pose(0, 0, 0), pose(10, -2, 1), 0);
        assertEquals(10, filter.getVelocity().getX().getIn(), 0);
        Pose previous = filter.getPose();
        assertFalse(filter.updateOdometry(pose(1, 0, 0), pose(Double.NaN, 0, 0), STEP));
        assertSame(previous, filter.getPose());
        assertEquals(10, filter.getVelocity().getX().getIn(), 0);
        filter.updateOdometry(pose(50, 0, 0), 1_000_000_000);
        assertEquals(0, filter.getVelocity().getX().getIn(), 0);
        assertEquals(0, filter.getAcceleration().getX().getIn(), 0);
        filter.reset(pose(100, 0, 0));
        assertEquals(0, filter.getVelocity().getX().getIn(), 0);
    }

    @Test public void reducesNativePoseNoiseOnKnownMotionWithoutAnotherSensor() {
        PoseEstimator filter = new PoseEstimator();
        Random random = new Random(7842);
        double rawPositionSquared = 0, filteredPositionSquared = 0;
        double rawHeadingSquared = 0, filteredHeadingSquared = 0;
        int samples = 0;
        for (int i = 0; i < 1800; i++) {
            double t = i * 0.02;
            Pose truth = pose(8 * Math.sin(0.7 * t), 6 * Math.cos(0.4 * t) - 6, 0.5 * Math.sin(0.3 * t));
            Pose measured = pose(truth.getX().getIn() + random.nextGaussian() * 0.05,
                    truth.getY().getIn() + random.nextGaussian() * 0.05,
                    truth.getHeading().getRad() + random.nextGaussian() * 0.01);
            assertTrue(filter.updateOdometry(measured, i * STEP));
            if (i >= 200) {
                rawPositionSquared += positionErrorSquared(measured, truth);
                filteredPositionSquared += positionErrorSquared(filter.getPose(), truth);
                rawHeadingSquared += square(Angle.wrap(measured.getHeading().getRad() - truth.getHeading().getRad()));
                filteredHeadingSquared += square(Angle.wrap(filter.getPose().getHeading().getRad() - truth.getHeading().getRad()));
                samples++;
            }
            assertBounds(filter.getTuning(), PoseEstimator.Tuning.defaults());
        }
        System.out.printf(java.util.Locale.US,
                "Native-reading simulation: raw position RMS %.5f in, filtered %.5f in; raw heading RMS %.5f rad, filtered %.5f rad%n",
                Math.sqrt(rawPositionSquared / samples), Math.sqrt(filteredPositionSquared / samples),
                Math.sqrt(rawHeadingSquared / samples), Math.sqrt(filteredHeadingSquared / samples));
        assertTrue("Pose filter must improve noise error including motion lag", filteredPositionSquared < rawPositionSquared);
        assertTrue("Heading filter must improve noise error including motion lag", filteredHeadingSquared < rawHeadingSquared);
    }

    @Test public void tracksConstantVelocityAcrossLoopRatesWithLimitedLag() {
        for (long period : new long[] {10_000_000L, 20_000_000L, 40_000_000L}) {
            PoseEstimator filter = new PoseEstimator();
            for (int i = 0; i < 300; i++) {
                double t = i * period / 1e9;
                filter.updateOdometry(pose(10 * t, -5 * t, 0.4 * t), i * period);
                if (i > 50) {
                    assertEquals(10 * t, filter.getPose().getX().getIn(), 0.03);
                    assertEquals(-5 * t, filter.getPose().getY().getIn(), 0.03);
                    assertEquals(0, Angle.wrap(filter.getPose().getHeading().getRad() - 0.4 * t), 0.005);
                    assertEquals(10, filter.getVelocity().getX().getIn(), 0.03);
                    assertEquals(-5, filter.getVelocity().getY().getIn(), 0.03);
                    assertEquals(0.4, filter.getVelocity().getHeading().getRad(), 0.005);
                }
            }
        }
    }

    @Test public void followsReversalAndStopInsteadOfTreatingMotionAsAnOutlier() {
        PoseEstimator filter = new PoseEstimator();
        double x = 0;
        for (int i = 0; i < 300; i++) {
            double speed = i < 100 ? 10 : i < 200 ? -10 : 0;
            x += speed * 0.02;
            filter.updateOdometry(pose(x, 0, 0), i * STEP);
            if (i == 130 || i == 230 || i == 299) {
                assertEquals(x, filter.getPose().getX().getIn(), 0.15);
                assertEquals(speed, filter.getVelocity().getX().getIn(), 0.5);
            }
        }
    }

    @Test public void wrapsHeadingWithoutFilteringAFullTurnDiscontinuity() {
        PoseEstimator filter = new PoseEstimator();
        filter.reset(pose(0, 0, Math.PI - 0.1));
        for (int i = 0; i < 200; i++) {
            double expected = Math.PI - 0.1 + i * 0.02;
            filter.updateOdometry(pose(0, 0, Angle.wrap(expected)), i * STEP);
            if (i > 20) { assertEquals(0, Angle.wrap(filter.getPose().getHeading().getRad() - expected), 0.005); }
        }
    }

    @Test public void rejectsInvalidAndRepeatedSamplesAndReacquiresAfterAnOutage() {
        PoseEstimator filter = new PoseEstimator();
        filter.updateOdometry(pose(1, 2, 0.1), 0);
        Pose previous = filter.getPose();
        assertFalse(filter.updateOdometry(pose(Double.NaN, 0, 0), STEP));
        assertFalse(filter.updateOdometry(pose(99, 0, 0), 0));
        assertSame(previous, filter.getPose());
        assertTrue(filter.updateOdometry(pose(10, 5, 0.5), 1_000_000_000));
        assertEquals(10, filter.getPose().getX().getIn(), 0);
        assertEquals(5, filter.getPose().getY().getIn(), 0);
    }

    @Test public void resetAndFreezeRetainCalibrationAndCovarianceIsDefensive() {
        PoseEstimator filter = new PoseEstimator();
        for (int i = 0; i < 300; i++) {
            filter.updateOdometry(pose(0.05 * Math.sin(i), 0, 0.005 * Math.sin(i)), i * STEP);
        }
        filter.setAutoTuning(false);
        PoseEstimator.Tuning learned = filter.getTuning();
        for (int i = 300; i < 400; i++) { filter.updateOdometry(pose(i * 0.1, 0, 0), i * STEP); }
        assertTuning(learned, filter.getTuning());
        filter.reset(pose(50, -2, 0.7));
        assertTuning(learned, filter.getTuning());
        assertEquals(50, filter.getPose().getX().getIn(), 0);
        filter.updateOdometry(pose(50, -2, 0.7), 0);
        assertTuning(learned, filter.getTuning());
        double[][] covariance = filter.getCovariance();
        assertTrue(covariance[0][0] > 0 && covariance[1][1] > 0 && covariance[2][2] > 0);
        covariance[0][0] = Double.NaN;
        assertTrue(Double.isFinite(filter.getCovariance()[0][0]));
    }

    @Test public void doesNotClaimToRemoveSystematicSensorDrift() {
        PoseEstimator filter = new PoseEstimator();
        double bias = 0;
        for (int i = 0; i < 400; i++) {
            bias = i * 0.01;
            filter.updateOdometry(pose(bias, 0, 0), i * STEP);
        }
        assertEquals("Without an independent reference, the filter follows sensor bias", bias,
                filter.getPose().getX().getIn(), 0.02);
    }

    @Test(expected = IllegalArgumentException.class) public void rejectsNonFiniteTuning() {
        new PoseEstimator.Tuning(new AdaptiveKalmanFilter.KalmanTuning(Double.NaN, 1),
                new AdaptiveKalmanFilter.KalmanTuning(1, 1), new AdaptiveKalmanFilter.KalmanTuning(1, 1));
    }

    private static void assertTuning(PoseEstimator.Tuning a, PoseEstimator.Tuning b) {
        assertEquals(a.x.measurementVariance, b.x.measurementVariance, 0);
        assertEquals(a.x.processVariance, b.x.processVariance, 0);
        assertEquals(a.y.measurementVariance, b.y.measurementVariance, 0);
        assertEquals(a.y.processVariance, b.y.processVariance, 0);
        assertEquals(a.heading.measurementVariance, b.heading.measurementVariance, 0);
        assertEquals(a.heading.processVariance, b.heading.processVariance, 0);
    }
    private static void assertBounds(PoseEstimator.Tuning value, PoseEstimator.Tuning baseline) {
        AdaptiveKalmanFilter.KalmanTuning[] values = {value.x, value.y, value.heading};
        AdaptiveKalmanFilter.KalmanTuning[] bases = {baseline.x, baseline.y, baseline.heading};
        for (int i = 0; i < 3; i++) {
            assertTrue(values[i].measurementVariance >= bases[i].measurementVariance * 0.25);
            assertTrue(values[i].measurementVariance <= bases[i].measurementVariance * 4);
            assertTrue(values[i].processVariance >= bases[i].processVariance * 0.1);
            assertTrue(values[i].processVariance <= bases[i].processVariance * 10);
        }
    }
    private static double positionErrorSquared(Pose a, Pose b) {
        return square(a.getX().getIn() - b.getX().getIn()) + square(a.getY().getIn() - b.getY().getIn());
    }
    private static double square(double x) { return x * x; }
    private static void assertPositiveCovariance(double[][] covariance) {
        for (int offset = 0; offset < 9; offset += 3) {
            for (int i = 0; i < 3; i++) {
                assertTrue(Double.isFinite(covariance[offset + i][offset + i]));
                assertTrue(covariance[offset + i][offset + i] > 0);
                for (int j = 0; j < 3; j++) {
                    assertEquals(covariance[offset + i][offset + j], covariance[offset + j][offset + i], 1e-9);
                }
            }
            double a = covariance[offset][offset], b = covariance[offset][offset + 1];
            double c = covariance[offset][offset + 2], d = covariance[offset + 1][offset + 1];
            double e = covariance[offset + 1][offset + 2], f = covariance[offset + 2][offset + 2];
            assertTrue(a * d - b * b > 0);
            assertTrue(a * (d * f - e * e) - b * (b * f - c * e) + c * (b * e - c * d) > 0);
        }
    }
    private static Pose pose(double x, double y, double h) {
        return new Pose(Vector.of(x, y, DistUnit.IN), Angle.fromRad(h));
    }
}
