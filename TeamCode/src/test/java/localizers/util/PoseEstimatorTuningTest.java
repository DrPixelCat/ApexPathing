package localizers.util;

import geometry.Angle;
import geometry.DistUnit;
import geometry.Pose;
import geometry.Vector;
import org.junit.Test;
import static org.junit.Assert.*;

/** Fits only on training samples, then evaluates frozen parameters on independent noisy motion. */
public class PoseEstimatorTuningTest {
    @Test public void noisyCalibrationLearnsFreezesAndImprovesHeldOutMotionAcrossLoopRates() {
        double tunedPositionError = 0, untunedPositionError = 0;
        double tunedVelocityError = 0, untunedVelocityError = 0;
        double tunedHeadingError = 0, untunedHeadingError = 0;
        double tunedAngularVelocityError = 0, untunedAngularVelocityError = 0;
        int evaluated = 0;
        for (double dt : new double[] {0.01, 0.02, 0.04}) {
            for (double noise : new double[] {0.025, 0.05, 0.10}) {
                PoseEstimator trained = new PoseEstimator();
                LocalizationNoise trainingNoise = noise(7842, noise);
                long period = Math.round(dt * 1e9);
                for (int i = 0; i < 1600; i++) {
                    LocalizationNoise.Sample sample = trainingNoise.sample(Pose.zero(), Pose.zero());
                    assertTrue(trained.updateOdometry(sample.position, sample.velocity, i * period));
                }
                PoseEstimator.Tuning learned = trained.getTuning();
                assertTrue("Training must change position noise", Math.abs(learned.x.measurementVariance - 0.0025) > 1e-5);
                assertTrue("Noise should be in a plausible range", learned.x.measurementVariance >= noise * noise * 0.5
                        && learned.x.measurementVariance <= noise * noise * 3);
                assertTrue("Native velocity noise must adapt", Math.abs(learned.xVelocityVariance - 0.25) > 1e-5);
                assertTrue("Process uncertainty must adapt", Math.abs(learned.x.processVariance - 625) > 1);
                trained.setAutoTuning(false);
                trained.reset(Pose.zero());
                assertTuningEqual(learned, trained.getTuning());
                PoseEstimator baseline = new PoseEstimator();
                baseline.setAutoTuning(false);
                // Both estimators receive exactly the same held-out observations. The held-out
                // seed and trajectory were not used to learn any parameter above.
                LocalizationNoise validationNoise = noise(1597, noise);
                for (int i = 0; i < 900; i++) {
                    double t = i * dt;
                    Pose truth = pose(8 * Math.sin(0.6 * t), 5 * Math.sin(0.4 * t), 0.4 * Math.sin(0.3 * t));
                    Pose speed = pose(4.8 * Math.cos(0.6 * t), 2 * Math.cos(0.4 * t), 0.12 * Math.cos(0.3 * t));
                    LocalizationNoise.Sample sample = validationNoise.sample(truth, speed);
                    trained.updateOdometry(sample.position, sample.velocity, i * period);
                    baseline.updateOdometry(sample.position, sample.velocity, i * period);
                    if (i > 100) {
                        tunedPositionError += error(trained.getPose(), truth);
                        untunedPositionError += error(baseline.getPose(), truth);
                        tunedVelocityError += error(trained.getVelocity(), speed);
                        untunedVelocityError += error(baseline.getVelocity(), speed);
                        tunedHeadingError += square(Angle.wrap(trained.getPose().getHeading().getRad() - truth.getHeading().getRad()));
                        untunedHeadingError += square(Angle.wrap(baseline.getPose().getHeading().getRad() - truth.getHeading().getRad()));
                        tunedAngularVelocityError += square(trained.getVelocity().getHeading().getRad() - speed.getHeading().getRad());
                        untunedAngularVelocityError += square(baseline.getVelocity().getHeading().getRad() - speed.getHeading().getRad());
                        evaluated++;
                    }
                }
                assertTuningEqual(learned, trained.getTuning());
            }
        }
        System.out.printf(java.util.Locale.US,
                "Frozen held-out tuning: position RMS %.5f vs untuned %.5f in; velocity RMS %.5f vs untuned %.5f in/s; %d samples%n",
                Math.sqrt(tunedPositionError / evaluated), Math.sqrt(untunedPositionError / evaluated),
                Math.sqrt(tunedVelocityError / evaluated), Math.sqrt(untunedVelocityError / evaluated), evaluated);
        assertTrue("Learned calibration should improve held-out position error", tunedPositionError < untunedPositionError);
        assertTrue("Learned calibration should improve held-out velocity error", tunedVelocityError < untunedVelocityError);
        assertTrue("Learned calibration should improve heading error", tunedHeadingError < untunedHeadingError);
        assertTrue("Learned calibration should improve angular velocity error", tunedAngularVelocityError < untunedAngularVelocityError);
    }

    @Test public void positionOnlyCalibrationLearnsWithoutAnInventedVelocityObservation() {
        PoseEstimator estimator = new PoseEstimator();
        LocalizationNoise noise = noise(9817, 0.1);
        for (int i = 0; i < 1200; i++) {
            estimator.updateOdometry(noise.sample(Pose.zero(), Pose.zero()).position, i * 20_000_000L);
        }
        assertTrue(estimator.getTuning().x.measurementVariance > 0.0025);
        assertEquals("Without native readings its measurement variance must not be learned",
                0.25, estimator.getTuning().xVelocityVariance, 0);
        estimator.setAutoTuning(false);
        PoseEstimator.Tuning frozen = estimator.getTuning();
        estimator.reset(pose(50, -10, 1));
        for (int i = 0; i < 400; i++) {
            estimator.updateOdometry(pose(50 + i * 0.2, -10, 1), i * 20_000_000L);
        }
        assertTuningEqual(frozen, estimator.getTuning());
    }

    private static LocalizationNoise noise(long seed, double sigma) {
        return new LocalizationNoise(seed, sigma, sigma * 0.2, sigma * 8, sigma * 1.6, 0.6);
    }

    private static double error(Pose estimate, Pose truth) {
        double x = estimate.getX().getIn() - truth.getX().getIn();
        double y = estimate.getY().getIn() - truth.getY().getIn();
        return x * x + y * y;
    }

    private static double square(double value) { return value * value; }

    private static Pose pose(double x, double y, double heading) {
        return new Pose(Vector.of(x, y, DistUnit.IN), Angle.fromRad(heading));
    }

    private static void assertTuningEqual(PoseEstimator.Tuning expected, PoseEstimator.Tuning actual) {
        AdaptiveKalmanFilter.KalmanTuning[] e = {expected.x, expected.y, expected.heading};
        AdaptiveKalmanFilter.KalmanTuning[] a = {actual.x, actual.y, actual.heading};
        for (int axis = 0; axis < 3; axis++) {
            assertEquals(e[axis].measurementVariance, a[axis].measurementVariance, 0);
            assertEquals(e[axis].processVariance, a[axis].processVariance, 0);
        }
        assertEquals(expected.xVelocityVariance, actual.xVelocityVariance, 0);
        assertEquals(expected.yVelocityVariance, actual.yVelocityVariance, 0);
        assertEquals(expected.headingVelocityVariance, actual.headingVelocityVariance, 0);
    }
}
