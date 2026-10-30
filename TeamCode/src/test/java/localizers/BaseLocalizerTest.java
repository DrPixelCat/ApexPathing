package localizers;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.junit.Test;

import paths.geometry.Angle;
import paths.geometry.DistUnit;
import paths.geometry.Pose;
import paths.geometry.Vector;

public class BaseLocalizerTest {
    private static final double EPSILON = 1e-9;

    @Test public void nativeVelocityKeepsUpdatingAfterTheFirstSamplingGap() {
        TimedLocalizer localizer = new TimedLocalizer();
        localizer.sample(Pose.zero(), Pose.zero(), 0);
        localizer.sample(pose(2.5, 0, 0), pose(10, 0, 1), 250_000_000);
        assertEquals(10, localizer.getRawVel().getX().getIn(), EPSILON);
        localizer.sample(pose(2.74, 0, 0), pose(12, 0, 2), 270_000_000);
        assertEquals(12, localizer.getRawVel().getX().getIn(), EPSILON);
        assertEquals(100, localizer.getRawAccel().getX().getIn(), EPSILON);
        assertEquals(2, localizer.getRawVel().getHeading().getRad(), EPSILON);
        assertEquals(270_000_000, localizer.getLastMeasurementTimeNanos());
        assertTrue(localizer.isLastMeasurementValid());
    }

    @Test public void poseDerivedVelocityUsesTheFullGapAndResumesNormalSampling() {
        TimedLocalizer localizer = new TimedLocalizer();
        localizer.sample(Pose.zero(), 0);
        localizer.sample(pose(2.5, 0, 0), 250_000_000);
        assertEquals(10, localizer.getRawVel().getX().getIn(), EPSILON);
        localizer.sample(pose(2.74, 0, 0), 270_000_000);
        assertEquals(12, localizer.getRawVel().getX().getIn(), EPSILON);
        assertEquals(100, localizer.getRawAccel().getX().getIn(), EPSILON);
        localizer.sample(pose(10.74, 0, 0), 1_270_000_000);
        assertEquals(8, localizer.getRawVel().getX().getIn(), EPSILON);
        localizer.sample(pose(10.94, 0, 0), 1_290_000_000);
        assertEquals(10, localizer.getRawVel().getX().getIn(), EPSILON);
        assertTrue(localizer.isLastMeasurementValid());
    }

    @Test public void poseOnlyLocalizerPublishesVelocityAndAccelerationFromJointState() {
        TimedLocalizer localizer = new TimedLocalizer();
        for (int i = 0; i < 300; i++) {
            double t = i * 0.02;
            localizer.sample(pose(1.5 * t * t, 0, 0.1 * t * t), i * 20_000_000L);
        }
        assertEquals(3 * 5.98, localizer.getVel().getX().getIn(), 0.01);
        assertEquals(3, localizer.getAccel().getX().getIn(), 0.02);
        assertEquals(0.2, localizer.getAccel().getHeading().getRad(), 0.005);
        // The raw finite difference is an interval average; the public velocity is instantaneous.
        assertEquals(3 * 5.97, localizer.getRawVel().getX().getIn(), EPSILON);
        assertTrue(Math.abs(localizer.getKinematicCovariance()[0][1]) > 0);
    }

    @Test public void nativeVelocityUpdatesThePositionStateAndInvalidSamplesAreAtomic() {
        TimedLocalizer observed = new TimedLocalizer(), positionOnly = new TimedLocalizer();
        for (int i = 0; i < 80; i++) {
            Pose reading = pose(i * 0.2, 0, 0);
            observed.sample(reading, pose(10, 0, 0), i * 20_000_000L);
            positionOnly.sample(reading, i * 20_000_000L);
        }
        double before = observed.getPose().getX().getIn();
        observed.sample(pose(16, 0, 0), pose(12, 0, 0), 1_600_000_000);
        positionOnly.sample(pose(16, 0, 0), 1_600_000_000);
        assertTrue(observed.getPose().getX().getIn() > positionOnly.getPose().getX().getIn());
        assertTrue(observed.getVel().getX().getIn() > 10);
        assertTrue(observed.getAccel().getX().getIn() > 0);
        Pose acceptedPose = observed.getPose(), acceptedVelocity = observed.getVel();
        observed.sample(pose(999, 0, 0), pose(Double.NaN, 0, 0), 1_620_000_000);
        assertEquals(acceptedPose, observed.getPose());
        assertEquals(acceptedVelocity, observed.getVel());
        assertEquals(16, observed.getRawPose().getX().getIn(), 0);
        assertTrue(observed.getPose().getX().getIn() > before);
        observed.setPose(pose(100, 0, 0));
        assertPoseIsZero(observed.getVel());
        assertPoseIsZero(observed.getAccel());
    }

    @Test public void existingCalibrationControlsConfigureAndFreezeTheJointEstimator() {
        TimedLocalizer localizer = new TimedLocalizer();
        localizer.setXKalmanTuning(new localizers.util.AdaptiveKalmanFilter.KalmanTuning(0.4, 900));
        assertEquals(0.4, localizer.getPoseKalmanTuning().xVelocityVariance, 0);
        assertEquals(900, localizer.getPoseKalmanTuning().x.processVariance, 0);
        localizer.setKalmanAutoTuning(true, false, true);
        assertTrue(localizer.isPoseAutoTuning(0));
        assertTrue(!localizer.isPoseAutoTuning(1));
        localizer.setIsTuning(false);
        for (int i = 0; i < 300; i++) {
            localizer.sample(pose(0.05 * Math.sin(i), 0, 0), i * 20_000_000L);
        }
        assertEquals(0.4, localizer.getXKalmanTuning().measurementVariance, 0);
        assertEquals(900, localizer.getXKalmanTuning().processVariance, 0);
        assertTrue(!localizer.isPoseAutoTuning());
    }

    @Test public void poseFilteringUsesNativeReadingsAndCanBeBypassed() {
        TimedLocalizer localizer = new TimedLocalizer();
        localizer.sample(Pose.zero(), 0);
        localizer.sample(pose(0.1, 0, 0), 20_000_000);
        assertEquals(0.1, localizer.getRawPose().getX().getIn(), 0);
        assertTrue(localizer.getPose().getX().getIn() > 0);
        assertTrue(localizer.getPose().getX().getIn() < 0.1);
        double filtered = localizer.getPose().getX().getIn();
        localizer.setVelocityFilterMode(BaseLocalizer.VelocityFilterMode.MOVING_AVERAGE);
        assertEquals(filtered, localizer.getPose().getX().getIn(), 0);
        localizer.setPoseFilteringEnabled(false);
        assertEquals(localizer.getRawPose(), localizer.getPose());
        localizer.sample(pose(0.2, 0, 0), 40_000_000);
        assertEquals(0.2, localizer.getPose().getX().getIn(), 0);
    }

    @Test public void headingDerivativesWrapAndInvalidSamplesPreserveNativeHistory() {
        TimedLocalizer localizer = new TimedLocalizer();
        localizer.setPose(pose(0, 0, Math.PI - 0.01));
        localizer.sample(pose(0, 0, Math.PI - 0.01), 0);
        localizer.sample(pose(0, 0, -Math.PI + 0.01), 20_000_000);
        assertEquals(1, localizer.getRawVel().getHeading().getRad(), EPSILON);
        localizer.sample(pose(Double.NaN, 0, 0), 40_000_000);
        assertTrue(!localizer.isLastMeasurementValid());
        assertEquals(20_000_000, localizer.getLastMeasurementTimeNanos());
        localizer.sample(pose(1, 0, -Math.PI + 0.02), 60_000_000);
        assertEquals(25, localizer.getRawVel().getX().getIn(), EPSILON);
        localizer.sample(pose(999, 0, 0), 60_000_000);
        assertTrue(!localizer.isLastMeasurementValid());
        assertEquals(1, localizer.getRawPose().getX().getIn(), 0);
    }

    @Test
    public void arcIntegrationFollowsExactQuarterCircle() {
        Pose result = BaseLocalizer.integrateArc(
                Pose.zero(), 15.0, 0.0, Math.PI / 2.0, Math.PI / 2.0);

        assertEquals(30.0 / Math.PI, result.getX().getIn(), EPSILON);
        assertEquals(30.0 / Math.PI, result.getY().getIn(), EPSILON);
        assertEquals(Math.PI / 2.0, result.getHeading().getRad(), EPSILON);
    }

    @Test
    public void arcIntegrationHasExactStraightLineLimit() {
        Pose start = pose(4.0, -2.0, Math.PI / 2.0);
        Pose result = BaseLocalizer.integrateArc(start, 2.0, 3.0, 0.0, Math.PI / 2.0);

        assertEquals(1.0, result.getX().getIn(), EPSILON);
        assertEquals(0.0, result.getY().getIn(), EPSILON);
        assertEquals(Math.PI / 2.0, result.getHeading().getRad(), EPSILON);
    }

    @Test
    public void nativeVelocityUsesSharedVelocityAndAccelerationFilter() throws Exception {
        NativeVelocityLocalizer localizer = new NativeVelocityLocalizer();
        localizer.setVelocityFilterMode(BaseLocalizer.VelocityFilterMode.MOVING_AVERAGE);

        localizer.setMeasuredVelocity(pose(0.0, 0.0, 0.0));
        localizer.update();
        Thread.sleep(5);
        localizer.setMeasuredVelocity(pose(14.0, 0.0, 0.0));
        localizer.update();

        assertEquals(14.0, localizer.getRawVel().getX().getIn(), EPSILON);
        assertEquals(7.0, localizer.getVel().getX().getIn(), EPSILON);
        assertTrue(localizer.getRawAccel().getX().getIn() > 0.0);
        assertTrue(localizer.getAccel().getX().getIn() > 0.0);
    }

    @Test
    public void adaptiveKalmanIsTheDefaultAndSupportsAngularOnlyMotion() throws Exception {
        NativeVelocityLocalizer localizer = new NativeVelocityLocalizer();
        assertEquals(BaseLocalizer.VelocityFilterMode.ADAPTIVE_KALMAN,
                localizer.getVelocityFilterMode());

        localizer.setMeasuredVelocity(pose(0.0, 0.0, 0.0));
        localizer.update();
        Thread.sleep(5);
        localizer.setMeasuredVelocity(pose(0.0, 0.0, 2.0));
        localizer.update();

        assertEquals(2.0, localizer.getRawVel().getHeading().getRad(), EPSILON);
        assertTrue(localizer.getVel().getHeading().getRad() > 0.0);
        assertEquals(0.0, localizer.getVel().getX().getIn(), EPSILON);
    }

    @Test
    public void kalmanTuningIsExplicitAndFrozenAfterOneTimeCalibration() {
        NativeVelocityLocalizer localizer = new NativeVelocityLocalizer();
        localizer.setIsTuning(true);
        assertTrue(localizer.isTuningVelocityFilter());
        localizer.setIsTuning(false);
        assertTrue(!localizer.isTuningVelocityFilter());
    }

    @Test
    public void poseResetClearsVelocityAccelerationAndFilterHistory() throws Exception {
        NativeVelocityLocalizer localizer = new NativeVelocityLocalizer();
        localizer.setMeasuredVelocity(pose(20.0, -4.0, 2.0));
        localizer.update();
        Thread.sleep(5);
        localizer.setMeasuredVelocity(pose(30.0, -8.0, 3.0));
        localizer.update();

        Pose stagedPose = pose(55.0, -12.0, 1.0);
        localizer.setPose(stagedPose);

        assertEquals(55.0, localizer.getPose().getX().getIn(), EPSILON);
        assertPoseIsZero(localizer.getVel());
        assertPoseIsZero(localizer.getRawVel());
        assertPoseIsZero(localizer.getAccel());
        assertPoseIsZero(localizer.getRawAccel());

        localizer.setMeasuredVelocity(Pose.zero());
        localizer.update();
        assertPoseIsZero(localizer.getVel());
        assertPoseIsZero(localizer.getAccel());
    }

    @Test
    public void poseDerivedLocalizerDoesNotInterpretResetAsMotion() throws Exception {
        PoseDerivedLocalizer localizer = new PoseDerivedLocalizer();
        localizer.update();
        Thread.sleep(5);
        localizer.setMeasuredPose(pose(1.0, 0.0, 0.0));
        localizer.update();

        localizer.setPose(pose(55.0, 0.0, 0.0));
        Thread.sleep(5);
        localizer.update();
        Thread.sleep(5);
        localizer.update();

        assertPoseIsZero(localizer.getRawVel());
        assertPoseIsZero(localizer.getVel());
        assertPoseIsZero(localizer.getRawAccel());
        assertPoseIsZero(localizer.getAccel());
    }

    private static void assertPoseIsZero(Pose pose) {
        assertEquals(0.0, pose.getX().getIn(), EPSILON);
        assertEquals(0.0, pose.getY().getIn(), EPSILON);
        assertEquals(0.0, pose.getHeading().getRad(), EPSILON);
    }

    private static Pose pose(double x, double y, double heading) {
        return new Pose(Vector.of(x, y, DistUnit.IN), Angle.fromRad(heading));
    }

    private static final class NativeVelocityLocalizer
            extends BaseLocalizer<TestConstants> {
        private Pose measuredVelocity = Pose.zero();

        NativeVelocityLocalizer() {
            super(new TestConstants());
        }

        void setMeasuredVelocity(Pose measuredVelocity) {
            this.measuredVelocity = measuredVelocity;
        }

        @Override
        public void update() {
            calculate(measuredVelocity);
        }

        @Override
        public void setPose(Pose newPose) {
            resetKinematicEstimate(newPose);
        }
    }

    private static final class PoseDerivedLocalizer
            extends BaseLocalizer<TestConstants> {
        private Pose measuredPose = Pose.zero();

        PoseDerivedLocalizer() {
            super(new TestConstants());
        }

        void setMeasuredPose(Pose measuredPose) {
            this.measuredPose = measuredPose;
        }

        @Override
        public void update() {
            pose = measuredPose;
            calculate(UpdateType.BOTH);
        }

        @Override
        public void setPose(Pose newPose) {
            measuredPose = newPose;
            resetKinematicEstimate(newPose);
        }
    }

    private static final class TestConstants
            implements BaseLocalizerConstants<TestConstants> {
        @Override
        public BaseLocalizer<?> build(HardwareMap hardwareMap) {
            return new NativeVelocityLocalizer();
        }
    }

    private static final class TimedLocalizer extends BaseLocalizer<TestConstants> {
        TimedLocalizer() { super(new TestConstants()); }
        void sample(Pose raw, long time) { pose = raw; calculate(UpdateType.BOTH, time); }
        void sample(Pose raw, Pose nativeVelocity, long time) { pose = raw; calculate(nativeVelocity, time); }
        @Override public void update() { calculate(UpdateType.BOTH); }
        @Override public void setPose(Pose pose) { resetKinematicEstimate(pose); }
    }
}
