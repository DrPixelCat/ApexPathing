package localizers;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.json.JSONObject;
import org.junit.Test;

import localizers.BaseLocalizer;
import localizers.BaseLocalizerConstants;
import localizers.util.PoseEstimator;
import localizers.util.AdaptiveKalmanFilter;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class LocalizationConstantsTest {
    @Test public void legacyVelocityCalibrationWithTinyVariancesRemainsLoadable() throws Exception {
        LocalizationConstants constants = LocalizationConstants.empty();
        TestConfig config = new TestConfig();
        constants.capture("DEFAULT", config, new TestLocalizer(config), LocalizationConstants.Status.ACCEPTED,
                LocalizationConstants.Status.ACCEPTED);
        JSONObject snapshot = constants.toJson();
        JSONObject filter = snapshot.getJSONObject("setups").getJSONObject("DEFAULT").getJSONObject("filter");
        filter.remove("pose");
        filter.getJSONObject("x").put("measurementVariance", 1e-24).put("processVariance", 1e-20);
        constants.restore(snapshot);
        TestLocalizer loaded = new TestLocalizer(config);
        assertTrue(constants.applyFilter("DEFAULT", config, loaded));
        assertEquals(1e-12, loaded.getXKalmanTuning().measurementVariance, 0);
        assertEquals(1e-12, loaded.getXKalmanTuning().processVariance, 0);
    }

    @Test public void jointTuningAndPerAxisAdaptationSurviveSavingAndLoading() throws Exception {
        LocalizationConstants constants = LocalizationConstants.empty();
        TestConfig config = new TestConfig();
        TestLocalizer original = new TestLocalizer(config);
        original.setXKalmanTuning(new AdaptiveKalmanFilter.KalmanTuning(0.7, 800));
        original.setYKalmanTuning(new AdaptiveKalmanFilter.KalmanTuning(0.6, 700));
        original.setHeadingKalmanTuning(new AdaptiveKalmanFilter.KalmanTuning(0.02, 30));
        original.setKalmanAutoTuning(true, false, true);
        constants.capture("DEFAULT", config, original, LocalizationConstants.Status.ACCEPTED,
                LocalizationConstants.Status.ACCEPTED);
        TestLocalizer loaded = new TestLocalizer(config);
        assertTrue(constants.applyFilter("DEFAULT", config, loaded));
        assertEquals(0.7, loaded.getPoseKalmanTuning().xVelocityVariance, 0);
        assertEquals(800, loaded.getPoseKalmanTuning().x.processVariance, 0);
        assertEquals(0.6, loaded.getYKalmanTuning().measurementVariance, 0);
        assertEquals(30, loaded.getHeadingKalmanTuning().processVariance, 0);
        assertTrue(loaded.isPoseAutoTuning(0));
        assertFalse(loaded.isPoseAutoTuning(1));
        assertTrue(loaded.isPoseAutoTuning(2));
    }

    @Test public void oldPoseModelUsesExistingJerkCalibrationDuringMigration() throws Exception {
        LocalizationConstants constants = LocalizationConstants.empty();
        TestConfig config = new TestConfig();
        constants.capture("DEFAULT", config, new TestLocalizer(config), LocalizationConstants.Status.ACCEPTED,
                LocalizationConstants.Status.ACCEPTED);
        JSONObject snapshot = constants.toJson();
        JSONObject filter = snapshot.getJSONObject("setups").getJSONObject("DEFAULT").getJSONObject("filter");
        JSONObject pose = filter.getJSONObject("pose");
        pose.remove("model");
        pose.getJSONObject("x").put("processVariance", 25).remove("velocityVariance");
        pose.getJSONObject("y").put("processVariance", 25).remove("velocityVariance");
        pose.getJSONObject("heading").put("processVariance", 0.25).remove("velocityVariance");
        filter.getJSONObject("x").put("processVariance", 1200).put("measurementVariance", 0.8);
        constants.restore(snapshot);
        TestLocalizer loaded = new TestLocalizer(config);
        assertTrue(constants.applyFilter("DEFAULT", config, loaded));
        assertEquals(1200, loaded.getPoseKalmanTuning().x.processVariance, 0);
        assertEquals(0.8, loaded.getPoseKalmanTuning().xVelocityVariance, 0);
        assertEquals(0.0025, loaded.getPoseKalmanTuning().x.measurementVariance, 0);
    }

    @Test public void sensorOnlyPoseCalibrationRoundTripsAndLegacyFilesLoad() throws Exception {
        LocalizationConstants constants = LocalizationConstants.empty();
        TestConfig config = new TestConfig();
        TestLocalizer original = new TestLocalizer(config);
        original.setPoseKalmanTuning(new PoseEstimator.Tuning(
                new AdaptiveKalmanFilter.KalmanTuning(0.02, 40),
                new AdaptiveKalmanFilter.KalmanTuning(0.03, 50),
                new AdaptiveKalmanFilter.KalmanTuning(0.001, 0.5)));
        original.setPoseAutoTuning(false);
        original.setPoseFilteringEnabled(false);
        constants.capture("TANK", config, original, LocalizationConstants.Status.ACCEPTED,
                LocalizationConstants.Status.ACCEPTED);
        TestLocalizer loaded = new TestLocalizer(config);
        assertTrue(constants.applyFilter("TANK", config, loaded));
        assertEquals(0.02, loaded.getPoseKalmanTuning().x.measurementVariance, 0);
        assertEquals(50, loaded.getPoseKalmanTuning().y.processVariance, 0);
        assertFalse(loaded.isPoseAutoTuning());
        assertFalse(loaded.isPoseFilteringEnabled());
        JSONObject legacy = constants.toJson();
        legacy.getJSONObject("setups").getJSONObject("TANK").getJSONObject("filter").remove("pose");
        constants.restore(legacy);
        assertTrue(constants.applyFilter("TANK", config, new TestLocalizer(config)));
    }

    @Test public void rejectsInvalidPoseCalibrationBeforeChangingVelocityMode() throws Exception {
        LocalizationConstants constants = LocalizationConstants.empty();
        TestConfig config = new TestConfig();
        TestLocalizer original = new TestLocalizer(config);
        original.setVelocityFilterMode(BaseLocalizer.VelocityFilterMode.MOVING_AVERAGE);
        constants.capture("DEFAULT", config, original, LocalizationConstants.Status.ACCEPTED,
                LocalizationConstants.Status.ACCEPTED);
        JSONObject broken = constants.toJson();
        broken.getJSONObject("setups").getJSONObject("DEFAULT").getJSONObject("filter")
                .getJSONObject("pose").getJSONObject("x").put("measurementVariance", -1);
        constants.restore(broken);
        TestLocalizer target = new TestLocalizer(config);
        assertFalse(constants.applyFilter("DEFAULT", config, target));
        assertEquals(BaseLocalizer.VelocityFilterMode.ADAPTIVE_KALMAN, target.getVelocityFilterMode());
    }
    @Test public void capturesIndependentGeometryStepsAndRestoresSnapshot() {
        LocalizationConstants constants = LocalizationConstants.empty();
        TestConfig config = new TestConfig();
        TestLocalizer localizer = new TestLocalizer(config);
        localizer.setVelocityFilterMode(BaseLocalizer.VelocityFilterMode.MOVING_AVERAGE);
        localizer.setFilterWindow(9);
        constants.capture("DEFAULT", config, localizer,
                LocalizationConstants.Status.ACCEPTED,
                LocalizationConstants.Status.NEEDS_VALIDATION);
        constants.markGeometryStepAccepted("DEFAULT", "FORWARD");
        JSONObject snapshot = constants.toJson();

        constants.markGeometryStepAccepted("DEFAULT", "ROTATION");
        assertEquals(LocalizationConstants.Status.ACCEPTED,
                constants.getGeometryStepStatus("DEFAULT", "ROTATION"));
        constants.restore(snapshot);
        assertEquals(LocalizationConstants.Status.UNCALIBRATED,
                constants.getGeometryStepStatus("DEFAULT", "ROTATION"));
        assertEquals(LocalizationConstants.Status.ACCEPTED,
                constants.getGeometryStepStatus("DEFAULT", "FORWARD"));

        TestConfig loaded = new TestConfig();
        loaded.scale = 1.0;
        assertTrue(constants.applyGeometry("DEFAULT", loaded));
        assertEquals(4.0, loaded.scale, 0.0);
        TestLocalizer filtered = new TestLocalizer(loaded);
        assertTrue(constants.applyFilter("DEFAULT", loaded, filtered));
        assertEquals(BaseLocalizer.VelocityFilterMode.MOVING_AVERAGE,
                filtered.getVelocityFilterMode());
        assertEquals(9, filtered.getFilterWindowSize());
    }

    @Test public void rejectsGeometryForDifferentLocalizerType() {
        LocalizationConstants constants = LocalizationConstants.empty();
        TestConfig config = new TestConfig();
        constants.capture("DEFAULT", config, new TestLocalizer(config),
                LocalizationConstants.Status.ACCEPTED,
                LocalizationConstants.Status.ACCEPTED);
        assertFalse(constants.applyGeometry("DEFAULT", new OtherConfig()));
    }

    private static class TestConfig implements BaseLocalizerConstants<TestConfig> {
        double scale = 4.0;
        @Override public JSONObject getCalibrationValues() {
            try { return new JSONObject().put("scale", scale); }
            catch (Exception e) { throw new IllegalStateException(e); }
        }
        @Override public void applyCalibrationValues(JSONObject values) {
            scale = values.optDouble("scale", scale);
        }
        @Override public BaseLocalizer<?> build(HardwareMap hardwareMap) {
            return new TestLocalizer(this);
        }
    }

    private static final class OtherConfig extends TestConfig { }

    private static final class TestLocalizer extends BaseLocalizer<TestConfig> {
        TestLocalizer(TestConfig config) { super(config); }
        @Override public void update() { calculate(UpdateType.BOTH); }
        @Override public void setPose(geometry.Pose pose) { resetKinematicEstimate(pose); }
    }
}
