package localizers;

import storage.ApexStorage;

import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;

import localizers.BaseLocalizer;
import localizers.BaseLocalizerConstants;
import localizers.util.AdaptiveKalmanFilter;
import localizers.util.PoseEstimator;

/** Persisted geometry, pose-filter and velocity-filter calibration for one robot configuration. */
public final class LocalizationConstants {
    public static final int SCHEMA_VERSION = 1;

    /** Validation state recorded independently for geometry and filters. */
    public enum Status { UNCALIBRATED, ACCEPTED, NEEDS_VALIDATION }

    private final JSONObject setups;
    private boolean followerRevalidationRequired;

    private LocalizationConstants(JSONObject setups, boolean followerRevalidationRequired) {
        this.setups = setups;
        this.followerRevalidationRequired = followerRevalidationRequired;
    }

    /** Loads persisted calibration. Invalid or unsupported files produce an empty calibration. */
    public static LocalizationConstants load() {
        File file = ApexStorage.getReadableLocalizationFile();
        if (!file.exists()) { return empty(); }
        try (BufferedReader reader = new BufferedReader(new FileReader(file))) {
            StringBuilder text = new StringBuilder();
            String line;
            while ((line = reader.readLine()) != null) { text.append(line); }
            JSONObject root = new JSONObject(text.toString());
            if (root.optInt("schemaVersion", -1) != SCHEMA_VERSION) { return empty(); }
            JSONObject setups = root.optJSONObject("setups");
            return new LocalizationConstants(setups == null ? new JSONObject() : setups,
                    root.optBoolean("followerRevalidationRequired", false));
        } catch (Exception ignored) {
            return empty();
        }
    }

    /** Creates an empty in-memory calibration. */
    public static LocalizationConstants empty() {
        return new LocalizationConstants(new JSONObject(), false);
    }

    /** Applies compatible saved geometry before hardware construction. */
    public boolean applyGeometry(String setupName, BaseLocalizerConstants<?> config) {
        JSONObject setup = compatibleSetup(setupName, config);
        if (setup == null) { return false; }
        JSONObject geometry = setup.optJSONObject("geometry");
        if (geometry == null) { return false; }
        config.applyCalibrationValues(geometry);
        return true;
    }

    /** Applies compatible filter settings after hardware construction. */
    public boolean applyFilter(String setupName, BaseLocalizerConstants<?> config,
                               BaseLocalizer<?> localizer) {
        JSONObject setup = compatibleSetup(setupName, config);
        JSONObject filter = setup == null ? null : setup.optJSONObject("filter");
        if (filter == null) { return false; }
        try {
            BaseLocalizer.VelocityFilterMode mode = BaseLocalizer.VelocityFilterMode.valueOf(
                    filter.getString("mode"));
            int window = mode == BaseLocalizer.VelocityFilterMode.MOVING_AVERAGE
                    ? Math.max(1, filter.getInt("window")) : localizer.getFilterWindowSize();
            AdaptiveKalmanFilter.KalmanTuning x = null, y = null, heading = null;
            if (mode == BaseLocalizer.VelocityFilterMode.ADAPTIVE_KALMAN) {
                x = readTuning(filter.getJSONObject("x"));
                y = readTuning(filter.getJSONObject("y"));
                heading = readTuning(filter.getJSONObject("heading"));
            }
            JSONObject pose = filter.optJSONObject("pose");
            if (filter.has("pose") && pose == null) { throw new IllegalArgumentException("Invalid pose record"); }
            PoseEstimator.Tuning poseTuning = null;
            if (pose != null) {
                if (!"SENSOR_SMOOTHING".equals(pose.getString("kind"))) {
                    throw new IllegalArgumentException("Unsupported pose filter");
                }
                boolean legacyModel = !pose.has("model");
                if (!legacyModel && !"CONSTANT_ACCELERATION".equals(pose.getString("model"))) {
                    throw new IllegalArgumentException("Unsupported kinematic model");
                }
                AdaptiveKalmanFilter.KalmanTuning px = readPoseAxis(pose.getJSONObject("x"), x, legacyModel);
                AdaptiveKalmanFilter.KalmanTuning py = readPoseAxis(pose.getJSONObject("y"), y, legacyModel);
                AdaptiveKalmanFilter.KalmanTuning ph = readPoseAxis(pose.getJSONObject("heading"), heading, legacyModel);
                poseTuning = new PoseEstimator.Tuning(px, py, ph,
                        readVelocityVariance(pose.getJSONObject("x"), x, 0.25, legacyModel),
                        readVelocityVariance(pose.getJSONObject("y"), y, 0.25, legacyModel),
                        readVelocityVariance(pose.getJSONObject("heading"), heading, 0.01, legacyModel));
            }
            localizer.setVelocityFilterMode(mode);
            if (mode == BaseLocalizer.VelocityFilterMode.MOVING_AVERAGE) {
                localizer.setFilterWindow(window);
            } else {
                localizer.setXKalmanTuning(x);
                localizer.setYKalmanTuning(y);
                localizer.setHeadingKalmanTuning(heading);
            }
            if (poseTuning != null) {
                localizer.setPoseKalmanTuning(poseTuning);
                boolean auto = pose.optBoolean("autoTuning", true);
                localizer.setPoseAutoTuning(pose.optBoolean("tuneX", auto),
                        pose.optBoolean("tuneY", auto), pose.optBoolean("tuneHeading", auto));
                localizer.setPoseFilteringEnabled(pose.optBoolean("enabled", true));
            }
            return true;
        } catch (Exception ignored) {
            return false;
        }
    }

    /** Captures current geometry and filter values after the operator accepts a result. */
    public void capture(String setupName, BaseLocalizerConstants<?> config,
                        BaseLocalizer<?> localizer, Status geometryStatus, Status filterStatus) {
        JSONObject setup = setup(setupName);
        try {
            setup.put("localizerType", config.getCalibrationType());
            setup.put("geometry", config.getCalibrationValues());
            setup.put("geometryStatus", geometryStatus.name());
            setup.put("filterStatus", filterStatus.name());
            JSONObject filter = new JSONObject();
            filter.put("mode", localizer.getVelocityFilterMode().name());
            filter.put("window", localizer.getFilterWindowSize());
            if (localizer.getVelocityFilterMode() == BaseLocalizer.VelocityFilterMode.ADAPTIVE_KALMAN) {
                filter.put("x", tuning(localizer.getXKalmanTuning()));
                filter.put("y", tuning(localizer.getYKalmanTuning()));
                filter.put("heading", tuning(localizer.getHeadingKalmanTuning()));
            }
            PoseEstimator.Tuning pose = localizer.getPoseKalmanTuning();
            filter.put("pose", new JSONObject().put("kind", "SENSOR_SMOOTHING")
                    .put("model", "CONSTANT_ACCELERATION")
                    .put("enabled", localizer.isPoseFilteringEnabled())
                    .put("autoTuning", localizer.isPoseAutoTuning())
                    .put("tuneX", localizer.isPoseAutoTuning(0))
                    .put("tuneY", localizer.isPoseAutoTuning(1))
                    .put("tuneHeading", localizer.isPoseAutoTuning(2))
                    .put("x", tuning(pose.x).put("velocityVariance", pose.xVelocityVariance))
                    .put("y", tuning(pose.y).put("velocityVariance", pose.yVelocityVariance))
                    .put("heading", tuning(pose.heading).put("velocityVariance", pose.headingVelocityVariance)));
            setup.put("filter", filter);
            followerRevalidationRequired = true;
        } catch (Exception e) {
            throw new IllegalStateException("Could not capture localization calibration", e);
        }
    }

    /** Marks filter validation stale after accepted geometry changes. */
    public void invalidateFilter(String setupName) {
        try { setup(setupName).put("filterStatus", Status.NEEDS_VALIDATION.name()); }
        catch (Exception e) { throw new IllegalStateException(e); }
    }

    /** Returns a saved status, defaulting to uncalibrated. */
    public Status getStatus(String setupName, boolean filter) {
        try {
            return Status.valueOf(setup(setupName).optString(
                    filter ? "filterStatus" : "geometryStatus", Status.UNCALIBRATED.name()));
        } catch (Exception ignored) { return Status.UNCALIBRATED; }
    }

    /** Marks one independent geometry procedure accepted for menu progress reporting. */
    public void markGeometryStepAccepted(String setupName, String step) {
        try {
            JSONObject setup = setup(setupName);
            JSONObject steps = setup.optJSONObject("geometrySteps");
            if (steps == null) { steps = new JSONObject(); setup.put("geometrySteps", steps); }
            steps.put(step, Status.ACCEPTED.name());
        } catch (Exception e) { throw new IllegalStateException(e); }
    }

    /** Returns the acceptance state of an individual geometry procedure. */
    public Status getGeometryStepStatus(String setupName, String step) {
        try {
            JSONObject steps = setup(setupName).optJSONObject("geometrySteps");
            return steps == null ? Status.UNCALIBRATED
                    : Status.valueOf(steps.optString(step, Status.UNCALIBRATED.name()));
        } catch (Exception ignored) { return Status.UNCALIBRATED; }
    }

    /** Returns whether follower tuning should be checked after localization changed. */
    public boolean isFollowerRevalidationRequired() { return followerRevalidationRequired; }

    /** Serializes the complete calibration file. */
    public JSONObject toJson() {
        try {
            JSONObject root = new JSONObject().put("schemaVersion", SCHEMA_VERSION)
                    .put("followerRevalidationRequired", followerRevalidationRequired)
                    .put("setups", setups);
            return new JSONObject(root.toString());
        } catch (Exception e) { throw new IllegalStateException(e); }
    }

    /** Restores an earlier snapshot after a failed transactional save. */
    public void restore(JSONObject snapshot) {
        try {
            JSONObject restored = snapshot.getJSONObject("setups");
            org.json.JSONArray oldNames = setups.names();
            if (oldNames != null) {
                for (int i = 0; i < oldNames.length(); i++) { setups.remove(oldNames.getString(i)); }
            }
            org.json.JSONArray names = restored.names();
            if (names != null) {
                for (int i = 0; i < names.length(); i++) {
                    String name = names.getString(i);
                    setups.put(name, restored.get(name));
                }
            }
            followerRevalidationRequired = snapshot.optBoolean(
                    "followerRevalidationRequired", false);
        } catch (Exception e) { throw new IllegalArgumentException("Invalid calibration snapshot", e); }
    }

    /** Saves through ApexStorage's backup-backed replacement. */
    public void save() throws java.io.IOException {
        try {
            ApexStorage.saveLocalization(toJson().toString(4));
        } catch (org.json.JSONException e) {
            throw new java.io.IOException("Could not serialize localization calibration", e);
        }
    }

    private JSONObject compatibleSetup(String name, BaseLocalizerConstants<?> config) {
        JSONObject setup = setups.optJSONObject(name);
        return setup != null && config.getCalibrationType().equals(
                setup.optString("localizerType")) ? setup : null;
    }

    private JSONObject setup(String name) {
        JSONObject setup = setups.optJSONObject(name);
        if (setup != null) { return setup; }
        setup = new JSONObject();
        try { setups.put(name, setup); }
        catch (Exception e) { throw new IllegalStateException(e); }
        return setup;
    }

    private static JSONObject tuning(AdaptiveKalmanFilter.KalmanTuning value) throws Exception {
        return new JSONObject().put("measurementVariance", value.measurementVariance)
                .put("processVariance", value.processVariance);
    }

    private static AdaptiveKalmanFilter.KalmanTuning readTuning(JSONObject value) throws Exception {
        double measurement = value.getDouble("measurementVariance");
        double process = value.getDouble("processVariance");
        if (!Double.isFinite(measurement) || measurement <= 0
                || !Double.isFinite(process) || process <= 0) {
            throw new IllegalArgumentException("Kalman variances must be finite and positive");
        }
        return new AdaptiveKalmanFilter.KalmanTuning(measurement, process);
    }

    private static AdaptiveKalmanFilter.KalmanTuning readPoseAxis(JSONObject value,
            AdaptiveKalmanFilter.KalmanTuning velocity, boolean legacyModel) throws Exception {
        AdaptiveKalmanFilter.KalmanTuning pose = readTuning(value);
        if (!legacyModel) { return pose; }
        // The old pose filter used white acceleration; the velocity filter already used white jerk.
        // Preserve its jerk calibration, or choose a starting baseline for pose-only files.
        double jerk = velocity == null ? Math.min(1e12, pose.processVariance * 100) : velocity.processVariance;
        return new AdaptiveKalmanFilter.KalmanTuning(pose.measurementVariance,
                Math.max(1e-12, Math.min(1e12, jerk)));
    }

    private static double readVelocityVariance(JSONObject pose, AdaptiveKalmanFilter.KalmanTuning velocity,
            double defaultVariance, boolean legacyModel) throws Exception {
        double value = pose.has("velocityVariance") ? pose.getDouble("velocityVariance")
                : velocity == null ? defaultVariance : velocity.measurementVariance;
        if (!Double.isFinite(value) || value <= 0) { throw new IllegalArgumentException("Invalid velocity variance"); }
        return legacyModel ? Math.max(1e-12, Math.min(1e12, value)) : value;
    }
}
