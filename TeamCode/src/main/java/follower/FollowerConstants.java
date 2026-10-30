package follower;

import feedforward.MotionCharacterization;
import paths.geometry.DistUnit;
import paths.geometry.Vector;
import storage.ApexStorage;

import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;

import drivetrains.BaseDrivetrain;
import controllers.FeedbackController.Coefficients;

/**
 * Class to hold constants for the Follower class. These constants are loaded from a JSON file
 * created by the tuners. A missing file uses defaults; malformed saved data is reported explicitly.
 *
 * @author Sohum Arora - 22985 Paraducks
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public class FollowerConstants {
    /** Optional measured speed-dependent authority, scoped to the active drivetrain profile. */
    public MotionCharacterization motionCharacterization;

    public double profilePowerLimit() {
        return motionCharacterization == null ? 1.0 : motionCharacterization.profilePowerLimit();
    }
    public double calibratedVelocityLimit(MotionCharacterization.Axis axis, double configured) {
        return motionCharacterization == null ? configured
                : Math.min(configured, motionCharacterization.velocityLimit(axis));
    }
    public double characterizedPower(MotionCharacterization.Axis axis, double velocity,
                                     double acceleration, double legacy) {
        return motionCharacterization == null ? legacy
                : motionCharacterization.power(axis, velocity, acceleration, legacy);
    }
    /** Conservative unsigned demand when a scalar profile does not encode travel direction. */
    public double characterizedAxisDemand(MotionCharacterization.Axis axis, double velocity,
                                          double acceleration, double legacy) {
        if (motionCharacterization == null) { return legacy; }
        return Math.max(Math.abs(characterizedPower(axis, velocity, acceleration, legacy)),
                Math.abs(characterizedPower(axis, -velocity, -acceleration, -legacy)));
    }
    /** Robot-relative axis powers; braking and legacy profiles keep their original vector. */
    public Vector characterizedTranslation(Vector unitDirection, double velocity,
                                           double acceleration, Vector legacy) {
        if (motionCharacterization == null || velocity * acceleration < -1e-6) { return legacy; }
        double x = unitDirection.getX().getIn(), y = unitDirection.getY().getIn();
        return Vector.of(
                characterizedPower(MotionCharacterization.Axis.FORWARD, velocity * x, acceleration * x, legacy.getX().getIn()),
                characterizedPower(MotionCharacterization.Axis.STRAFE, velocity * y, acceleration * y, legacy.getY().getIn()),
                DistUnit.IN);
    }
    /** Legacy pulse/coast settings retained for constants.json compatibility; profiled paths
     * now use signed acceleration feedforward and velocityFeedbackGain instead. */
    public double brakingVelocityFeedbackGain = 0.0;
    public double brakingOverspeedToleranceIn = 3.0;
    public double brakingPulsePower = 0.30;
    public double tankBrakingPulsePower = 0.10;
    public int brakingPulseCooldownCycles = 1;
    public double brakingNeutralPowerMargin = 0.02;
    /** Scale applied to curved-path velocity ceilings before reachability sweeps. */
    public double profileCornerSpeedScale = 1.0;
    /** Optional conservative velocity power slope for profile reachability. */
    public double profileVelocityPowerCoefficient = 0.0;
    /** Legacy corner pulse thresholds retained for constants.json compatibility. */
    public double brakingCornerToleranceIn = 3.0;
    public double tankBrakingCornerToleranceIn = 3.0;
    /** Set only after the active profile has been checked on straight and corner courses. */
    public boolean profiledTrackingTuned = false;
    /** Desired first-order velocity-error time constant used after kV/kA characterization. */
    public static final double VELOCITY_FEEDBACK_TIME_CONSTANT_SECONDS = 0.10;
    /** Feedback-power limits protect translation from large localization-error commands. */
    public static final double MAX_TRANSLATION_VELOCITY_FEEDBACK_GAIN = 0.10;
    /** Angular velocity feedback is capped more tightly to avoid noise-driven oscillation. */
    public static final double MAX_ANGULAR_VELOCITY_FEEDBACK_GAIN = 0.25;
    /**
     * Note to developers:
     * If you want to add new constants, create the variable here and add it to the loadValues() and
     * valuesToJson() methods. This will ensure that the new constants are loaded from the JSON file and
     * saved back to it.
     */
    private static FollowerConstants instance;
    public BaseDrivetrain.DrivetrainType drivetrainType =
            BaseDrivetrain.DrivetrainType.MECANUM;
    public Coefficients angularCoeffs = new Coefficients().p(0.0).d(0.0).s(0.0);
    public Coefficients translationalCoeffs = new Coefficients().p(0.0).d(0.0).s(0.0);

    public double velocityFeedbackGain = 0.0;
    public double angularVelocityFeedbackGain = 0.0;
    public double translationalKV = 0.0, translationalKA = 0.0;
    public double angularKV = 0.0, angularKA = 0.0;
    /** Moving-friction feedforward terms; feedback kS remains the larger breakaway value. */
    public double translationalFeedforwardKS = 0.0, angularFeedforwardKS = 0.0;
    public double kCentripetal = 0.0;

    public double forwardVelLimitIn = 0.0;
    public double forwardAccelLimitIn = 0.0;
    public double strafeVelLimitIn = 0.0;
    public double strafeAccelLimitIn = 0.0;
    public double angularVelLimitRad = 0.0;
    public double angularAccelLimitRad = 0.0;
    /** Optional tank traction limit in in/s squared; zero means unconfigured. */
    public double maxCentripetalAccelIn = 0.0;

    public enum Profile { DEFAULT, TANK, HOLONOMIC }
    private final java.util.EnumMap<Profile, JSONObject> profiles =
            new java.util.EnumMap<>(Profile.class);
    private Profile activeProfile = Profile.DEFAULT;
    private JSONObject unassignedLegacy;
    private boolean allowUntuned;

    private FollowerConstants() { reload(); }
    private FollowerConstants(boolean empty) { }

    public Profile getActiveProfile() { return activeProfile; }
    public boolean hasUnassignedLegacy() { return unassignedLegacy != null; }
    public boolean requiresStrafeLimits() {
        return drivetrainType != BaseDrivetrain.DrivetrainType.TANK
                && activeProfile != Profile.TANK;
    }

    /** Select hardware without guessing which mode produced a legacy dual calibration. */
    public void configure(BaseDrivetrain.DrivetrainType type, Profile profile, boolean tuning) {
        allowUntuned = tuning;
        if (type != drivetrainType) {
            profiles.clear();
            unassignedLegacy = null;
            activeProfile = null;
            loadValues(new JSONObject());
        }
        drivetrainType = type;
        selectProfile(type == BaseDrivetrain.DrivetrainType.DUAL_ACTUATED ? profile : Profile.DEFAULT);
    }

    private void validateProfile(Profile profile) {
        if (profile == null || (drivetrainType == BaseDrivetrain.DrivetrainType.DUAL_ACTUATED
                && profile == Profile.DEFAULT)) {
            throw new IllegalArgumentException("Dual-actuated tuning requires TANK or HOLONOMIC");
        }
    }

    public void selectProfile(Profile profile) {
        validateProfile(profile);
        if (profile == activeProfile) { return; }
        if (!profiles.containsKey(profile) && !allowUntuned
                && drivetrainType == BaseDrivetrain.DrivetrainType.DUAL_ACTUATED) {
            throw new IllegalStateException("No saved follower profile for " + profile
                    + ". Run Follower Tuner for this mode first.");
        }
        if (activeProfile != null) { profiles.put(activeProfile, valuesToJson()); }
        loadValues(profiles.containsKey(profile) ? profiles.get(profile) : new JSONObject());
        activeProfile = profile;
    }

    /** Independent runtime configuration; constructing one follower cannot modify another. */
    public FollowerConstants copy() {
        FollowerConstants result = new FollowerConstants(true);
        result.motionCharacterization = motionCharacterization;
        result.brakingVelocityFeedbackGain = brakingVelocityFeedbackGain;
        result.brakingOverspeedToleranceIn = brakingOverspeedToleranceIn;
        result.brakingPulsePower = brakingPulsePower;
        result.tankBrakingPulsePower = tankBrakingPulsePower;
        result.brakingPulseCooldownCycles = brakingPulseCooldownCycles;
        result.brakingNeutralPowerMargin = brakingNeutralPowerMargin;
        result.profileCornerSpeedScale = profileCornerSpeedScale;
        result.profileVelocityPowerCoefficient = profileVelocityPowerCoefficient;
        result.brakingCornerToleranceIn = brakingCornerToleranceIn;
        result.tankBrakingCornerToleranceIn = tankBrakingCornerToleranceIn;
        result.profiledTrackingTuned = profiledTrackingTuned;
        result.drivetrainType = drivetrainType;
        result.velocityFeedbackGain = velocityFeedbackGain;
        result.angularVelocityFeedbackGain = angularVelocityFeedbackGain;
        result.translationalKV = translationalKV;
        result.translationalKA = translationalKA;
        result.angularKV = angularKV;
        result.angularKA = angularKA;
        result.translationalFeedforwardKS = translationalFeedforwardKS;
        result.angularFeedforwardKS = angularFeedforwardKS;
        result.kCentripetal = kCentripetal;
        result.forwardVelLimitIn = forwardVelLimitIn;
        result.forwardAccelLimitIn = forwardAccelLimitIn;
        result.strafeVelLimitIn = strafeVelLimitIn;
        result.strafeAccelLimitIn = strafeAccelLimitIn;
        result.angularVelLimitRad = angularVelLimitRad;
        result.angularAccelLimitRad = angularAccelLimitRad;
        result.maxCentripetalAccelIn = maxCentripetalAccelIn;
        result.allowUntuned = allowUntuned;
        result.activeProfile = activeProfile;
        result.angularCoeffs = copyCoefficients(angularCoeffs);
        result.translationalCoeffs = copyCoefficients(translationalCoeffs);
        for (Profile profile : profiles.keySet()) {
            result.profiles.put(profile, copyJson(profiles.get(profile)));
        }
        result.unassignedLegacy = unassignedLegacy == null ? null : copyJson(unassignedLegacy);
        return result;
    }

    private static Coefficients copyCoefficients(Coefficients source) {
        return new Coefficients().p(source.kP).d(source.kD).s(source.kS)
                .sqrt(source.kSqrt).vel(source.kV).accel(source.kA);
    }
    private static JSONObject copyJson(JSONObject source) {
        try { return new JSONObject(source.toString()); }
        catch (org.json.JSONException failure) { throw new IllegalArgumentException("Invalid constants", failure); }
    }

    /** Detached values for path generation; does not change the active drivetrain mode. */
    public FollowerConstants forProfile(Profile profile) {
        if (drivetrainType != BaseDrivetrain.DrivetrainType.DUAL_ACTUATED) { return copy(); }
        validateProfile(profile);
        if (profile != activeProfile && !profiles.containsKey(profile) && !allowUntuned) {
            throw new IllegalStateException("No saved follower profile for " + profile);
        }
        FollowerConstants result = new FollowerConstants(true);
        result.drivetrainType = drivetrainType;
        result.activeProfile = profile;
        result.loadValues(profile == activeProfile ? valuesToJson()
                : profiles.containsKey(profile) ? profiles.get(profile) : new JSONObject());
        return result;
    }

    /** Explicit operator assignment; legacy data is never automatically copied to both modes. */
    public void assignLegacyToActiveProfile() {
        if (unassignedLegacy == null || activeProfile == null) { return; }
        loadValues(unassignedLegacy);
        profiles.put(activeProfile, valuesToJson());
        unassignedLegacy = null;
    }

    public static FollowerConstants fromJson(JSONObject json) {
        FollowerConstants result = new FollowerConstants(true);
        result.readJson(json);
        return result;
    }

    private void readJson(JSONObject json) {
        int version = json.optInt("schemaVersion", 1);
        if (version != 1 && version != 2) {
            throw new IllegalArgumentException("Unsupported follower constants schema: " + version);
        }
        BaseDrivetrain.DrivetrainType type;
        try {
            String savedType = json.getString("drivetrainType");
            // Migrate tuning data from the removed three-wheel-specific drivetrain.
            type = BaseDrivetrain.DrivetrainType.valueOf("KIWI".equals(savedType) ? "OMNI" : savedType);
        }
        catch (Exception e) { throw new IllegalArgumentException("Invalid drivetrainType in follower constants", e); }
        profiles.clear();
        drivetrainType = type;
        unassignedLegacy = json.optJSONObject("unassignedLegacy");
        activeProfile = null;
        loadValues(new JSONObject());
        if (version == 1) {
            if (type == BaseDrivetrain.DrivetrainType.DUAL_ACTUATED) { unassignedLegacy = json; }
            else { activeProfile = Profile.DEFAULT; loadValues(json); }
        } else {
            JSONObject saved = json.optJSONObject("profiles");
            if (saved != null) {
                for (Profile profile : Profile.values()) {
                    JSONObject values = saved.optJSONObject(profile.name());
                    if (values != null) { profiles.put(profile, values); }
                }
            }
            if (type != BaseDrivetrain.DrivetrainType.DUAL_ACTUATED) {
                activeProfile = Profile.DEFAULT;
                loadValues(profiles.containsKey(Profile.DEFAULT)
                        ? profiles.get(Profile.DEFAULT) : new JSONObject());
            }
        }
    }

    public JSONObject toJson() {
        if (activeProfile != null) { profiles.put(activeProfile, valuesToJson()); }
        JSONObject json = new JSONObject();
        JSONObject saved = new JSONObject();
        try {
            json.put("schemaVersion", 2);
            json.put("drivetrainType", drivetrainType.name());
            for (Profile profile : profiles.keySet()) { saved.put(profile.name(), profiles.get(profile)); }
            json.put("profiles", saved);
            if (unassignedLegacy != null) { json.put("unassignedLegacy", unassignedLegacy); }
        } catch (org.json.JSONException e) { throw new IllegalStateException(e); }
        return json;
    }

    public static FollowerConstants getInstance() {
        if (instance == null) {
            instance = new FollowerConstants();
        }
        return instance;
    }

    private double loadDouble(JSONObject json, String key) {
        if (!json.has(key)) { return 0; }
        try {
            double value = json.getDouble(key);
            if (!Double.isFinite(value)) { throw new IllegalArgumentException("Non-finite " + key); }
            return value;
        } catch (org.json.JSONException failure) {
            throw new IllegalArgumentException("Invalid numeric setting: " + key, failure);
        }
    }

    public void reload() {
        File file = ApexStorage.getReadableConstantsFile();
        if (!file.exists()) { return; }

        JSONObject json;
        try (BufferedReader reader = new BufferedReader(new FileReader(file))) {
            StringBuilder sb = new StringBuilder();
            String line = reader.readLine();
            while (line != null) {
                sb.append(line);
                line = reader.readLine();
            }
            json = new JSONObject(sb.toString());
        } catch (Exception e) {
            throw new IllegalStateException("Cannot read follower constants: " + file, e);
        }

        readJson(json);
    }

    private void loadValues(JSONObject json) {
        JSONObject characterization = json.optJSONObject("motionCharacterization");
        motionCharacterization = characterization == null ? null
                : MotionCharacterization.fromJson(characterization, characterization.has("STRAFE"));
        angularCoeffs.p(loadDouble(json, "headingP"));
        angularCoeffs.d(loadDouble(json, "headingD"));
        angularCoeffs.s(loadDouble(json, "headingS"));

        translationalCoeffs.p(loadDouble(json, "translationalP"));
        translationalCoeffs.d(loadDouble(json, "translationalD"));
        translationalCoeffs.s(loadDouble(json, "translationalS"));

        translationalKV = loadDouble(json, "translationKV");
        translationalKA = loadDouble(json, "translationKA");
        angularKV = loadDouble(json, "angularKV");
        angularKA = loadDouble(json, "angularKA");
        angularFeedforwardKS = json.has("angularFeedforwardS")
                ? loadDouble(json, "angularFeedforwardS") : angularCoeffs.kS;
        translationalFeedforwardKS = json.has("translationalFeedforwardS")
                ? loadDouble(json, "translationalFeedforwardS") : translationalCoeffs.kS;
        velocityFeedbackGain = loadDouble(json, "velocityFeedbackGain");
        brakingVelocityFeedbackGain = json.has("brakingVelocityFeedbackGain")
                ? loadDouble(json, "brakingVelocityFeedbackGain") : velocityFeedbackGain;
        brakingOverspeedToleranceIn = json.has("brakingOverspeedToleranceIn")
                ? loadDouble(json, "brakingOverspeedToleranceIn") : 3.0;
        brakingPulsePower = json.has("brakingPulsePower")
                ? loadDouble(json, "brakingPulsePower") : 0.30;
        tankBrakingPulsePower = json.has("tankBrakingPulsePower")
                ? loadDouble(json, "tankBrakingPulsePower") : 0.10;
        brakingPulseCooldownCycles = json.has("brakingPulseCooldownCycles")
                ? json.optInt("brakingPulseCooldownCycles") : 1;
        brakingNeutralPowerMargin = json.has("brakingNeutralPowerMargin")
                ? loadDouble(json, "brakingNeutralPowerMargin") : 0.02;
        angularVelocityFeedbackGain = loadDouble(json, "angularVelocityFeedbackGain");
        kCentripetal = loadDouble(json, "kCentripetal");
        profileCornerSpeedScale = json.has("profileCornerSpeedScale")
                ? loadDouble(json, "profileCornerSpeedScale") : 1.0;
        profileVelocityPowerCoefficient = json.has("profileVelocityPowerCoefficient")
                ? loadDouble(json, "profileVelocityPowerCoefficient") : 0.0;
        brakingCornerToleranceIn = json.has("brakingCornerToleranceIn")
                ? loadDouble(json, "brakingCornerToleranceIn") : 3.0;
        tankBrakingCornerToleranceIn = json.has("tankBrakingCornerToleranceIn")
                ? loadDouble(json, "tankBrakingCornerToleranceIn") : 3.0;
        profiledTrackingTuned = json.optBoolean("profiledTrackingTuned", false);

        forwardVelLimitIn = loadDouble(json, "forwardVelLimitIn");
        forwardAccelLimitIn = loadDouble(json, "forwardAccelLimitIn");
        strafeVelLimitIn = loadDouble(json, "strafeVelLimitIn");
        strafeAccelLimitIn = loadDouble(json, "strafeAccelLimitIn");
        angularVelLimitRad = loadDouble(json, "angularVelLimitRad");
        angularAccelLimitRad = loadDouble(json, "angularAccelLimitRad");
        maxCentripetalAccelIn = loadDouble(json, "maxCentripetalAccelIn");
    }

    private JSONObject valuesToJson() {
        JSONObject json = new JSONObject();
        try {
            if (motionCharacterization != null) {
                json.put("motionCharacterization", motionCharacterization.toJson());
            }
            json.put("headingP", angularCoeffs.kP);
            json.put("headingD", angularCoeffs.kD);
            json.put("headingS", angularCoeffs.kS);
            json.put("translationalP", translationalCoeffs.kP);
            json.put("translationalD", translationalCoeffs.kD);
            json.put("translationalS", translationalCoeffs.kS);
            json.put("translationKV", translationalKV);
            json.put("translationKA", translationalKA);
            json.put("angularKV", angularKV);
            json.put("angularKA", angularKA);
            json.put("angularFeedforwardS", angularFeedforwardKS);
            json.put("translationalFeedforwardS", translationalFeedforwardKS);
            json.put("velocityFeedbackGain", velocityFeedbackGain);
            json.put("brakingVelocityFeedbackGain", brakingVelocityFeedbackGain);
            json.put("brakingOverspeedToleranceIn", brakingOverspeedToleranceIn);
            json.put("brakingPulsePower", brakingPulsePower);
            json.put("tankBrakingPulsePower", tankBrakingPulsePower);
            json.put("brakingPulseCooldownCycles", brakingPulseCooldownCycles);
            json.put("brakingNeutralPowerMargin", brakingNeutralPowerMargin);
            json.put("angularVelocityFeedbackGain", angularVelocityFeedbackGain);
            json.put("kCentripetal", kCentripetal);
            json.put("profileCornerSpeedScale", profileCornerSpeedScale);
            json.put("profileVelocityPowerCoefficient", profileVelocityPowerCoefficient);
            json.put("brakingCornerToleranceIn", brakingCornerToleranceIn);
            json.put("tankBrakingCornerToleranceIn", tankBrakingCornerToleranceIn);
            json.put("profiledTrackingTuned", profiledTrackingTuned);
            json.put("forwardVelLimitIn", forwardVelLimitIn);
            json.put("forwardAccelLimitIn", forwardAccelLimitIn);
            json.put("strafeVelLimitIn", strafeVelLimitIn);
            json.put("strafeAccelLimitIn", strafeAccelLimitIn);
            json.put("angularVelLimitRad", angularVelLimitRad);
            json.put("angularAccelLimitRad", angularAccelLimitRad);
            json.put("maxCentripetalAccelIn", maxCentripetalAccelIn);
        } catch (Exception e) {
            throw new IllegalStateException("Cannot save non-finite follower constants", e);
        }
        return json;
    }

    /**
     * Derives velocity feedback from the identified first-order plant
     * {@code power = kV * velocity + kA * acceleration}.
     *
     * <p>With proportional velocity feedback, the error time constant is
     * {@code kA / (kV + gain)}. Solving for the configured response time gives
     * {@code gain = kA / timeConstant - kV}.</p>
     */
    public void deriveVelocityFeedbackGains() {
        velocityFeedbackGain = velocityFeedbackGain(
                translationalKV, translationalKA,
                MAX_TRANSLATION_VELOCITY_FEEDBACK_GAIN);
        brakingVelocityFeedbackGain = velocityFeedbackGain;
        angularVelocityFeedbackGain = velocityFeedbackGain(
                angularKV, angularKA,
                MAX_ANGULAR_VELOCITY_FEEDBACK_GAIN);
    }

    static double velocityFeedbackGain(double kV, double kA, double maximum) {
        if (!Double.isFinite(kV) || !Double.isFinite(kA) ||
                kV < 0.0 || kA <= 0.0 || maximum <= 0.0) {
            return 0.0;
        }
        return Math.max(0.0, Math.min(maximum,
                kA / VELOCITY_FEEDBACK_TIME_CONSTANT_SECONDS - kV));
    }

    /** The inertia coefficient applies to signed acceleration, including planned braking. */
    public double getTranslationalKA(double velocity, double acceleration) {
        return translationalKA;
    }
}
