package setup.hardware;

import org.json.JSONObject;
import java.util.LinkedHashMap;
import java.util.Map;

/** Portable operator-owned robot settings. Drafts are separate from the saved profile. */
public final class RobotProfile {
    public static final int VERSION = 2;
    public final Map<String, String> values = new LinkedHashMap<>();
    public JSONObject filter = new JSONObject();
    public boolean sensorsChecked, geometryMeasured, filterMeasured;

    public RobotProfile() {
        set("drive", "MECANUM"); set("localizer", "PINPOINT");
        set("fl", "frontLeftMotor"); set("fr", "frontRightMotor");
        set("bl", "backLeftMotor"); set("br", "backRightMotor");
        set("flReverse", "false"); set("frReverse", "true");
        set("blReverse", "false"); set("brReverse", "true");
        set("device", "pinpoint"); set("forward", ""); set("right", ""); set("strafe", "");
        set("imu", "imu"); set("logo", "UP"); set("usb", "FORWARD");
        set("forwardReverse", "false"); set("rightReverse", "false"); set("strafeReverse", "false");
        set("ticksPerInch", "505.38314138"); set("offsetX", "0"); set("offsetY", "0");
        set("encoderModel", "GOBILDA_4_BAR");
        set("distanceIn", "24");
        set("wheelCount", "3"); set("firstWheelDeg", "0");
        set("wheelbaseIn", "14"); set("trackWidthIn", "14");
        set("steerP", "0"); set("steerD", "0"); set("steerS", "0");
        set("actuatorCount", "1"); set("initialState", "HOLONOMIC"); set("transitionSeconds", "0.5");
        for (String corner : new String[]{"fl", "fr", "bl", "br"}) {
            set(corner + "Servo", ""); set(corner + "Encoder", "");
            set(corner + "SteerReverse", "false"); set(corner + "EncoderReverse", "false");
            set(corner + "ZeroDeg", "0");
        }
        for (int i = 1; i <= 8; i++) {
            set("wheel" + i, ""); set("wheel" + i + "Reverse", "false");
            set("actuator" + i, ""); set("actuator" + i + "Tank", "0"); set("actuator" + i + "Holonomic", "1");
        }
    }

    public String get(String key) { return values.getOrDefault(key, ""); }
    public void set(String key, String value) { values.put(key, value); }
    public double number(String key) { return Double.parseDouble(get(key)); }
    public boolean flag(String key) { return Boolean.parseBoolean(get(key)); }
    public boolean threeWheel() { return get("localizer").equals("THREE_WHEEL"); }
    public boolean pinpoint() { return get("localizer").equals("PINPOINT"); }
    public boolean holonomic() { return !get("drive").startsWith("TANK") &&
            !(get("drive").equals("DUAL_ACTUATED") && get("initialState").equals("TANK")); }
    public int count(String key, int minimum) {
        double n = number(key);
        if (!Double.isFinite(n) || n != Math.rint(n) || n < minimum || n > 8) {
            throw new IllegalArgumentException("Choose " + key + " between " + minimum + " and 8");
        }
        return (int) n;
    }
    public String[] motorKeys() {
        if (get("drive").equals("OMNI")) {
            String[] keys = new String[count("wheelCount", 3)];
            for (int i = 0; i < keys.length; i++) { keys[i] = "wheel" + (i + 1); }
            return keys;
        }
        return get("drive").equals("TANK_2") ? new String[]{"fl", "fr"} : new String[]{"fl", "fr", "bl", "br"};
    }
    public int channels() { return threeWheel() ? 3 : 2; }
    public int strafeChannel() { return threeWheel() ? 2 : 1; }

    public void geometryChanged() { filter = new JSONObject(); geometryMeasured = false; filterMeasured = false; }

    public void validate() {
        if (!java.util.Arrays.asList("MECANUM", "TANK_2", "TANK_4", "COAXIAL_SWERVE", "DUAL_ACTUATED", "OMNI").contains(get("drive"))) {
            throw new IllegalArgumentException("Choose a supported drive layout");
        }
        if (!java.util.Arrays.asList("PINPOINT", "TWO_WHEEL", "THREE_WHEEL").contains(get("localizer"))) {
            throw new IllegalArgumentException("Choose a supported localizer");
        }
        unique(motorKeys(), "drive motors");
        if (get("drive").equals("OMNI")) { finite("firstWheelDeg"); }
        if (get("drive").equals("COAXIAL_SWERVE")) {
            positive("wheelbaseIn"); positive("trackWidthIn"); positive("steerP"); nonnegative("steerD"); nonnegative("steerS");
            unique(new String[]{"flServo", "frServo", "blServo", "brServo"}, "steering servos");
            unique(new String[]{"flEncoder", "frEncoder", "blEncoder", "brEncoder"}, "steering encoders");
            for (String corner : motorKeys()) {
                finite(corner + "ZeroDeg"); direction(corner + "SteerReverse"); direction(corner + "EncoderReverse");
            }
        }
        if (get("drive").equals("DUAL_ACTUATED")) {
            int count = count("actuatorCount", 1);
            String[] names = new String[count];
            for (int i = 1; i <= count; i++) {
                names[i - 1] = "actuator" + i;
                for (String state : new String[]{"Tank", "Holonomic"}) {
                    String key = "actuator" + i + state; finite(key);
                    if (number(key) < 0 || number(key) > 1) { throw new IllegalArgumentException("Servo positions must be between 0 and 1"); }
                }
            }
            unique(names, "actuation servos"); nonnegative("transitionSeconds");
            if (!java.util.Arrays.asList("TANK", "HOLONOMIC").contains(get("initialState"))) { throw new IllegalArgumentException("Choose the startup drive state"); }
        }
        if (pinpoint()) { requireName("device"); }
        else {
            unique(threeWheel() ? new String[]{"forward", "right", "strafe"} : new String[]{"forward", "strafe"}, "pod encoders");
            if (!threeWheel()) {
                requireName("imu");
                String logoAxis = axis(get("logo")), usbAxis = axis(get("usb"));
                if (logoAxis.equals(usbAxis)) { throw new IllegalArgumentException("Hub logo and USB must face perpendicular directions"); }
            }
        }
        positive("ticksPerInch"); positive("distanceIn");
        if (!java.util.Arrays.asList("GOBILDA_4_BAR", "GOBILDA_SWINGARM", "CUSTOM").contains(get("encoderModel"))) {
            throw new IllegalArgumentException("Choose the odometry wheel type");
        }
        if (filterMeasured && filter.optJSONObject("velocity") == null) {
            throw new IllegalArgumentException("Saved filter settings are incomplete");
        }
        if (filter.has("velocity")) {
            try {
                JSONObject velocity = filter.getJSONObject("velocity");
                for (String axis : new String[]{"x", "y", "heading"}) {
                    for (String noise : new String[]{"R", "Q"}) {
                        double value = velocity.getJSONObject(axis).getDouble(noise);
                        if (!Double.isFinite(value) || value <= 0) { throw new IllegalArgumentException("Invalid filter variance"); }
                    }
                }
            } catch (Exception e) { throw new IllegalArgumentException("Saved filter settings are incomplete or invalid", e); }
        }
        finite("offsetX"); finite("offsetY");
        if (threeWheel() && number("offsetX") <= 0) {
            throw new IllegalArgumentException("Enter the approximate distance between the parallel pods");
        }
        for (String key : new String[]{"flReverse", "frReverse", "blReverse", "brReverse",
                "forwardReverse", "rightReverse", "strafeReverse"}) {
            direction(key);
        }
        for (String key : motorKeys()) { direction(key + "Reverse"); }
    }

    private void direction(String key) {
        if (!get(key).equals("true") && !get(key).equals("false")) { throw new IllegalArgumentException("Invalid direction setting"); }
    }
    private void nonnegative(String key) {
        finite(key);
        if (number(key) < 0) { throw new IllegalArgumentException("Enter a nonnegative " + key); }
    }

    private void requireName(String key) {
        if (get(key).trim().isEmpty()) { throw new IllegalArgumentException("Choose " + key + " from the configured devices"); }
    }
    private void unique(String[] keys, String description) {
        java.util.Set<String> names = new java.util.HashSet<>();
        for (String key : keys) {
            requireName(key);
            if (!names.add(get(key))) { throw new IllegalArgumentException("Assign different devices to the " + description); }
        }
    }
    private static String axis(String direction) {
        switch (direction) {
            case "UP": case "DOWN": return "z";
            case "LEFT": case "RIGHT": return "y";
            case "FORWARD": case "BACKWARD": return "x";
            default: throw new IllegalArgumentException("Choose the hub mounting directions");
        }
    }
    private void finite(String key) {
        if (!Double.isFinite(number(key))) { throw new IllegalArgumentException("Invalid " + key); }
    }
    private void positive(String key) {
        finite(key);
        if (number(key) <= 0) { throw new IllegalArgumentException("Enter a positive " + key); }
    }

    public JSONObject toJson() {
        try {
            JSONObject fields = new JSONObject();
            for (Map.Entry<String, String> value : values.entrySet()) { fields.put(value.getKey(), value.getValue()); }
            return new JSONObject().put("version", VERSION).put("settings", fields).put("filter", filter)
                    .put("sensorsChecked", sensorsChecked).put("geometryMeasured", geometryMeasured).put("filterMeasured", filterMeasured);
        } catch (Exception e) { throw new IllegalStateException("Cannot serialize robot setup", e); }
    }

    public static RobotProfile fromJson(JSONObject json) {
        try {
            int version = json.getInt("version");
            if (version != 1 && version != VERSION) { throw new IllegalArgumentException("Unsupported robot setup version"); }
            RobotProfile profile = new RobotProfile();
            JSONObject fields = json.getJSONObject("settings");
            // New layout fields are optional for existing v1 profiles; active fields are validated below.
            for (String key : profile.values.keySet()) {
                if (fields.has(key)) { profile.set(key, fields.getString(key)); }
                else if (version == VERSION || LEGACY_KEYS.contains(key)) { throw new IllegalArgumentException("Missing " + key); }
            }
            JSONObject filter = json.optJSONObject("filter");
            profile.filter = filter == null ? new JSONObject() : new JSONObject(filter.toString());
            profile.sensorsChecked = json.optBoolean("sensorsChecked");
            profile.geometryMeasured = json.optBoolean("geometryMeasured");
            profile.filterMeasured = json.optBoolean("filterMeasured");
            profile.validate();
            return profile;
        } catch (Exception e) { throw new IllegalArgumentException("Robot setup is invalid. Run Robot Setup again.", e); }
    }

    private static final java.util.Set<String> LEGACY_KEYS = new java.util.HashSet<>(java.util.Arrays.asList(
            "drive", "localizer", "fl", "fr", "bl", "br", "flReverse", "frReverse", "blReverse", "brReverse",
            "device", "forward", "right", "strafe", "imu", "logo", "usb", "forwardReverse", "rightReverse",
            "strafeReverse", "ticksPerInch", "offsetX", "offsetY", "encoderModel", "distanceIn"));

    public RobotProfile copy() {
        RobotProfile result = new RobotProfile();
        result.values.clear(); result.values.putAll(values);
        try { result.filter = new JSONObject(filter.toString()); }
        catch (Exception e) { throw new IllegalStateException(e); }
        result.sensorsChecked = sensorsChecked; result.geometryMeasured = geometryMeasured; result.filterMeasured = filterMeasured;
        return result;
    }
}
