package setup;

import org.json.JSONObject;
import java.util.LinkedHashMap;
import java.util.Map;

/** Portable operator-owned robot settings. Drafts are separate from the saved profile. */
public final class RobotProfile {
    public static final int VERSION = 1;
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
    }

    public String get(String key) { return values.getOrDefault(key, ""); }
    public void set(String key, String value) { values.put(key, value); }
    public double number(String key) { return Double.parseDouble(get(key)); }
    public boolean flag(String key) { return Boolean.parseBoolean(get(key)); }
    public boolean threeWheel() { return get("localizer").equals("THREE_WHEEL"); }
    public boolean pinpoint() { return get("localizer").equals("PINPOINT"); }
    public boolean holonomic() { return get("drive").equals("MECANUM"); }
    public int channels() { return threeWheel() ? 3 : 2; }
    public int strafeChannel() { return threeWheel() ? 2 : 1; }

    public void geometryChanged() { filter = new JSONObject(); geometryMeasured = false; filterMeasured = false; }

    public void validate() {
        if (!java.util.Arrays.asList("MECANUM", "TANK_2", "TANK_4").contains(get("drive"))) {
            throw new IllegalArgumentException("Choose a supported drive layout");
        }
        if (!java.util.Arrays.asList("PINPOINT", "TWO_WHEEL", "THREE_WHEEL").contains(get("localizer"))) {
            throw new IllegalArgumentException("Choose a supported localizer");
        }
        unique(get("drive").equals("TANK_2") ? new String[]{"fl", "fr"} : new String[]{"fl", "fr", "bl", "br"}, "drive motors");
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
            if (!get(key).equals("true") && !get(key).equals("false")) { throw new IllegalArgumentException("Invalid direction setting"); }
        }
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
            if (json.getInt("version") != VERSION) { throw new IllegalArgumentException("Unsupported robot setup version"); }
            RobotProfile profile = new RobotProfile();
            JSONObject fields = json.getJSONObject("settings");
            // Require every setting: incomplete/corrupt files must not silently become defaults.
            for (String key : profile.values.keySet()) { profile.set(key, fields.getString(key)); }
            JSONObject filter = json.optJSONObject("filter");
            profile.filter = filter == null ? new JSONObject() : new JSONObject(filter.toString());
            profile.sensorsChecked = json.optBoolean("sensorsChecked");
            profile.geometryMeasured = json.optBoolean("geometryMeasured");
            profile.filterMeasured = json.optBoolean("filterMeasured");
            profile.validate();
            return profile;
        } catch (Exception e) { throw new IllegalArgumentException("Robot setup is invalid. Run Robot Setup again.", e); }
    }

    public RobotProfile copy() {
        RobotProfile result = new RobotProfile();
        result.values.clear(); result.values.putAll(values);
        try { result.filter = new JSONObject(filter.toString()); }
        catch (Exception e) { throw new IllegalStateException(e); }
        result.sensorsChecked = sensorsChecked; result.geometryMeasured = geometryMeasured; result.filterMeasured = filterMeasured;
        return result;
    }
}
