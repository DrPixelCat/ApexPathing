package setup;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/** Contextual input descriptions; devices come from the actual hardware map. */
public final class SetupField {
    public final String key, label, role;
    public final List<String> options;
    public final double increment;

    private SetupField(String key, String label, String role, List<String> options, double increment) {
        this.key = key; this.label = label; this.role = role; this.options = options; this.increment = increment;
    }
    private static SetupField choice(String key, String label, String... options) {
        return new SetupField(key, label, "", Arrays.asList(options), 0);
    }
    private static SetupField device(String key, String label, String role, SetupHardware hardware) {
        return new SetupField(key, label, role, hardware.devices(role), 0);
    }
    private static SetupField number(String key, String label, double increment) {
        return new SetupField(key, label, "", null, increment);
    }

    public static List<SetupField> forProfile(RobotProfile p, SetupHardware hardware) {
        List<SetupField> fields = new ArrayList<>();
        fields.add(choice("drive", "Drive layout", "MECANUM", "TANK_2", "TANK_4"));
        fields.add(choice("localizer", "Position sensor", "PINPOINT", "TWO_WHEEL", "THREE_WHEEL"));
        String[] motors = p.get("drive").equals("TANK_2") ? new String[]{"fl", "fr"} : new String[]{"fl", "fr", "bl", "br"};
        String[] labels = {"Front left", "Front right", "Back left", "Back right"};
        for (int i = 0; i < motors.length; i++) {
            fields.add(device(motors[i], labels[i] + " motor", "motor", hardware));
            fields.add(choice(motors[i] + "Reverse", labels[i] + " motor direction", "false", "true"));
        }
        if (p.pinpoint()) { fields.add(device("device", "Pinpoint device", "pinpoint", hardware)); }
        else {
            fields.add(device("forward", p.threeWheel() ? "Left parallel pod" : "Forward pod", "motor", hardware));
            if (p.threeWheel()) { fields.add(device("right", "Right parallel pod", "motor", hardware)); }
            fields.add(device("strafe", "Sideways pod", "motor", hardware));
            if (!p.threeWheel()) {
                fields.add(device("imu", "Hub IMU", "imu", hardware));
                fields.add(choice("logo", "Hub logo faces", "UP", "DOWN", "FORWARD", "BACKWARD", "LEFT", "RIGHT"));
                fields.add(choice("usb", "Hub USB faces", "FORWARD", "BACKWARD", "LEFT", "RIGHT", "UP", "DOWN"));
            }
        }
        fields.add(choice("encoderModel", "Odometry wheels", "GOBILDA_4_BAR", "GOBILDA_SWINGARM", "CUSTOM"));
        if (p.get("encoderModel").equals("CUSTOM")) { fields.add(number("ticksPerInch", "Encoder ticks per inch", 1)); }
        if (p.threeWheel()) { fields.add(number("offsetX", "Approximate parallel-pod spacing (inches)", .1)); }
        fields.add(number("distanceIn", "Marked forward test distance (inches)", 1));
        // Directions are corrected by guided hand-motion tests; no sign-convention knowledge needed.
        return fields;
    }

    public boolean accepts(String value) {
        if (options != null) { return options.contains(value); }
        try { return Double.isFinite(Double.parseDouble(value)) && Double.parseDouble(value) > 0; }
        catch (Exception e) { return false; }
    }
}
