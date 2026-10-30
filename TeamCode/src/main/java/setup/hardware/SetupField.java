package setup.hardware;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/** Contextual input descriptions; devices come from the actual hardware map. */
public final class SetupField {
    public final String key, label, role;
    public final List<String> options;
    public final double increment;
    private double minimum = Double.MIN_VALUE, maximum = Double.MAX_VALUE;
    private boolean integral;

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
    private static SetupField range(String key, String label, double step, double min, double max, boolean integer) {
        SetupField f = number(key, label, step); f.minimum = min; f.maximum = max; f.integral = integer; return f;
    }

    public static List<SetupField> forProfile(RobotProfile p, SetupHardware hardware) {
        List<SetupField> fields = new ArrayList<>();
        fields.add(choice("drive", "Drive layout", "MECANUM", "TANK_2", "TANK_4", "COAXIAL_SWERVE", "DUAL_ACTUATED", "OMNI"));
        fields.add(choice("localizer", "Position sensor", "PINPOINT", "TWO_WHEEL", "THREE_WHEEL"));
        if (p.get("drive").equals("OMNI")) {
            fields.add(range("wheelCount", "Omni wheel count (uniform spacing)", 1, 3, 8, true));
            fields.add(range("firstWheelDeg", "First wheel radial angle CCW from forward (degrees)", 1, -360, 360, false));
        }
        String[] motors = p.motorKeys();
        String[] labels = {"Front left", "Front right", "Back left", "Back right"};
        for (int i = 0; i < motors.length; i++) {
            String label = p.get("drive").equals("OMNI") ? "Wheel " + (i + 1) + " (CCW order)" : labels[i];
            fields.add(device(motors[i], label + " motor", "motor", hardware));
            fields.add(choice(motors[i] + "Reverse", label + " motor direction", "false", "true"));
        }
        if (p.get("drive").equals("COAXIAL_SWERVE")) {
            for (int i = 0; i < motors.length; i++) {
                String key = motors[i];
                fields.add(device(key + "Servo", labels[i] + " steering CR servo", "crservo", hardware));
                fields.add(choice(key + "SteerReverse", labels[i] + " steering servo direction", "false", "true"));
                fields.add(device(key + "Encoder", labels[i] + " steering analog encoder", "analog", hardware));
                fields.add(choice(key + "EncoderReverse", labels[i] + " steering encoder direction", "false", "true"));
                fields.add(range(key + "ZeroDeg", labels[i] + " encoder reading with wheel forward (degrees)", 1, -360, 360, false));
            }
            fields.add(number("wheelbaseIn", "Front-to-back module spacing (inches)", .1));
            fields.add(number("trackWidthIn", "Left-to-right module spacing (inches)", .1));
            fields.add(number("steerP", "Steering proportional gain (per radian)", .01));
            fields.add(range("steerD", "Steering derivative gain", .001, 0, Double.MAX_VALUE, false));
            fields.add(range("steerS", "Steering static gain", .01, 0, 1, false));
        }
        if (p.get("drive").equals("DUAL_ACTUATED")) {
            fields.add(range("actuatorCount", "Actuation servo count", 1, 1, 8, true));
            for (int i = 1; i <= p.count("actuatorCount", 1); i++) {
                String key = "actuator" + i;
                fields.add(device(key, "Actuation servo " + i, "servo", hardware));
                fields.add(range(key + "Tank", "Servo " + i + " traction position", .01, 0, 1, false));
                fields.add(range(key + "Holonomic", "Servo " + i + " holonomic position", .01, 0, 1, false));
            }
            fields.add(choice("initialState", "Startup drive state", "HOLONOMIC", "TANK"));
            fields.add(range("transitionSeconds", "Actuation settling time (seconds)", .1, 0, 30, false));
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
        fields.add(choice("forwardReverse", p.threeWheel() ? "Left parallel pod direction" : "Forward pod direction", "false", "true"));
        if (p.threeWheel()) { fields.add(choice("rightReverse", "Right parallel pod direction", "false", "true")); }
        fields.add(choice("strafeReverse", "Sideways pod direction", "false", "true"));
        return fields;
    }

    public boolean accepts(String value) {
        if (options != null) { return options.contains(value); }
        try { double n = Double.parseDouble(value); return Double.isFinite(n) && n >= minimum && n <= maximum && (!integral || n == Math.rint(n)); }
        catch (Exception e) { return false; }
    }
}
