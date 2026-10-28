package setup;

import java.io.File;
import java.io.IOException;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.UUID;
import tuning.logging.CsvLog;

/** Background capture for every setup screen, decision and hardware sample. */
public final class SetupLog implements AutoCloseable {
    private final File directory;
    private final Map<String, CsvLog> tables = new LinkedHashMap<>();
    private long sequence;
    private double previousTime = Double.NaN;

    public SetupLog(File root) {
        directory = new File(root, "robot-setup-" + System.currentTimeMillis() + "-" + UUID.randomUUID().toString().substring(0, 8));
        if (!directory.mkdirs()) { throw new IllegalStateException("Could not create setup log folder"); }
    }
    public File getDirectory() { return directory; }
    private void row(String table, String[] headers, Object... values) {
        if (!tables.containsKey(table)) {
            try { tables.put(table, new CsvLog(new File(directory, table + ".csv"), headers)); }
            catch (IOException e) { throw new IllegalStateException("Could not open setup CSV", e); }
        }
        tables.get(table).row(values);
    }
    public void event(double time, String step, String event, String detail) {
        row("events", new String[]{"time_s", "step", "event", "detail"}, time, step, event, detail);
        flush();
    }
    public void profile(double time, String stage, RobotProfile profile) {
        row("profiles", new String[]{"time_s", "stage", "profile_json"}, time, stage, profile.toJson().toString());
        flush();
    }
    public void measurement(double time, String step, String name, double value, String unit) {
        row("measurements", new String[]{"time_s", "step", "measurement", "value", "unit"}, time, step, name, value, unit);
    }
    public void motorTest(double time, String role, String name, boolean reversed, boolean running, MotorTestReading reading) {
        row("motor_tests", new String[]{"time_s", "role", "device", "reversed", "state", "command_power",
                        "reported_power", "encoder_ticks", "velocity_ticks_s", "battery_v"},
                time, role, name, reversed, running ? "held" : "stopped", running ? 0.5 : 0,
                reading == null ? null : reading.power, reading == null ? null : reading.ticks,
                reading == null ? null : reading.velocityTicksPerSecond, reading == null ? null : reading.batteryVolts);
    }
    public void sample(double time, String step, SetupSnapshot s) {
        long id = sequence++;
        row("motion", new String[]{"sample", "time_s", "loop_dt_s", "step", "valid", "battery_v",
                        "x_in", "y_in", "heading_rad", "raw_x_in", "raw_y_in", "raw_heading_rad",
                        "vx_in_s", "vy_in_s", "omega_rad_s", "raw_vx_in_s", "raw_vy_in_s", "raw_omega_rad_s",
                        "ax_in_s2", "ay_in_s2", "alpha_rad_s2", "raw_ax_in_s2", "raw_ay_in_s2", "raw_alpha_rad_s2"},
                id, time, time - previousTime, step, s.valid, s.battery,
                s.pose[0], s.pose[1], s.pose[2], s.rawPose[0], s.rawPose[1], s.rawPose[2],
                s.velocity[0], s.velocity[1], s.velocity[2], s.rawVelocity[0], s.rawVelocity[1], s.rawVelocity[2],
                s.acceleration[0], s.acceleration[1], s.acceleration[2], s.rawAcceleration[0], s.rawAcceleration[1], s.rawAcceleration[2]);
        previousTime = time;
        for (int i = 0; i < s.pods.length; i++) {
            row("pods", new String[]{"sample", "time_s", "step", "channel", "travel_in", "raw_ticks"},
                    id, time, step, i, s.pods[i], s.ticks.length > i ? s.ticks[i] : null);
        }
        for (int axis = 0; axis < 3; axis++) {
            row("kalman", new String[]{"sample", "time_s", "step", "axis", "R", "Q"},
                    id, time, step, axis, s.noise[axis * 2], s.noise[axis * 2 + 1]);
        }
        for (Map.Entry<String, Double> diagnostic : s.diagnostics.entrySet()) {
            row("diagnostics", new String[]{"sample", "time_s", "step", "name", "value"},
                    id, time, step, diagnostic.getKey(), diagnostic.getValue());
        }
    }
    public void flush() { for (CsvLog table : tables.values()) { table.flush(); } }
    @Override public void close() {
        IOException failure = null;
        for (CsvLog table : tables.values()) { try { table.close(); } catch (IOException e) { failure = e; } }
        if (failure != null) { throw new IllegalStateException("Could not finish setup logs", failure); }
    }
}
