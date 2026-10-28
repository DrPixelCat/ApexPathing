package setup;

import org.json.JSONObject;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;
import java.io.File;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.Arrays;
import java.util.List;
import static org.junit.Assert.*;
import static setup.RobotSetupWizard.Action.CONFIRM;
import static setup.RobotSetupWizard.Action.DOWN;
import static setup.RobotSetupWizard.Screen.*;
import static setup.RobotSetupWizard.Task.*;

/** Executes the production state machine, persistence, and CSV capture with physical pod geometry. */
public class RobotSetupWorkflowTest {
    @Rule public TemporaryFolder temp = new TemporaryFolder();

    static class Hardware implements SetupHardware {
        RobotProfile profile;
        boolean learning, connected = true;
        int builds, stops;
        double heading;
        double[] ticks = new double[3];
        double[] polarity = {1, 1, 1};
        @Override public List<String> devices(String role) {
            return role.equals("pinpoint") ? Arrays.asList("pinpoint") : role.equals("imu") ? Arrays.asList("imu")
                    : Arrays.asList("frontLeftMotor", "frontRightMotor", "backLeftMotor", "backRightMotor", "forward", "right", "strafe");
        }
        @Override public void prepare(RobotProfile p) { profile = p.copy(); builds++; ticks = new double[3]; heading = 0; learning = false; }
        @Override public SetupSnapshot update() {
            SetupSnapshot result = new SetupSnapshot(); result.valid = connected;
            result.pods = new double[profile.channels()]; result.ticks = new int[result.pods.length];
            String[] reverse = profile.threeWheel() ? new String[]{"forwardReverse", "rightReverse", "strafeReverse"}
                    : new String[]{"forwardReverse", "strafeReverse"};
            for (int i = 0; i < result.pods.length; i++) {
                result.ticks[i] = (int) Math.round(ticks[i] * polarity[i]);
                result.pods[i] = result.ticks[i] * (profile.flag(reverse[i]) ? -1 : 1) / profile.number("ticksPerInch");
            }
            result.rawPose[2] = Math.atan2(Math.sin(heading), Math.cos(heading));
            result.pose[2] = result.rawPose[2];
            result.noise = new double[]{.15, 42, .25, 20, .01, 7};
            result.rawVelocity[0] = 7; result.velocity[0] = 6.5;
            result.diagnostics.put("adapting", learning ? 1.0 : 0.0);
            return result;
        }
        void motion(double forward, double left, double turn) {
            // Real resolution=500 ticks/in. Calibration must recover it from the nominal default.
            if (profile.threeWheel()) {
                ticks[0] += (forward + 5 * turn) * 500; ticks[1] += (forward - 5 * turn) * 500;
                ticks[2] += (left - 3 * turn) * 500;
            } else {
                ticks[0] += (forward + (profile.pinpoint() ? -2 : 2) * turn) * 500;
                ticks[1] += (left - 3 * turn) * 500;
            }
            heading += turn;
        }
        @Override public void learn(boolean enabled) { learning = enabled; }
        @Override public JSONObject captureFilter() {
            try {
                return new JSONObject().put("velocity", new JSONObject()
                        .put("x", new JSONObject().put("R", .15).put("Q", 42))
                        .put("y", new JSONObject().put("R", .25).put("Q", 20))
                        .put("heading", new JSONObject().put("R", .01).put("Q", 7)));
            } catch (Exception e) { throw new AssertionError(e); }
        }
        @Override public void stop() { stops++; }
    }

    class Session implements AutoCloseable {
        final Hardware hardware = new Hardware();
        final SetupStore store;
        final SetupLog log;
        final RobotSetupWizard wizard;
        double time;
        Session(File root, RobotProfile existing) throws Exception {
            store = new SetupStore(root);
            if (existing != null) { store.save(existing); }
            log = new SetupLog(new File(root, "logs"));
            wizard = new RobotSetupWizard(hardware, store, log);
        }
        void tick(double seconds) { time += seconds; wizard.tick(time); }
        void confirm() { wizard.act(CONFIRM); }
        void ready() { tick(3.1); assertEquals(READY, wizard.getScreen()); }
        void begin() {
            confirm();
            int guard = 0;
            while (wizard.getScreen() == CONFIGURE && guard++ < 40) { confirm(); }
            assertEquals(WARMUP, wizard.getScreen()); ready();
        }
        void forward() { assertEquals(FORWARD, wizard.getTask()); confirm(); hardware.motion(24, 0, 0); tick(.02); confirm(); assertEquals(REVIEW, wizard.getScreen()); }
        void left() { assertEquals(LEFT, wizard.getTask()); confirm(); hardware.motion(0, 6, 0); tick(.02); confirm(); assertEquals(REVIEW, wizard.getScreen()); }
        void turn(double radians) {
            confirm();
            for (int i = 0; i < 24; i++) { hardware.motion(0, 0, radians / 24); tick(.02); }
            confirm(); assertEquals(REVIEW, wizard.getScreen());
        }
        void geometry() {
            forward(); confirm(); ready(); left(); confirm(); ready();
            assertEquals(TURN, wizard.getTask()); turn(Math.PI / 2); confirm(); ready();
            assertEquals(SPIN_CCW, wizard.getTask()); turn(2 * Math.PI); confirm(); ready();
            assertEquals(SPIN_CW, wizard.getTask()); turn(-2 * Math.PI); confirm(); ready();
            assertEquals(FILTER, wizard.getTask());
        }
        void filter() {
            confirm(); assertTrue(hardware.learning); assertEquals(FILTER_REST, wizard.getScreen());
            tick(3.1); assertEquals(FILTER_MOVE, wizard.getScreen());
            confirm(); assertEquals("Short capture must not be accepted", FILTER_MOVE, wizard.getScreen());
            hardware.motion(12, 0, .5); tick(10.1); confirm();
            assertFalse(hardware.learning); assertEquals(REVIEW, wizard.getScreen());
        }
        @Override public void close() { wizard.close(); }
    }

    @Test public void firstSetupMeasuresPersistsAndLogsWithoutDisplayingResults() throws Exception {
        File root = temp.newFolder();
        File logs;
        try (Session s = new Session(root, null)) {
            s.begin(); s.geometry(); s.filter();
            String review = String.join("\n", s.wizard.lines());
            assertFalse(review.contains("42")); assertFalse(review.contains("0.15")); assertFalse(review.contains("Q="));
            s.confirm(); assertEquals(SAVE, s.wizard.getScreen());
            assertFalse("Draft measurements are not saved early", s.store.getFile().exists());
            s.confirm(); assertEquals(COMPLETE, s.wizard.getScreen());
            RobotProfile saved = s.store.load();
            assertEquals(500, saved.number("ticksPerInch"), .01);
            assertEquals(2, saved.number("offsetX"), .002); assertEquals(-3, saved.number("offsetY"), .002);
            assertEquals(42, saved.filter.getJSONObject("velocity").getJSONObject("x").getDouble("Q"), 0);
            assertTrue(saved.sensorsChecked); assertTrue(saved.geometryMeasured); assertTrue(saved.filterMeasured);
            assertTrue(s.hardware.stops > 20);
            logs = s.log.getDirectory();
        }
        String motion = read(new File(logs, "motion.csv"));
        assertTrue(motion.contains("raw_vx_in_s")); assertTrue(motion.contains("filtered") || motion.contains("vx_in_s"));
        assertTrue(read(new File(logs, "kalman.csv")).contains("42.0"));
        assertTrue(read(new File(logs, "profiles.csv")).contains("filter_candidate"));
        assertTrue(read(new File(logs, "events.csv")).contains("COMPLETE"));
        assertTrue(read(new File(logs, "measurements.csv")).contains("measured_offset_x"));
    }

    @Test public void usesAnExternalTurnForThreeWheelGeometryAndPreservesTwoWheelSigns() throws Exception {
        for (String type : new String[]{"TWO_WHEEL", "THREE_WHEEL"}) {
            RobotProfile profile = profile(type);
            try (Session s = new Session(temp.newFolder(), profile)) {
                s.begin(); s.geometry();
                assertEquals(type.equals("THREE_WHEEL") ? 10 : 2, s.wizard.getDraft().number("offsetX"), .003);
                assertEquals(-3, s.wizard.getDraft().number("offsetY"), .003);
            }
        }
    }

    @Test public void sensorCorrectionRequiresAnotherCheckBeforeProceeding() throws Exception {
        try (Session s = new Session(temp.newFolder(), null)) {
            s.hardware.polarity[0] = -1;
            s.begin(); s.forward();
            assertTrue(String.join("\n", s.wizard.lines()).contains("Correct direction and repeat"));
            s.confirm(); s.ready();
            assertEquals(FORWARD, s.wizard.getTask()); assertTrue(s.wizard.getDraft().flag("forwardReverse"));
            s.forward(); s.confirm(); s.ready(); assertEquals(LEFT, s.wizard.getTask());
            assertFalse(s.store.getFile().exists());
        }
    }

    @Test public void repeatAndKeepPreviousDoNotApplyMeasurementCandidates() throws Exception {
        RobotProfile existing = profile("PINPOINT");
        try (Session s = new Session(temp.newFolder(), existing)) {
            s.begin(); s.forward(); s.wizard.act(DOWN); s.confirm();
            assertEquals(READY, s.wizard.getScreen()); assertEquals(existing.number("ticksPerInch"), s.wizard.getDraft().number("ticksPerInch"), 0);
            s.forward(); s.wizard.act(DOWN); s.wizard.act(DOWN); s.confirm(); s.ready();
            assertEquals(LEFT, s.wizard.getTask()); assertEquals(existing.number("ticksPerInch"), s.wizard.getDraft().number("ticksPerInch"), 0);
        }
    }

    @Test public void interruptingFilterCollectionRestoresLearningAndKeepsSavedProfile() throws Exception {
        File root = temp.newFolder();
        byte[] previous;
        Session s = new Session(root, profile("PINPOINT"));
        s.begin(); previous = Files.readAllBytes(s.store.getFile().toPath()); s.geometry();
        s.confirm(); s.tick(3.1); assertTrue(s.hardware.learning);
        s.close(); assertFalse(s.hardware.learning);
        assertArrayEquals(previous, Files.readAllBytes(s.store.getFile().toPath()));
        assertTrue(read(new File(s.log.getDirectory(), "events.csv")).contains("session_aborted"));
        assertTrue(read(new File(s.log.getDirectory(), "motion.csv")).contains("FILTER_REST"));
    }

    @Test public void invalidPacketsAndNoMotionCannotBecomeAcceptedMeasurements() throws Exception {
        try (Session s = new Session(temp.newFolder(), null)) {
            s.begin(); s.confirm(); s.tick(.02); s.confirm();
            assertEquals(READY, s.wizard.getScreen()); assertEquals(FORWARD, s.wizard.getTask());
            s.confirm(); s.hardware.connected = false; s.tick(.02);
            assertEquals(READY, s.wizard.getScreen());
            s.confirm(); assertEquals(READY, s.wizard.getScreen());
            assertFalse(s.store.getFile().exists());
        }
    }

    @Test public void fieldsOnlyAskForDevicesUsedByTheChosenRobot() {
        Hardware hardware = new Hardware();
        RobotProfile p = profile("PINPOINT");
        assertTrue(SetupField.forProfile(p, hardware).stream().anyMatch(f -> f.key.equals("device")));
        assertFalse(SetupField.forProfile(p, hardware).stream().anyMatch(f -> f.key.equals("imu")));
        p.set("drive", "TANK_2"); p.set("localizer", "TWO_WHEEL");
        assertFalse(SetupField.forProfile(p, hardware).stream().anyMatch(f -> f.key.equals("bl")));
        assertTrue(SetupField.forProfile(p, hardware).stream().anyMatch(f -> f.key.equals("logo")));
        p.set("localizer", "THREE_WHEEL");
        assertFalse(SetupField.forProfile(p, hardware).stream().anyMatch(f -> f.key.equals("imu")));
        assertTrue(SetupField.forProfile(p, hardware).stream().anyMatch(f -> f.key.equals("right")));
    }

    @Test public void corruptSavedSetupCanBeRepairedThroughTheWizardWithoutDeletingFiles() throws Exception {
        File root = temp.newFolder();
        SetupStore store = new SetupStore(root);
        Files.write(store.getFile().toPath(), "{broken".getBytes(StandardCharsets.UTF_8));
        try (Session s = new Session(root, null)) {
            assertTrue(String.join("\n", s.wizard.lines()).contains("couldn't be read"));
            s.confirm(); assertEquals(CONFIGURE, s.wizard.getScreen());
        }
        assertEquals("{broken", read(store.getFile()));
    }

    static RobotProfile profile(String type) {
        RobotProfile p = new RobotProfile(); p.set("localizer", type);
        p.set("forward", "forward"); p.set("right", "right"); p.set("strafe", "strafe");
        if (type.equals("THREE_WHEEL")) { p.set("offsetX", "12"); }
        return p;
    }
    static String read(File file) throws Exception { return new String(Files.readAllBytes(file.toPath()), StandardCharsets.UTF_8); }
}
