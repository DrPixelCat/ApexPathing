package setup.hardware;

import static org.firstinspires.ftc.teamcode.apexpathing.RobotSetup.handleSetupAction;

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
import static setup.hardware.RobotSetupWizard.Action.CONFIRM;
import static setup.hardware.RobotSetupWizard.Action.DOWN;
import static setup.hardware.RobotSetupWizard.Screen.*;

/** Executes the production state machine, persistence, and CSV capture without localization collection. */
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
        final Hardware hardware = new Hardware() {
            @Override public void prepare(RobotProfile p) { throw new AssertionError("Setup must not initialize localization"); }
            @Override public SetupSnapshot update() { throw new AssertionError("Setup must not sample localization"); }
            @Override public void learn(boolean enabled) {
                assertFalse("Setup must not collect filter data", enabled);
                super.learn(enabled);
            }
        };
        final SetupStore store;
        final SetupLog log;
        final RobotSetupWizard wizard;
        Session(File root, RobotProfile existing) throws Exception {
            store = new SetupStore(root);
            if (existing != null) { store.save(existing); }
            log = new SetupLog(new File(root, "logs"));
            wizard = new RobotSetupWizard(hardware, store, log);
        }
        void confirm() { wizard.tick(1); handleSetupAction(wizard, CONFIRM); }
        void begin() {
            confirm();
            int guard = 0;
            while (wizard.getScreen() == CONFIGURE && guard++ < 60) { confirm(); }
            assertEquals(SAVE, wizard.getScreen());
        }
        @Override public void close() { wizard.close(); }
    }

    @Test public void allEncoderLayoutsSaveWithoutLocalizationMeasurements() throws Exception {
        for (String type : new String[]{"PINPOINT", "TWO_WHEEL", "THREE_WHEEL"}) {
            RobotProfile p = profile(type);
            p.set("forwardReverse", "true"); p.set("strafeReverse", "true");
            try (Session s = new Session(temp.newFolder(), p)) {
                handleSetupAction(s.wizard, DOWN); // Edit existing settings through the full picker.
                s.begin();
                assertEquals(0, s.hardware.builds);
                s.confirm();
                assertEquals(COMPLETE, s.wizard.getScreen());
                RobotProfile saved = s.store.load();
                assertEquals(p.number("ticksPerInch"), saved.number("ticksPerInch"), 0);
                assertTrue(saved.flag("forwardReverse")); assertTrue(saved.flag("strafeReverse"));
                assertFalse(saved.sensorsChecked); assertFalse(saved.geometryMeasured); assertFalse(saved.filterMeasured);
            }
        }
    }

    @Test public void firstSetupRequiresExplicitSaveAndStopKeepsDraftUnsaved() throws Exception {
        File root = temp.newFolder(); File logs;
        try (Session s = new Session(root, null)) {
            s.begin();
            assertFalse(s.store.getFile().exists());
            assertTrue(String.join("\n", s.wizard.lines()).contains("Localization Tuner"));
            logs = s.log.getDirectory();
        }
        assertFalse(new SetupStore(root).getFile().exists());
        assertTrue(read(new File(logs, "events.csv")).contains("session_aborted"));
        assertFalse(read(new File(logs, "profiles.csv")).contains("candidate"));
    }

    @Test public void resavingHardwarePreservesExistingLocalizationCalibration() throws Exception {
        RobotProfile p = profile("PINPOINT");
        p.set("offsetX", "2.5"); p.set("offsetY", "-3");
        p.filter = new Hardware().captureFilter();
        p.sensorsChecked = true; p.geometryMeasured = true; p.filterMeasured = true;
        try (Session s = new Session(temp.newFolder(), p)) {
            s.begin(); s.confirm();
            RobotProfile saved = s.store.load();
            assertEquals(p.toJson().toString(), saved.toJson().toString());
        }
    }

    @Test public void failedSaveRequiresStopAndPreservesTheSavedProfile() throws Exception {
        try (Session s = new Session(temp.newFolder(), profile("PINPOINT"))) {
            byte[] saved = Files.readAllBytes(s.store.getFile().toPath());
            s.begin();
            assertTrue(new File(s.store.getFile().getPath() + ".tmp").mkdir());
            s.confirm();
            assertEquals(ERROR, s.wizard.getScreen());
            assertTrue(String.join("\n", s.wizard.lines()).contains(setup.WorkflowText.STOP_HINT));
            s.confirm(); assertEquals(ERROR, s.wizard.getScreen());
            assertArrayEquals(saved, Files.readAllBytes(s.store.getFile().toPath()));
        }
    }

    @Test public void viewingAScreenDoesNotChangeHardwareDraftOrStorage() throws Exception {
        try (Session s = new Session(temp.newFolder(), null)) {
            for (int i = 0; i < 2; i++) {
                int stops = s.hardware.stops;
                String draft = s.wizard.getDraft().toJson().toString();
                assertEquals(s.wizard.lines(), s.wizard.lines());
                assertEquals(stops, s.hardware.stops);
                assertEquals(draft, s.wizard.getDraft().toJson().toString());
                assertFalse(s.store.getFile().exists());
                if (i == 0) { s.begin(); }
            }
        }
    }

    @Test public void closeReleasesLearningAndClosesLogsEvenIfHardwareStopFails() throws Exception {
        boolean[] failStop = {false};
        Hardware hardware = new Hardware() {
            @Override public void stop() {
                super.stop();
                if (failStop[0]) { throw new IllegalStateException("Disconnected during STOP"); }
            }
        };
        File root = temp.newFolder();
        SetupStore store = new SetupStore(root);
        SetupLog log = new SetupLog(new File(root, "logs"));
        RobotSetupWizard wizard = new RobotSetupWizard(hardware, store, log);
        hardware.learn(true); failStop[0] = true;
        assertThrows(IllegalStateException.class, wizard::close);
        assertFalse(hardware.learning);
        assertThrows(IllegalStateException.class, () -> log.event(1, "closed", "unexpected", ""));
        int stops = hardware.stops;
        wizard.close(); wizard.tick(10); handleSetupAction(wizard, CONFIRM);
        assertEquals(stops, hardware.stops);
        assertEquals(WELCOME, wizard.getScreen());
        assertFalse(store.getFile().exists());
    }

    @Test public void pickerIncludesEncoderDirectionsButNoCalibrationMeasurements() {
        Hardware hardware = new Hardware();
        for (String type : new String[]{"PINPOINT", "TWO_WHEEL", "THREE_WHEEL"}) {
            List<SetupField> fields = SetupField.forProfile(profile(type), hardware);
            assertTrue(fields.stream().anyMatch(f -> f.key.equals("forwardReverse")));
            assertTrue(fields.stream().anyMatch(f -> f.key.equals("strafeReverse")));
            assertEquals(type.equals("THREE_WHEEL"), fields.stream().anyMatch(f -> f.key.equals("rightReverse")));
            assertFalse(fields.stream().anyMatch(f -> f.key.equals("distanceIn") || f.key.startsWith("offset")));
        }
    }

    @Test public void corruptSavedSetupCanBeRepairedWithoutDeletingFiles() throws Exception {
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
