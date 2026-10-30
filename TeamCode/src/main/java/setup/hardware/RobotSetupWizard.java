package setup.hardware;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import setup.WorkflowText;
import setup.AdjustmentStep;

/** Setup draft, field editor, views, and hardware safeguards. RobotSetup owns the workflow. */
public final class RobotSetupWizard implements AutoCloseable {
    public enum Action { CONFIRM, UP, DOWN, LEFT, RIGHT, FINER, COARSER }
    public enum Screen { WELCOME, CONFIGURE, SAVE, COMPLETE, ERROR }

    private final SetupHardware hardware;
    private final SetupStore store;
    private final SetupLog log;
    private Screen screen;
    private SetupScreen activeScreen;
    private RobotProfile draft;
    private ConfigureScreen configuration;
    private boolean savedBefore;
    private boolean closed;
    private double time;
    private String message = "";
    private boolean testRequiresRelease;
    private boolean testedReversed;
    private String testedRole;
    private String testedName;

    public RobotSetupWizard(SetupHardware hardware, SetupStore store, SetupLog log) throws IOException {
        this.hardware = hardware;
        this.store = store;
        this.log = log;
        RobotProfile saved = null;
        try { saved = store.load(); }
        catch (IOException unreadable) {
            log.event(0, "WELCOME", "load_failed", unreadable.toString());
            try { saved = store.loadPrevious(); }
            catch (IOException ignored) { /* The operator can start with a fresh draft. */ }
            message = saved == null
                    ? "Saved setup couldn't be read. Begin a new setup, or STOP to keep the existing files."
                    : "Previous setup recovered. It replaces the saved setup only if you choose Save.";
        }
        savedBefore = saved != null;
        draft = saved == null ? new RobotProfile() : saved.copy();
        log.event(0, "WELCOME", "session_start", savedBefore ? "Existing setup loaded" : "First setup");
        log.profile(0, "initial", draft);

        transition(Screen.WELCOME);
    }

    public Screen getScreen() { return screen; }
    public RobotProfile getDraft() { return draft.copy(); }

    /** Updates the session clock without collecting localization measurements. */
    public void tick(double now) {
        if (!closed) { time = now; }
    }

    /** Stops held tests before navigation and records the action once. */
    public boolean beginAction(Action action) {
        if (closed) { return false; }
        testRequiresRelease = true;
        stopMotorTest();
        log.event(time, getScreen().name(), "button", action.name());
        return true;
    }

    public void chooseWelcome(Action action) {
        if (action == Action.UP || action == Action.DOWN) {
            WelcomeScreen welcome = (WelcomeScreen) activeScreen;
            welcome.choice = 1 - welcome.choice;
        }
    }

    public boolean useSavedSettings() {
        return savedBefore && ((WelcomeScreen) activeScreen).choice == 0;
    }

    public void beginConfiguration() { message = ""; transition(Screen.CONFIGURE); }
    public boolean editConfiguration(Action action) { return configuration.edit(action); }
    public void showSave() { transition(Screen.SAVE); }
    public void complete() {
        message = "Robot setup saved. You can stop this OpMode.";
        transition(Screen.COMPLETE);
    }

    public boolean saveSettings() {
        try {
            log.profile(time, "save_requested", draft);
            store.save(draft);
            log.profile(time, "saved", draft);
            return true;
        } catch (IOException failure) {
            log.event(time, "SAVE", "save_failed", failure.toString());
            fail("Couldn't save. Check storage. Previous setup is kept.");
            return false;
        }
    }

    public List<String> lines() { return activeScreen.view(); }

    private void transition(Screen next) {
        if (screen == next) { return; }
        if (activeScreen != null) { activeScreen.exit(); }
        screen = next;
        switch (next) {
            case WELCOME: activeScreen = new WelcomeScreen(); break;
            case CONFIGURE: activeScreen = new ConfigureScreen(); break;
            case SAVE: activeScreen = new SaveScreen(); break;
            case COMPLETE: activeScreen = new CompleteScreen(); break;
            case ERROR: activeScreen = new ErrorScreen(); break;
            default: throw new IllegalArgumentException("Unknown setup screen: " + next);
        }
        activeScreen.enter();
    }

    private abstract class SetupScreen {
        public void exit() { }
        public void enter() {
            log.event(time, getScreen().name(), "screen", getScreen().name());
        }

        public List<String> view() {
            List<String> lines = new ArrayList<>();
            lines.add("Robot hardware setup");
            if (!message.isEmpty()) { lines.add(message); }
            describe(lines);
            if (getScreen() != Screen.COMPLETE) { lines.add(WorkflowText.STOP_HINT); }
            return lines;
        }

        abstract void describe(List<String> lines);
    }

    private final class WelcomeScreen extends SetupScreen {
        private int choice;

        @Override void describe(List<String> lines) {
            lines.add("Configure robot hardware, encoders, and optional motor tests.");
            lines.add("Calibrate distance, pod offsets, and filtering in Localization Tuner.");
            if (savedBefore) { menu(lines, choice, "Use saved robot settings", "Edit robot settings"); }
            else { lines.add("A: Begin setup"); }
        }
    }

    private final class ConfigureScreen extends SetupScreen {
        private int field;
        private final AdjustmentStep precision = new AdjustmentStep(1, .001, 100, 10);

        @Override public void enter() { super.enter(); configuration = this; }
        @Override public void exit() { stopMotorTest(); configuration = null; }

        boolean edit(Action action) {
            List<SetupField> fields = SetupField.forProfile(draft, hardware);
            SetupField item = fields.get(Math.min(field, fields.size() - 1));
            if (action == Action.FINER) { precision.finer(); }
            if (action == Action.COARSER) { precision.coarser(); }
            int direction = action == Action.RIGHT || action == Action.UP ? 1 : action == Action.LEFT || action == Action.DOWN ? -1 : 0;
            if (direction != 0) {
                String before = draft.get(item.key), after;
                if (item.options != null) {
                    if (item.options.isEmpty()) { message = "No matching device is configured. Add it in the Robot Controller configuration first."; return false; }
                    int index = item.options.indexOf(before);
                    after = item.options.get(index < 0 ? 0 : (index + direction + item.options.size()) % item.options.size());
                } else { after = Double.toString(draft.number(item.key) + direction * item.increment * precision.get()); }
                if (!item.accepts(after)) { message = "Choose a value within this field's range."; return false; }
                draft.set(item.key, after);
                if (!before.equals(after)) {
                    draft.sensorsChecked = false; draft.geometryChanged();
                    if (item.key.equals("localizer")) { draft.set("offsetX", draft.threeWheel() ? "12" : "0"); draft.set("offsetY", "0"); }
                    if (item.key.equals("encoderModel") && !after.equals("CUSTOM")) {
                        draft.set("ticksPerInch", Double.toString((after.equals("GOBILDA_4_BAR") ? 19.89436789 : 13.26291192) * 25.4));
                    }
                    log.profile(time, "edited_" + item.key, draft);
                }
                message = "";
            }
            if (action == Action.CONFIRM) {
                if (!item.accepts(draft.get(item.key))) {
                    message = item.options == null ? "Choose a value within this field's range." : "Choose a value before continuing.";
                    return false;
                }
                if (field + 1 < fields.size()) { field++; precision.reset(); message = ""; }
                else { return true; }
            }
            return false;
        }

        @Override void describe(List<String> lines) {
            List<SetupField> fields = SetupField.forProfile(draft, hardware);
            SetupField item = fields.get(Math.min(field, fields.size() - 1));
            lines.add("Setting " + (field + 1) + " of " + fields.size());
            lines.add(item.label);
            lines.add(item.accepts(draft.get(item.key))
                    ? item.role.isEmpty() ? friendly(draft.get(item.key)) : draft.get(item.key)
                    : "Choose a configured device");
            lines.add(item.options == null ? "Up/Down: adjust; bumpers: finer/coarser" : "Up/Down: choose");
            if (item.options == null) { lines.add("Adjustment: " + item.increment * precision.get()); }
            String motor = motorRole();
            if (motor != null) {
                lines.add("Selected motor: " + draft.get(motor) + " (" + friendly(draft.get(motor + "Reverse")) + ")");
                lines.add("Keep drive wheels clear.");
                if (draft.get("drive").equals("OMNI")) { lines.add("Positive power must drive along the CCW tangent."); }
                lines.add(testedRole != null ? "Testing selected direction. Release X to stop."
                        : testRequiresRelease ? "Release X before holding it to test."
                        : "Hold X: test selected direction at 0.5 power. Release to stop.");
            }
            if (draft.get("drive").equals("COAXIAL_SWERVE")) {
                lines.add("Align modules by hand for drive motor tests. Steering remains stopped.");
            }
            if (item.key.endsWith("ZeroDeg")) { lines.add("Point this wheel forward by hand. Press X to record its forward direction."); }
            if (draft.get("drive").equals("DUAL_ACTUATED")) { lines.add("Servo positions are saved only; setup does not actuate them."); }
            lines.add("A: Next");
        }
    }

    private final class SaveScreen extends SetupScreen {
        @Override void describe(List<String> lines) {
            lines.add("Save these settings for future OpModes?");
            lines.add("Next: run Localization Tuner for localization calibration.");
            lines.add("A: Save robot setup");
        }
    }

    private final class CompleteScreen extends SetupScreen {
        @Override public void enter() { super.enter(); hardware.stop(); }
        @Override void describe(List<String> lines) {
            lines.add("All session data has been recorded. Stop this OpMode when ready.");
        }
    }

    private final class ErrorScreen extends SetupScreen {
        @Override public void enter() { super.enter(); hardware.learn(false); hardware.stop(); }
        @Override void describe(List<String> lines) { lines.add("Your previously saved setup is retained."); }
    }

    /** Level-triggered every loop, independent from the edge-triggered navigation controls. */
    public void updateMotorTest(boolean held) {
        if (closed) { return; }
        if (!held) { stopMotorTest(); testRequiresRelease = false; return; }
        if (!testRequiresRelease && getScreen() == Screen.CONFIGURE && draft.get("drive").equals("COAXIAL_SWERVE")) {
            List<SetupField> fields = SetupField.forProfile(draft, hardware);
            String key = fields.get(Math.min(configuration.field, fields.size() - 1)).key;
            if (key.endsWith("ZeroDeg")) {
                testRequiresRelease = true;
                try {
                    String corner = key.substring(0, 2);
                    double angle = hardware.steeringAngleDegrees(draft.get(corner + "Encoder"));
                    if (!Double.isFinite(angle) || angle < 0 || angle > 360) { throw new IllegalStateException("Invalid steering angle"); }
                    draft.set(key, Double.toString(angle)); draft.sensorsChecked = false; draft.geometryChanged();
                    log.profile(time, "captured_" + key, draft);
                    message = "Forward direction recorded. Press A to continue.";
                } catch (RuntimeException failure) {
                    fail("Encoder capture failed. Check the selected encoder.");
                    log.event(time, "CONFIGURE", "steering_capture_failed", failure.toString());
                }
                return;
            }
        }
        String role = motorRole();
        if (testRequiresRelease || role == null) {
            stopMotorTest(); testRequiresRelease = true; return;
        }
        String name = draft.get(role);
        if (!hardware.devices("motor").contains(name)) {
            stopMotorTest(); testRequiresRelease = true;
            message = "Choose a configured motor before testing."; return;
        }
        try {
            MotorTestReading reading = hardware.testMotor(name, draft.flag(role + "Reverse"));
            if (testedRole == null) {
                testedRole = role; testedName = name; testedReversed = draft.flag(role + "Reverse");
                log.event(time, "CONFIGURE", "motor_test_started", name + "; reversed=" + testedReversed + "; power=0.5");
            }
            message = "";
            log.motorTest(time, role, name, testedReversed, true, reading);
        } catch (RuntimeException failure) {
            hardware.stop(); testedRole = null; testRequiresRelease = true;
            fail("Motor test stopped. Check the selected device.");
            log.event(time, "CONFIGURE", "motor_test_failed", failure.toString());
        }
    }

    private String motorRole() {
        if (getScreen() != Screen.CONFIGURE) { return null; }
        List<SetupField> fields = SetupField.forProfile(draft, hardware);
        String key = fields.get(Math.min(configuration.field, fields.size() - 1)).key;
        for (String role : draft.motorKeys()) {
            if (key.equals(role) || key.equals(role + "Reverse")) { return role; }
        }
        return null;
    }

    private void stopMotorTest() {
        if (testedRole == null) { return; }
        hardware.stop();
        String role = testedRole; testedRole = null;
        log.motorTest(time, role, testedName, testedReversed, false, null);
        log.event(time, "CONFIGURE", "motor_test_stopped", testedName + "; power=0");
    }

    public boolean validateConfiguration() {
        try {
            draft.validate();
            for (SetupField field : SetupField.forProfile(draft, hardware)) {
                if (!field.accepts(draft.get(field.key))) {
                    throw new IllegalArgumentException("Check " + field.label);
                }
            }
            hardware.stop();
            message = "";
            return true;
        } catch (RuntimeException failure) {
            log.event(time, "CONFIGURE", "hardware_error", failure.toString());
            fail(failure instanceof IllegalArgumentException ? failure.getMessage()
                    : "Couldn't check this setup. Check device assignments.");
            return false;
        }
    }

    private void fail(String explanation) {
        message = explanation;
        hardware.stop();
        hardware.learn(false);
        transition(Screen.ERROR);
    }

    private static String friendly(String value) {
        if (value.equals("true")) { return "Reversed"; }
        if (value.equals("false")) { return "Normal"; }
        return value.replace('_', ' ');
    }

    private static void menu(List<String> lines, int selection, String... options) {
        for (int i = 0; i < options.length; i++) {
            lines.add((selection == i ? "> " : "  ") + options[i]);
        }
        lines.add("Up/Down: choose    A: Confirm");
    }

    @Override public void close() {
        if (closed) { return; }
        closed = true;
        try {
            try { hardware.stop(); }
            finally {
                try { stopMotorTest(); }
                finally {
                    try { activeScreen.exit(); }
                    finally { hardware.learn(false); }
                }
            }
            if (getScreen() != Screen.COMPLETE) {
                log.event(time, getScreen().name(), "session_aborted", "Saved profile was not changed");
            }
        } finally { log.close(); }
    }
}
