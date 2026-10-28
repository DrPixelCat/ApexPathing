package setup;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

/** Instruction-first setup with an explicit hold-to-test control during motor assignment. */
public final class RobotSetupWizard implements AutoCloseable {
    public enum Action { CONFIRM, BACK, UP, DOWN, LEFT, RIGHT, FINER, COARSER }
    public enum Screen { WELCOME, CONFIGURE, WARMUP, READY, RECORDING, REVIEW, FILTER_REST, FILTER_MOVE, SAVE, COMPLETE }
    public enum Task { FORWARD, LEFT, TURN, SPIN_CCW, SPIN_CW, FILTER }
    private final SetupHardware hardware;
    private final SetupStore store;
    private final SetupLog log;
    private RobotProfile draft, candidate;
    private Screen screen = Screen.WELCOME;
    private Task task = Task.FORWARD;
    private SetupSnapshot current = new SetupSnapshot(), start;
    private boolean prepared, correction, savedBefore, closed;
    private int field, choice;
    private double time, enteredAt, lastHeading, sweep, precision = 1;
    private String message = "";
    private double[] ccwOffsets;
    private boolean testRequiresRelease, testedReversed;
    private String testedRole, testedName;

    public RobotSetupWizard(SetupHardware hardware, SetupStore store, SetupLog log) throws IOException {
        this.hardware = hardware; this.store = store; this.log = log;
        RobotProfile saved = null;
        try { saved = store.load(); }
        catch (IOException unreadable) {
            log.event(0, "WELCOME", "load_failed", unreadable.toString());
            try { saved = store.loadPrevious(); } catch (IOException ignored) { /* offer a fresh draft */ }
            message = saved == null ? "Saved setup couldn't be read. Begin a new setup, or stop to keep the existing files."
                    : "Previous setup recovered for review. It will only replace the saved setup if you choose Save.";
        }
        savedBefore = saved != null;
        draft = saved == null ? new RobotProfile() : saved.copy();
        log.event(0, "WELCOME", "session_start", savedBefore ? "Existing setup loaded" : "First setup");
        log.profile(0, "initial", draft);
    }
    public Screen getScreen() { return screen; }
    public Task getTask() { return task; }
    public RobotProfile getDraft() { return draft.copy(); }

    /** Level-triggered every loop, independent from the edge-triggered navigation controls. */
    public void updateMotorTest(boolean held) {
        if (closed) { return; }
        if (!held) { stopMotorTest(); testRequiresRelease = false; return; }
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
            message = "Motor test stopped. Check the selected device, then release X before retrying.";
            log.event(time, "CONFIGURE", "motor_test_failed", failure.toString());
        }
    }

    private String motorRole() {
        if (screen != Screen.CONFIGURE) { return null; }
        List<SetupField> fields = SetupField.forProfile(draft, hardware);
        String key = fields.get(Math.min(field, fields.size() - 1)).key;
        for (String role : new String[]{"fl", "fr", "bl", "br"}) {
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

    /** Called once per loop, before button handling. Invalid samples never count as measurements. */
    public void tick(double now) {
        if (closed) { return; }
        time = now;
        if (!prepared || screen == Screen.COMPLETE) { return; }
        hardware.stop();
        current = hardware.update();
        log.sample(time, screen + ":" + task, current);
        if (!current.valid) {
            if (screen == Screen.RECORDING) { message = "Sensor connection interrupted. Repeat this movement."; transition(Screen.READY); }
            else if (screen == Screen.FILTER_REST || screen == Screen.FILTER_MOVE) {
                hardware.learn(false); hardware.prepare(draft.copy());
                message = "Sensor connection interrupted. Repeat collection."; transition(Screen.WARMUP);
            }
            return;
        }
        if (screen == Screen.RECORDING) {
            sweep += wrap(current.rawPose[2] - lastHeading);
            lastHeading = current.rawPose[2];
        }
        if (screen == Screen.WARMUP && time - enteredAt >= 3) { transition(Screen.READY); }
        if (screen == Screen.FILTER_REST && time - enteredAt >= 3) { transition(Screen.FILTER_MOVE); }
    }

    public void act(Action action) {
        if (closed) { return; }
        // Navigating or changing a direction cannot transfer a held test onto another selection.
        testRequiresRelease = true;
        stopMotorTest();
        log.event(time, screen + ":" + task, "button", action.name());
        if (screen == Screen.WELCOME) {
            if (action == Action.UP || action == Action.DOWN) { choice = 1 - choice; }
            if (action == Action.CONFIRM) {
                if (savedBefore && choice == 0) { prepare(); }
                else { field = 0; transition(Screen.CONFIGURE); }
            }
        } else if (screen == Screen.CONFIGURE) {
            configure(action);
        } else if (screen == Screen.WARMUP) {
            if (action == Action.BACK) { stopForEdit(); }
        } else if (screen == Screen.READY) {
            if (action == Action.BACK) { stopForEdit(); }
            if (action == Action.CONFIRM && current.valid) {
                message = "";
                if (task == Task.FILTER) { hardware.learn(true); transition(Screen.FILTER_REST); }
                else {
                    start = new SetupSnapshot(); start.pods = current.pods.clone();
                    sweep = 0; lastHeading = current.rawPose[2]; transition(Screen.RECORDING);
                }
            }
        } else if (screen == Screen.RECORDING) {
            if (action == Action.BACK) { message = "Movement canceled. Your saved settings are unchanged."; transition(Screen.READY); }
            if (action == Action.CONFIRM) { finishMovement(); }
        } else if (screen == Screen.REVIEW) {
            if (action == Action.UP || action == Action.DOWN) { choice = (choice + (action == Action.UP ? 2 : 1)) % 3; }
            if (action == Action.BACK) { transition(Screen.READY); }
            if (action == Action.CONFIRM) { acceptReview(); }
        } else if (screen == Screen.FILTER_REST || screen == Screen.FILTER_MOVE) {
            if (action == Action.BACK) {
                hardware.learn(false); hardware.prepare(draft.copy());
                message = "Collection canceled. Previous filter settings restored."; transition(Screen.WARMUP);
            }
            if (action == Action.CONFIRM && screen == Screen.FILTER_MOVE && time - enteredAt >= 10) {
                hardware.learn(false);
                candidate = draft.copy(); candidate.filter = hardware.captureFilter(); candidate.filterMeasured = true;
                correction = false; choice = 0; transition(Screen.REVIEW);
                log.profile(time, "filter_candidate", candidate);
            }
        } else if (screen == Screen.SAVE) {
            if (action == Action.UP || action == Action.DOWN) { choice = 1 - choice; }
            if (action == Action.BACK) { task = Task.FILTER; transition(Screen.READY); }
            if (action == Action.CONFIRM) {
                if (choice == 1) { hardware.learn(false); hardware.stop(); transition(Screen.COMPLETE); message = "Previous saved setup kept."; }
                else {
                    try {
                        log.profile(time, "save_requested", draft);
                        store.save(draft);
                        log.profile(time, "saved", draft);
                        hardware.stop(); transition(Screen.COMPLETE); message = "Robot setup saved. You can stop this OpMode.";
                    } catch (IOException e) {
                        log.event(time, "SAVE", "save_failed", e.toString());
                        message = "Couldn't save. Check storage, then press A to retry. Previous setup is kept.";
                    }
                }
            }
        }
    }

    private void configure(Action action) {
        List<SetupField> fields = SetupField.forProfile(draft, hardware);
        SetupField item = fields.get(Math.min(field, fields.size() - 1));
        if (action == Action.FINER) { precision = Math.max(.001, precision / 10); }
        if (action == Action.COARSER) { precision = Math.min(100, precision * 10); }
        int direction = action == Action.RIGHT || action == Action.UP ? 1 : action == Action.LEFT || action == Action.DOWN ? -1 : 0;
        if (direction != 0) {
            String before = draft.get(item.key), after;
            if (item.options != null) {
                if (item.options.isEmpty()) { message = "No matching device is configured. Add it in the Robot Controller configuration first."; return; }
                int index = item.options.indexOf(before);
                after = item.options.get(index < 0 ? 0 : (index + direction + item.options.size()) % item.options.size());
            } else { after = Double.toString(draft.number(item.key) + direction * item.increment * precision); }
            draft.set(item.key, after);
            if (!before.equals(after)) {
                draft.sensorsChecked = false; draft.geometryChanged(); ccwOffsets = null;
                if (item.key.equals("localizer")) { draft.set("offsetX", draft.threeWheel() ? "12" : "0"); draft.set("offsetY", "0"); }
                if (item.key.equals("encoderModel") && !after.equals("CUSTOM")) {
                    draft.set("ticksPerInch", Double.toString((after.equals("GOBILDA_4_BAR") ? 19.89436789 : 13.26291192) * 25.4));
                }
                log.profile(time, "edited_" + item.key, draft);
            }
            message = "";
        }
        if (action == Action.BACK) {
            if (field == 0) { choice = 0; transition(Screen.WELCOME); } else { field--; precision = 1; }
        }
        if (action == Action.CONFIRM) {
            if (!item.accepts(draft.get(item.key))) {
                message = item.options == null ? "Enter a value above zero before continuing." : "Choose a value before continuing.";
                return;
            }
            if (field + 1 < fields.size()) { field++; precision = 1; message = ""; }
            else { prepare(); }
        }
    }

    private void prepare() {
        try {
            draft.validate(); hardware.stop(); hardware.prepare(draft.copy()); prepared = true;
            task = Task.FORWARD; ccwOffsets = null; message = ""; transition(Screen.WARMUP);
        } catch (RuntimeException e) {
            hardware.stop(); prepared = false;
            log.event(time, "CONFIGURE", "hardware_error", e.toString());
            message = e instanceof IllegalArgumentException ? e.getMessage()
                    : "Couldn't connect this setup. Check device assignments and hub mounting.";
            transition(Screen.CONFIGURE);
        }
    }
    private void stopForEdit() {
        hardware.learn(false); hardware.stop(); prepared = false; field = 0; message = ""; transition(Screen.CONFIGURE);
    }

    private void finishMovement() {
        if (!current.valid || current.pods.length != draft.channels() || start == null || start.pods.length != current.pods.length) {
            retry("Waiting for the sensors. Try again when connected."); return;
        }
        double[] delta = new double[current.pods.length];
        for (int i = 0; i < delta.length; i++) {
            delta[i] = current.pods[i] - start.pods[i];
            log.measurement(time, task.name(), "pod_" + i + "_delta", delta[i], "in");
        }
        log.measurement(time, task.name(), "reported_rotation", sweep, "rad");
        candidate = draft.copy(); correction = false;
        if (task == Task.FORWARD) {
            int count = draft.threeWheel() ? 2 : 1;
            double travel = 0;
            for (int i = 0; i < count; i++) {
                if (!Double.isFinite(delta[i]) || Math.abs(delta[i]) < .1) { retry("No clear forward movement. Repeat the marked distance."); return; }
                if (delta[i] < 0) { flip(i == 0 ? "forwardReverse" : "rightReverse"); correction = true; }
                travel += Math.abs(delta[i]) / count;
            }
            if (!correction) {
                candidate.set("ticksPerInch", Double.toString(draft.number("ticksPerInch") * travel / draft.number("distanceIn")));
                candidate.geometryChanged();
            }
        } else if (task == Task.LEFT) {
            double travel = delta[draft.strafeChannel()];
            if (!Double.isFinite(travel) || Math.abs(travel) < .1) { retry("No clear sideways movement. Move left and try again."); return; }
            if (travel < 0) { flip("strafeReverse"); correction = true; }
        } else if (task == Task.TURN) {
            if (!Double.isFinite(sweep) || sweep < .1) {
                retry(sweep < -.1 ? "Heading ran backwards. Check hub mounting or the left/right pod assignments."
                        : "No clear turn. Turn counterclockwise and try again."); return;
            }
            candidate.sensorsChecked = true;
        } else {
            double reference = task == Task.SPIN_CCW ? 2 * Math.PI : -2 * Math.PI;
            double[] measured = offsets(draft, delta, reference);
            if (!Double.isFinite(measured[0]) || !Double.isFinite(measured[1]) || (draft.threeWheel() && measured[0] <= 0)
                    || Math.abs(sweep) < .1) { retry("The full turn was not captured. Keep the center fixed and repeat one complete turn."); return; }
            log.measurement(time, task.name(), "external_reference", reference, "rad");
            log.measurement(time, task.name(), "measured_offset_x", measured[0], "in");
            log.measurement(time, task.name(), "measured_offset_y", measured[1], "in");
            if (task == Task.SPIN_CW && ccwOffsets != null) {
                measured[0] = (measured[0] + ccwOffsets[0]) / 2; measured[1] = (measured[1] + ccwOffsets[1]) / 2;
            }
            candidate.set("offsetX", Double.toString(measured[0])); candidate.set("offsetY", Double.toString(measured[1]));
            candidate.geometryChanged(); candidate.geometryMeasured = task == Task.SPIN_CW && ccwOffsets != null;
        }
        choice = 0; message = ""; transition(Screen.REVIEW); log.profile(time, "candidate_" + task, candidate);
    }

    public static double[] offsets(RobotProfile profile, double[] travel, double reference) {
        if (!Double.isFinite(reference) || Math.abs(reference) < .1) { throw new IllegalArgumentException("A physical reference rotation is required"); }
        return profile.threeWheel() ? new double[]{(travel[0] - travel[1]) / reference, travel[2] / reference}
                : new double[]{travel[0] / reference * (profile.pinpoint() ? -1 : 1), travel[1] / reference};
    }

    private void flip(String field) { candidate.set(field, Boolean.toString(!candidate.flag(field))); candidate.geometryChanged(); }
    private void retry(String explanation) { message = explanation; transition(Screen.READY); }

    private void acceptReview() {
        if (choice == 1) {
            if (task == Task.FILTER) { hardware.prepare(draft.copy()); }
            message = ""; transition(task == Task.FILTER ? Screen.WARMUP : Screen.READY); return;
        }
        if (choice == 0) {
            draft = candidate.copy(); log.profile(time, "applied_to_draft_" + task, draft);
            if (task == Task.SPIN_CCW) { ccwOffsets = new double[]{draft.number("offsetX"), draft.number("offsetY")}; }
            hardware.prepare(draft.copy());
            if (correction) { message = "Direction corrected. Repeat the movement to check it."; transition(Screen.WARMUP); return; }
        } else {
            log.event(time, task.name(), "kept_previous_setting", "Operator chose to keep the draft's earlier value");
            if (correction) { message = "Repeat this check, or go back to edit sensor assignments."; transition(Screen.READY); return; }
            if (task == Task.FILTER) { hardware.prepare(draft.copy()); }
        }
        choice = 0; message = "";
        switch (task) {
            case FORWARD: task = Task.LEFT; break;
            case LEFT: task = Task.TURN; break;
            case TURN: task = Task.SPIN_CCW; break;
            case SPIN_CCW: task = Task.SPIN_CW; break;
            case SPIN_CW: task = Task.FILTER; break;
            case FILTER: transition(Screen.SAVE); return;
        }
        // Every rebuild can recalibrate an IMU; always give it a stationary warmup.
        transition(Screen.WARMUP);
    }

    private void transition(Screen next) {
        screen = next; enteredAt = time;
        log.event(time, next + ":" + task, "screen", next.name());
    }
    private static double wrap(double value) { return Math.atan2(Math.sin(value), Math.cos(value)); }
    private static String friendly(String value) {
        if (value.equals("true")) { return "Reversed"; }
        if (value.equals("false")) { return "Normal"; }
        return value.replace('_', ' ');
    }

    /** Only instructions, operator choices, and progress are shown; numerical results stay in CSV. */
    public List<String> lines() {
        List<String> lines = new ArrayList<>();
        lines.add(screen == Screen.CONFIGURE || screen == Screen.WELCOME ? "1 / 3 - Your robot"
                : task == Task.FORWARD || task == Task.LEFT || task == Task.TURN ? "2 / 3 - Sensor checks" : "3 / 3 - Localization");
        if (!message.isEmpty()) { lines.add(message); }
        switch (screen) {
            case WELCOME:
                lines.add("Set up the robot with guided hand movements and optional motor tests.");
                lines.add("Measurements are recorded automatically.");
                if (savedBefore) { menu(lines, "Use saved robot settings", "Edit robot settings"); }
                else { lines.add("A: Begin setup"); }
                break;
            case CONFIGURE:
                List<SetupField> fields = SetupField.forProfile(draft, hardware);
                SetupField item = fields.get(Math.min(field, fields.size() - 1));
                lines.add("Setting " + (field + 1) + " of " + fields.size()); lines.add(item.label);
                lines.add(item.accepts(draft.get(item.key)) ? item.role.isEmpty() ? friendly(draft.get(item.key)) : draft.get(item.key)
                        : "Choose a configured device");
                lines.add(item.options == null ? "Up/Down: adjust; bumpers: finer/coarser" : "Up/Down: choose");
                if (item.options == null) { lines.add("Adjustment: " + item.increment * precision); }
                String motor = motorRole();
                if (motor != null) {
                    lines.add("Selected motor: " + draft.get(motor) + " (" + friendly(draft.get(motor + "Reverse")) + ")");
                    lines.add("Keep drive wheels clear.");
                    lines.add(testedRole != null ? "Testing selected direction. Release X to stop."
                            : testRequiresRelease ? "Release X before holding it to test."
                            : "Hold X: test selected direction at 0.5 power. Release to stop.");
                }
                lines.add("A: Next    B: Back");
                break;
            case WARMUP:
                lines.add("Keep the robot still while the sensors settle.");
                lines.add("Continuing shortly...    B: Edit setup"); break;
            case READY:
                instruction(lines);
                if (!current.valid) { lines.add("Waiting for the sensor connection."); }
                lines.add("Press A before moving. Start from your floor mark.");
                lines.add(task == Task.FILTER ? "A: Begin collection    B: Edit setup" : "A: Begin movement    B: Edit setup"); break;
            case RECORDING:
                lines.add("Recording"); instruction(lines); lines.add("When the movement is finished, stop the robot.");
                lines.add("A: Finished    B: Cancel this movement"); break;
            case REVIEW:
                lines.add(correction ? "A sensor ran opposite to the expected direction."
                        : task == Task.FILTER ? "Filter collection recorded." : "Movement recorded.");
                menu(lines, correction ? "Correct direction and repeat" : task == Task.FILTER ? "Use collected filter settings" : "Use this measurement",
                        "Repeat this step", "Keep previous setting");
                lines.add("B: Back"); break;
            case FILTER_REST:
                lines.add("Keep the robot completely still for a moment."); lines.add("B: Cancel collection"); break;
            case FILTER_MOVE:
                lines.add(draft.holonomic() ? "Move smoothly forward, backward, sideways, and through turns."
                        : "Move smoothly forward, backward, and through turns.");
                if (time - enteredAt < 10) {
                    lines.add("Collecting motion data: " + (int) Math.ceil(10 - (time - enteredAt)) + " seconds remaining.");
                    lines.add("B: Cancel collection");
                } else {
                    lines.add("Enough data is recorded. Stop when you are ready.");
                    lines.add("A: Finish collection    B: Cancel");
                }
                break;
            case SAVE:
                lines.add("Your choices are applied to this session. Save them for future OpModes?");
                menu(lines, "Save robot setup", "Keep the previously saved setup"); lines.add("B: Back"); break;
            case COMPLETE:
                lines.add("All session data has been recorded. Stop this OpMode when ready."); break;
        }
        return lines;
    }
    private void menu(List<String> lines, String... options) {
        for (int i = 0; i < options.length; i++) { lines.add((choice == i ? "> " : "  ") + options[i]); }
        lines.add("Up/Down: choose    A: Confirm");
    }
    private void instruction(List<String> lines) {
        switch (task) {
            case FORWARD:
                lines.add("Mark a straight " + draft.get("distanceIn") + " inch distance on the floor.");
                lines.add("Push the robot forward from the start mark to the end mark without turning."); break;
            case LEFT:
                lines.add(draft.holonomic() ? "Push the robot straight to its left a short distance without turning."
                        : "With the chassis stationary, roll the sideways odometry wheel by hand as if the robot moves left.");
                break;
            case TURN: lines.add("Turn the robot counterclockwise about a quarter turn."); break;
            case SPIN_CCW: case SPIN_CW:
                lines.add("Mark the robot center and starting direction on the floor.");
                lines.add("Keeping that center fixed, turn exactly one full turn " + (task == Task.SPIN_CCW ? "counterclockwise." : "clockwise."));
                lines.add("Use the floor marks to finish at the original direction."); break;
            case FILTER: lines.add("Next, collect stationary and smooth-motion filter data by hand."); break;
        }
    }

    @Override public void close() {
        if (closed) { return; }
        closed = true;
        try {
            stopMotorTest();
            hardware.learn(false); hardware.stop();
            if (screen != Screen.COMPLETE) { log.event(time, screen + ":" + task, "session_aborted", "Saved profile was not changed"); }
        } finally { log.close(); }
    }
}
