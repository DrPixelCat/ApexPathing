package tuning.follower.sysid;

import util.DeltaTime;

import com.qualcomm.robotcore.util.ElapsedTime;
import follower.Follower;
import geometry.AngleUnit;
import geometry.DistUnit;
import geometry.GeometryFactory;
import geometry.Pose;
import tuning.TuningCsvWriter;
import tuning.follower.TunerContext;
import tuning.follower.TuningPhase;

/**
 * Independent axis characterization using quasistatic and dynamic tests in both directions.
 *
 * @author Joel H
 */
public final class FeedforwardTuner extends TuningPhase {
    public enum Axis { DRIVE, HEADING }
    private enum Stage { READY, RUNNING, REVIEW }
    private static final String[] RUNS = {"Quasistatic forward", "Quasistatic reverse", "Dynamic forward", "Dynamic reverse"};
    private final Axis axis;
    private final ElapsedTime timer = new ElapsedTime();
    private Stage stage = Stage.READY;
    private SysIdRegression regression;
    private SysIdRegression.Fit fit;
    private int run;
    private Pose startPose;
    private double lastHeading, travel, previousPower, previousVelocity;
    private final DeltaTime timing = new DeltaTime();
    private double windowTime, windowVelocity, windowPower, windowStartVelocity;
    private final double[] breakawayPower = {Double.NaN, Double.NaN};
    private String csvPath = "Not started";
    private TuningCsvWriter csv;
    private int selectedGain, nextDirection = 1, manualDirection;
    private ManualFeedforwardProfile manualProfile;
    private String manualResult = "No trial yet";
    private boolean manualSaturated;
    private Runnable restoreSimulatorMotorModel = () -> { };

    public FeedforwardTuner(TunerContext context) { this(context, Axis.DRIVE); }
    public FeedforwardTuner(TunerContext context, Axis axis) { super(context); this.axis = axis; }
    @Override protected String getPhaseName() { return axis + " Feedforward SysId"; }
    @Override protected String getTuningConstants() { return axis + " feedforward kS, kV, kA"; }
    @Override protected boolean manualTuneIsPossible() { return true; }
    @Override protected boolean autoTuneIsPossible() { return true; }
    @Override protected boolean manualTuned() {
        // Poll edges during motion too, so an ignored press cannot become a queued idle action.
        boolean left = opMode.gamepad1.leftBumperWasPressed();
        boolean right = opMode.gamepad1.rightBumperWasPressed();
        boolean test = opMode.gamepad1.xWasPressed();
        boolean accept = opMode.gamepad1.aWasPressed();
        double change = manualChange();
        if (stage == Stage.RUNNING) {
            sampleManual();
        } else {
            if (left) { selectedGain = (selectedGain + 2) % 3; }
            if (right) { selectedGain = (selectedGain + 1) % 3; }
            if (change != 0) { setManualGain(selectedGain, Math.max(0, manualGain(selectedGain) + change)); }
            if (test) {
                if (stationary()) { beginManual(); }
                else { manualResult = "Release sticks and wait until stationary."; }
            } else if (accept) {
                if (validManualGains()) { applyManualGains(); return true; }
                manualResult = "Use finite nonnegative kS/kA and positive kV before saving.";
            }
        }
        addTunableValue("kS", manualGain(0), selectedGain == 0);
        addTunableValue("kV", manualGain(1), selectedGain == 1);
        addTunableValue("kA", manualGain(2), selectedGain == 2);
        context.getTelemetry().addData("Increment", number(increment));
        showManualPositioning();
        reportManualErrors();
        if (stage == Stage.RUNNING) {
            context.getTelemetry().addLine("Stop: cancel test");
        } else {
            context.getTelemetry().addLine("LB/RB select; Dpad Up/Down edit; Left/Right increment");
            context.getTelemetry().addLine("X: run from rest; A: accept; Stop: cancel");
        }
        return false;
    }

    private double manualGain(int index) {
        if (axis == Axis.DRIVE) {
            return index == 0 ? context.constants.translationalFeedforwardKS
                    : index == 1 ? context.constants.translationalKV : context.constants.translationalKA;
        }
        return index == 0 ? context.constants.angularFeedforwardKS
                : index == 1 ? context.constants.angularKV : context.constants.angularKA;
    }

    private void setManualGain(int index, double value) {
        if (axis == Axis.DRIVE) {
            if (index == 0) { context.constants.translationalFeedforwardKS = value; }
            else if (index == 1) { context.constants.translationalKV = value; }
            else { context.constants.translationalKA = value; }
        } else {
            if (index == 0) { context.constants.angularFeedforwardKS = value; }
            else if (index == 1) { context.constants.angularKV = value; }
            else { context.constants.angularKA = value; }
        }
    }

    private boolean validManualGains() {
        return Double.isFinite(manualGain(0)) && manualGain(0) >= 0
                && Double.isFinite(manualGain(1)) && manualGain(1) > 0
                && Double.isFinite(manualGain(2)) && manualGain(2) >= 0;
    }

    private boolean stationary() {
        Pose velocity = context.getFollower().getVelocity();
        return context.getFollower().getLocalizer().isLastMeasurementValid()
                && Double.isFinite(velocity.getVec().getMag().getIn())
                && velocity.getVec().getMag().getIn() <= .5
                && Double.isFinite(velocity.getHeading(AngleUnit.RAD))
                && Math.abs(velocity.getHeading(AngleUnit.RAD)) <= .05;
    }

    private void showManualPositioning() {
        int direction = stage == Stage.RUNNING ? manualDirection : nextDirection;
        context.getTelemetry().addData(stage == Stage.RUNNING ? "Running" : "Next direction",
                axis == Axis.DRIVE ? (direction > 0 ? "Forward" : "Backward")
                        : (direction > 0 ? "CCW (+)" : "CW (-)"));
        if (stage == Stage.RUNNING) { return; }
        context.getTelemetry().addLine(axis == Axis.DRIVE
                ? "Clear lane: 60 in " + (direction > 0 ? "ahead" : "behind") + " + braking room."
                : "Clear rotation: 3 rad (~172 deg) + braking room.");
    }

    private void beginManual() {
        Follower f = context.getFollower();
        f.stop(); f.disableControllers();
        manualDirection = nextDirection;
        nextDirection = -nextDirection;
        manualProfile = new ManualFeedforwardProfile(manualDirection * (axis == Axis.DRIVE ? 24 : 1.5));
        manualProfile.begin(measuredVelocity());
        startPose = f.getPose(); lastHeading = startPose.getHeading().getRad(); travel = 0;
        manualSaturated = false; manualResult = "Running";
        timer.reset(); stage = Stage.RUNNING;
        commandManual(0);
    }

    private void sampleManual() {
        Follower f = context.getFollower();
        double time = timer.seconds(), measured = measuredVelocity();
        double heading = f.getPose().getHeading().getRad();
        travel += Math.abs(Math.atan2(Math.sin(heading - lastHeading), Math.cos(heading - lastHeading)));
        lastHeading = heading;
        double distance = f.getPose().getVec().minus(startPose.getVec()).getMag().getIn();
        if (!f.getLocalizer().isLastMeasurementValid() || !Double.isFinite(measured)
                || !Double.isFinite(distance) || !Double.isFinite(travel)) {
            finishManual("Aborted: invalid localization; results incomplete."); return;
        }
        manualProfile.sample(time, measured);
        if (axis == Axis.DRIVE ? distance >= 60 : travel >= 4) {
            finishManual("Aborted: travel limit reached; results incomplete."); return;
        }
        if (time >= ManualFeedforwardProfile.END_SECONDS) {
            finishManual("Completed " + (axis == Axis.DRIVE ? (manualDirection > 0 ? "forward" : "backward")
                    : (manualDirection > 0 ? "CCW" : "CW")) + " trial"); return;
        }
        commandManual(time);
    }

    private void commandManual(double time) {
        double v = manualProfile.velocity(time), a = manualProfile.acceleration(time);
        double ks = manualGain(0), kv = manualGain(1), ka = manualGain(2);
        double command = ks * (v == 0 ? Math.signum(a) : Math.signum(v)) + kv * v + ka * a;
        if (!Double.isFinite(command)) { finishManual("Aborted: invalid feedforward constants."); return; }
        double limit = Math.min(.85, context.getFollower().getDrivetrain().getConstants().maxPower);
        manualSaturated |= Math.abs(command) > limit;
        command = Math.max(-limit, Math.min(limit, command));
        context.getFollower().getDrivetrain().moveWithVectors(axis == Axis.DRIVE ? command : 0, 0,
                axis == Axis.HEADING ? command : 0);
    }

    private void finishManual(String result) {
        stopMotion(); stage = Stage.READY; manualResult = result;
    }

    private void reportManualErrors() {
        context.getTelemetry().addData("Trial", manualResult);
        if (manualProfile == null) { return; }
        String units = axis == Axis.DRIVE ? " in/s" : " rad/s";
        double acceleration = manualProfile.accelerationMeanError(), cruise = manualProfile.cruiseMeanError();
        context.getTelemetry().addData("Accel mean error",
                Double.isFinite(acceleration) ? number(acceleration) + units : "N/A: no samples");
        context.getTelemetry().addData("Cruise mean error",
                Double.isFinite(cruise) ? number(cruise) + units : "N/A: target not reached / no samples");
        if (manualSaturated) { context.getTelemetry().addLine("Power clipped: errors include saturation."); }
    }

    private void applyManualGains() {
        context.constants.motionCharacterization = null;
        updateFollowerGains();
    }
    @Override protected boolean routineMotionActive() { return stage == Stage.RUNNING; }
    @Override protected void showPreRunInstructions() {
        if (manualMode) {
            showManualPositioning();
            context.getTelemetry().addLine(axis == Axis.DRIVE
                    ? "Profile: 24 in/s, 48 in; 1s accel/cruise/brake each."
                    : "Profile: 1.5 rad/s, 3 rad; 1s accel/cruise/brake each.");
            context.getTelemetry().addLine("Mean error = measured - target; braking excluded.");
            context.getTelemetry().addLine("Release sticks; X runs from rest; Stop cancels.");
            return;
        }
        context.getTelemetry().addLine(axis == Axis.DRIVE
                ? "Clear 96 inches in BOTH directions, plus braking room."
                : "Clear room for two full turns in BOTH directions.");
    }
    @Override protected void init() {
        context.getFollower().stop(); context.getFollower().enableControllers();
        enableSimulatorMotorOverride();
        regression = new SysIdRegression(); run = 0; stage = Stage.READY;
        breakawayPower[0] = breakawayPower[1] = Double.NaN;
        fit = null;
        selectedGain = 0; nextDirection = 1; manualDirection = 1;
        manualProfile = null; manualResult = "No trial yet";
    }
    @Override protected void onExit() {
        try {
            context.getFollower().enableControllers();
            closeCsv();
        } finally {
            restoreSimulatorMotorModel.run();
            restoreSimulatorMotorModel = () -> { };
        }
    }

    private void enableSimulatorMotorOverride() {
        try {
            // Desktop-only adapter; no simulator dependency is packaged on the robot.
            Class<?> adapter = Class.forName("org.firstinspires.ftc.teamcode.sim.SimulatorSysIdMotorOverride");
            restoreSimulatorMotorModel = (Runnable) adapter.getMethod("open",
                    com.qualcomm.robotcore.hardware.HardwareMap.class).invoke(null, opMode.hardwareMap);
        } catch (ClassNotFoundException robotRuntime) {
            // Actual robot hardware retains its normal motor-controller behavior.
        } catch (ReflectiveOperationException failure) {
            throw new IllegalStateException("Could not configure simulator SysId motor model", failure);
        }
    }
    @Override protected boolean autoTuned() {
        if (stage == Stage.READY) {
            showPreRunInstructions();
            context.getTelemetry().addData("Next test", RUNS[run]);
            context.getTelemetry().addLine(axis == Axis.DRIVE
                    ? (run % 2 == 0 ? "Allow 96 inches in front of the robot."
                    : "Allow 96 inches behind the robot.") : "Allow two full turns in both directions.");
            context.getTelemetry().addLine("Release sticks. Press A when stationary.");
            if (opMode.gamepad1.aWasPressed()) {
                Pose v = context.getFollower().getVelocity();
                if (!Double.isFinite(v.getVec().getMag().getIn()) || !Double.isFinite(v.getHeading(AngleUnit.RAD))
                        || v.getVec().getMag().getIn() > .5 || Math.abs(v.getHeading(AngleUnit.RAD)) > .05) {
                    context.getTelemetry().addLine("Release sticks and wait until stationary.");
                } else { beginTest(); }
            }
        } else if (stage == Stage.RUNNING) { sample(); }
        else {
            if (fit != null && fit.valid && breakawayValid()) { applyFit(); return true; }
            context.getTelemetry().addLine("Feedforward acquisition failed. Stop and rerun this phase.");
        }
        return false;
    }
    private void beginTest() {
        Follower f = context.getFollower();
        if (Boolean.getBoolean("apex.simulation.unlockTunerPhases")) {
            GeometryFactory staging = new GeometryFactory().setDistUnit(DistUnit.IN);
            // Every SysId run needs the full 96-inch travel. Reusing the previous
            // simulator endpoint can hit a field boundary and corrupt kA.
            positionRobotForSimulation(axis == Axis.DRIVE
                    ? staging.pose(run % 2 == 0 ? -55.0 : 55.0, 0.0, 0.0)
                    : staging.pose(0.0, 0.0, 0.0));
        }
        f.stop(); f.disableControllers(); startPose = f.getPose();
        lastHeading = startPose.getHeading().getRad(); travel = previousPower = 0;
        timing.reset(0L);
        previousVelocity = measuredVelocity();
        resetWindow();
        if (csv == null) {
            csv = TuningCsvWriter.openBuffered("sysid_" + axis.name().toLowerCase(java.util.Locale.US),
                    "record", "test", "time_s", "velocity", "acceleration", "applied_power", "included");
            csvPath = csv.getPath();
        }
        timer.reset(); stage = Stage.RUNNING;
    }
    private double measuredVelocity() {
        Follower f = context.getFollower();
        return axis == Axis.HEADING ? f.getVelocity().getHeading(AngleUnit.RAD)
                : f.getVelocity().getVec().rotate(f.getPose().getHeading().times(-1)).getX().getIn();
    }
    private void resetWindow() {
        windowTime = windowVelocity = windowPower = 0; windowStartVelocity = previousVelocity;
    }
    private void sample() {
        context.getTelemetry().addData("Test", RUNS[run]);
        Follower f = context.getFollower();
        if (!f.getLocalizer().isLastMeasurementValid()) {
            stopMotion();
            throw new IllegalStateException("Feedforward tuning requires valid localization");
        }
        long timeNanos = (long) timer.nanoseconds();
        double time = DeltaTime.seconds(timeNanos), v = measuredVelocity();
        double dt = timing.update(timeNanos);
        double heading = f.getPose().getHeading().getRad();
        travel += Math.abs(Math.atan2(Math.sin(heading - lastHeading), Math.cos(heading - lastHeading)));
        lastHeading = heading;
        double distance = f.getPose().getVec().minus(startPose.getVec()).getMag().getIn();
        if (!Double.isFinite(v) || !Double.isFinite(distance) || !Double.isFinite(travel)
                || !Double.isFinite(previousPower)) {
            stopMotion();
            csv.writeRow("invalid", RUNS[run]);
            csv.flush();
            stage = Stage.REVIEW;
            return;
        }
        // Integrate the PRECEDING delivered command over its measurement interval. Windows
        // reduce differentiation noise without mixing stationary friction into the moving model.
        double threshold = axis == Axis.DRIVE ? .5 : .05;
        if (run < 2 && Double.isNaN(breakawayPower[run])) {
            breakawayPower[run] = breakawayCrossingPower(run, previousVelocity,
                    v, previousPower, threshold);
        }
        boolean moving = Math.abs(v) >= threshold && Math.abs(previousVelocity) >= threshold
                && Math.signum(v) == Math.signum(previousVelocity) && dt > 0 && dt <= .1;
        csv.writeRow("raw", RUNS[run], time, v, "", previousPower, moving);
        if (moving) {
            windowTime += dt; windowVelocity += .5 * (v + previousVelocity) * dt;
            windowPower += previousPower * dt;
            if (windowTime >= .1) {
                double meanV = windowVelocity / windowTime, a = (v - windowStartVelocity) / windowTime;
                double p = windowPower / windowTime;
                boolean included = regression.add(run, time, meanV, a, p);
                csv.writeRow("window", RUNS[run], time, meanV, a, p, included);
                previousVelocity = v; resetWindow();
            }
        } else { previousVelocity = v; resetWindow(); }
        previousVelocity = v;
        boolean limit = axis == Axis.DRIVE ? distance >= 96 : travel >= 4 * Math.PI;
        if (limit || time >= (run < 2 ? 9 : 2)) {
            stopMotion(); csv.flush();
            if (++run == 4) {
                closeCsv(); fit = regression.fit(axis == Axis.DRIVE ? 2 : .2);
                writeFitCsv();
                renderDesktopGraphs();
                stage = Stage.REVIEW;
            } else { stage = Stage.READY; }
            return;
        }
        double command = (run % 2 == 0 ? 1 : -1) * (run < 2 ? Math.min(.9, .1 * time) : .5);
        f.getDrivetrain().moveWithVectors(axis == Axis.DRIVE ? command : 0, 0,
                axis == Axis.HEADING ? command : 0);
        double delivered = Math.max(Math.max(Math.abs(f.getDrivetrain().getLastFlPower()), Math.abs(f.getDrivetrain().getLastFrPower())),
                Math.max(Math.abs(f.getDrivetrain().getLastBlPower()), Math.abs(f.getDrivetrain().getLastBrPower())));
        previousPower = Math.copySign(delivered, command);
    }
    static double breakawayCrossingPower(int run, double before, double after,
                                         double power, double velocityThreshold) {
        double direction = run % 2 == 0 ? 1.0 : -1.0;
        if (run >= 2 || !Double.isFinite(before) || !Double.isFinite(after)
                || !Double.isFinite(power) || !Double.isFinite(velocityThreshold)
                || direction * before >= velocityThreshold
                || direction * after < velocityThreshold
                || direction * power < 0.03) {
            return Double.NaN;
        }
        return Math.abs(power);
    }
    private boolean breakawayValid() {
        return Double.isFinite(breakawayPower[0]) && Double.isFinite(breakawayPower[1]);
    }
    private double staticBreakawayPower() {
        // One direction can be inflated by a delayed localization sample. The smaller
        // measured launch is the conservative static term for the bidirectional PDS search.
        return Math.min(breakawayPower[0], breakawayPower[1]);
    }
    private void stopMotion() { context.getFollower().stop(); context.getFollower().enableControllers(); }
    private void closeCsv() {
        if (csv != null) { csv.close(); csv = null; }
    }
    private void writeFitCsv() {
        TuningCsvWriter reviewed = TuningCsvWriter.open("sysid_fit_" + axis.name().toLowerCase(java.util.Locale.US),
                "test", "time_s", "velocity", "acceleration", "applied_power", "fitted_power", "residual", "kS", "kV", "kA", "r_squared", "valid");
        for (SysIdRegression.Sample s : regression.samples) {
            double predicted = fit.predict(s);
            reviewed.writeRow(s.run, s.time, s.velocity, s.acceleration, s.power, predicted,
                    s.power - predicted, fit.kS, fit.kV, fit.kA, fit.rSquared, fit.valid);
        }
        reviewed.close();
        csvPath = reviewed.getError() == null ? reviewed.getPath() : "CSV error: " + reviewed.getError();
    }
    private void renderDesktopGraphs() {
        // XChart/AWT remain on the desktop test classpath, never in the Android APK.
        try {
            Class<?> renderer = Class.forName("tuning.follower.sysid.SysIdGraphs");
            renderer.getMethod("render", String.class, String.class).invoke(null, csvPath, axis.name());
        } catch (ClassNotFoundException ignored) {
            // Android does not include the desktop renderer.
        } catch (ReflectiveOperationException e) {
            com.qualcomm.robotcore.util.RobotLog.ee("Apex SysId", e, "Could not render SysId graphs.");
        }
    }
    private void applyFit() {
        context.constants.motionCharacterization = null;
        if (axis == Axis.DRIVE) {
            context.constants.translationalFeedforwardKS = fit.kS;
            context.constants.translationalKV = fit.kV; context.constants.translationalKA = fit.kA;
            context.constants.translationalCoeffs.s(staticBreakawayPower());
        } else {
            context.constants.angularFeedforwardKS = fit.kS;
            context.constants.angularKV = fit.kV; context.constants.angularKA = fit.kA;
            context.constants.angularCoeffs.s(staticBreakawayPower());
        }
        updateFollowerGains();
    }
    private void updateFollowerGains() {
        context.getFollower().setFeedforwardGains(context.constants.translationalKV, context.constants.translationalKA,
                context.constants.angularKV, context.constants.angularKA);
        double previousDriveFeedback = context.constants.velocityFeedbackGain;
        double previousHeadingFeedback = context.constants.angularVelocityFeedbackGain;
        context.constants.deriveVelocityFeedbackGains();
        if (axis == Axis.DRIVE) { context.constants.angularVelocityFeedbackGain = previousHeadingFeedback; }
        else { context.constants.velocityFeedbackGain = previousDriveFeedback; }
        context.getFollower().setVelocityFeedback(context.constants.velocityFeedbackGain, context.constants.angularVelocityFeedbackGain);
    }
    @Override protected void reportResults() {
        if (manualMode) {
            context.getTelemetry().addData("kS / kV / kA", number(manualGain(0)) + " / " + number(manualGain(1)) + " / " + number(manualGain(2)));
            reportManualErrors(); return;
        }
        if (fit == null) { return; }
        context.getTelemetry().addData("kS / kV / kA", number(fit.kS) + " / " + number(fit.kV) + " / " + number(fit.kA));
    }
}
