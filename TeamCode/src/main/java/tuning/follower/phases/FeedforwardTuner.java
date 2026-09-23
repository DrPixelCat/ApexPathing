package tuning.follower.phases;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import core.Follower;
import geometry.AngleUnit;
import geometry.Pose;
import tuning.TuningCsvWriter;
import tuning.follower.TunerContext;
import tuning.follower.TuningPhase;

/** Independent axis characterization using quasistatic and dynamic tests in both directions. */
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
    private double lastHeading, travel, previousPower, previousTime, previousVelocity;
    private double windowTime, windowVelocity, windowPower, windowStartVelocity;
    private String status = "Not run", csvPath = "Not started", graphPath = "Not generated";
    private TuningCsvWriter csv;

    public FeedforwardTuner(TunerContext context) { this(context, Axis.DRIVE); }
    public FeedforwardTuner(TunerContext context, Axis axis) { super(context); this.axis = axis; }
    @Override protected String getPhaseName() { return axis + " Feedforward SysId"; }
    @Override protected boolean manualTuneIsPossible() { return false; }
    @Override protected boolean autoTuneIsPossible() { return true; }
    @Override protected boolean manualTuned() { return false; }
    @Override protected boolean routineMotionActive() { return stage == Stage.RUNNING; }
    @Override protected void showPreRunInstructions() {
        context.getTelemetry().addLine("Four tests: slow ramp and power step, each forward and reverse.");
        context.getTelemetry().addLine("Each test starts from rest with A. X aborts the current test.");
        context.getTelemetry().addLine(axis == Axis.DRIVE
                ? "Clear 96 inches in BOTH directions, plus braking room."
                : "Clear room for two full turns in BOTH directions.");
        context.getTelemetry().addLine("Fits kS, kV and kA together; saves CSV and desktop PNG results graphs.");
    }
    @Override protected void init() {
        context.getFollower().stop(); context.getFollower().enableControllers();
        regression = new SysIdRegression(); run = 0; stage = Stage.READY;
        status = "Ready"; fit = null;
    }
    @Override public boolean run(LinearOpMode opMode) {
        try { return super.run(opMode); }
        finally { context.getFollower().stop(); context.getFollower().enableControllers(); closeCsv(); }
    }
    @Override protected boolean autoTuned() {
        if (stage == Stage.READY) {
            context.getTelemetry().addData("Next test", RUNS[run]);
            context.getTelemetry().addLine("Reposition with sticks. Release sticks. Press A from rest.");
            if (opMode.gamepad1.aWasPressed()) {
                Pose v = context.getFollower().getVelocity();
                if (!Double.isFinite(v.getVec().getMag().getIn()) || !Double.isFinite(v.getHeading(AngleUnit.RAD))
                        || v.getVec().getMag().getIn() > .5 || Math.abs(v.getHeading(AngleUnit.RAD)) > .05) {
                    status = "Wait until stationary.";
                } else { beginTest(); }
            }
        } else if (stage == Stage.RUNNING) { sample(); }
        else {
            reportResults();
            context.getTelemetry().addLine("X: discard all four tests and retry.");
            if (opMode.gamepad1.xWasPressed()) { init(); }
            else if (fit.valid) {
                context.getTelemetry().addLine("Press A to accept this axis's kS, kV and kA.");
                if (opMode.gamepad1.aWasPressed()) { applyFit(); return true; }
            } else { context.getTelemetry().addLine("Fit rejected: need all four tests, speed range, nonnegative gains and R² >= 0.90."); }
        }
        context.getTelemetry().addData("Status", status);
        context.getTelemetry().update();
        return false;
    }
    private void beginTest() {
        Follower f = context.getFollower();
        f.stop(); f.disableControllers(); startPose = f.getPose();
        lastHeading = startPose.getHeading().getRad(); travel = previousPower = previousTime = 0;
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
        Follower f = context.getFollower();
        double time = timer.seconds(), v = measuredVelocity(), dt = time - previousTime;
        double heading = f.getPose().getHeading().getRad();
        travel += Math.abs(Math.atan2(Math.sin(heading - lastHeading), Math.cos(heading - lastHeading)));
        lastHeading = heading;
        double distance = f.getPose().getVec().minus(startPose.getVec()).getMag().getIn();
        if (!Double.isFinite(v) || !Double.isFinite(distance) || !Double.isFinite(travel)
                || !Double.isFinite(previousPower)) { abort("Invalid measurement; current test discarded."); return; }
        // Integrate the PRECEDING delivered command over its measurement interval. Windows
        // reduce differentiation noise without mixing stationary friction into the moving model.
        double threshold = axis == Axis.DRIVE ? .5 : .05;
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
        previousVelocity = v; previousTime = time;
        if (opMode.gamepad1.xWasPressed()) { abort("Aborted; reposition and retry current test."); return; }
        boolean limit = axis == Axis.DRIVE ? distance >= 96 : travel >= 4 * Math.PI;
        if (limit || time >= (run < 2 ? 9 : 2)) {
            stopMotion(); csv.flush();
            status = limit ? "Travel limit reached." : "Test complete.";
            if (++run == 4) {
                closeCsv(); fit = regression.fit(axis == Axis.DRIVE ? 2 : .2);
                writeFitCsv();
                graphPath = renderDesktopGraphs();
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
        context.getTelemetry().addData("SysId test", RUNS[run]);
        context.getTelemetry().addData("Power / velocity", number(previousPower) + " / " + number(v));
        context.getTelemetry().addLine("X: abort test. Red STOP: end tuning.");
    }
    private void abort(String message) {
        stopMotion();
        regression.samples.removeIf(s -> s.run == run);
        csv.writeRow("aborted", RUNS[run]); csv.flush(); stage = Stage.READY; status = message;
    }
    private void stopMotion() { context.getFollower().stop(); context.getFollower().enableControllers(); }
    private void closeCsv() {
        if (csv != null) { csv.close(); if (csv.getError() != null) { status = "CSV error: " + csv.getError(); } csv = null; }
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
    private String renderDesktopGraphs() {
        // XChart/AWT remain on the desktop test classpath, never in the Android APK.
        try {
            Class<?> renderer = Class.forName("tuning.follower.phases.SysIdGraphs");
            return (String) renderer.getMethod("render", String.class, String.class).invoke(null, csvPath, axis.name());
        } catch (ClassNotFoundException e) {
            return "Run desktop SysIdGraphs on the saved fit CSV.";
        } catch (ReflectiveOperationException e) {
            return "Graph error: " + e.getMessage();
        }
    }
    private void applyFit() {
        if (axis == Axis.DRIVE) {
            context.constants.translationalFeedforwardKS = fit.kS;
            context.constants.translationalKV = fit.kV; context.constants.translationalKA = fit.kA;
        } else {
            context.constants.angularFeedforwardKS = fit.kS;
            context.constants.angularKV = fit.kV; context.constants.angularKA = fit.kA;
        }
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
        if (fit == null) { return; }
        context.getTelemetry().addData("kS / kV / kA", number(fit.kS) + " / " + number(fit.kV) + " / " + number(fit.kA));
        context.getTelemetry().addData("R² / RMS power error", number(fit.rSquared) + " / " + number(fit.rms));
        context.getTelemetry().addData("Units", axis == Axis.DRIVE ? "normalized power, inches, seconds" : "normalized power, radians, seconds");
        context.getTelemetry().addData("CSV", csvPath);
        context.getTelemetry().addData("Results graphs", graphPath);
    }
}
