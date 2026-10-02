package tuning.follower.phases;

import com.qualcomm.robotcore.util.ElapsedTime;
import core.Follower;
import geometry.AngleUnit;
import geometry.DistUnit;
import geometry.GeometryFactory;
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
    private final double[] breakawayPower = {Double.NaN, Double.NaN};
    private String csvPath = "Not started";
    private TuningCsvWriter csv;

    public FeedforwardTuner(TunerContext context) { this(context, Axis.DRIVE); }
    public FeedforwardTuner(TunerContext context, Axis axis) { super(context); this.axis = axis; }
    @Override protected String getPhaseName() { return axis + " Feedforward SysId"; }
    @Override protected String getTuningConstants() { return axis + " feedforward kS, kV, kA"; }
    @Override protected boolean manualTuneIsPossible() { return false; }
    @Override protected boolean autoTuneIsPossible() { return true; }
    @Override protected boolean manualTuned() { return false; }
    @Override protected boolean routineMotionActive() { return stage == Stage.RUNNING; }
    @Override protected void showPreRunInstructions() {
        context.getTelemetry().addLine(axis == Axis.DRIVE
                ? "Clear 96 inches in BOTH directions, plus braking room."
                : "Clear room for two full turns in BOTH directions.");
    }
    @Override protected void init() {
        context.getFollower().stop(); context.getFollower().enableControllers();
        regression = new SysIdRegression(); run = 0; stage = Stage.READY;
        breakawayPower[0] = breakawayPower[1] = Double.NaN;
        fit = null;
    }
    @Override protected void onExit() {
        context.getFollower().enableControllers();
        closeCsv();
    }
    @Override protected boolean autoTuned() {
        if (stage == Stage.READY) {
            showPreRunInstructions();
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
            context.getTelemetry().addLine("Feedforward validation failed. Stop and rerun this phase.");
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
        previousVelocity = v; previousTime = time;
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
            Class<?> renderer = Class.forName("tuning.follower.phases.SysIdGraphs");
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
    }
}
