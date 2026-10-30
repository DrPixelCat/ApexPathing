package setup.follower.characterization;

import paths.geometry.Vector;
import util.DeltaTime;

import setup.follower.TuningMeasurements;

import com.qualcomm.robotcore.util.ElapsedTime;
import follower.Follower;
import feedforward.MotionCharacterization;
import feedforward.MotionCharacterization.Axis;
import feedforward.MotorPowerModel;
import paths.geometry.AngleUnit;
import paths.geometry.DistUnit;
import paths.geometry.GeometryFactory;
import paths.geometry.Pose;
import setup.logging.TuningCsvWriter;
import setup.follower.TunerContext;
import setup.follower.TuningPhase;

/**
 * Measures speed-dependent holding power and acceleration authority before controller tuning.
 *
 * @author Joel H
 */
public final class MotionCharacterizationPhase extends TuningPhase {
    private static final int RUNS_PER_DIRECTION = 4;
    private final ElapsedTime timer = new ElapsedTime();
    private final MotorPowerModel[][] models = new MotorPowerModel[3][2];
    private Axis axis = Axis.FORWARD;
    private int direction, run;
    private boolean running, complete;
    private String failure;
    private final MotorAuthorityRegression[] regressions = new MotorAuthorityRegression[2];
    private TuningCsvWriter csv;
    private Pose start;
    private double previousVelocity, previousPower, lastHeading, rotation;
    private double windowTime, windowVelocity, windowPower, windowPowerVelocity, stableAfter;
    private double windowStartVelocity;
    private final DeltaTime timing = new DeltaTime();

    public MotionCharacterizationPhase(TunerContext context) { super(context); }
    @Override protected String getPhaseName() { return "Motion Characterization"; }
    @Override protected String getTuningConstants() { return "Holding power and acceleration authority vs speed"; }
    @Override protected boolean manualTuneIsPossible() { return false; }
    @Override protected boolean autoTuneIsPossible() { return true; }
    @Override protected boolean manualTuned() { return false; }
    @Override protected boolean routineMotionActive() { return running; }
    @Override protected void showPreRunInstructions() {
        context.getView().addLine("Clear 96 inches of travel, plus stopping room; allow four full turns.");
    }
    @Override protected void init() {
        axis = Axis.FORWARD;
        direction = run = 0;
        running = complete = false;
        failure = null;
        for (MotorPowerModel[] row : models) { java.util.Arrays.fill(row, null); }
        resetRegressions();
        csv = TuningCsvWriter.openBuffered("motion_characterization",
                "record", "axis", "direction", "run", "held_out", "time_s",
                "velocity", "acceleration", "applied_power", "gain0", "gainSlope",
                "drag0", "dragSlope", "max_velocity", "validation_rms", "power_velocity",
                "native_velocity", "measurement_time_ns", "filtered_velocity");
    }
    @Override protected boolean autoTuned() {
        if (failure != null) {
            context.getView().addLine("Characterization acquisition failed: " + failure);
            context.getView().addLine("Stop and rerun this phase.");
            return false;
        }
        if (complete) { return true; }
        if (running) { sample(); return complete; }
        context.getView().addLine(axis == Axis.ANGULAR
                ? "Allow four full clear turns around the robot."
                : "Leave 96 inches clear " + (axis == Axis.FORWARD
                        ? (direction == 0 ? "in front" : "behind")
                        : (direction == 0 ? "to the left" : "to the right")) + ", plus stopping room.");
        context.getView().addData("Acquisition run", (run + 1) + "/" + RUNS_PER_DIRECTION + ", " + (direction == 0 ? "+" : "-") + " direction");
        context.getView().addLine("Release sticks. Press A when positioned and stationary.");
        if (confirmPressed()
                && context.getFollower().getVelocity().getVec().getMag().getIn() < .5
                && Math.abs(context.getFollower().getVelocity().getHeading(AngleUnit.RAD)) < .05) {
            beginRun();
        }
        return false;
    }
    private void beginRun() {
        Follower f = context.getFollower();
        double sign = direction == 0 ? 1 : -1;
        GeometryFactory staging = new GeometryFactory().setDistUnit(DistUnit.IN);
        positionRobotForSimulation(axis == Axis.FORWARD ? staging.pose(-55 * sign, 0, 0)
                : axis == Axis.STRAFE ? staging.pose(0, -55 * sign, 0) : Pose.zero());
        f.stop(); f.disableControllers(); start = f.getPose();
        previousVelocity = speed(); previousPower = rotation = 0;
        timing.reset();
        lastHeading = start.getHeading().getRad(); resetWindow();
        stableAfter = 0; timer.reset(); running = true;
    }
    private double speed() {
        Follower f = context.getFollower();
        // Identify the plant from sensor observations. The adaptive navigation estimator
        // has its own transient response, which must not become motor drag/inertia.
        return speed(f.getLocalizer().getRawVel(), f.getLocalizer().getRawPose());
    }
    private double speed(Pose velocity, Pose pose) {
        double sign = direction == 0 ? 1 : -1;
        if (axis == Axis.ANGULAR) { return sign * velocity.getHeading(AngleUnit.RAD); }
        Vector body = velocity.getVec().rotate(pose.getHeading().times(-1));
        return sign * (axis == Axis.FORWARD ? body.getX().getIn() : body.getY().getIn());
    }
    private void resetWindow() {
        windowTime = windowVelocity = windowPower = windowPowerVelocity = 0;
        windowStartVelocity = previousVelocity;
    }
    /** Two repeats at each acquisition power level, alternating direction after every run. */
    static double command(int run) {
        return run < 2 ? .65 : 1;
    }
    private void sample() {
        Follower f = context.getFollower();
        if (!f.getLocalizer().isLastMeasurementValid()) {
            reject("Invalid localization measurement"); return;
        }
        double time = timer.seconds(), v = speed();
        long measurementTime = f.getLocalizer().getLastMeasurementTimeNanos();
        double dt = timing.update(measurementTime);
        double heading = f.getPose().getHeading().getRad();
        rotation += Math.abs(Math.atan2(Math.sin(heading - lastHeading), Math.cos(heading - lastHeading)));
        lastHeading = heading;
        double distance = f.getPose().getVec().minus(start.getVec()).getMag().getIn();
        if (!Double.isFinite(v) || !Double.isFinite(distance) || !Double.isFinite(rotation)) {
            reject("Invalid localization measurement"); return;
        }
        csv.writeRow("raw", axis, direction, run, false, time, v, "", previousPower,
                "", "", "", "", "", "", "",
                v, measurementTime, speed(f.getVelocity(), f.getPose()));
        double threshold = axis == Axis.ANGULAR ? .1 : 1;
        if (dt > .08) { stableAfter = time + .2; }
        if (v > threshold && previousVelocity > threshold && dt > 0 && dt <= .08 && time >= stableAfter) {
            windowTime += dt; windowVelocity += .5 * (previousVelocity + v) * dt;
            windowPower += previousPower * dt;
            windowPowerVelocity += previousPower * .5 * (previousVelocity + v) * dt;
            if (windowTime >= .15) {
                double velocity = windowVelocity / windowTime;
                // All regression terms describe the same time integral of the plant equation.
                // A least-squares velocity slope uses a different temporal weighting.
                double acceleration = (v - windowStartVelocity) / windowTime;
                double power = windowPower / windowTime;
                regressions[direction].add(velocity, acceleration, power, windowPowerVelocity / windowTime);
                csv.writeRow("window", axis, direction, run, false, time, velocity, acceleration, power,
                        "", "", "", "", "", "", windowPowerVelocity / windowTime);
                previousVelocity = v; resetWindow();
            }
        } else { previousVelocity = v; resetWindow(); }
        previousVelocity = v;
        if (time >= 2 || distance >= 96 || rotation >= 8 * Math.PI) { finishRun(); return; }
        double power = (direction == 0 ? 1 : -1) * command(run)
                * f.getDrivetrain().getConstants().maxPower;
        f.getDrivetrain().moveWithVectors(axis == Axis.FORWARD ? power : 0,
                axis == Axis.STRAFE ? power : 0, axis == Axis.ANGULAR ? power : 0);
        double delivered = TuningMeasurements.axisCommandPower(f.getDrivetrain(), axis);
        if (Math.abs(delivered - previousPower) > .02) { resetWindow(); }
        previousPower = delivered;
    }
    private void resetRegressions() {
        for (int i = 0; i < regressions.length; i++) { regressions[i] = new MotorAuthorityRegression(); }
    }
    private void finishRun() {
        stopMotion(); csv.flush();
        direction = 1 - direction;
        if (direction == 1) { return; }
        // Increment the per-direction run count only after the negative run completes the pair.
        if (++run < RUNS_PER_DIRECTION) { return; }
        try {
            for (int i = 0; i < 2; i++) {
                MotorPowerModel model = regressions[i].fit();
                models[axis.ordinal()][i] = model;
                csv.writeRow("model", axis, i, "", false, "", "", "", "",
                        model.gain0, model.gainSlope, model.drag0, model.dragSlope,
                        model.maxVelocity, model.validationRms);
            }
            run = 0; resetRegressions();
            if (axis == Axis.ANGULAR) {
                context.constants.motionCharacterization = new MotionCharacterization(models,
                        fHolonomic(), context.getFollower().getDrivetrain().getConstants().maxPower);
                context.constants.profileVelocityPowerCoefficient = 0;
                complete = true;
            } else { axis = axis == Axis.FORWARD && fHolonomic() ? Axis.STRAFE : Axis.ANGULAR; }
        } catch (RuntimeException e) { reject(e.getMessage()); }
    }
    private boolean fHolonomic() { return context.getFollower().getDrivetrain().isHolonomic(); }
    private void stopMotion() { context.getFollower().stop(); context.getFollower().enableControllers(); running = false; }
    private void reject(String message) { stopMotion(); failure = message; csv.writeRow("rejected", axis, direction, run, message); csv.flush(); }
    @Override protected void onExit() { context.getFollower().enableControllers(); if (csv != null) { csv.close(); } }
    @Override protected void reportResults() {
        for (Axis a : Axis.values()) {
            if (a == Axis.STRAFE && !fHolonomic()) { continue; }
            context.getView().addData(a + " calibrated speed",
                    number(context.constants.motionCharacterization.velocityLimit(a)));
        }
    }
}
