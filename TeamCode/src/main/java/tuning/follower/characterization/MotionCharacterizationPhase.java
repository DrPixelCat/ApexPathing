package tuning.follower.characterization;

import tuning.follower.TuningMeasurements;

import com.qualcomm.robotcore.util.ElapsedTime;
import follower.Follower;
import feedforward.MotionCharacterization;
import feedforward.MotionCharacterization.Axis;
import feedforward.MotorPowerModel;
import geometry.AngleUnit;
import geometry.DistUnit;
import geometry.GeometryFactory;
import geometry.Pose;
import tuning.TuningCsvWriter;
import tuning.follower.TunerContext;
import tuning.follower.TuningPhase;

/** Measures speed-dependent holding power and acceleration authority before controller tuning. */
public final class MotionCharacterizationPhase extends TuningPhase {
    private final ElapsedTime timer = new ElapsedTime();
    private final MotorPowerModel[][] models = new MotorPowerModel[3][2];
    private Axis axis = Axis.FORWARD;
    private int direction, run;
    private boolean running, complete;
    private String failure;
    private MotorAuthorityRegression regression;
    private TuningCsvWriter csv;
    private Pose start;
    private double previousVelocity, previousTime, previousPower, lastHeading, rotation;
    private double windowTime, windowVelocity, windowPower, windowPowerVelocity, stableAfter;
    private double sampleTimeSum, sampleTimeSquared, sampleVelocitySum, sampleTimeVelocity;
    private int windowSamples;

    public MotionCharacterizationPhase(TunerContext context) { super(context); }
    @Override protected String getPhaseName() { return "Motion Characterization"; }
    @Override protected String getTuningConstants() { return "Holding power and acceleration authority vs speed"; }
    @Override protected boolean manualTuneIsPossible() { return false; }
    @Override protected boolean autoTuneIsPossible() { return true; }
    @Override protected boolean manualTuned() { return false; }
    @Override protected boolean routineMotionActive() { return running; }
    @Override protected void showPreRunInstructions() {
        context.getTelemetry().addLine("Clear 96 inches of travel, plus stopping room; allow four full turns.");
    }
    @Override protected void init() {
        axis = Axis.FORWARD;
        direction = run = 0;
        running = complete = false;
        failure = null;
        for (MotorPowerModel[] row : models) { java.util.Arrays.fill(row, null); }
        regression = new MotorAuthorityRegression();
        csv = TuningCsvWriter.openBuffered("motion_characterization",
                "record", "axis", "direction", "run", "held_out", "time_s",
                "velocity", "acceleration", "applied_power", "gain0", "gainSlope",
                "drag0", "dragSlope", "max_velocity", "validation_rms", "power_velocity");
    }
    @Override protected boolean autoTuned() {
        if (failure != null) {
            context.getTelemetry().addLine("Characterization validation failed: " + failure);
            context.getTelemetry().addLine("Stop and rerun this phase.");
            return false;
        }
        if (complete) { return true; }
        if (running) { sample(); return complete; }
        context.getTelemetry().addLine(axis == Axis.ANGULAR
                ? "Allow four full clear turns around the robot."
                : "Leave 96 inches clear " + (axis == Axis.FORWARD
                        ? (direction == 0 ? "in front" : "behind")
                        : (direction == 0 ? "to the left" : "to the right")) + ", plus stopping room.");
        context.getTelemetry().addLine("Release sticks. Press A when positioned and stationary.");
        if (opMode.gamepad1.aWasPressed()
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
        previousVelocity = speed(); previousTime = previousPower = rotation = 0;
        lastHeading = start.getHeading().getRad(); resetWindow();
        stableAfter = 0; timer.reset(); running = true;
    }
    private double speed() {
        Follower f = context.getFollower();
        double sign = direction == 0 ? 1 : -1;
        if (axis == Axis.ANGULAR) { return sign * f.getVelocity().getHeading(AngleUnit.RAD); }
        geometry.Vector body = f.getVelocity().getVec().rotate(f.getPose().getHeading().times(-1));
        return sign * (axis == Axis.FORWARD ? body.getX().getIn() : body.getY().getIn());
    }
    private void resetWindow() {
        windowTime = windowVelocity = windowPower = windowPowerVelocity = 0;
        windowSamples = 1; sampleTimeSum = sampleTimeSquared = sampleTimeVelocity = 0;
        sampleVelocitySum = previousVelocity;
    }
    /** Two repeated power levels identify response; a separate sequence is entirely withheld. */
    static double command(int run, double time) {
        return run < 2 ? .65 : run < 4 ? 1 : time < .75 ? .55 : .95;
    }
    private void sample() {
        Follower f = context.getFollower();
        if (!f.getLocalizer().isLastMeasurementValid()) {
            reject("Invalid localization measurement"); return;
        }
        double time = timer.seconds(), v = speed(), dt = time - previousTime;
        double heading = f.getPose().getHeading().getRad();
        rotation += Math.abs(Math.atan2(Math.sin(heading - lastHeading), Math.cos(heading - lastHeading)));
        lastHeading = heading;
        double distance = f.getPose().getVec().minus(start.getVec()).getMag().getIn();
        if (!Double.isFinite(v) || !Double.isFinite(distance) || !Double.isFinite(rotation)) {
            reject("Invalid localization measurement"); return;
        }
        csv.writeRow("raw", axis, direction, run, run >= 4, time, v, "", previousPower);
        double threshold = axis == Axis.ANGULAR ? .1 : 1;
        if (dt > .08) { stableAfter = time + .2; }
        if (v > threshold && previousVelocity > threshold && dt > 0 && dt <= .08 && time >= stableAfter) {
            windowTime += dt; windowVelocity += .5 * (previousVelocity + v) * dt;
            windowPower += previousPower * dt;
            windowPowerVelocity += previousPower * .5 * (previousVelocity + v) * dt;
            windowSamples++; sampleTimeSum += windowTime; sampleTimeSquared += windowTime * windowTime;
            sampleVelocitySum += v; sampleTimeVelocity += windowTime * v;
            if (windowTime >= .15) {
                double velocity = windowVelocity / windowTime;
                double acceleration = (windowSamples * sampleTimeVelocity - sampleTimeSum * sampleVelocitySum)
                        / (windowSamples * sampleTimeSquared - sampleTimeSum * sampleTimeSum);
                double power = windowPower / windowTime;
                regression.add(run >= 4, velocity, acceleration, power, windowPowerVelocity / windowTime);
                csv.writeRow("window", axis, direction, run, run >= 4, time, velocity, acceleration, power,
                        "", "", "", "", "", "", windowPowerVelocity / windowTime);
                previousVelocity = v; resetWindow();
            }
        } else { previousVelocity = v; resetWindow(); }
        previousVelocity = v; previousTime = time;
        if (time >= 2 || distance >= 96 || rotation >= 8 * Math.PI) { finishRun(); return; }
        double power = (direction == 0 ? 1 : -1) * command(run, time)
                * f.getDrivetrain().getConstants().maxPower;
        f.getDrivetrain().moveWithVectors(axis == Axis.FORWARD ? power : 0,
                axis == Axis.STRAFE ? power : 0, axis == Axis.ANGULAR ? power : 0);
        double delivered = TuningMeasurements.maxMotorPower(f.getDrivetrain());
        if (Math.abs(delivered - previousPower) > .02) { resetWindow(); }
        previousPower = delivered;
    }
    private void finishRun() {
        stopMotion(); csv.flush();
        if (++run < 6) { return; }
        // Dropped timing gaps must not turn a short validation into an under-sampled fit.
        // Repeat the withheld sequence for coverage, never add it to the acquisition data.
        if (!regression.validationCovered() && run < 8) { return; }
        try {
            MotorPowerModel model = regression.fit();
            models[axis.ordinal()][direction] = model;
            csv.writeRow("model", axis, direction, "", true, "", "", "", "",
                    model.gain0, model.gainSlope, model.drag0, model.dragSlope,
                    model.maxVelocity, model.validationRms);
            run = 0; regression = new MotorAuthorityRegression();
            if (++direction == 2) {
                direction = 0;
                if (axis == Axis.ANGULAR) {
                    context.constants.motionCharacterization = new MotionCharacterization(models,
                            fHolonomic(), context.getFollower().getDrivetrain().getConstants().maxPower);
                    context.constants.profileVelocityPowerCoefficient = 0;
                    complete = true;
                } else { axis = axis == Axis.FORWARD && fHolonomic() ? Axis.STRAFE : Axis.ANGULAR; }
            }
        } catch (RuntimeException e) { reject(e.getMessage()); }
    }
    private boolean fHolonomic() { return context.getFollower().getDrivetrain().isHolonomic(); }
    private void stopMotion() { context.getFollower().stop(); context.getFollower().enableControllers(); running = false; }
    private void reject(String message) { stopMotion(); failure = message; csv.writeRow("rejected", axis, direction, run, message); csv.flush(); }
    @Override protected void onExit() { context.getFollower().enableControllers(); if (csv != null) { csv.close(); } }
    @Override protected void reportResults() {
        for (Axis a : Axis.values()) {
            if (a == Axis.STRAFE && !fHolonomic()) { continue; }
            context.getTelemetry().addData(a + " calibrated speed",
                    number(context.constants.motionCharacterization.velocityLimit(a)));
        }
    }
}
