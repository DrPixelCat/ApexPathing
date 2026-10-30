package setup.follower.sysid;

import util.DeltaTime;

import com.qualcomm.robotcore.util.ElapsedTime;
import follower.Follower;
import paths.geometry.AngleUnit;
import paths.geometry.DistUnit;
import paths.geometry.GeometryFactory;
import paths.geometry.Pose;
import setup.logging.TuningCsvWriter;
import setup.follower.TunerContext;
import setup.follower.TuningMeasurements;
import setup.follower.TuningPhase;

/**
 * Fits kS and kV from slow power ramps, keeping the existing kA unchanged.
 *
 * @author Joel H
 */
public final class FeedforwardTuner extends TuningPhase {
    public enum Axis { DRIVE, HEADING }
    private enum Stage { READY, RUNNING, REVIEW }
    static final double RAMP_RATE = .1;
    static final double MAX_RAMP_POWER = .9;
    static final double MAX_DRIVE_TRAVEL_IN = 48;
    static final double MAX_HEADING_TRAVEL_RAD = 3 * Math.PI;

    /** Drive uses forward/reverse ramps; heading uses one counterclockwise ramp. */
    static int trialDirection(Axis axis, int run) {
        return axis == Axis.DRIVE && run == 1 ? -1 : 1;
    }

    /** Returns the slow ramp command, before the drivetrain's configured power limit. */
    static double trialPower(Axis axis, int run, double seconds) {
        return trialDirection(axis, run) * Math.min(MAX_RAMP_POWER, RAMP_RATE * Math.max(0, seconds));
    }

    private int trialCount() { return axis == Axis.DRIVE ? 2 : 1; }
    private String trialName() { return "Quasistatic " + automaticDirection().toLowerCase(java.util.Locale.US); }

    private String automaticDirection() {
        boolean forward = trialDirection(axis, run) > 0;
        if (axis == Axis.DRIVE) {
            return forward ? "Forward" : "Reverse";
        }
        return forward ? "CCW" : "CW";
    }
    private final Axis axis;
    private final ElapsedTime timer = new ElapsedTime();
    private Stage stage;
    private boolean acceptedManual;
    private boolean acceptedAutomatic;
    private boolean selectPrevious;
    private boolean selectNext;
    private boolean testPressed;
    private double gainChange;
    private SysIdRegression regression;
    private SysIdRegression.Fit fit;
    private int run;
    private Pose startPose;
    private double lastHeading, travel, previousPower, previousVelocity, lastStationaryPower;
    private final DeltaTime timing = new DeltaTime();
    private double windowTime, windowVelocity, windowPower, windowStartVelocity;
    private final double[] breakawayPower = {Double.NaN, Double.NaN};
    private String csvPath = "Not started";
    private TuningCsvWriter csv;
    private int selectedGain, nextDirection = 1, manualDirection;
    private ManualFeedforwardProfile manualProfile;
    private String manualResult = "No trial yet";
    private boolean manualSaturated;

    public FeedforwardTuner(TunerContext context) { this(context, Axis.DRIVE); }
    public FeedforwardTuner(TunerContext context, Axis axis) { super(context); this.axis = axis; }
    @Override protected String getPhaseName() { return axis + " Feedforward SysId"; }
    @Override protected String getTuningConstants() {
        return axis + (manualMode ? " feedforward kS, kV, kA" : " feedforward kS, kV");
    }
    @Override protected boolean manualTuneIsPossible() { return true; }
    @Override protected boolean autoTuneIsPossible() { return true; }
    @Override protected boolean manualTuned() {
        // Consume ignored edges during motion rather than queuing an idle action.
        selectPrevious = opMode.gamepad1.leftBumperWasPressed();
        selectNext = opMode.gamepad1.rightBumperWasPressed();
        testPressed = opMode.gamepad1.xWasPressed();
        gainChange = manualChange();
        updateAcquisition();
        if (acceptedManual) { return true; }
        addTunableValue("kS", manualGain(0), selectedGain == 0);
        addTunableValue("kV", manualGain(1), selectedGain == 1);
        addTunableValue("kA", manualGain(2), selectedGain == 2);
        context.getView().addData("Increment", number(increment.get()));
        showManualPositioning();
        reportManualErrors();
        if (isIn(Stage.RUNNING)) {
            context.getView().addLine("Test in progress.");
        } else {
            context.getView().addLine("LB/RB select; Dpad Up/Down edit; Left/Right increment");
            context.getView().addLine("X: run from rest; A: accept");
        }
        return false;
    }

    private void updateManualReady() {
        if (selectPrevious) { selectedGain = (selectedGain + 2) % 3; }
        if (selectNext) { selectedGain = (selectedGain + 1) % 3; }
        if (gainChange != 0) { setManualGain(selectedGain, Math.max(0, manualGain(selectedGain) + gainChange)); }
        if (testPressed) {
            if (stationary()) { beginManual(); }
            else { manualResult = "Release sticks and wait until stationary."; }
        } else if (confirmPressed()) {
            if (validManualGains()) { applyManualGains(); acceptedManual = true; return; }
            manualResult = "Use finite nonnegative kS/kA and positive kV before saving.";
        }
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
        int direction = isIn(Stage.RUNNING) ? manualDirection : nextDirection;
        context.getView().addData(isIn(Stage.RUNNING) ? "Running" : "Next direction",
                axis == Axis.DRIVE ? (direction > 0 ? "Forward" : "Backward")
                        : (direction > 0 ? "CCW (+)" : "CW (-)"));
        if (isIn(Stage.RUNNING)) { return; }
        context.getView().addLine(axis == Axis.DRIVE
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
        timer.reset(); changeStage(Stage.RUNNING);
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
        if (result.startsWith("Aborted:")) {
            stopMotion();
            throw new IllegalStateException(result);
        }
        stopMotion(); changeStage(Stage.READY); manualResult = result;
    }

    private void reportManualErrors() {
        context.getView().addData("Trial", manualResult);
        if (manualProfile == null) { return; }
        String units = axis == Axis.DRIVE ? " in/s" : " rad/s";
        double acceleration = manualProfile.accelerationMeanError(), cruise = manualProfile.cruiseMeanError();
        context.getView().addData("Accel mean error",
                Double.isFinite(acceleration) ? number(acceleration) + units : "N/A: no samples");
        context.getView().addData("Cruise mean error",
                Double.isFinite(cruise) ? number(cruise) + units : "N/A: target not reached / no samples");
        if (manualSaturated) { context.getView().addLine("Power clipped: errors include saturation."); }
    }

    private void applyManualGains() {
        context.constants.motionCharacterization = null;
        updateFollowerGains();
    }
    private boolean isIn(Stage state) { return stage == state; }

    @Override protected boolean routineMotionActive() { return isIn(Stage.RUNNING); }
    @Override protected void showPreRunInstructions() {
        if (manualMode) {
            showManualPositioning();
            context.getView().addLine(axis == Axis.DRIVE
                    ? "Profile: 24 in/s, 48 in; 1s accel/cruise/brake each."
                    : "Profile: 1.5 rad/s, 3 rad; 1s accel/cruise/brake each.");
            context.getView().addLine("Mean error = measured - target; braking excluded.");
            context.getView().addLine("Release sticks; X runs from rest.");
            return;
        }
        if (axis == Axis.DRIVE && run == 0) {
            context.getView().addLine("Initial pose: x = -12 in, y = -12 in from center; heading = 45 degrees.");
        }
        context.getView().addLine(axis == Axis.DRIVE
                ? "Forward and reverse ramps, each from rest. Do not reposition between trials."
                : "One CCW ramp, from rest; allow 1.5 turns plus stopping room.");
        context.getView().addLine("Power rises by 0.1 per second. Fits kS/kV; existing kA is retained.");
    }
    @Override protected void init() {
        context.getFollower().stop(); context.getFollower().enableControllers();
        regression = new SysIdRegression();
        if (!manualMode && !(context.getFollower().getDrivetrain().getConstants().maxPower > 0)) {
            throw new IllegalStateException("Feedforward ramps require a positive motor power limit.");
        }
        if (!manualMode && Boolean.getBoolean("apex.simulation.unlockTunerPhases")) {
            GeometryFactory staging = new GeometryFactory().setDistUnit(DistUnit.IN).setAngleUnit(AngleUnit.DEG);
            // Place once before acquisition; every trial then retains the real pose and velocity.
            positionRobotForSimulation(axis == Axis.DRIVE ? staging.pose(-12, -12, 45) : Pose.zero());
        }
        run = 0;
        acceptedManual = false;
        acceptedAutomatic = false;
        breakawayPower[0] = breakawayPower[1] = Double.NaN;
        fit = null;
        selectedGain = 0; nextDirection = 1; manualDirection = 1;
        manualProfile = null; manualResult = "No trial yet";
        stage = Stage.READY;
    }
    @Override protected void onExit() {
        closeCsv();
    }

    private void updateAcquisition() {
        switch (stage) {
            case READY:
                if (manualMode) { updateManualReady(); }
                else { updateAutomaticReady(); }
                break;
            case RUNNING:
                if (manualMode) { sampleManual(); }
                else { sample(); }
                break;
            case REVIEW:
                if (fit != null && fit.valid && breakawayValid()) {
                    applyFit();
                    acceptedAutomatic = true;
                } else {
                    context.getView().addLine("Feedforward acquisition failed.");
                    if (fit != null) {
                        context.getView().addData("R-squared / power RMS", number(fit.rSquared) + " / " + number(fit.rms));
                    }
                    context.getView().addLine("The ramps need sufficient moving data and a consistent linear power-speed fit. No fit saved.");
                }
                break;
        }
    }

    private void changeStage(Stage next) {
        if (stage == next) { return; }
        if (stage == Stage.RUNNING) { stopMotion(); }
        stage = next;
    }

    @Override protected boolean autoTuned() {
        updateAcquisition();
        return acceptedAutomatic;
    }

    private void updateAutomaticReady() {
        showPreRunInstructions();
        context.getView().addData("Next test", trialName());
        context.getView().addData("Direction", automaticDirection());
        context.getView().addLine(axis == Axis.DRIVE
                ? (trialDirection(axis, run) > 0 ? "Allow 48 inches ahead, plus braking room."
                : "Allow 48 inches behind, plus braking room.") : "Allow 1.5 turns plus stopping room.");
        context.getView().addLine("Release sticks. Press A when stationary.");
        if (confirmPressed()) {
            Pose v = context.getFollower().getVelocity();
            if (!Double.isFinite(v.getVec().getMag().getIn()) || !Double.isFinite(v.getHeading(AngleUnit.RAD))
                    || v.getVec().getMag().getIn() > .5 || Math.abs(v.getHeading(AngleUnit.RAD)) > .05) {
                context.getView().addLine("Release sticks and wait until stationary.");
            } else { beginTest(); }
        }
    }

    private void beginTest() {
        Follower f = context.getFollower();
        f.stop(); f.disableControllers(); startPose = f.getLocalizer().getRawPose();
        lastHeading = startPose.getHeading().getRad(); travel = previousPower = lastStationaryPower = 0;
        timing.reset(f.getLocalizer().getLastMeasurementTimeNanos());
        previousVelocity = automaticVelocity();
        resetWindow();
        if (csv == null) {
            csv = TuningCsvWriter.openBuffered("sysid_" + axis.name().toLowerCase(java.util.Locale.US),
                    "record", "test", "time_s", "velocity", "acceleration", "applied_power", "included");
            csvPath = csv.getPath();
        }
        timer.reset(); changeStage(Stage.RUNNING);
        commandAutomatic(0);
    }
    private double measuredVelocity() {
        Follower f = context.getFollower();
        return axis == Axis.HEADING ? f.getVelocity().getHeading(AngleUnit.RAD)
                : f.getVelocity().getVec().rotate(f.getPose().getHeading().times(-1)).getX().getIn();
    }

    /** Uses native measurements so navigation-filter lag does not shift the fitted line. */
    private double automaticVelocity() {
        Follower follower = context.getFollower();
        Pose velocity = follower.getRawVelocity();
        return axis == Axis.HEADING ? velocity.getHeading(AngleUnit.RAD)
                : velocity.getVec().rotate(follower.getLocalizer().getRawPose().getHeading().times(-1)).getX().getIn();
    }
    private void resetWindow() {
        windowTime = windowVelocity = windowPower = 0; windowStartVelocity = previousVelocity;
    }
    private void sample() {
        context.getView().addData("Test", trialName());
        context.getView().addData("Direction", automaticDirection());
        Follower f = context.getFollower();
        if (!f.getLocalizer().isLastMeasurementValid()) {
            stopMotion();
            throw new IllegalStateException("Feedforward tuning requires valid localization");
        }
        long timeNanos = (long) timer.nanoseconds();
        double time = DeltaTime.seconds(timeNanos), v = automaticVelocity();
        double dt = timing.update(f.getLocalizer().getLastMeasurementTimeNanos());
        double heading = f.getLocalizer().getRawPose().getHeading().getRad();
        travel += Math.abs(Math.atan2(Math.sin(heading - lastHeading), Math.cos(heading - lastHeading)));
        lastHeading = heading;
        double distance = f.getLocalizer().getRawPose().getVec().minus(startPose.getVec()).getMag().getIn();
        if (!Double.isFinite(v) || !Double.isFinite(distance) || !Double.isFinite(travel)
                || !Double.isFinite(previousPower)) {
            stopMotion();
            csv.writeRow("invalid", trialName());
            csv.flush();
            changeStage(Stage.REVIEW);
            return;
        }
        // Integrate the PRECEDING delivered command over its measurement interval. Windows
        // reduce differentiation noise without mixing stationary friction into the moving model.
        double threshold = axis == Axis.DRIVE ? .5 : .05;
        if (Double.isNaN(breakawayPower[run]) && !timing.isTimeAnomalyDetected()) {
            breakawayPower[run] = breakawayCrossingPower(run, previousVelocity,
                    v, previousPower, lastStationaryPower, threshold);
            if (Math.abs(v) < threshold && previousPower * (run == 0 ? 1 : -1) >= 0) {
                // This command has completed an observed stationary interval. Do not save
                // the command from the later interval in which movement is first detected.
                lastStationaryPower = previousPower;
            }
        }
        boolean moving = Math.abs(v) >= threshold && Math.abs(previousVelocity) >= threshold
                && Math.signum(v) == Math.signum(previousVelocity) && !timing.isTimeAnomalyDetected();
        csv.writeRow("raw", trialName(), time, v, "", previousPower, moving);
        if (moving) {
            windowTime += dt; windowVelocity += .5 * (v + previousVelocity) * dt;
            windowPower += previousPower * dt;
            if (windowTime >= .1) {
                double meanV = windowVelocity / windowTime, a = (v - windowStartVelocity) / windowTime;
                double p = windowPower / windowTime;
                boolean included = regression.add(run, time, meanV, a, p);
                csv.writeRow("window", trialName(), time, meanV, a, p, included);
                previousVelocity = v; resetWindow();
            }
        } else { previousVelocity = v; resetWindow(); }
        previousVelocity = v;
        boolean limit = axis == Axis.DRIVE ? distance >= MAX_DRIVE_TRAVEL_IN : travel >= MAX_HEADING_TRAVEL_RAD;
        double maximumPower = Math.min(MAX_RAMP_POWER, f.getDrivetrain().getConstants().maxPower);
        if (limit || time >= maximumPower / RAMP_RATE) {
            stopMotion(); csv.flush();
            if (!Double.isFinite(breakawayPower[run])) {
                closeCsv();
                throw new IllegalStateException("No breakaway detected within the ramp limits; no fit saved.");
            }
            csv.writeRow("breakaway", trialName(), time, "", "", breakawayPower[run], true);
            if (++run == trialCount()) {
                closeCsv(); fit = QuasistaticFit.fit(regression, trialCount(), axis == Axis.DRIVE ? 2 : .2,
                        manualGain(2));
                writeFitCsv();
                renderDesktopGraphs();
                changeStage(Stage.REVIEW);
            } else { changeStage(Stage.READY); }
            return;
        }
        commandAutomatic(time);
    }

    /** Applies the requested duty and remembers the power delivered during the next interval. */
    private void commandAutomatic(double time) {
        Follower follower = context.getFollower();
        double command = trialPower(axis, run, time);
        follower.getDrivetrain().moveWithVectors(axis == Axis.DRIVE ? command : 0, 0,
                axis == Axis.HEADING ? command : 0);
        double delivered = TuningMeasurements.axisCommandPower(follower.getDrivetrain(),
                axis == Axis.DRIVE ? feedforward.MotionCharacterization.Axis.FORWARD
                        : feedforward.MotionCharacterization.Axis.ANGULAR);
        previousPower = Math.copySign(delivered, command);
    }
    static double breakawayCrossingPower(int run, double before, double after,
                                         double power, double stationaryPower, double velocityThreshold) {
        double direction = run % 2 == 0 ? 1.0 : -1.0;
        if (run < 0 || run >= 2 || !Double.isFinite(before) || !Double.isFinite(after)
                || !Double.isFinite(power) || !Double.isFinite(velocityThreshold)
                || !Double.isFinite(stationaryPower) || velocityThreshold <= 0
                || Math.abs(before) >= velocityThreshold
                || direction * after < velocityThreshold
                || direction * power <= 0 || direction * stationaryPower < 0
                || Math.abs(stationaryPower) > Math.abs(power)) {
            return Double.NaN;
        }
        return Math.abs(stationaryPower);
    }
    private boolean breakawayValid() {
        return Double.isFinite(breakawayPower[0]) && (trialCount() == 1 || Double.isFinite(breakawayPower[1]));
    }
    private double staticBreakawayPower() {
        // One direction can be inflated by a delayed localization sample. The smaller
        // measured launch is the conservative static term for the bidirectional PDS search.
        return trialCount() == 1 ? breakawayPower[0] : Math.min(breakawayPower[0], breakawayPower[1]);
    }
    private void stopMotion() { context.getFollower().stop(); context.getFollower().enableControllers(); }
    private void closeCsv() {
        if (csv != null) { csv.close(); csv = null; }
    }
    private void writeFitCsv() {
        TuningCsvWriter reviewed = TuningCsvWriter.open("sysid_fit_" + axis.name().toLowerCase(java.util.Locale.US),
                "test", "time_s", "velocity", "acceleration", "applied_power", "fitted_power", "residual", "kS", "kV", "kA", "r_squared", "valid");
        for (SysIdRegression.Sample s : regression.samples) {
            double predicted = fit.kS * Math.signum(s.velocity) + fit.kV * s.velocity;
            reviewed.writeRow(s.run, s.time, s.velocity, s.acceleration, s.power, predicted,
                    s.power - predicted, fit.kS, fit.kV, fit.kA, fit.rSquared, fit.valid);
        }
        reviewed.close();
        csvPath = reviewed.getError() == null ? reviewed.getPath() : "CSV error: " + reviewed.getError();
    }
    private void renderDesktopGraphs() {
        // XChart/AWT remain on the desktop test classpath, never in the Android APK.
        try {
            Class<?> renderer = Class.forName("setup.follower.sysid.SysIdGraphs");
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
            context.constants.translationalKV = fit.kV;
            context.constants.translationalCoeffs.s(staticBreakawayPower());
        } else {
            context.constants.angularFeedforwardKS = fit.kS;
            context.constants.angularKV = fit.kV;
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
            context.getView().addData("kS / kV / kA", number(manualGain(0)) + " / " + number(manualGain(1)) + " / " + number(manualGain(2)));
            reportManualErrors(); return;
        }
        if (fit == null) { return; }
        context.getView().addData("kS / kV", number(fit.kS) + " / " + number(fit.kV));
        context.getView().addData("Existing kA (unchanged)", number(manualGain(2)));
        context.getView().addData("R-squared / power RMS", number(fit.rSquared) + " / " + number(fit.rms));
    }
}
