package tuning.follower.tracking;

import follower.FollowerConstants;

import geometry.Angle;
import geometry.ArcPose;
import geometry.Dist;
import geometry.DistUnit;
import geometry.GeometryFactory;
import geometry.Pose;
import geometry.Vector;
import paths.heading.InterpolationStyle;
import paths.movements.Path;
import tuning.TuningCsvWriter;
import tuning.follower.TunerContext;
import tuning.follower.TuningPhase;

/** Measures profiled braking on the active drivetrain and edits its saved profile. */
public final class ProfiledTrackingPhase extends TuningPhase {
    private enum Setting {
        VELOCITY_GAIN, PROFILE_VELOCITY_POWER, CORNER_SPEED_SCALE
    }

    private static final double TRIAL_TIMEOUT_SECONDS = 25.0;
    private static final double BRAKING_ACCELERATION_THRESHOLD = -5.0;
    private static final double CORNER_CURVATURE_THRESHOLD = 0.005;
    private static final double ACCELERATION_THRESHOLD = 5.0;

    private static final class TrialSummary {
        final double brakingRms;
        final double brakingBias;
        final double cornerBias;
        final double accelerationRms;
        final int accelerationSamples;
        final int brakingSamples;
        final int cornerSamples;
        final double peakTarget;
        final double peakMeasured;
        final double rawRise;
        final double peakCrossTrack;
        final double peakHeadingErrorDeg;
        final double scoreInches;
        final int pulses;

        TrialSummary(double brakingRms, double brakingBias, double cornerBias,
                     double accelerationRms, int accelerationSamples, int brakingSamples,
                     int cornerSamples, double peakTarget,
                     double peakMeasured, double rawRise, double peakCrossTrack,
                     double peakHeadingErrorDeg, double scoreInches, int pulses) {
            this.brakingRms = brakingRms;
            this.brakingBias = brakingBias;
            this.cornerBias = cornerBias;
            this.accelerationRms = accelerationRms;
            this.accelerationSamples = accelerationSamples;
            this.brakingSamples = brakingSamples;
            this.cornerSamples = cornerSamples;
            this.peakTarget = peakTarget;
            this.peakMeasured = peakMeasured;
            this.rawRise = rawRise;
            this.peakCrossTrack = peakCrossTrack;
            this.peakHeadingErrorDeg = peakHeadingErrorDeg;
            this.scoreInches = scoreInches;
            this.pulses = pulses;
        }

        String qualityIssue() {
            if (accelerationSamples < 10 || brakingSamples < 10 || cornerSamples < 5) {
                return "Insufficient acceleration, braking, or corner samples";
            }
            if (accelerationRms > Math.max(4.0, 0.18 * peakTarget)) {
                return "Acceleration tracking failed: RMS " + accelerationRms + " in/s";
            }
            if (brakingRms > Math.max(4.0, 0.16 * peakTarget)) {
                return "Braking tracking failed: RMS " + brakingRms + " in/s";
            }
            if (peakCrossTrack > 2.0) {
                return "Cross-track tracking failed: peak " + peakCrossTrack + " in";
            }
            if (peakHeadingErrorDeg > 12.0) {
                return "Heading tracking failed: peak " + peakHeadingErrorDeg + " deg";
            }
            return null;
        }
    }

    private Setting selectedSetting = Setting.VELOCITY_GAIN;
    private Path activePath;
    private boolean courseMeasured;
    private boolean lastPowerOpposing;
    private int trial;
    private int brakingSamples;
    private int cornerSamples;
    private int accelerationSamples;
    private int pulseCount;
    private double squaredBrakingError;
    private double signedBrakingError;
    private double squaredAccelerationError;
    private double cornerError;
    private double peakTarget;
    private double peakMeasured;
    private double largestRawSpeedRise;
    private double peakCrossTrack;
    private double peakHeadingErrorDeg;
    private double peakPathT;
    private double scoreInches;
    private double previousRawSpeed = Double.NaN;
    private long previousSampleNanos;
    private long trialStartedNanos;
    private String status = "Ready";
    private TuningCsvWriter csv;
    private TrialSummary courseSummary;
    private Path referencePath;
    private Path forwardPath;
    private Path forwardReferencePath;
    private Path reverseReferencePath;
    private ProfiledTrackingOptimizer optimizer;
    private ProfiledTrackingOptimizer.Settings originalSettings;
    private boolean originallyTuned;

    public ProfiledTrackingPhase(TunerContext context) { super(context); }

    @Override protected String getPhaseName() { return "Trajectory Tracking"; }
    @Override protected String getTuningConstants() {
        return context.constants.motionCharacterization == null
                ? "Velocity tracking gain, profile velocity power, corner speed scale"
                : "Velocity tracking gain, corner speed scale";
    }
    @Override protected boolean manualTuneIsPossible() { return true; }
    @Override protected boolean autoTuneIsPossible() { return true; }

    @Override protected void showPreRunInstructions() {
        if (forwardPath != null) {
            context.getTelemetry().addLine("Each run retraces the same course in the opposite direction.");
            context.getTelemetry().addLine("Leave the robot at the last endpoint; if a run missed it, reposition there.");
            return;
        }
        context.getTelemetry().addLine("Place the robot at a corner of a clear 36 by 36 in area.");
        context.getTelemetry().addLine("Face along the edge; leave the area forward and to the right, plus stopping room.");
    }

    @Override protected void init() {
        if (!positive(context.constants.translationalKV)
                || !nonnegative(context.constants.translationalKA)
                || !nonnegative(context.constants.translationalFeedforwardKS)
                || !positive(context.constants.angularKV)
                || !nonnegative(context.constants.angularKA)
                || !nonnegative(context.constants.angularFeedforwardKS)
                || !positive(context.constants.translationalCoeffs.kP)
                || !positive(context.constants.angularCoeffs.kP)
                || !positive(context.constants.forwardVelLimitIn)
                || !positive(context.constants.forwardAccelLimitIn)
                || !positive(context.constants.angularVelLimitRad)
                || !positive(context.constants.angularAccelLimitRad)
                || (!tank() && (!positive(context.constants.strafeVelLimitIn)
                || !positive(context.constants.strafeAccelLimitIn)
                || !positive(context.constants.kCentripetal)))) {
            throw new IllegalStateException("Trajectory tracking requires accepted feedforward, "
                    + "heading/drive feedback, motion limits, and holonomic centripetal tuning.");
        }
        courseMeasured = false;
        courseSummary = null;
        originalSettings = ProfiledTrackingOptimizer.Settings.capture(context.constants, tank());
        originallyTuned = context.constants.profiledTrackingTuned;
        trial = 0;
        status = "Ready";
        activePath = null;
        referencePath = null;
        forwardPath = forwardReferencePath = reverseReferencePath = null;
        optimizer = manualMode ? null : new ProfiledTrackingOptimizer(context.constants,
                tank(), context.getFollower().getDrivetrain().getConstants().maxPower);
        context.constants.profiledTrackingTuned = false;
        increment = 0.01;
        csv = TuningCsvWriter.open("profiled_tracking", "trial", "course", "time_s",
                "path_t", "target_in_s", "reference_target_in_s", "measured_in_s", "raw_in_s", "target_accel_in_s2",
                "curvature_in_inv", "endpoint_blend", "feedforward_power", "velocity_feedback_power",
                "front_left_power", "front_right_power", "cross_track_error_in",
                "heading_error_rad",
                "velocity_gain", "legacy_overspeed_tolerance_in_s", "legacy_corner_tolerance_in_s",
                "pulse_power", "cooldown_cycles", "neutral_margin", "profile_velocity_power",
                "corner_speed_scale");
    }

    private static boolean positive(double value) { return Double.isFinite(value) && value > 0.0; }
    private static boolean nonnegative(double value) { return Double.isFinite(value) && value >= 0.0; }

    private boolean tank() { return !context.getFollower().getDrivetrain().isHolonomic(); }

    private double cornerTolerance() {
        return tank() ? context.constants.tankBrakingCornerToleranceIn
                : context.constants.brakingCornerToleranceIn;
    }

    private double pulsePower() {
        return tank() ? context.constants.tankBrakingPulsePower
                : context.constants.brakingPulsePower;
    }

    private void startTrial() {
        if (context.getFollower().getVelocity().getVec().getMag().getIn() > 1.0) {
            status = "Wait until stationary before starting";
            return;
        }
        if (forwardPath == null && Boolean.getBoolean("apex.simulation.unlockTunerPhases")) {
            GeometryFactory staging = new GeometryFactory().setDistUnit(DistUnit.IN);
            positionRobotForSimulation(staging.pose(-20.0, 20.0, 0.0));
        }
        if (!manualMode) { optimizer.candidate().apply(context.constants, tank()); }
        if (forwardPath == null) {
            Pose start = context.getFollower().getPose();
            GeometryFactory factory = new GeometryFactory(context.getFollower()).setDistUnit(DistUnit.IN);
            Pose corner = localArc(start, 36.0, 0.0, 7.0);
            Pose end = localPose(start, 36.0, -36.0, -90.0);
            forwardPath = tank()
                    ? factory.tankPath(start, corner, end)
                            .interpolateWith(InterpolationStyle.TANGENT_FORWARD).quickBuild()
                    : factory.holonomicPath(start, corner, end)
                            .interpolateWith(InterpolationStyle.TANGENT_OPTIMAL).quickBuild();
            forwardReferencePath = tank() ? factory.tankPath(start, corner, end)
                    .interpolateWith(InterpolationStyle.TANGENT_FORWARD).buildToMemory()
                    : factory.holonomicPath(start, corner, end)
                    .interpolateWith(InterpolationStyle.TANGENT_OPTIMAL).buildToMemory();
            // Build both references before any candidate changes the planning constants.
            reverseReferencePath = forwardReferencePath.reversed();
        }
        boolean reverse = trial % 2 != 0;
        activePath = reverse ? forwardPath.reversed() : forwardPath;
        referencePath = reverse ? reverseReferencePath : forwardReferencePath;
        trial++;
        brakingSamples = cornerSamples = accelerationSamples = pulseCount = 0;
        squaredBrakingError = signedBrakingError = squaredAccelerationError
                = cornerError = peakTarget = peakMeasured
                = largestRawSpeedRise = 0.0;
        peakCrossTrack = peakHeadingErrorDeg = 0.0;
        peakPathT = 0.0;
        scoreInches = 0.0;
        previousRawSpeed = Double.NaN;
        previousSampleNanos = 0L;
        lastPowerOpposing = false;
        trialStartedNanos = System.nanoTime();
        status = "Running single-corner path " + (reverse ? "in reverse" : "forward");
        context.getFollower().clearFailure();
        context.getFollower().follow(activePath);
    }

    private static Vector localVector(Pose origin, double x, double y) {
        return origin.getVec().plus(Vector.of(x, y, DistUnit.IN).rotate(origin.getHeading()));
    }

    private static Pose localPose(Pose origin, double x, double y, double headingDeg) {
        return new Pose(localVector(origin, x, y),
                origin.getHeading().plus(Angle.fromDeg(headingDeg)));
    }

    private static ArcPose localArc(Pose origin, double x, double y, double radiusIn) {
        return new ArcPose(localVector(origin, x, y), Dist.fromIn(radiusIn));
    }

    private void sample() {
        trajectory.TrajectorySample target = context.getFollower().getTrajectoryTarget();
        if (target == null) { return; }
        double t = target.pathT;
        if (!Double.isFinite(t) || t < 0.0 || t >= 1.0) { return; }
        peakPathT = Math.max(peakPathT, t);
        double traveled = target.progressIn;
        double targetSpeed = context.getFollower().getTrackingVelocityTarget();
        double referenceSpeed = referencePath.getTrajectoryField().sample(referencePath.getParametricPath()
                .getPosition(referencePath.getParametricPath().getTAtDistance(traveled))).tangentialVelocity();
        double measured = context.getFollower().getTrackingProgressVelocity();
        double raw = context.getFollower().getRawVelocity().getVec().dot(target.tangent).getIn();
        double curvature = activePath.getParametricPath().getSignedCurvature(t);
        double headingError = context.getFollower().getPose().getHeading().getShortestAngleTo(target.heading).getRad();
        double blend = context.getFollower().getTrackingEndpointBlend();
        double feedforward = context.getFollower().getTrackingFeedforward();
        double elapsed = (System.nanoTime() - trialStartedNanos) * 1e-9;
        if (csv != null) {
            csv.writeRow(trial, "SINGLE_CORNER", elapsed, t, targetSpeed, referenceSpeed, measured, raw,
                    target.tangentialAcceleration(), curvature, blend, feedforward,
                    context.getFollower().getTrackingVelocityFeedback(),
                    context.getFollower().getDrivetrain().getLastFlPower(),
                    context.getFollower().getDrivetrain().getLastFrPower(),
                    context.getFollower().getCrossTrackErrorIn(),
                    headingError,
                    context.constants.velocityFeedbackGain,
                    context.constants.brakingOverspeedToleranceIn, cornerTolerance(), pulsePower(),
                    context.constants.brakingPulseCooldownCycles,
                    context.constants.brakingNeutralPowerMargin,
                    context.constants.profileVelocityPowerCoefficient,
                    context.constants.profileCornerSpeedScale);
        }
        if (blend >= 0.05 || !Double.isFinite(measured) || !Double.isFinite(targetSpeed)) {
            return;
        }
        long now = System.nanoTime();
        double dt = previousSampleNanos == 0L ? 0.0 : (now - previousSampleNanos) * 1e-9;
        previousSampleNanos = now;
        if (dt > 0.0 && dt <= 0.1 && Double.isFinite(referenceSpeed)) {
            // Integrating speed error yields inches. Overspeed costs four times underspeed;
            // the reference LUT stays fixed so a slower proposal cannot win by moving its target.
            double referenceError = measured - referenceSpeed;
            scoreInches += dt * (Math.abs(referenceError)
                    + 3.0 * Math.max(0.0, referenceError));
        }
        peakTarget = Math.max(peakTarget, Math.abs(targetSpeed));
        peakMeasured = Math.max(peakMeasured, measured);
        peakCrossTrack = Math.max(peakCrossTrack,
                Math.abs(context.getFollower().getCrossTrackErrorIn()));
        peakHeadingErrorDeg = Math.max(peakHeadingErrorDeg,
                Math.abs(Math.toDegrees(headingError)));
        if (target.tangentialAcceleration() > ACCELERATION_THRESHOLD) {
            double error = measured - targetSpeed;
            squaredAccelerationError += error * error;
            accelerationSamples++;
        }
        boolean opposing = feedforward < -1e-6;
        if (opposing && !lastPowerOpposing) { pulseCount++; }
        lastPowerOpposing = opposing;
        if (target.tangentialAcceleration() < BRAKING_ACCELERATION_THRESHOLD) {
            double error = measured - Math.abs(targetSpeed);
            squaredBrakingError += error * error;
            signedBrakingError += error;
            brakingSamples++;
            if (Math.abs(curvature) > CORNER_CURVATURE_THRESHOLD) {
                cornerError += error;
                cornerSamples++;
            }
            if (Double.isFinite(previousRawSpeed)) {
                largestRawSpeedRise = Math.max(largestRawSpeedRise, raw - previousRawSpeed);
            }
            previousRawSpeed = raw;
        } else {
            previousRawSpeed = Double.NaN;
        }
    }

    private void finishTrial(boolean timedOut) {
        Pose actualEnd = context.getFollower().getPose();
        double endpointPositionError = actualEnd.getVec()
                .minus(activePath.getEndPose().getVec()).getMag().getIn();
        double endpointHeadingError = Math.abs(actualEnd.getHeading()
                .getShortestAngleTo(activePath.getEndPose().getHeading()).getDeg());
        context.getFollower().stop();
        activePath = null;
        if (csv != null) { csv.flush(); }
        if (timedOut || peakPathT < 0.90 || brakingSamples == 0 || !Double.isFinite(scoreInches)
                || scoreInches <= 0.0 || cornerSamples == 0
                || endpointPositionError > 3.0 || endpointHeadingError > 5.0) {
            if (!manualMode && optimizer.canRejectCurrentCandidate()) {
                optimizer.rejectCurrentCandidate();
                status = "Unusable candidate trial; trying next setting";
                return;
            }
            status = timedOut ? "Trial timed out; reposition and retry"
                    : endpointPositionError > 3.0 || endpointHeadingError > 5.0
                    ? "Endpoint missed: " + number(endpointPositionError) + " in / "
                            + number(endpointHeadingError) + " deg; verify earlier tuning phases"
                    : "No usable braking/corner samples; check limits and localization";
            return;
        }
        TrialSummary summary = new TrialSummary(Math.sqrt(squaredBrakingError / brakingSamples),
                signedBrakingError / brakingSamples,
                cornerSamples == 0 ? Double.NaN : cornerError / cornerSamples,
                accelerationSamples == 0 ? Double.POSITIVE_INFINITY
                        : Math.sqrt(squaredAccelerationError / accelerationSamples),
                accelerationSamples, brakingSamples, cornerSamples,
                peakTarget, peakMeasured, largestRawSpeedRise,
                peakCrossTrack, peakHeadingErrorDeg,
                scoreInches + peakCrossTrack
                        + 7.0 * Math.toRadians(peakHeadingErrorDeg), pulseCount);
        courseSummary = summary;
        String qualityIssue = summary.qualityIssue();
        if (qualityIssue != null) {
            courseMeasured = false;
            status = qualityIssue + "; verify earlier tuning phases";
            if (!manualMode && optimizer.canRejectCurrentCandidate()) {
                optimizer.rejectCurrentCandidate();
            } else if (!manualMode && optimizer.isValidating()) {
                optimizer.rejectValidation();
            }
            return;
        }
        courseMeasured = true;
        status = "Single-corner path measured";
        if (!manualMode) {
            optimizer.record(summary.scoreInches, summary.cornerBias);
            status = optimizer.status();
        }
    }

    private void updateTrial() {
        if (activePath == null) { return; }
        if (context.getFollower().isBusy()) { sample(); }
        if (!context.getFollower().isBusy()
                || (System.nanoTime() - trialStartedNanos) * 1e-9 > TRIAL_TIMEOUT_SECONDS) {
            finishTrial(context.getFollower().isBusy());
        }
    }

    @Override protected boolean autoTuned() {
        updateTrial();
        if (optimizer.complete()) {
            optimizer.best().apply(context.constants, tank());
            accept();
            return true;
        }
        if (activePath == null) {
            if (!courseMeasured && trial > 0) { context.getTelemetry().addLine(status); }
            showPreRunInstructions();
            context.getTelemetry().addLine("Press A when positioned and stationary.");
            if (opMode.gamepad1.aWasPressed()) { startTrial(); }
        }
        return false;
    }

    @Override protected boolean manualTuned() {
        if (activePath != null) {
            updateTrial();
        } else {
            if (opMode.gamepad1.leftBumperWasPressed()) {
                selectSetting(-1);
            } else if (opMode.gamepad1.rightBumperWasPressed()) {
                selectSetting(1);
            }
            double change = manualChange();
            if (change != 0.0) { changeSetting(selectedSetting, change); }
            if (opMode.gamepad1.xWasPressed()) { startTrial(); }
            if (opMode.gamepad1.aWasPressed() && courseMeasured) {
                accept();
                return true;
            }
        }
        reportProgress();
        context.getTelemetry().addData("Step", number(increment));
        for (Setting setting : settings()) {
            addTunableValue(label(setting), value(setting), setting == selectedSetting);
        }
        context.getTelemetry().addLine("LB/RB select; dpad L/R step; dpad Up/Down edit");
        context.getTelemetry().addLine("X run; A save after path passes");

        return false;
    }

    private Setting[] settings() {
        return context.constants.motionCharacterization == null ? Setting.values()
                : new Setting[]{Setting.VELOCITY_GAIN, Setting.CORNER_SPEED_SCALE};
    }

    private void selectSetting(int direction) {
        do {
            selectedSetting = Setting.values()[(selectedSetting.ordinal() + direction
                    + Setting.values().length) % Setting.values().length];
        } while (context.constants.motionCharacterization != null && selectedSetting == Setting.PROFILE_VELOCITY_POWER);
    }

    private void reportProgress() {
        context.getTelemetry().addData("Profile", context.constants.getActiveProfile());
        if (context.constants.motionCharacterization == null) {
            context.getTelemetry().addData("Effective profile velocity power per in/s",
                    number(context.constants.profileVelocityPowerCoefficient > 0.0
                            ? context.constants.profileVelocityPowerCoefficient : context.constants.translationalKV));
        }
        context.getTelemetry().addData("Course", "single corner, tangent heading");
        context.getTelemetry().addData("State", status);
        context.getTelemetry().addData("Measured", courseMeasured);
        if (activePath != null && brakingSamples > 0) {
            context.getTelemetry().addData("Current braking RMS in/s",
                    number(Math.sqrt(squaredBrakingError / brakingSamples)));
        }
        reportSummary("Single corner", courseSummary);
        context.getTelemetry().addData("CSV", csv == null ? "Unavailable" : csv.getPath());
        if (activePath == null) {
            context.getTelemetry().addLine(courseMeasured
                    ? "A: save measured path" : "X: run single-corner path");
        }
    }

    private void reportSummary(String name, TrialSummary summary) {
        if (summary == null) { return; }
        context.getTelemetry().addData(name + " braking RMS in/s", number(summary.brakingRms));
        context.getTelemetry().addData(name + " acceleration RMS in/s", number(summary.accelerationRms));
        context.getTelemetry().addData(name + " braking bias in/s", number(summary.brakingBias));
        if (Double.isFinite(summary.cornerBias)) {
            context.getTelemetry().addData(name + " curved braking bias in/s",
                    number(summary.cornerBias));
        }
        context.getTelemetry().addData(name + " peak target / measured in/s",
                number(summary.peakTarget) + " / " + number(summary.peakMeasured));
        context.getTelemetry().addData(name + " opposing commands", summary.pulses);
        context.getTelemetry().addData(name + " max raw rise/sample in/s", number(summary.rawRise));
        context.getTelemetry().addData(name + " peak cross-track / heading",
                number(summary.peakCrossTrack) + " in / "
                        + number(summary.peakHeadingErrorDeg) + " deg");
        context.getTelemetry().addData(name + " selection score", number(summary.scoreInches) + " in");
    }

    private void accept() {
        context.constants.profiledTrackingTuned = true;
    }

    @Override protected boolean routineMotionActive() { return activePath != null; }
    @Override protected void onExit() {
        if (originalSettings != null && !context.constants.profiledTrackingTuned) {
            originalSettings.apply(context.constants, tank());
            context.constants.profiledTrackingTuned = originallyTuned;
        }
        if (csv != null) { csv.close(); }
    }
    @Override protected void reportResults() {
        for (Setting setting : settings()) {
            context.getTelemetry().addData(label(setting), number(value(setting)));
        }
    }

    static String label(Setting setting) {
        switch (setting) {
            case VELOCITY_GAIN: return "Velocity tracking gain (power per in/s)";
            case PROFILE_VELOCITY_POWER: return "Profile velocity power (power per in/s)";
            case CORNER_SPEED_SCALE: return "Corner speed scale";
            default: throw new AssertionError(setting);
        }
    }

    private double value(Setting setting) {
        switch (setting) {
            case VELOCITY_GAIN: return context.constants.velocityFeedbackGain;
            case PROFILE_VELOCITY_POWER: return context.constants.profileVelocityPowerCoefficient;
            case CORNER_SPEED_SCALE: return context.constants.profileCornerSpeedScale;
            default: throw new AssertionError(setting);
        }
    }

    private void changeSetting(Setting setting, double change) {
        switch (setting) {
            case VELOCITY_GAIN:
                context.constants.velocityFeedbackGain = Math.max(0.0,
                        Math.min(FollowerConstants.MAX_TRANSLATION_VELOCITY_FEEDBACK_GAIN,
                                context.constants.velocityFeedbackGain + change));
                break;
            case PROFILE_VELOCITY_POWER:
                context.constants.profileVelocityPowerCoefficient = Math.max(0.0,
                        context.constants.profileVelocityPowerCoefficient + change);
                break;
            case CORNER_SPEED_SCALE:
                context.constants.profileCornerSpeedScale = unit(
                        context.constants.profileCornerSpeedScale + change);
                break;
            default: throw new AssertionError(setting);
        }
        // A saved measurement no longer verifies a changed controller/profile setting.
        courseMeasured = false;
        courseSummary = null;
        context.constants.profiledTrackingTuned = false;
        status = "Value changed; rerun single-corner path";
    }

    private static double unit(double value) { return Math.max(0.0, Math.min(1.0, value)); }
}
