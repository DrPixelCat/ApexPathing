package org.firstinspires.ftc.teamcode.apexpathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;
import java.util.Arrays;

import storage.ApexStorage;
import config.ApexConstants;
import follower.Follower;
import follower.FollowerConstants;
import trajectory.profile.MotionParameters;
import geometry.GeometryFactory;
import geometry.AngleUnit;
import geometry.DistUnit;
import geometry.PathSegment;
import geometry.Pose;
import geometry.Vector;
import paths.movements.Path;
import paths.heading.InterpolationStyle;

/**
 * Single-path autonomous test with explicit tank and holonomic geometry. Make sure the
 * robot has been tuned with the {@link FollowerTuner} before running this OpMode. This OpMode will
 * follow the initial curve and stop. The holonomic path twists through custom heading nodes.
 *
 * @author Sohum Arora - 22985 Paraducks
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
@Autonomous(name = "Apex Auto Test", group = "Apex Pathing")
public class AutoTest extends LinearOpMode {
    private static final double STAGE_TIMEOUT_SECONDS = 50.0;
    private static final double POSITION_TOLERANCE_INCHES = 3.0;
    private static final double HEADING_TOLERANCE_DEGREES = 5.0;
    private static final double TELEMETRY_INTERVAL_SECONDS = 0.10;
    private static final double VELOCITY_LOG_INTERVAL_SECONDS = 0.02;
    private static final long CONTROL_LOOP_NANOS = 20_000_000L;
    // This is a coarse stress-test curve, not the endpoint acceptance tolerance. Leave enough
    // margin for the measured transient (~7.2 in in FTCodeSim) while still catching gross drift.
    private static final double CROSS_TRACK_TOLERANCE_INCHES = 15;

    AutoRoute path;
    AutoState currentState = AutoState.OUTBOUND_CURVE;
    private final ElapsedTime stageTimer = new ElapsedTime();
    private final ElapsedTime telemetryTimer = new ElapsedTime();
    private String failureReason = "None";
    private int passedStages;
    private double lastPositionError;
    private double lastHeadingError;
    private double maximumCrossTrackError;
    private double lastMaximumCrossTrackError;
    private FileWriter outboundVelocityCsv;
    private String outboundVelocityCsvPath = "Not started";
    private String outboundVelocityCsvError;
    private int outboundVelocityRowsSinceFlush;
    private double lastOutboundVelocityLogSeconds = Double.NEGATIVE_INFINITY;
    private static final String[] DEMAND_NAMES = {"cross-track", "tangent correction",
            "heading correction", "centripetal", "forward velocity", "heading velocity",
            "drive feedforward", "heading feedforward"};
    private final double[] peakDemand = new double[DEMAND_NAMES.length];
    private final int[] dominantSaturationFrames = new int[DEMAND_NAMES.length];
    private long demandSamples;
    private long saturatedDemandSamples;
    private double squaredTotalDemand;
    private double peakTotalDemand;
    private final double[] stagePeakDemand = new double[3];
    private final double[] stageSquaredDemand = new double[3];
    private final long[] stageSaturatedSamples = new long[3];
    private double movementDuration;
    private double firstToleranceTime = Double.NaN;

    enum AutoState {
        OUTBOUND_CURVE,
        COMPLETE,
        FAILED
    }

    @Override
    public void runOpMode() {
        resetRunState();
        Follower follower = Follower.create(createConstants(), hardwareMap);
        try {
            configureFollower(follower);
            if (useTankPath()) {
                FollowerConstants constants = follower.getConstants().forProfile(FollowerConstants.Profile.TANK);
                if (!positive(constants.forwardVelLimitIn) || !positive(constants.forwardAccelLimitIn)
                        || !positive(constants.angularVelLimitRad) || !positive(constants.angularAccelLimitRad)) {
                    fail(follower, "Tank follower constants are incomplete. Run localization and follower tuning first.");
                    telemetry.addLine(failureReason);
                    telemetry.update();
                    return;
                }
            }
            path = useTankPath() ? buildTankPath() : buildHolonomicPath();

            telemetry.addLine(useTankPath()
                    ? "Apex tank test: one curve."
                    : "Apex twist test: one curve, headings 0 -> 83 -> -71 -> 146 deg.");
            telemetry.addLine("Press Start to begin");
            telemetry.update();

            waitForStart();
            if (!opModeIsActive()) { return; }

            follower.setPose(Pose.zero());
            openOutboundVelocityCsv();
            startPath(follower);

            while (opModeIsActive()) {
                long loopStartedNanos = System.nanoTime();
                follower.update();
                sampleCommandDemand(follower);
                logOutboundVelocitySample(follower);
                Pose pose = follower.getPose();
                recordFirstToleranceEntry(pose);
                maximumCrossTrackError = Math.max(maximumCrossTrackError,
                        Math.abs(follower.getCrossTrackErrorIn()));

                if (!isTerminal(currentState)) {
                    if (stageTimer.seconds() > STAGE_TIMEOUT_SECONDS) {
                        fail(follower, "Stage exceeded " + STAGE_TIMEOUT_SECONDS + " seconds");
                    } else if (!follower.isBusy()) {
                        finishStage(follower, pose);
                    }
                }

                if (telemetryTimer.seconds() >= TELEMETRY_INTERVAL_SECONDS ||
                        isTerminal(currentState)) {
                    if (currentState == AutoState.COMPLETE) {
                        telemetry.addLine("PASS: Apex path check completed.");
                    } else if (currentState == AutoState.FAILED) {
                        telemetry.addLine("FAIL: " + failureReason);
                    }

                    telemetry.addData("Current check", currentState);
                    telemetry.addData("Passed checks", passedStages + " / 1");
                    telemetry.addData("Stage time (s)", stageTimer.seconds());
                    telemetry.addData("Follower busy", follower.isBusy());
                    telemetry.addData("Callback state", path.callbackMessage);
                    telemetry.addData("Last endpoint position error (in)", lastPositionError);
                    telemetry.addData("Last endpoint heading error (deg)", lastHeadingError);
                    telemetry.addData("Current maximum cross-track error (in)",
                            maximumCrossTrackError);
                    telemetry.addData("Last maximum cross-track error (in)",
                            lastMaximumCrossTrackError);
                    telemetry.addData("Outbound velocity CSV", outboundVelocityCsvPath);
                    telemetry.addData("Command demand", getCommandDemandReport());
                    if (outboundVelocityCsvError != null) {
                        telemetry.addData("Velocity CSV warning", outboundVelocityCsvError);
                    }
                    telemetry.addData("X (in)", pose.getX().getIn());
                    telemetry.addData("Y (in)", pose.getY().getIn());
                    telemetry.addData("Heading (deg)", pose.getHeading().getDeg());
                    telemetry.update();
                    telemetryTimer.reset();
                }
                if (isTerminal(currentState)) { break; }
                waitForNextControlLoop(loopStartedNanos);
            }

        } finally {
            closeOutboundVelocityCsv();
            follower.stop();
        }
    }

    /** Selects hardware while keeping all autonomous execution in this class. */
    protected ApexConstants createConstants() { return new Constants(); }
    /** Optional per-run follower configuration, also used by desktop diagnostics. */
    protected void configureFollower(Follower follower) { }

    /** Explicit route selection; a dual-actuated drive's current mode does not choose the path. */
    protected boolean useTankPath() { return false; }

    private AutoRoute buildTankPath() {
        GeometryFactory factory = new GeometryFactory()
                .setDistUnit(DistUnit.IN).setAngleUnit(AngleUnit.DEG);
        AutoRoute route = new AutoRoute();
        route.testPath = factory.tankPath(Pose.zero(),
                        factory.arcPose(30, 0, 7), factory.arcPose(30, -30, 7),
                        factory.arcPose(-30, -30, 7), factory.arcPose(-30, 30, 7),
                        factory.pose(30, 30, 0))
                .interpolateWith(InterpolationStyle.TANGENT_FORWARD)
                .addDistanceCallback(.5, route::outboundCallback).quickBuild();
        return route;
    }

    private AutoRoute buildHolonomicPath() {
        GeometryFactory factory = new GeometryFactory()
                .setDistUnit(DistUnit.IN).setAngleUnit(AngleUnit.DEG);
        AutoRoute route = new AutoRoute();
        route.testPath = factory.holonomicPath(Pose.zero(),
                        factory.arcPose(30, 0, 7), factory.arcPose(30, -30, 7),
                        factory.arcPose(-30, -30, 7), factory.arcPose(-30, 30, 7),
                        factory.pose(30, 30, 146))
                // Fixed arbitrary headings make the twisting stress test repeatable.
                .addHeadingNode(.30, factory.angle(83))
                .addHeadingNode(.65, factory.angle(-71))
                .addDistanceCallback(.5, route::outboundCallback).buildToMemory();
        return route;
    }

    static final class AutoRoute {
        Path testPath;
        String callbackMessage = "Callback not triggered yet";
        boolean outboundCallbackTriggered;

        void outboundCallback() {
            outboundCallbackTriggered = true;
            callbackMessage = "Outbound distance callback triggered!";
        }
    }

    private static boolean positive(double value) {
        return Double.isFinite(value) && value > 0.0;
    }

    public int getPassedStages() { return passedStages; }

    public String getResult() {
        if (currentState == AutoState.COMPLETE) { return "PASS"; }
        if (currentState == AutoState.FAILED) { return "FAIL " + failureReason; }
        return "NOT COMPLETE";
    }

    private void finishStage(Follower follower, Pose actualPose) {
        if (follower.getMovementOutcome() == Follower.MovementOutcome.FAILED) {
            fail(follower, follower.getTrajectoryFailure());
            return;
        }
        movementDuration = stageTimer.seconds();
        closeOutboundVelocityCsv();
        Pose expectedPose = path.testPath.getEndPose();
        lastPositionError = actualPose.distanceTo(expectedPose).getIn();
        lastHeadingError = Math.abs(actualPose.getHeading()
                .getShortestAngleTo(expectedPose.getHeading()).getDeg());
        lastMaximumCrossTrackError = maximumCrossTrackError;

        if (!Double.isFinite(lastPositionError) || !Double.isFinite(lastHeadingError) ||
                !Double.isFinite(maximumCrossTrackError)) {
            fail(follower, "Non-finite localization/tracking result in " + currentState);
            return;
        }
        if (lastPositionError > POSITION_TOLERANCE_INCHES ||
                lastHeadingError > HEADING_TOLERANCE_DEGREES) {
            fail(follower, "Endpoint tolerance missed in " + currentState +
                    ": position=" + lastPositionError + " in, heading=" +
                    lastHeadingError + " deg");
            return;
        }
        if (maximumCrossTrackError > CROSS_TRACK_TOLERANCE_INCHES) {
            fail(follower, "Cross-track error exceeded " + CROSS_TRACK_TOLERANCE_INCHES +
                    " inches in " + currentState + ": " + maximumCrossTrackError + " in");
            return;
        }
        if (!path.outboundCallbackTriggered) {
            fail(follower, "Expected callback did not run in " + currentState);
            return;
        }

        passedStages++;
        currentState = nextState(currentState);
        follower.stop();
    }

    private void startPath(Follower follower) {
        maximumCrossTrackError = 0.0;
        stageTimer.reset();
        follower.follow(path.testPath);
    }

    private void fail(Follower follower, String reason) {
        closeOutboundVelocityCsv();
        follower.stop();
        failureReason = reason;
        currentState = AutoState.FAILED;
    }

    static AutoState nextState(AutoState state) {
        switch (state) {
            case OUTBOUND_CURVE: return AutoState.COMPLETE;
            default: return state;
        }
    }

    private static boolean isTerminal(AutoState state) {
        return state == AutoState.COMPLETE || state == AutoState.FAILED;
    }

    /** Exposes the initial curve for simulation verification and diagnostics. */
    public Path getOutboundPath() { return path == null ? null : path.testPath; }

    public String getOutboundVelocityCsvPath() { return outboundVelocityCsvPath; }

    public String getOutboundVelocityCsvError() { return outboundVelocityCsvError; }

    /** Returns the elapsed motion time for the single completed path. */
    public double getTotalMovementTimeSeconds() {
        return movementDuration;
    }

    /** Yields to hardware while maintaining a complete 20 ms control-loop period. */
    private void waitForNextControlLoop(long loopStartedNanos) {
        long deadline = loopStartedNanos + CONTROL_LOOP_NANOS;
        if (deadline - System.nanoTime() > 6_000_000L) { sleep(5); }
        while (opModeIsActive() && System.nanoTime() < deadline) { Thread.yield(); }
    }

    /** Returns the average active follower update period in milliseconds. */
    public double getAverageLoopMilliseconds() {
        return demandSamples == 0 ? 0.0 :
                1000.0 * getTotalMovementTimeSeconds() / demandSamples;
    }

    /** Returns a concise timing breakdown for performance regression tests. */
    public String getMovementTimingReport() {
        return String.format(Locale.US,
                "outbound=%.3fs(+%.3fs settle), total=%.3fs",
                movementDuration, settlingDelay(),
                getTotalMovementTimeSeconds());
    }

    private double settlingDelay() {
        return Double.isFinite(firstToleranceTime)
                ? Math.max(0.0, movementDuration - firstToleranceTime)
                : movementDuration;
    }

    private void recordFirstToleranceEntry(Pose actualPose) {
        if (isTerminal(currentState) || Double.isFinite(firstToleranceTime)) {
            return;
        }
        Pose expected = path.testPath.getEndPose();
        double positionError = actualPose.distanceTo(expected).getIn();
        double headingError = Math.abs(actualPose.getHeading()
                .getShortestAngleTo(expected.getHeading()).getDeg());
        if (positionError <= 1.0 && headingError <= 2.0) {
            firstToleranceTime = stageTimer.seconds();
        }
    }

    /** Summarizes raw controller demand before command normalization. */
    public String getCommandDemandReport() {
        double rms = demandSamples == 0 ? 0.0 :
                Math.sqrt(squaredTotalDemand / demandSamples);
        StringBuilder report = new StringBuilder(String.format(Locale.US,
                "samples=%d, saturated=%.1f%%, total peak=%.3f, total RMS=%.3f",
                demandSamples, demandSamples == 0 ? 0.0 :
                        100.0 * saturatedDemandSamples / demandSamples,
                peakTotalDemand, rms));
        for (int i = 0; i < DEMAND_NAMES.length; i++) {
            report.append(String.format(Locale.US, ", %s peak=%.3f, dominant=%d",
                    DEMAND_NAMES[i], peakDemand[i], dominantSaturationFrames[i]));
        }
        String[] stageNames = {"corrective", "velocity", "feedforward"};
        for (int i = 0; i < stageNames.length; i++) {
            double stageRms = demandSamples == 0 ? 0.0 :
                    Math.sqrt(stageSquaredDemand[i] / demandSamples);
            report.append(String.format(Locale.US,
                    ", %s stage peak=%.3f RMS=%.3f over-capacity=%.1f%%",
                    stageNames[i], stagePeakDemand[i], stageRms,
                    demandSamples == 0 ? 0.0 :
                            100.0 * stageSaturatedSamples[i] / demandSamples));
        }
        return report.toString();
    }

    private void sampleCommandDemand(Follower follower) {
        Follower.CommandDemand demand = follower.getLastCommandDemand();
        if (!demand.available) { return; }
        double[] components = {demand.crossTrack, demand.tangentCorrection,
                demand.headingCorrection, demand.centripetal, demand.forwardVelocity,
                demand.headingVelocity, demand.driveFeedforward, demand.headingFeedforward};
        double[] stages = {demand.correctiveTotal, demand.velocityTotal,
                demand.feedforwardTotal};
        demandSamples++;
        squaredTotalDemand += demand.total * demand.total;
        peakTotalDemand = Math.max(peakTotalDemand, demand.total);
        int dominant = 0;
        for (int i = 0; i < components.length; i++) {
            peakDemand[i] = Math.max(peakDemand[i], components[i]);
            if (components[i] > components[dominant]) { dominant = i; }
        }
        if (demand.total > 1.0) {
            saturatedDemandSamples++;
            dominantSaturationFrames[dominant]++;
        }
        for (int i = 0; i < stages.length; i++) {
            stagePeakDemand[i] = Math.max(stagePeakDemand[i], stages[i]);
            stageSquaredDemand[i] += stages[i] * stages[i];
            if (stages[i] > 1.0) { stageSaturatedSamples[i]++; }
        }
    }

    private void resetRunState() {
        closeOutboundVelocityCsv();
        currentState = AutoState.OUTBOUND_CURVE;
        failureReason = "None";
        passedStages = 0;
        lastPositionError = lastHeadingError = maximumCrossTrackError =
                lastMaximumCrossTrackError = 0.0;
        outboundVelocityCsvPath = "Not started";
        outboundVelocityCsvError = null;
        outboundVelocityRowsSinceFlush = 0;
        lastOutboundVelocityLogSeconds = Double.NEGATIVE_INFINITY;
        telemetryTimer.reset();
        demandSamples = saturatedDemandSamples = 0;
        squaredTotalDemand = peakTotalDemand = 0.0;
        Arrays.fill(peakDemand, 0.0);
        Arrays.fill(dominantSaturationFrames, 0);
        Arrays.fill(stagePeakDemand, 0.0);
        Arrays.fill(stageSquaredDemand, 0.0);
        Arrays.fill(stageSaturatedSamples, 0);
        movementDuration = 0.0;
        firstToleranceTime = Double.NaN;
    }

    private void openOutboundVelocityCsv() {
        try {
            File directory = ApexStorage.getDirectory();
            if (!directory.exists() && !directory.mkdirs()) {
                throw new IOException("Could not create " + directory.getAbsolutePath());
            }
            String timestamp = new SimpleDateFormat(
                    "yyyyMMdd_HHmmss_SSS", Locale.US).format(new Date());
            File file = new File(directory,
                    "auto-test-outbound-velocity_" + timestamp + ".csv");
            outboundVelocityCsv = new FileWriter(file);
            outboundVelocityCsvPath = file.getAbsolutePath();
            outboundVelocityCsv.write(
                    "elapsed_s,path_distance_in,target_velocity_in_s,raw_velocity_in_s," +
                            "kalman_velocity_in_s,target_accel_in_s2,target_omega_rad_s," +
                            "target_alpha_rad_s2,profile_power,cross_track_error_in," +
                            "curvature_in_inv,target_heading_rad,heading_error_rad," +
                            "cross_track_power,tangent_correction_power,heading_correction_power," +
                            "centripetal_power,forward_velocity_power,heading_velocity_power," +
                            "drive_feedforward_power,heading_feedforward_power,total_demand," +
                            "command_power,saturated,controller_target_in_s,controller_measured_in_s," +
                            "progress_velocity_in_s,signed_translation_ff,signed_velocity_feedback," +
                            "left_front_command,right_front_command,endpoint_blend," +
                            "actual_omega_rad_s,correction_scale,feedforward_scale,velocity_scale," +
                            "allocated_drive_ff_magnitude,allocated_heading_ff_magnitude," +
                            "remaining_after_correction,remaining_after_feedforward," +
                            "remaining_after_velocity,expected_motor_power,velocity_goal_distance_in,current_speed_ceiling_in_s," +
                            "centered_braking_active,centered_braking_mean_power\n");
        } catch (IOException e) {
            outboundVelocityCsv = null;
            outboundVelocityCsvPath = "Unavailable";
            outboundVelocityCsvError = e.getMessage();
        }
    }

    private void logOutboundVelocitySample(Follower follower) {
        if (outboundVelocityCsv == null || currentState != AutoState.OUTBOUND_CURVE) { return; }
        double elapsed = stageTimer.seconds();
        // Always retain the terminal measurement, even if the last periodic sample was
        // less than 20 ms ago; otherwise the CSV can report a pre-completion speed.
        if (follower.isBusy()
                && elapsed - lastOutboundVelocityLogSeconds < VELOCITY_LOG_INTERVAL_SECONDS) { return; }
        lastOutboundVelocityLogSeconds = elapsed;

        PathSegment segment = path.testPath.getParametricPath();
        trajectory.TrajectorySample sample = follower.getTrajectoryTarget();
        if (sample == null) { return; }
        double t = sample.pathT;
        double traveled = sample.progressIn;
        MotionParameters target = new MotionParameters(sample.tangentialVelocity(), sample.tangentialAcceleration(),
                sample.angularVelocity, sample.angularAcceleration, sample.progressIn);
        target.setMotorPower(follower.getLastCommandDemand().feedforwardTotal);
        Vector tangent = sample.tangent;
        double rawVelocity = follower.getRawVelocity().getVec().dot(tangent).getIn();
        double kalmanVelocity = follower.getVelocity().getVec().dot(tangent).getIn();
        double targetHeading = sample.heading.getRad();
        double headingError = follower.getPose().getHeading().getShortestAngleTo(
                geometry.Angle.fromRad(targetHeading)).getRad();
        double crossTrackError = follower.getCrossTrackErrorIn();
        double curvature = segment.getSignedCurvature(t);
        Follower.CommandDemand demand = follower.getLastCommandDemand();
        double commandPower = follower.getDrivetrain().getMaxMotorPower();

        try {
            outboundVelocityCsv.write(String.format(
                    Locale.US,
                    "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f," +
                            "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f," +
                            "%.6f,%.6f,%.6f,%.6f,%s,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f," +
                            "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%s,%.6f%n",
                    elapsed, traveled, target.getTangentialVel(), rawVelocity,
                    kalmanVelocity, target.getTangentialAccel(), target.getAngularVel(),
                    target.getAngularAccel(), target.getMotorPower(), crossTrackError, curvature,
                    targetHeading, headingError, demand.crossTrack, demand.tangentCorrection,
                    demand.headingCorrection, demand.centripetal, demand.forwardVelocity,
                    demand.headingVelocity, demand.driveFeedforward, demand.headingFeedforward,
                    demand.total,
                    commandPower, commandPower >= 0.98,
                    follower.getTrackingVelocityTarget(), follower.getTrackingMeasuredVelocity(),
                    follower.getTrackingProgressVelocity(), follower.getTrackingFeedforward(),
                    follower.getTrackingVelocityFeedback(),
                    follower.getDrivetrain().getLastFlPower(),
                    follower.getDrivetrain().getLastFrPower(), follower.getTrackingEndpointBlend(),
                    follower.getVelocity().getHeading().getRad(),
                    demand.correctionScale, demand.feedforwardScale, demand.velocityScale,
                    demand.driveFeedforward * demand.feedforwardScale,
                    demand.headingFeedforward * demand.feedforwardScale,
                    Math.max(0, 1 - demand.correctiveTotal * demand.correctionScale),
                    Math.max(0, 1 - demand.correctiveTotal * demand.correctionScale
                            - demand.feedforwardTotal * demand.feedforwardScale),
                    Math.max(0, 1 - demand.correctiveTotal * demand.correctionScale
                            - demand.feedforwardTotal * demand.feedforwardScale
                            - demand.velocityTotal * demand.velocityScale), sample.expectedMotorPower,
                    follower.getVelocityGoalDistanceIn(), follower.getCurrentSpeedCeiling(),
                    follower.isCenteredBrakingActive(), follower.getCenteredBrakingMeanPower()));
            outboundVelocityRowsSinceFlush++;
            if (outboundVelocityRowsSinceFlush >= 25) {
                outboundVelocityCsv.flush();
                outboundVelocityRowsSinceFlush = 0;
            }
        } catch (IOException e) {
            outboundVelocityCsvError = e.getMessage();
            closeOutboundVelocityCsv();
        }
    }

    private void closeOutboundVelocityCsv() {
        if (outboundVelocityCsv == null) { return; }
        try {
            outboundVelocityCsv.close();
        } catch (IOException e) {
            outboundVelocityCsvError = e.getMessage();
        } finally {
            outboundVelocityCsv = null;
        }
    }
}
