package org.firstinspires.ftc.teamcode.sim;

import com.qualcomm.robotcore.eventloop.opmode.SimLinearOpModeBridge;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import storage.ApexStorage;
import follower.Follower;
import follower.FollowerConstants;
import localizers.LocalizationConstants;
import org.firstinspires.ftc.teamcode.apexpathing.Constants;
import org.firstinspires.ftc.teamcode.apexpathing.FollowerTuner;
import org.junit.Test;

import java.io.File;
import java.nio.file.Files;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

import tuning.follower.TunerContext;
import tuning.follower.tracking.ProfiledTrackingPhase;

import static org.junit.Assert.assertTrue;

/** Checks the profiled phase with actual simulated motion and saved telemetry. */
public class ProfiledTrackingPhaseSimulationTest {
    @Test(timeout = 85_000L)
    public void manualEditInvalidatesReviewUntilSingleCornerIsRerun() throws Exception {
        String previousDirectory = System.getProperty(ApexStorage.DIRECTORY_PROPERTY);
        File directory = Files.createTempDirectory("apex-manual-profiled-tuner-").toFile();
        System.setProperty(ApexStorage.DIRECTORY_PROPERTY, directory.getAbsolutePath());
        SimLinearOpModeBridge.Session session = null;
        try {
            ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
            Constants robot = new Constants();
            Follower seed = new Follower(robot, hardware.hardwareMap, true);
            LocalizationConstants localization = LocalizationConstants.empty();
            localization.capture("DEFAULT", robot.localizerConstants(), seed.getLocalizer(),
                    LocalizationConstants.Status.ACCEPTED,
                    LocalizationConstants.Status.ACCEPTED);
            localization.save();
            FollowerConstants values = FollowerConstants.getInstance();
            SimulatorTuning.apply(values);
            values.profiledTrackingTuned = true;
            double oldGain = values.velocityFeedbackGain;
            List<String> frames = Collections.synchronizedList(new ArrayList<>());
            ApexSimTelemetry telemetry = new ApexSimTelemetry(frames::add);
            telemetry.setMsTransmissionInterval(0);
            AtomicBoolean complete = new AtomicBoolean();
            Follower[] follower = new Follower[1];
            LinearOpMode opMode = new LinearOpMode() {
                @Override public void runOpMode() {
                    TunerContext context = new TunerContext(this);
                    follower[0] = new Follower(new Constants(), hardwareMap, true);
                    context.setFollower(follower[0]);
                    waitForStart();
                    complete.set(new ProfiledTrackingPhase(context).run(this));
                }
            };
            opMode.hardwareMap = hardware.hardwareMap;
            opMode.telemetry = telemetry;
            opMode.gamepad1 = new Gamepad();
            opMode.gamepad2 = new Gamepad();
            session = SimLinearOpModeBridge.initialize(opMode, () -> { });
            SimLinearOpModeBridge.start(session);
            pump(session, opMode, telemetry, hardware, 80);
            opMode.gamepad1.dpad_right = true;
            pump(session, opMode, telemetry, hardware, 40);
            opMode.gamepad1.dpad_right = false;
            pump(session, opMode, telemetry, hardware, 40);
            assertTrue(latest(frames).contains("Manual"));
            pressA(session, opMode, telemetry, hardware);
            waitForFrame(session, opMode, telemetry, hardware, frames,
                    "LB/RB select", 2_000L);
            opMode.gamepad1.dpad_up = true;
            pump(session, opMode, telemetry, hardware, 40);
            opMode.gamepad1.dpad_up = false;
            pump(session, opMode, telemetry, hardware, 40);
            assertTrue("Gain edit did not apply", values.velocityFeedbackGain > oldGain);
            assertTrue("Edit must invalidate prior review", !values.profiledTrackingTuned);
            pressA(session, opMode, telemetry, hardware);
            assertTrue("A must not accept before new measurements", !complete.get());

            // The unlocked simulator stages the first trial at this field pose.
            geometry.Pose courseStart = new geometry.GeometryFactory()
                    .setDistUnit(geometry.DistUnit.IN).pose(-20.0, 20.0, 0.0);
            runManualCourse(session, opMode, telemetry, hardware, frames,
                    "Single-corner path measured");
            assertTrue("First run should leave the starting point",
                    follower[0].getPose().getVec().minus(courseStart.getVec()).getMag().getIn() > 30.0);
            // A new measurement must retrace the original course even after settings change.
            opMode.gamepad1.dpad_up = true;
            pump(session, opMode, telemetry, hardware, 40);
            opMode.gamepad1.dpad_up = false;
            pump(session, opMode, telemetry, hardware, 40);
            runManualCourse(session, opMode, telemetry, hardware, frames,
                    "Single-corner path measured");
            assertTrue("Return run should end at the original starting point",
                    follower[0].getPose().getVec().minus(courseStart.getVec()).getMag().getIn() < 3.0);
            assertTrue("Return run should preserve the original heading",
                    Math.abs(follower[0].getPose().getHeading()
                            .getShortestAngleTo(courseStart.getHeading()).getDeg()) < 5.0);
            pressA(session, opMode, telemetry, hardware);
            assertTrue(values.profiledTrackingTuned);
            waitForFrame(session, opMode, telemetry, hardware, frames,
                    "Press A to save and continue", 2_000L);
            pressA(session, opMode, telemetry, hardware);
            pump(session, opMode, telemetry, hardware, 100);
            assertTrue("The measured path should permit completion", complete.get());
        } finally {
            if (session != null) { SimLinearOpModeBridge.stop(session); }
            if (previousDirectory == null) { System.clearProperty(ApexStorage.DIRECTORY_PROPERTY); }
            else { System.setProperty(ApexStorage.DIRECTORY_PROPERTY, previousDirectory); }
        }
    }

    @Test(timeout = 300_000L)
    public void measuresSingleCornerBeforeCompletingTheActiveProfile() throws Exception {
        String previousDirectory = System.getProperty(ApexStorage.DIRECTORY_PROPERTY);
        File directory = Files.createTempDirectory("apex-profiled-tuner-").toFile();
        System.setProperty(ApexStorage.DIRECTORY_PROPERTY, directory.getAbsolutePath());
        SimLinearOpModeBridge.Session session = null;
        try {
            ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
            Constants robot = new Constants();
            Follower seed = new Follower(robot, hardware.hardwareMap, true);
            LocalizationConstants localization = LocalizationConstants.empty();
            localization.capture("DEFAULT", robot.localizerConstants(), seed.getLocalizer(),
                    LocalizationConstants.Status.ACCEPTED,
                    LocalizationConstants.Status.ACCEPTED);
            localization.save();
            FollowerConstants values = FollowerConstants.getInstance();
            SimulatorTuning.apply(values);
            values.profiledTrackingTuned = false;

            List<String> frames = Collections.synchronizedList(new ArrayList<>());
            ApexSimTelemetry telemetry = new ApexSimTelemetry(frames::add);
            telemetry.setMsTransmissionInterval(0);
            FollowerTuner tuner = new FollowerTuner();
            tuner.hardwareMap = hardware.hardwareMap;
            tuner.telemetry = telemetry;
            tuner.gamepad1 = new Gamepad();
            tuner.gamepad2 = new Gamepad();
            session = SimLinearOpModeBridge.initialize(tuner, () -> { });
            pump(session, tuner, telemetry, hardware, 100);
            assertTrue("Trajectory tracking should be the first incomplete phase: " + latest(frames),
                    latest(frames).contains("TRAJECTORY TRACKING <"));
            pressA(session, tuner, telemetry, hardware);
            SimLinearOpModeBridge.start(session);

            long deadline = System.nanoTime() + 270_000_000_000L;
            while (!latest(frames).contains("Follower tuning complete")
                    && System.nanoTime() < deadline) {
                if (latest(frames).contains("Press A")) {
                    pressA(session, tuner, telemetry, hardware);
                }
                pump(session, tuner, telemetry, hardware, 20);
            }
            assertTrue("Profiled tuner did not finish: " + latest(frames),
                    latest(frames).contains("Follower tuning complete"));
            File[] csv = directory.listFiles((dir, name) ->
                    name.startsWith("profiled_tracking_") && name.endsWith(".csv"));
            assertTrue("Missing profile trace", csv != null && csv.length == 1);
            String trace = new String(Files.readAllBytes(csv[0].toPath()),
                    StandardCharsets.UTF_8);
            assertTrue("Single-corner profile was not sampled",
                    trace.contains(",SINGLE_CORNER,"));
            assertTrue("Active profile was not marked reviewed", values.profiledTrackingTuned);
        } finally {
            if (session != null) { SimLinearOpModeBridge.stop(session); }
            if (previousDirectory == null) { System.clearProperty(ApexStorage.DIRECTORY_PROPERTY); }
            else { System.setProperty(ApexStorage.DIRECTORY_PROPERTY, previousDirectory); }
        }
    }

    private static void waitForFrame(SimLinearOpModeBridge.Session session, LinearOpMode opMode,
                                     ApexSimTelemetry telemetry, ApexSimulation.Hardware hardware,
                                     List<String> frames, String expected, long timeoutMs)
            throws Exception {
        long deadline = System.nanoTime() + timeoutMs * 1_000_000L;
        while (!latest(frames).contains(expected) && System.nanoTime() < deadline) {
            pump(session, opMode, telemetry, hardware, 20);
        }
        assertTrue("Timed out waiting for " + expected + ": " + latest(frames),
                latest(frames).contains(expected));
    }

    private static void runManualCourse(SimLinearOpModeBridge.Session session,
                                        LinearOpMode opMode, ApexSimTelemetry telemetry,
                                        ApexSimulation.Hardware hardware, List<String> frames,
                                        String measured) throws Exception {
        for (int attempt = 0; attempt < 3; attempt++) {
            opMode.gamepad1.x = true;
            pump(session, opMode, telemetry, hardware, 40);
            opMode.gamepad1.x = false;
            pump(session, opMode, telemetry, hardware, 40);
            long deadline = System.nanoTime() + 20_000_000_000L;
            while (System.nanoTime() < deadline) {
                String frame = latest(frames);
                if (frame.contains(measured)) { return; }
                if (frame.contains("No usable braking/corner samples")
                        || frame.contains("Trial timed out")) { break; }
                pump(session, opMode, telemetry, hardware, 20);
            }
        }
        assertTrue("Timed out waiting for " + measured + ": " + latest(frames),
                latest(frames).contains(measured));
    }

    private static void pressA(SimLinearOpModeBridge.Session session, LinearOpMode tuner,
                               ApexSimTelemetry telemetry, ApexSimulation.Hardware hardware)
            throws Exception {
        tuner.gamepad1.a = true;
        pump(session, tuner, telemetry, hardware, 35);
        tuner.gamepad1.a = false;
        pump(session, tuner, telemetry, hardware, 35);
    }

    private static String latest(List<String> frames) {
        return frames.isEmpty() ? "" : frames.get(frames.size() - 1);
    }

    private static void pump(SimLinearOpModeBridge.Session session, LinearOpMode tuner,
                             ApexSimTelemetry telemetry, ApexSimulation.Hardware hardware,
                             long milliseconds) throws Exception {
        long deadline = System.nanoTime() + milliseconds * 1_000_000L;
        while (System.nanoTime() < deadline) {
            long now = System.nanoTime();
            double remaining = Math.max(0.0, Math.min(0.05,
                    (now - hardware.lastPhysicsUpdateNanos) * 1e-9));
            hardware.lastPhysicsUpdateNanos = now;
            while (remaining > 1e-9) {
                double step = Math.min(0.005, remaining);
                ApexSimulation.stepPhysics(hardware, step);
                remaining -= step;
            }
            SimLinearOpModeBridge.eventLoopIteration(session, tuner.gamepad1, tuner.gamepad2);
            telemetry.update();
            Thread.sleep(5);
        }
    }
}
