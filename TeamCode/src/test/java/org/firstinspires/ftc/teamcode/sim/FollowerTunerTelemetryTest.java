package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.eventloop.opmode.SimLinearOpModeBridge;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.codeblooded.ftcodesim.hardware.devices.SimMotor;
import org.firstinspires.ftc.teamcode.apexpathing.FollowerTuner;
import org.firstinspires.ftc.teamcode.apexpathing.Constants;
import org.junit.Test;

import java.io.File;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import controllers.FeedbackController.Coefficients;
import storage.ApexStorage;
import follower.FollowerConstants;
import follower.Follower;
import localizers.LocalizationConstants;

/** End-to-end coverage for the automatic follower-tuning workflow. */
public class FollowerTunerTelemetryTest {
    @Test(timeout = 30_000L)
    public void stopDiscardsUnacceptedManualEdits() throws Exception {
        String previousDirectory = System.getProperty(ApexStorage.DIRECTORY_PROPERTY);
        File directory = java.nio.file.Files.createTempDirectory("apex-tuner-stop-").toFile();
        System.setProperty(ApexStorage.DIRECTORY_PROPERTY, directory.getAbsolutePath());
        SimLinearOpModeBridge.Session session = null;
        try {
            ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
            seedAcceptedLocalization(hardware);
            FollowerConstants constants = FollowerConstants.getInstance();
            SimulatorTuning.apply(constants);
            constants.translationalCoeffs.kP = 0.0; // Start at the first incomplete phase: Drive.
            constants.profiledTrackingTuned = true;
            ApexStorage.saveConstants(constants.toJson().toString(4));
            byte[] saved = java.nio.file.Files.readAllBytes(ApexStorage.getConstantsFile().toPath());
            List<String> frames = Collections.synchronizedList(new ArrayList<>());
            ApexSimTelemetry telemetry = new ApexSimTelemetry(frames::add);
            telemetry.setMsTransmissionInterval(0);
            FollowerTuner tuner = new FollowerTuner();
            tuner.hardwareMap = hardware.hardwareMap;
            tuner.telemetry = telemetry;
            tuner.gamepad1 = new Gamepad();
            tuner.gamepad2 = new Gamepad();
            session = SimLinearOpModeBridge.initialize(tuner, () -> { });
            pump(session, tuner, telemetry, hardware, 60);
            SimLinearOpModeBridge.start(session);
            pump(session, tuner, telemetry, hardware, 80);
            for (int step = 0; step < 8 && !latestFrame(frames).contains(" DRIVE <"); step++) {
                tuner.gamepad1.dpad_down = true;
                pump(session, tuner, telemetry, hardware, 80);
                tuner.gamepad1.dpad_down = false;
                pump(session, tuner, telemetry, hardware, 80);
            }
            pressA(session, tuner, telemetry, hardware);
            assertTrue(latestFrame(frames).contains("Next phase: Drive Controller"));
            tuner.gamepad1.dpad_right = true;
            pump(session, tuner, telemetry, hardware, 80);
            tuner.gamepad1.dpad_right = false;
            pump(session, tuner, telemetry, hardware, 80);
            assertTrue(latestFrame(frames).contains("Mode Manual"));
            pressA(session, tuner, telemetry, hardware);
            tuner.gamepad1.dpad_up = true;
            pump(session, tuner, telemetry, hardware, 80);
            tuner.gamepad1.dpad_up = false;
            pump(session, tuner, telemetry, hardware, 80);
            assertTrue("Manual edit should change the pending gain", constants.translationalCoeffs.kP > 0.0);
            SimLinearOpModeBridge.stop(session);
            session = null;
            org.junit.Assert.assertArrayEquals("Stop must leave the saved calibration untouched", saved,
                    java.nio.file.Files.readAllBytes(ApexStorage.getConstantsFile().toPath()));
            for (double power : frontMotorPowers(hardware)) {
                org.junit.Assert.assertEquals(0.0, power, 0.0);
            }
        } finally {
            if (session != null) { SimLinearOpModeBridge.stop(session); }
            if (previousDirectory == null) { System.clearProperty(ApexStorage.DIRECTORY_PROPERTY); }
            else { System.setProperty(ApexStorage.DIRECTORY_PROPERTY, previousDirectory); }
        }
    }

    @Test(timeout = 30_000L)
    public void idleScreensAllowDrivingAndActiveRoutineOwnsTheDrivetrain() throws Exception {
        resetFollowerConstants();
        ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
        seedAcceptedLocalization(hardware);
        List<String> frames = Collections.synchronizedList(new ArrayList<>());
        ApexSimTelemetry telemetry = new ApexSimTelemetry(frames::add);
        telemetry.setMsTransmissionInterval(0);

        FollowerTuner tuner = new FollowerTuner();
        tuner.hardwareMap = hardware.hardwareMap;
        tuner.telemetry = telemetry;
        tuner.gamepad1 = new Gamepad();
        tuner.gamepad2 = new Gamepad();

        SimLinearOpModeBridge.Session session = SimLinearOpModeBridge.initialize(tuner, () -> { });
        try {
            pump(session, tuner, telemetry, hardware, 60);
            SimLinearOpModeBridge.start(session);

            tuner.gamepad1.left_stick_y = -1.0f;
            pump(session, tuner, telemetry, hardware, 80);
            assertTrue("phase picker should pass through forward stick input",
                    frontMotorProduct(hardware) > 0.0);
            assertTrue(latestFrame(frames).trim().endsWith("Robot can be driven with sticks"));

            tuner.gamepad1.left_stick_y = 0.0f;
            pump(session, tuner, telemetry, hardware, 30);
            // Select heading SysId to exercise its idle and active motor ownership.
            tuner.gamepad1.dpad_down = true;
            pump(session, tuner, telemetry, hardware, 30);
            tuner.gamepad1.dpad_down = false;
            pump(session, tuner, telemetry, hardware, 30);
            pressA(session, tuner, telemetry, hardware);

            tuner.gamepad1.left_stick_y = -1.0f;
            pump(session, tuner, telemetry, hardware, 80);
            assertTrue("mode selector should pass through forward stick input",
                    frontMotorProduct(hardware) > 0.0);

            tuner.gamepad1.left_stick_y = 0.0f;
            pump(session, tuner, telemetry, hardware, 30);
            pressA(session, tuner, telemetry, hardware);

            tuner.gamepad1.left_stick_y = -1.0f;
            pump(session, tuner, telemetry, hardware, 80);
            assertTrue("positioning prompt should pass through forward stick input",
                    frontMotorProduct(hardware) > 0.0);
            assertTrue(latestFrame(frames).trim().endsWith("Robot can be driven with sticks"));

            tuner.gamepad1.left_stick_y = 0.0f;
            pump(session, tuner, telemetry, hardware, 30);
            pump(session, tuner, telemetry, hardware, 1000);
            pressA(session, tuner, telemetry, hardware);
            tuner.gamepad1.left_stick_y = -1.0f;
            pump(session, tuner, telemetry, hardware, 80);
            double[] frontPowers = frontMotorPowers(hardware);
            assertTrue("active heading routine should override forward stick input: " +
                            java.util.Arrays.toString(frontPowers),
                    frontPowers[0] * frontPowers[1] <= 0.0);
            assertTrue(latestFrame(frames).contains("Tuning HEADING feedforward kS, kV, kA"));
            assertTrue(!latestFrame(frames).contains("Robot can be driven with sticks"));
            tuner.gamepad1.x = tuner.gamepad1.b = true;
            pump(session, tuner, telemetry, hardware, 80);
            tuner.gamepad1.x = tuner.gamepad1.b = false;
            assertTrue("X/B must not abort the active tuning run", frontMotorProduct(hardware) <= 0.0);
            assertTrue(!latestFrame(frames).contains("Robot can be driven with sticks"));
        } finally {
            SimLinearOpModeBridge.stop(session);
        }
        for (double power : frontMotorPowers(hardware)) {
            org.junit.Assert.assertEquals("Stop must remove motor power", 0.0, power, 0.0);
        }
    }

    @Test(timeout = 1_200_000L)
    public void automaticWorkflowCompletesWithoutVelocityFeedbackTuner() throws Exception {
        automaticWorkflow(false);
    }

    void automaticWorkflow(boolean afterAcceptedFeedforward) throws Exception {
        resetFollowerConstants();
        if (afterAcceptedFeedforward) {
            SimulatorTuning.apply(FollowerConstants.getInstance());
            FollowerConstants.getInstance().motionCharacterization = null;
        }
        ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
        seedAcceptedLocalization(hardware);
        List<String> frames = Collections.synchronizedList(new ArrayList<>());
        ApexSimTelemetry telemetry = new ApexSimTelemetry(frames::add);
        telemetry.setMsTransmissionInterval(0);

        FollowerTuner tuner = new FollowerTuner();
        tuner.hardwareMap = hardware.hardwareMap;
        tuner.telemetry = telemetry;
        tuner.gamepad1 = new Gamepad();
        tuner.gamepad2 = new Gamepad();

        SimLinearOpModeBridge.Session session = SimLinearOpModeBridge.initialize(tuner, () -> { });
        try {
            pump(session, tuner, telemetry, hardware, 100);
            tuner.gamepad1.a = true;
            pump(session, tuner, telemetry, hardware, 30);
            tuner.gamepad1.a = false;
            SimLinearOpModeBridge.start(session);

            long deadline = System.nanoTime() + 1_140_000_000_000L;
            long lastProgressNanos = 0;
            java.util.Set<String> verifiedControllers = new java.util.HashSet<>();
            while (!latestFrame(frames).contains("Follower tuning complete for") &&
                    System.nanoTime() < deadline) {
                String frame = latestFrame(frames);
                if (System.nanoTime() - lastProgressNanos > 1_000_000_000L) {
                    java.nio.file.Files.write(new File(ApexStorage.getDirectory(), "tuning-progress.txt").toPath(),
                            frame.getBytes(java.nio.charset.StandardCharsets.UTF_8));
                    lastProgressNanos = System.nanoTime();
                }
                assertTrue("Automatic feedforward validation failed:\n" + frame,
                        !frame.contains("validation failed"));
                String controller = frame.contains("Tuning Heading kP, kD") ? "heading" : "drive";
                if (frame.contains("Press X to run a verification step") && verifiedControllers.add(controller)) {
                    tuner.gamepad1.x = true;
                    pump(session, tuner, telemetry, hardware, 40);
                    tuner.gamepad1.x = false;
                } else if (frame.contains("Press A")) {
                    tuner.gamepad1.a = true;
                    pump(session, tuner, telemetry, hardware, 30);
                    tuner.gamepad1.a = false;
                }
                pump(session, tuner, telemetry, hardware, 20);
            }

            assertTrue("Automatic follower tuning did not finish. Latest telemetry:\n" +
                            latestFrame(frames),
                    latestFrame(frames).contains("Follower tuning complete for"));
            org.junit.Assert.assertEquals("Both PD phases must retain X verification", 2, verifiedControllers.size());
            for (String frame : frames) {
                if (!frame.contains("Tuning ")) { continue; }
                assertTrue("Automatic telemetry must omit progress diagnostics:\n" + frame,
                        !frame.contains("Status ") && !frame.contains("CSV ")
                                && !frame.contains("Best cost") && !frame.contains("Trials ")
                                && !frame.contains("Robot is "));
                if (frame.contains("Robot can be driven with sticks")) {
                    assertTrue("Drive availability belongs at the bottom", frame.trim()
                            .endsWith("Robot can be driven with sticks"));
                }
            }
        } finally {
            SimLinearOpModeBridge.stop(session);
        }
    }

    private static void pump(SimLinearOpModeBridge.Session session, FollowerTuner tuner,
                             ApexSimTelemetry telemetry, ApexSimulation.Hardware hardware,
                             long milliseconds) throws Exception {
        long deadline = System.nanoTime() + milliseconds * 1_000_000L;
        while (System.nanoTime() < deadline) {
            long now = System.nanoTime();
            double remaining = Math.max(0, Math.min(0.25,
                    (now - hardware.lastPhysicsUpdateNanos) * 1e-9));
            hardware.lastPhysicsUpdateNanos = now;
            while (remaining > 1e-9) {
                double step = Math.min(0.005, remaining);
                stepPhysics(hardware, step);
                remaining -= step;
            }
            SimLinearOpModeBridge.eventLoopIteration(
                    session, tuner.gamepad1, tuner.gamepad2);
            telemetry.update();
            Thread.sleep(5);
        }
    }

    private static void pressA(SimLinearOpModeBridge.Session session, FollowerTuner tuner,
                               ApexSimTelemetry telemetry, ApexSimulation.Hardware hardware)
            throws Exception {
        tuner.gamepad1.a = true;
        pump(session, tuner, telemetry, hardware, 30);
        tuner.gamepad1.a = false;
        pump(session, tuner, telemetry, hardware, 30);
    }

    private static double frontMotorProduct(ApexSimulation.Hardware hardware) {
        double[] powers = frontMotorPowers(hardware);
        return powers[0] * powers[1];
    }

    private static double[] frontMotorPowers(ApexSimulation.Hardware hardware) {
        SimMotor left = (SimMotor) hardware.hardwareMap.get(
                DcMotorEx.class, ApexSimulation.FRONT_LEFT_MOTOR);
        SimMotor right = (SimMotor) hardware.hardwareMap.get(
                DcMotorEx.class, ApexSimulation.FRONT_RIGHT_MOTOR);
        return new double[] { left.getPower(), right.getPower() };
    }

    private static void stepPhysics(ApexSimulation.Hardware hardware, double seconds)
            throws Exception {
        ApexSimulation.stepPhysics(hardware, seconds);
    }

    private static String latestFrame(List<String> frames) {
        return frames.isEmpty() ? "" : frames.get(frames.size() - 1);
    }

    private static void resetFollowerConstants() {
        if (System.getProperty(ApexStorage.DIRECTORY_PROPERTY) == null) {
            File directory = new File(System.getProperty("user.dir"), "build/ftcodesim-data");
            System.setProperty(ApexStorage.DIRECTORY_PROPERTY, directory.getAbsolutePath());
        }
        FollowerConstants constants = FollowerConstants.getInstance();
        constants.motionCharacterization = null;
        constants.angularCoeffs = new Coefficients().p(0.0).d(0.0).s(0.0);
        constants.translationalCoeffs = new Coefficients().p(0.0).d(0.0).s(0.0);
        constants.angularKV = 0.0;
        constants.angularKA = 0.0;
        constants.angularFeedforwardKS = 0.0;
        constants.translationalFeedforwardKS = 0.0;
        constants.translationalKV = 0.0;
        constants.translationalKA = 0.0;
        constants.kCentripetal = 0.0;
        constants.velocityFeedbackGain = 0.0;
        constants.angularVelocityFeedbackGain = 0.0;
        constants.forwardVelLimitIn = 0.0;
        constants.forwardAccelLimitIn = 0.0;
        constants.strafeVelLimitIn = 0.0;
        constants.strafeAccelLimitIn = 0.0;
        constants.angularVelLimitRad = 0.0;
        constants.angularAccelLimitRad = 0.0;
    }

    private static void seedAcceptedLocalization(ApexSimulation.Hardware hardware) throws Exception {
        Constants robot = new Constants();
        Follower seed = new Follower(robot, hardware.hardwareMap, true);
        LocalizationConstants localization = LocalizationConstants.empty();
        localization.capture("DEFAULT", robot.localizerConstants(), seed.getLocalizer(),
                LocalizationConstants.Status.ACCEPTED,
                LocalizationConstants.Status.ACCEPTED);
        localization.save();
    }
}
