package org.firstinspires.ftc.teamcode.sim;

import com.qualcomm.robotcore.eventloop.opmode.SimLinearOpModeBridge;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.apexpathing.Constants;
import org.firstinspires.ftc.teamcode.apexpathing.RobotSetup;
import org.json.JSONObject;
import org.junit.Test;
import setup.*;
import storage.ApexStorage;
import localizers.BaseLocalizer;
import localizers.LocalizationConstants;
import localizers.util.AdaptiveKalmanFilter;
import localizers.util.PoseEstimator;
import follower.Follower;
import java.io.File;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.Collections;
import java.util.List;
import java.util.ArrayList;
import static org.junit.Assert.*;

/** Real SDK lifecycle, gamepad edges, simulator inventory, and joint-estimator round trips. */
public class RobotSetupSimulationTest {
    @Test public void advancedCalibrationUpdatesTheGuidedProfileInsteadOfACompetingFile() throws Exception {
        String old = System.getProperty(ApexStorage.DIRECTORY_PROPERTY);
        File root = Files.createTempDirectory("apex-setup-advanced-").toFile();
        System.setProperty(ApexStorage.DIRECTORY_PROPERTY, root.getAbsolutePath());
        try {
            new SetupStore(root).save(new RobotProfile());
            ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
            localizers.LocalizerSet localizers = new localizers.LocalizerSet(new Constants(), hardware.hardwareMap,
                    drivetrains.BaseDrivetrain.DrivetrainType.MECANUM, follower.FollowerConstants.Profile.DEFAULT);
            ((localizers.Pinpoint.Constants) localizers.getConfig()).setOffsets(5, -3, geometry.DistUnit.IN);
            localizers.rebuildActive();
            localizers.getCalibration().capture("DEFAULT", localizers.getConfig(), localizers.getLocalizer(),
                    LocalizationConstants.Status.ACCEPTED, LocalizationConstants.Status.NEEDS_VALIDATION);
            localizers.getCalibration().markGeometryStepAccepted("DEFAULT", "ROTATION");
            localizers.saveCalibration();
            RobotProfile reloaded = new SetupStore(root).load();
            assertEquals(5, reloaded.number("offsetX"), 0); assertEquals(-3, reloaded.number("offsetY"), 0);
            assertTrue(reloaded.geometryMeasured); assertFalse(reloaded.filterMeasured);
            assertFalse("No second calibration owner", new File(root, "localization.json").exists());
            assertEquals(5, ((localizers.Pinpoint.Constants) new Constants().localizerConstants()).offsets.getX().getIn(), 0);
        } finally { restore(old); }
    }

    @Test public void simulatorHardwareSupportsTheSetupPickerAndCompleteNativeFilterPersistence() throws Exception {
        String old = System.getProperty(ApexStorage.DIRECTORY_PROPERTY);
        File root = Files.createTempDirectory("apex-setup-filter-").toFile();
        System.setProperty(ApexStorage.DIRECTORY_PROPERTY, root.getAbsolutePath());
        try {
            ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
            SdkSetupHardware adapter = new SdkSetupHardware(hardware.hardwareMap);
            assertTrue(adapter.devices("motor").contains(ApexSimulation.FRONT_LEFT_MOTOR));
            assertTrue(adapter.devices("pinpoint").contains(ApexSimulation.PINPOINT));
            RobotProfile profile = new RobotProfile(); adapter.prepare(profile);
            BaseLocalizer<?> l = adapter.getLocalizer();
            l.setPoseKalmanTuning(new PoseEstimator.Tuning(new AdaptiveKalmanFilter.KalmanTuning(.004, 77),
                    new AdaptiveKalmanFilter.KalmanTuning(.005, 88), new AdaptiveKalmanFilter.KalmanTuning(.0003, 9),
                    .12, .14, .02));
            profile.filter = adapter.captureFilter(); profile.filterMeasured = true;
            new SetupStore(root).save(profile);
            Follower follower = Follower.create(new Constants(), hardware.hardwareMap, true);
            PoseEstimator.Tuning reloaded = follower.getLocalizer().getPoseKalmanTuning();
            assertEquals(.004, reloaded.x.measurementVariance, 0); assertEquals(77, reloaded.x.processVariance, 0);
            assertEquals(.005, reloaded.y.measurementVariance, 0); assertEquals(88, reloaded.y.processVariance, 0);
            assertEquals(.12, reloaded.xVelocityVariance, 0); assertEquals(.02, reloaded.headingVelocityVariance, 0);
            assertFalse(follower.getLocalizer().isPoseAutoTuning());
        } finally { restore(old); }
    }

    @Test(timeout = 20_000) public void gamepadSetupFlowRecordsDataAndStopDoesNotSaveADraft() throws Exception {
        String old = System.getProperty(ApexStorage.DIRECTORY_PROPERTY);
        File root = Files.createTempDirectory("apex-setup-ui-").toFile();
        System.setProperty(ApexStorage.DIRECTORY_PROPERTY, root.getAbsolutePath());
        List<String> frames = Collections.synchronizedList(new ArrayList<>());
        final boolean[] learning = {false};
        final double[] motorPower = {0};
        SetupHardware backend = new SetupHardware() {
            @Override public List<String> devices(String role) {
                return role.equals("pinpoint") ? java.util.Arrays.asList("pinpoint")
                        : java.util.Arrays.asList("frontLeftMotor", "frontRightMotor", "backLeftMotor", "backRightMotor");
            }
            @Override public void prepare(RobotProfile p) { }
            @Override public SetupSnapshot update() { SetupSnapshot s = new SetupSnapshot(); s.pods = new double[2]; return s; }
            @Override public void learn(boolean enabled) { learning[0] = enabled; }
            @Override public JSONObject captureFilter() { return new JSONObject(); }
            @Override public MotorTestReading testMotor(String name, boolean reversed) {
                motorPower[0] = .5; return new MotorTestReading(.5, 10, 100, 12.5);
            }
            @Override public void stop() { motorPower[0] = 0; }
        };
        RobotSetup opMode = new RobotSetup() { @Override protected SetupHardware createSetupHardware() { return backend; } };
        ApexSimTelemetry telemetry = new ApexSimTelemetry(frames::add); telemetry.setMsTransmissionInterval(0);
        opMode.telemetry = telemetry; opMode.gamepad1 = new Gamepad(); opMode.gamepad2 = new Gamepad();
        SimLinearOpModeBridge.Session session = SimLinearOpModeBridge.initialize(opMode, () -> { });
        try {
            await(session, opMode, frames, "Press Start", 2000);
            SimLinearOpModeBridge.start(session);
            await(session, opMode, frames, "Begin setup", 2000);
            pressA(session, opMode);
            await(session, opMode, frames, "Drive layout", 2000);
            pressA(session, opMode); pressA(session, opMode);
            await(session, opMode, frames, "Front left motor", 2000);
            opMode.gamepad1.x = true; pump(session, opMode, 120);
            assertEquals(.5, motorPower[0], 0);
            await(session, opMode, frames, "Release X to stop", 2000);
            opMode.gamepad1.x = false; pump(session, opMode, 80);
            assertEquals(0, motorPower[0], 0);
            for (int i = 0; i < 24 && latest(frames).contains("Setting "); i++) { pressA(session, opMode); }
            await(session, opMode, frames, "Keep the robot still", 2000);
            await(session, opMode, frames, "Begin movement", 5000);
            pressA(session, opMode); await(session, opMode, frames, "Recording", 2000);
        } finally {
            SimLinearOpModeBridge.stop(session);
            restore(old);
        }
        assertFalse(learning[0]); assertFalse(new File(root, "robot-setup.json").exists());
        assertEquals(0, motorPower[0], 0);
        File[] logs = new File(root, "logs").listFiles(); assertNotNull(logs); assertEquals(1, logs.length);
        String motion = new String(Files.readAllBytes(new File(logs[0], "motion.csv").toPath()), StandardCharsets.UTF_8);
        assertTrue(motion.contains("RECORDING:FORWARD"));
        String events = new String(Files.readAllBytes(new File(logs[0], "events.csv").toPath()), StandardCharsets.UTF_8);
        assertTrue(events.contains("session_aborted"));
        assertTrue(events.contains("motor_test_started")); assertTrue(events.contains("motor_test_stopped"));
        for (String frame : frames) { assertFalse(frame.contains("R=")); assertFalse(frame.contains("raw_vx")); }
    }

    private static void pressA(SimLinearOpModeBridge.Session session, RobotSetup mode) throws Exception {
        mode.gamepad1.a = true; pump(session, mode, 80);
        mode.gamepad1.a = false; pump(session, mode, 80);
    }
    private static void pump(SimLinearOpModeBridge.Session session, RobotSetup mode, long millis) throws Exception {
        long until = System.nanoTime() + millis * 1_000_000;
        while (System.nanoTime() < until) {
            SimLinearOpModeBridge.eventLoopIteration(session, mode.gamepad1, mode.gamepad2); Thread.sleep(5);
        }
    }
    private static void await(SimLinearOpModeBridge.Session session, RobotSetup mode, List<String> frames, String text, long millis) throws Exception {
        long until = System.nanoTime() + millis * 1_000_000;
        while (!latest(frames).contains(text) && System.nanoTime() < until) { pump(session, mode, 20); }
        assertTrue("Expected " + text + "; got " + latest(frames), latest(frames).contains(text));
    }
    private static String latest(List<String> frames) { return frames.isEmpty() ? "" : frames.get(frames.size() - 1); }
    private static void restore(String old) {
        if (old == null) { System.clearProperty(ApexStorage.DIRECTORY_PROPERTY); }
        else { System.setProperty(ApexStorage.DIRECTORY_PROPERTY, old); }
    }
}
