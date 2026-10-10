package org.firstinspires.ftc.teamcode.sim;

import util.DeltaTime;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.eventloop.opmode.SimLinearOpModeBridge;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.codeblooded.ftcodesim.hardware.devices.SimMotor;
import org.codeblooded.ftcodesim.ascope.boundaries.MotionVector;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.teamcode.apexpathing.LocalizationTuner;
import org.json.JSONObject;
import org.junit.Test;

import java.io.File;
import java.lang.reflect.Array;
import java.lang.reflect.Proxy;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;

import config.ApexConstants;
import storage.ApexStorage;
import localizers.LocalizationConstants;
import drivetrains.BaseDrivetrainConstants;
import drivetrains.DualActuated;
import drivetrains.Mecanum;
import drivetrains.Motor;
import drivetrains.Tank;
import geometry.DistUnit;
import geometry.Pose;
import localizers.BaseLocalizer;
import localizers.BaseLocalizerConstants;
import localizers.Pinpoint;
import localizers.util.LocalizationNoise;
import localizers.util.PoseEstimator;
import tuning.localizer.calibration.CalibrationCandidate;
import tuning.localizer.LocalizationTunerContext;

/** End-to-end simulator coverage for every phase of the localization-tuning workflow. */
public class LocalizationTunerWorkflowTest {
    @Test(timeout = 15_000L)
    public void interruptedFilterCollectionFreezesLearningAndClosesCsvWithoutSaving() throws Exception {
        File storage = new File("build/localization-tuner-test/" + System.nanoTime());
        String oldStorage = System.getProperty(ApexStorage.DIRECTORY_PROPERTY);
        System.setProperty(ApexStorage.DIRECTORY_PROPERTY, storage.getAbsolutePath());
        ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
        List<String> frames = Collections.synchronizedList(new ArrayList<>());
        ApexSimTelemetry telemetry = new ApexSimTelemetry(frames::add);
        telemetry.setMsTransmissionInterval(0);
        LocalizationTuner tuner = configuredTuner(new PinpointOffsetTuner(), hardware, telemetry);
        SimLinearOpModeBridge.Session session = SimLinearOpModeBridge.initialize(tuner, () -> { });
        boolean stopped = false;
        try {
            await(session, tuner, telemetry, hardware, frames, "Choose a test", 2_000L);
            SimLinearOpModeBridge.start(session);
            moveSelectionDown(session, tuner, telemetry, hardware, 4);
            press(session, tuner, telemetry, hardware, Button.A);
            await(session, tuner, telemetry, hardware, frames, "Selected Unified adaptive Kalman", 2_000L);
            java.lang.reflect.Field initialContext = LocalizationTuner.class.getDeclaredField("context");
            initialContext.setAccessible(true);
            BaseLocalizer<?> initialLocalizer = ((LocalizationTunerContext) initialContext.get(tuner)).getLocalizer();
            // Cleanup restores the prior policy; make this interruption fixture start frozen.
            initialLocalizer.setKalmanAutoTuning(false, false, false);
            initialLocalizer.setPoseAutoTuning(false);
            press(session, tuner, telemetry, hardware, Button.A);
            await(session, tuner, telemetry, hardware, frames, "Move X", 5_000L);
            java.lang.reflect.Field contextField = LocalizationTuner.class.getDeclaredField("context");
            contextField.setAccessible(true);
            BaseLocalizer<?> localizer = ((LocalizationTunerContext) contextField.get(tuner)).getLocalizer();
            assertTrue(localizer.isPoseAutoTuning());
            SimLinearOpModeBridge.stop(session); stopped = true;
            assertFalse(localizer.isPoseAutoTuning());
            assertFalse(localizer.isTuningVelocityFilter());
            assertMotorsStopped(hardware);
            assertFalse("An interrupted collection must not save accepted calibration",
                    new File(storage, "localization.json").exists());
            File[] csv = storage.listFiles((directory, name) -> name.startsWith("localization_filter")
                    && name.endsWith(".csv"));
            assertTrue(csv != null && csv.length == 1);
            String text = new String(Files.readAllBytes(csv[0].toPath()), StandardCharsets.UTF_8);
            assertTrue("Cleanup must flush buffered collection data", text.contains("STILL"));
            PoseEstimator.Tuning frozen = localizer.getPoseKalmanTuning();
            for (int i = 0; i < 30; i++) { localizer.update(); Thread.sleep(5); }
            assertEquals(frozen.x.processVariance, localizer.getPoseKalmanTuning().x.processVariance, 0);
        } finally {
            if (!stopped) { SimLinearOpModeBridge.stop(session); }
            restoreProperty(ApexStorage.DIRECTORY_PROPERTY, oldStorage);
        }
    }

    @Test(timeout = 90_000L)
    public void noisyKalmanCollectionCompletesFreezesSavesAndReloadsTheJointEstimator() throws Exception {
        File storage = new File("build/localization-tuner-test/" + System.nanoTime());
        String oldStorage = System.getProperty(ApexStorage.DIRECTORY_PROPERTY);
        System.setProperty(ApexStorage.DIRECTORY_PROPERTY, storage.getAbsolutePath());
        ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
        SimApexPinpoint sensor = (SimApexPinpoint) hardware.hardwareMap.get(Pinpoint.Driver.class,
                ApexSimulation.PINPOINT);
        sensor.setMeasurementNoise(new LocalizationNoise(7842, 0.10, 0.02, 0.8, 0.16, 0.6));
        List<String> frames = Collections.synchronizedList(new ArrayList<>());
        ApexSimTelemetry telemetry = new ApexSimTelemetry(frames::add);
        telemetry.setMsTransmissionInterval(0);
        LocalizationTuner tuner = configuredTuner(new PinpointOffsetTuner(), hardware, telemetry);
        SimLinearOpModeBridge.Session session = SimLinearOpModeBridge.initialize(tuner, () -> { });
        boolean stopped = false;
        try {
            await(session, tuner, telemetry, hardware, frames, "Choose a test", 2_000L);
            SimLinearOpModeBridge.start(session);
            moveSelectionDown(session, tuner, telemetry, hardware, 4);
            await(session, tuner, telemetry, hardware, frames, "FILTER <", 2_000L);
            runKalmanCollection(session, tuner, telemetry, hardware, frames);
            assertMotorsStopped(hardware);
            JSONObject setup = readSavedSetup(storage);
            assertEquals("ACCEPTED", setup.getString("filterStatus"));
            JSONObject pose = setup.getJSONObject("filter").getJSONObject("pose");
            assertEquals("CONSTANT_ACCELERATION", pose.getString("model"));
            assertFalse(pose.getBoolean("autoTuning"));
            assertFalse(pose.getBoolean("tuneX"));
            assertFalse(pose.getBoolean("tuneY"));
            assertFalse(pose.getBoolean("tuneHeading"));
            double learnedR = pose.getJSONObject("x").getDouble("measurementVariance");
            double learnedVelocityR = pose.getJSONObject("x").getDouble("velocityVariance");
            assertTrue("Injected noise should change position tuning", Math.abs(learnedR - 0.0025) > 1e-5);
            assertTrue("Injected noise should change native velocity tuning", Math.abs(learnedVelocityR - 0.25) > 1e-5);
            File[] csv = storage.listFiles((directory, name) -> name.startsWith("localization_filter")
                    && name.endsWith(".csv"));
            assertTrue(csv != null && csv.length == 1 && csv[0].length() > 100);
            String csvText = new String(Files.readAllBytes(csv[0].toPath()), StandardCharsets.UTF_8);
            assertTrue(csvText.contains("STILL"));
            assertTrue(csvText.contains("MOVE"));
            assertTrue(csvText.contains("STOP"));

            SimLinearOpModeBridge.stop(session); stopped = true;
            Pinpoint.Constants config = pinpointConfig();
            LocalizationConstants saved = LocalizationConstants.load();
            assertTrue(saved.applyGeometry("DEFAULT", config));
            BaseLocalizer<?> reloaded = config.build(hardware.hardwareMap);
            assertTrue(saved.applyFilter("DEFAULT", config, reloaded));
            assertFalse(reloaded.isPoseAutoTuning());
            PoseEstimator.Tuning frozen = reloaded.getPoseKalmanTuning();
            assertEquals(learnedR, frozen.x.measurementVariance, 0);
            assertEquals(learnedVelocityR, frozen.xVelocityVariance, 0);
            for (int i = 0; i < 100; i++) { reloaded.update(); Thread.sleep(5); }
            assertEquals(frozen.x.measurementVariance, reloaded.getPoseKalmanTuning().x.measurementVariance, 0);
            assertEquals(frozen.x.processVariance, reloaded.getPoseKalmanTuning().x.processVariance, 0);
            assertEquals(frozen.xVelocityVariance, reloaded.getPoseKalmanTuning().xVelocityVariance, 0);
            System.out.printf(java.util.Locale.US,
                    "Noisy tuner saved and reloaded: position R=%.7f, native velocity R=%.7f, jerk Q=%.7f%n",
                    frozen.x.measurementVariance, frozen.xVelocityVariance, frozen.x.processVariance);
        } finally {
            if (!stopped) { SimLinearOpModeBridge.stop(session); }
            restoreProperty(ApexStorage.DIRECTORY_PROPERTY, oldStorage);
        }
    }

    @Test(timeout = 120_000L)
    public void completesEveryPhaseWithImuAndPersistsReviewableResults() throws Exception {
        File storage = new File("build/localization-tuner-test/" + System.nanoTime());
        String oldStorage = System.getProperty(ApexStorage.DIRECTORY_PROPERTY);
        System.setProperty(ApexStorage.DIRECTORY_PROPERTY, storage.getAbsolutePath());

        ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
        hardware.hardwareMap.register("imu", simulatedImu(hardware));
        List<String> frames = Collections.synchronizedList(new ArrayList<>());
        ApexSimTelemetry telemetry = new ApexSimTelemetry(frames::add);
        telemetry.setMsTransmissionInterval(0);

        LocalizationTuner tuner = new PinpointOffsetTuner();
        tuner.hardwareMap = hardware.hardwareMap;
        tuner.telemetry = telemetry;
        tuner.gamepad1 = new Gamepad();
        tuner.gamepad2 = new Gamepad();

        SimLinearOpModeBridge.Session session = SimLinearOpModeBridge.initialize(tuner, () -> { });
        try {
            await(session, tuner, telemetry, hardware, frames,
                    "Choose a test", 3_000L);
            assertContains(latestFrame(frames), "[READY] DIRECTIONS <");
            SimLinearOpModeBridge.start(session);

            runDirections(session, tuner, telemetry, hardware, frames);
            runDistance(session, tuner, telemetry, hardware, frames, "Forward distance");
            runDistance(session, tuner, telemetry, hardware, frames, "Strafe distance");
            runAutomaticSpin(session, tuner, telemetry, hardware, frames);
            runKalmanCollection(session, tuner, telemetry, hardware, frames);
            await(session, tuner, telemetry, hardware, frames,
                    "[DONE] DIRECTIONS <", 3_000L);
            String menu = latestFrame(frames);
            assertContains(menu, "[DONE] DIRECTIONS <");
            assertFalse("Validation must not appear as a tuner phase", menu.contains(" TEST <"));
            validateLocalization(session, tuner, telemetry, hardware);

            File saved = new File(storage, "localization.json");
            assertTrue("localization.json should be saved", saved.isFile());
            JSONObject root = new JSONObject(new String(Files.readAllBytes(saved.toPath()),
                    StandardCharsets.UTF_8));
            assertEquals(LocalizationConstants.SCHEMA_VERSION, root.getInt("schemaVersion"));
            assertTrue(root.getBoolean("followerRevalidationRequired"));
            JSONObject setup = root.getJSONObject("setups").getJSONObject("DEFAULT");
            assertEquals(Pinpoint.Constants.class.getName(), setup.getString("localizerType"));
            assertEquals("ACCEPTED", setup.getString("geometryStatus"));
            assertEquals("ACCEPTED", setup.getString("filterStatus"));
            JSONObject steps = setup.getJSONObject("geometrySteps");
            assertEquals("ACCEPTED", steps.getString("DIRECTIONS"));
            assertEquals("ACCEPTED", steps.getString("FORWARD"));
            assertEquals("ACCEPTED", steps.getString("STRAFE"));
            assertEquals("ACCEPTED", steps.getString("ROTATION"));
            JSONObject geometry = setup.getJSONObject("geometry");
            assertEquals(Pinpoint.EncoderDirection.FORWARD.name(),
                    geometry.getString("xPodDirection"));
            assertEquals(Pinpoint.EncoderDirection.FORWARD.name(),
                    geometry.getString("yPodDirection"));
            assertEquals(5.0, geometry.getDouble("xIn"), 0.05);
            assertEquals(-3.0, geometry.getDouble("yIn"), 0.05);
            JSONObject filter = setup.getJSONObject("filter");
            assertEquals(BaseLocalizer.VelocityFilterMode.ADAPTIVE_KALMAN.name(),
                    filter.getString("mode"));
            assertPositiveTuning(filter.getJSONObject("x"));
            assertPositiveTuning(filter.getJSONObject("y"));
            assertPositiveTuning(filter.getJSONObject("heading"));

            File[] csvFiles = storage.listFiles((directory, name) ->
                    name.startsWith("localization_filter") && name.endsWith(".csv"));
            assertTrue("filter phase should produce a non-empty CSV",
                    csvFiles != null && csvFiles.length == 1 && csvFiles[0].length() > 100L);

            Pinpoint.Driver pinpoint = hardware.hardwareMap.get(Pinpoint.Driver.class,
                    ApexSimulation.PINPOINT);
            int[] rawTicks = pinpoint.getEncoderPositions();
            assertTrue("simulated Pinpoint should expose raw encoder movement",
                    rawTicks[0] != 0 || rawTicks[1] != 0);
        } finally {
            SimLinearOpModeBridge.stop(session);
            restoreProperty(ApexStorage.DIRECTORY_PROPERTY, oldStorage);
        }
    }

    @Test(timeout = 20_000L)
    public void movingAverageSelectionIsSafe() throws Exception {
        File storage = new File("build/localization-tuner-test/" + System.nanoTime());
        String oldStorage = System.getProperty(ApexStorage.DIRECTORY_PROPERTY);
        System.setProperty(ApexStorage.DIRECTORY_PROPERTY, storage.getAbsolutePath());
        ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
        List<String> frames = Collections.synchronizedList(new ArrayList<>());
        ApexSimTelemetry telemetry = new ApexSimTelemetry(frames::add);
        telemetry.setMsTransmissionInterval(0);
        LocalizationTuner tuner = configuredTuner(new LocalizationTuner(), hardware, telemetry);
        SimLinearOpModeBridge.Session session = SimLinearOpModeBridge.initialize(tuner, () -> { });
        try {
            await(session, tuner, telemetry, hardware, frames,
                    "Choose a test", 2_000L);
            SimLinearOpModeBridge.start(session);
            moveSelectionDown(session, tuner, telemetry, hardware, 4);
            await(session, tuner, telemetry, hardware, frames, "FILTER <", 2_000L);
            press(session, tuner, telemetry, hardware, Button.A);
            await(session, tuner, telemetry, hardware, frames,
                    "Position, velocity and acceleration filter", 2_000L);
            assertContains(latestFrame(frames), "Selected Unified adaptive Kalman");
            press(session, tuner, telemetry, hardware, Button.DPAD_RIGHT);
            assertContains(latestFrame(frames), "Selected Moving average", "Window");
            press(session, tuner, telemetry, hardware, Button.DPAD_UP);
        } finally {
            SimLinearOpModeBridge.stop(session);
            restoreProperty(ApexStorage.DIRECTORY_PROPERTY, oldStorage);
        }
    }

    @Test(timeout = 8_000L)
    public void customLocalizerMarksUnsupportedGeometryPhasesNotApplicable() throws Exception {
        File storage = new File("build/localization-tuner-test/" + System.nanoTime());
        String oldStorage = System.getProperty(ApexStorage.DIRECTORY_PROPERTY);
        System.setProperty(ApexStorage.DIRECTORY_PROPERTY, storage.getAbsolutePath());
        ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
        List<String> frames = Collections.synchronizedList(new ArrayList<>());
        ApexSimTelemetry telemetry = new ApexSimTelemetry(frames::add);
        telemetry.setMsTransmissionInterval(0);
        LocalizationTuner tuner = configuredTuner(new MonitorOnlyTuner(), hardware, telemetry);
        SimLinearOpModeBridge.Session session = SimLinearOpModeBridge.initialize(tuner, () -> { });
        try {
            await(session, tuner, telemetry, hardware, frames,
                    "Choose a test", 2_000L);
            SimLinearOpModeBridge.start(session);
            press(session, tuner, telemetry, hardware, Button.DPAD_DOWN);
            await(session, tuner, telemetry, hardware, frames, "FORWARD DISTANCE <", 2_000L);
            press(session, tuner, telemetry, hardware, Button.A);
            assertContains(latestFrame(frames), "Choose a test",
                    "[N/A] FORWARD DISTANCE <");
        } finally {
            SimLinearOpModeBridge.stop(session);
            restoreProperty(ApexStorage.DIRECTORY_PROPERTY, oldStorage);
        }
    }

    @Test(timeout = 8_000L)
    public void tankAndDualActuatedMenusFollowActiveKinematicsAndLocalizer() throws Exception {
        String oldStorage = System.getProperty(ApexStorage.DIRECTORY_PROPERTY);
        File storage = new File("build/localization-tuner-test/" + System.nanoTime());
        System.setProperty(ApexStorage.DIRECTORY_PROPERTY, storage.getAbsolutePath());
        try {
            verifyTankMenu();
            verifyDualActuatedMenuSwitch();
        } finally {
            restoreProperty(ApexStorage.DIRECTORY_PROPERTY, oldStorage);
        }
    }

    @Test public void failedGeometrySaveRollsBackRuntimeAndStatus() throws Exception {
        File storageRoot = new File("build/localization-tuner-test/" + System.nanoTime());
        assertTrue(storageRoot.getParentFile().isDirectory() || storageRoot.getParentFile().mkdirs());
        Files.write(storageRoot.toPath(), new byte[] {1});
        String oldStorage = System.getProperty(ApexStorage.DIRECTORY_PROPERTY);
        System.setProperty(ApexStorage.DIRECTORY_PROPERTY, storageRoot.getAbsolutePath());
        ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
        LocalizationTuner tuner = configuredTuner(new LocalizationTuner(), hardware,
                new ApexSimTelemetry(frame -> { }));
        try {
            LocalizationTunerContext context = new LocalizationTunerContext(tuner,
                    new org.firstinspires.ftc.teamcode.apexpathing.Constants());
            JSONObject before = context.getConfig().getCalibrationValues();
            JSONObject changed = new JSONObject(before.toString());
            changed.put("customEncoderResolutionIn", 2.0);
            boolean accepted = context.acceptGeometry("FORWARD",
                    new CalibrationCandidate(changed, Collections.singletonList("test")));
            assertTrue("save to a regular file path must fail", !accepted);
            assertEquals(before.toString(), context.getConfig().getCalibrationValues().toString());
            assertEquals(LocalizationConstants.Status.UNCALIBRATED,
                    context.getCalibration().getGeometryStepStatus("DEFAULT", "FORWARD"));
            assertContains(context.getLastSaveError(), "Cannot create tuning-data directory");
        } finally {
            restoreProperty(ApexStorage.DIRECTORY_PROPERTY, oldStorage);
        }
    }

    @Test(timeout = 10_000L)
    public void imuFailureDuringAutomaticSpinStopsDriveWithoutRetrying() throws Exception {
        File storage = new File("build/localization-tuner-test/" + System.nanoTime());
        String oldStorage = System.getProperty(ApexStorage.DIRECTORY_PROPERTY);
        System.setProperty(ApexStorage.DIRECTORY_PROPERTY, storage.getAbsolutePath());
        ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
        hardware.hardwareMap.register("imu", simulatedImu(hardware, 8));
        List<String> frames = Collections.synchronizedList(new ArrayList<>());
        ApexSimTelemetry telemetry = new ApexSimTelemetry(frames::add);
        telemetry.setMsTransmissionInterval(0);
        LocalizationTuner tuner = configuredTuner(new LocalizationTuner(), hardware, telemetry);
        SimLinearOpModeBridge.Session session = SimLinearOpModeBridge.initialize(tuner, () -> { });
        try {
            await(session, tuner, telemetry, hardware, frames,
                    "Choose a test", 2_000L);
            SimLinearOpModeBridge.start(session);
            moveSelectionDown(session, tuner, telemetry, hardware, 3);
            press(session, tuner, telemetry, hardware, Button.A);
            await(session, tuner, telemetry, hardware, frames,
                    "Reference IMU", 2_000L);
            press(session, tuner, telemetry, hardware, Button.A);
            await(session, tuner, telemetry, hardware, frames,
                    "Could not record Device 'imu' stopped returning orientation", 3_000L);
            assertContains(latestFrame(frames),
                    "Could not record Device 'imu' stopped returning orientation",
                    "Stop the OpMode and run the phase again.");
            assertMotorsStopped(hardware);
            assertTrue("a failed recording must not create calibration",
                    !new File(storage, "localization.json").exists());
        } finally {
            SimLinearOpModeBridge.stop(session);
            restoreProperty(ApexStorage.DIRECTORY_PROPERTY, oldStorage);
        }
    }

    @Test(timeout = 120_000L)
    public void healthyImuCanBeOverriddenByManualReferenceAndSaved() throws Exception {
        File storage = new File("build/localization-tuner-test/" + System.nanoTime());
        String oldStorage = System.getProperty(ApexStorage.DIRECTORY_PROPERTY);
        System.setProperty(ApexStorage.DIRECTORY_PROPERTY, storage.getAbsolutePath());
        ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
        hardware.hardwareMap.register("imu", simulatedImu(hardware));
        List<String> frames = Collections.synchronizedList(new ArrayList<>());
        ApexSimTelemetry telemetry = new ApexSimTelemetry(frames::add);
        telemetry.setMsTransmissionInterval(0);
        LocalizationTuner tuner = configuredTuner(new PinpointOffsetTuner(), hardware, telemetry);
        SimLinearOpModeBridge.Session session = SimLinearOpModeBridge.initialize(tuner, () -> { });
        try {
            await(session, tuner, telemetry, hardware, frames,
                    "Choose a test", 2_000L);
            SimLinearOpModeBridge.start(session);
            moveSelectionDown(session, tuner, telemetry, hardware, 3);
            press(session, tuner, telemetry, hardware, Button.A);
            await(session, tuner, telemetry, hardware, frames,
                    "Reference IMU", 2_000L);
            press(session, tuner, telemetry, hardware, Button.DPAD_RIGHT);
            assertContains(latestFrame(frames), "Reference Manual");
            press(session, tuner, telemetry, hardware, Button.A);
            await(session, tuner, telemetry, hardware, frames, "Measured rotation", 60_000L);
            press(session, tuner, telemetry, hardware, Button.A);
            await(session, tuner, telemetry, hardware, frames, "A: save", 2_000L);
            press(session, tuner, telemetry, hardware, Button.A);
            await(session, tuner, telemetry, hardware, frames, "FILTER <", 2_000L);

            JSONObject setup = readSavedSetup(storage);
            assertEquals("ACCEPTED",
                    setup.getJSONObject("geometrySteps").getString("ROTATION"));
            assertTrue(setup.getJSONObject("geometry").getDouble("xIn") > 0.0);
            assertTrue(setup.getJSONObject("geometry").getDouble("yIn") < 0.0);
        } finally {
            SimLinearOpModeBridge.stop(session);
            restoreProperty(ApexStorage.DIRECTORY_PROPERTY, oldStorage);
        }
    }

    @Test(timeout = 90_000L)
    public void movingAverageCollectionSavesAndReloadsModeAndWindow() throws Exception {
        File storage = new File("build/localization-tuner-test/" + System.nanoTime());
        String oldStorage = System.getProperty(ApexStorage.DIRECTORY_PROPERTY);
        System.setProperty(ApexStorage.DIRECTORY_PROPERTY, storage.getAbsolutePath());
        ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
        List<String> frames = Collections.synchronizedList(new ArrayList<>());
        ApexSimTelemetry telemetry = new ApexSimTelemetry(frames::add);
        telemetry.setMsTransmissionInterval(0);
        LocalizationTuner tuner = configuredTuner(new LocalizationTuner(), hardware, telemetry);
        SimLinearOpModeBridge.Session session = SimLinearOpModeBridge.initialize(tuner, () -> { });
        try {
            await(session, tuner, telemetry, hardware, frames,
                    "Choose a test", 2_000L);
            SimLinearOpModeBridge.start(session);
            moveSelectionDown(session, tuner, telemetry, hardware, 4);
            press(session, tuner, telemetry, hardware, Button.A);
            await(session, tuner, telemetry, hardware, frames,
                    "Selected Unified adaptive Kalman", 2_000L);
            press(session, tuner, telemetry, hardware, Button.DPAD_RIGHT);
            press(session, tuner, telemetry, hardware, Button.DPAD_UP);
            press(session, tuner, telemetry, hardware, Button.DPAD_UP);
            assertContains(latestFrame(frames), "Selected Moving average", "Window 9");
            press(session, tuner, telemetry, hardware, Button.A);
            await(session, tuner, telemetry, hardware, frames,
                    "Move X", 5_000L);
            tuner.gamepad1.left_stick_y = -0.45f;
            tuner.gamepad1.left_stick_x = -0.20f;
            tuner.gamepad1.right_stick_x = -0.60f;
            await(session, tuner, telemetry, hardware, frames,
                    "Release the sticks and keep still", 60_000L);
            tuner.gamepad1.left_stick_y = 0.0f;
            tuner.gamepad1.left_stick_x = 0.0f;
            tuner.gamepad1.right_stick_x = 0.0f;
            await(session, tuner, telemetry, hardware, frames, "A: save", 5_000L);
            assertContains(latestFrame(frames), "Estimator Moving average", "Window 9");
            press(session, tuner, telemetry, hardware, Button.A);
            await(session, tuner, telemetry, hardware, frames, "DIRECTIONS <", 2_000L);

            JSONObject filter = readSavedSetup(storage).getJSONObject("filter");
            assertEquals(BaseLocalizer.VelocityFilterMode.MOVING_AVERAGE.name(),
                    filter.getString("mode"));
            assertEquals(9, filter.getInt("window"));
            Pinpoint.Constants reloadedConfig = pinpointConfig();
            LocalizationConstants persisted = LocalizationConstants.load();
            persisted.applyGeometry("DEFAULT", reloadedConfig);
            BaseLocalizer<?> reloaded = reloadedConfig.build(hardware.hardwareMap);
            assertTrue(persisted.applyFilter("DEFAULT", reloadedConfig, reloaded));
            assertEquals(BaseLocalizer.VelocityFilterMode.MOVING_AVERAGE,
                    reloaded.getVelocityFilterMode());
            assertEquals(9, reloaded.getFilterWindowSize());
        } finally {
            SimLinearOpModeBridge.stop(session);
            restoreProperty(ApexStorage.DIRECTORY_PROPERTY, oldStorage);
        }
    }

    @Test(timeout = 15_000L)
    public void acceptedGeometryPhaseSurvivesOpModeRestart() throws Exception {
        File storage = new File("build/localization-tuner-test/" + System.nanoTime());
        String oldStorage = System.getProperty(ApexStorage.DIRECTORY_PROPERTY);
        System.setProperty(ApexStorage.DIRECTORY_PROPERTY, storage.getAbsolutePath());
        ApexSimulation.Hardware firstHardware = ApexSimulation.createHardware();
        List<String> firstFrames = Collections.synchronizedList(new ArrayList<>());
        ApexSimTelemetry firstTelemetry = new ApexSimTelemetry(firstFrames::add);
        firstTelemetry.setMsTransmissionInterval(0);
        LocalizationTuner first = configuredTuner(new LocalizationTuner(), firstHardware,
                firstTelemetry);
        SimLinearOpModeBridge.Session firstSession = SimLinearOpModeBridge.initialize(first,
                () -> { });
        try {
            await(firstSession, first, firstTelemetry, firstHardware, firstFrames,
                    "Choose a test", 2_000L);
            SimLinearOpModeBridge.start(firstSession);
            moveSelectionDown(firstSession, first, firstTelemetry, firstHardware, 1);
            runDistance(firstSession, first, firstTelemetry, firstHardware, firstFrames,
                    "Forward distance");
        } finally {
            SimLinearOpModeBridge.stop(firstSession);
        }

        ApexSimulation.Hardware secondHardware = ApexSimulation.createHardware();
        List<String> secondFrames = Collections.synchronizedList(new ArrayList<>());
        ApexSimTelemetry secondTelemetry = new ApexSimTelemetry(secondFrames::add);
        secondTelemetry.setMsTransmissionInterval(0);
        LocalizationTuner second = configuredTuner(new LocalizationTuner(), secondHardware,
                secondTelemetry);
        SimLinearOpModeBridge.Session secondSession = SimLinearOpModeBridge.initialize(second,
                () -> { });
        try {
            await(secondSession, second, secondTelemetry, secondHardware, secondFrames,
                    "Choose a test", 2_000L);
            press(secondSession, second, secondTelemetry, secondHardware, Button.DPAD_DOWN);
            assertContains(latestFrame(secondFrames), "[DONE] FORWARD DISTANCE <");
        } finally {
            SimLinearOpModeBridge.stop(secondSession);
            restoreProperty(ApexStorage.DIRECTORY_PROPERTY, oldStorage);
        }
    }

    private static void runDirections(SimLinearOpModeBridge.Session session,
            LocalizationTuner tuner, ApexSimTelemetry telemetry,
            ApexSimulation.Hardware hardware, List<String> frames) throws Exception {
        selectAndBegin(session, tuner, telemetry, hardware, frames, "Direction check");
        await(session, tuner, telemetry, hardware, frames, "A: save", 5_000L);
        assertContains(latestFrame(frames), "Forward CORRECT",
                "Strafe CORRECT", "Turn CORRECT", "IMU spin axis (x, y, z)");
        press(session, tuner, telemetry, hardware, Button.A);
        await(session, tuner, telemetry, hardware, frames, "FORWARD DISTANCE <", 2_000L);
    }

    private static void runDistance(SimLinearOpModeBridge.Session session,
            LocalizationTuner tuner, ApexSimTelemetry telemetry,
            ApexSimulation.Hardware hardware, List<String> frames, String phase) throws Exception {
        selectAndBegin(session, tuner, telemetry, hardware, frames, phase);
        // The trial uses wall time on a separate OpMode thread. Allow scheduler/GC pauses
        // in the combined simulation suite without changing the trial or its assertions.
        await(session, tuner, telemetry, hardware, frames, "Measured distance", 10_000L);
        assertTrue("localizer must report enough travel to fit " + phase + "; entered="
                        + parseMeasuredInches(latestFrame(frames)) + ", sim pose="
                        + hardware.drivetrain.position.x + ","
                        + hardware.drivetrain.position.y + ","
                        + hardware.drivetrain.position.theta,
                parseMeasuredInches(latestFrame(frames)) >= 0.5);
        enterMinimumPhysicalDistance(session, tuner, telemetry, hardware, frames);
        press(session, tuner, telemetry, hardware, Button.A);
        await(session, tuner, telemetry, hardware, frames, "Trial direction negative", 2_000L);
        await(session, tuner, telemetry, hardware, frames, "Measured distance", 10_000L);
        enterMinimumPhysicalDistance(session, tuner, telemetry, hardware, frames);
        press(session, tuner, telemetry, hardware, Button.A);
        await(session, tuner, telemetry, hardware, frames, "A: save", 2_000L);
        assertContains(latestFrame(frames), "Both powered directions were measured.");
        press(session, tuner, telemetry, hardware, Button.A);
        String next = phase.startsWith("Forward") ? "STRAFE DISTANCE <" : "ROTATION <";
        await(session, tuner, telemetry, hardware, frames, next, 2_000L);
    }

    private static void runAutomaticSpin(SimLinearOpModeBridge.Session session,
            LocalizationTuner tuner, ApexSimTelemetry telemetry,
            ApexSimulation.Hardware hardware, List<String> frames) throws Exception {
        press(session, tuner, telemetry, hardware, Button.A);
        await(session, tuner, telemetry, hardware, frames, "Rotation geometry", 2_000L);
        assertContains(latestFrame(frames), "Reference IMU",
                "Robot spins left at 45 deg/s for 5 rotations.");
        // Previous translation trials leave simulated wheel momentum. Start the spin only after
        // the plant actually rests, so translation is not mistaken for a pod-offset error.
        long settleDeadline = System.nanoTime() + 5_000_000_000L;
        while ((Math.hypot(hardware.drivetrain.velocity.x, hardware.drivetrain.velocity.y) > 0.01
                || Math.abs(hardware.drivetrain.velocity.theta) > 0.01) && System.nanoTime() < settleDeadline) {
            pump(session, tuner, telemetry, hardware, 50L);
        }
        assertTrue("Spin calibration must begin at rest",
                Math.hypot(hardware.drivetrain.velocity.x, hardware.drivetrain.velocity.y) <= 0.01);
        press(session, tuner, telemetry, hardware, Button.A);
        await(session, tuner, telemetry, hardware, frames, "A: save", 60_000L);
        assertContains(latestFrame(frames), "IMU axis");
        press(session, tuner, telemetry, hardware, Button.A);
        await(session, tuner, telemetry, hardware, frames, "FILTER <", 2_000L);
    }

    private static void runKalmanCollection(SimLinearOpModeBridge.Session session,
            LocalizationTuner tuner, ApexSimTelemetry telemetry,
            ApexSimulation.Hardware hardware, List<String> frames) throws Exception {
        press(session, tuner, telemetry, hardware, Button.A);
        await(session, tuner, telemetry, hardware, frames,
                "Position, velocity and acceleration filter", 2_000L);
        assertContains(latestFrame(frames), "Selected Unified adaptive Kalman");
        press(session, tuner, telemetry, hardware, Button.A);
        await(session, tuner, telemetry, hardware, frames, "Move X",
                5_000L);
        tuner.gamepad1.left_stick_y = -0.55f;
        tuner.gamepad1.left_stick_x = -0.25f;
        tuner.gamepad1.right_stick_x = -0.60f;
        await(session, tuner, telemetry, hardware, frames,
                "Release the sticks and keep still", 60_000L);
        tuner.gamepad1.left_stick_y = 0.0f;
        tuner.gamepad1.left_stick_x = 0.0f;
        tuner.gamepad1.right_stick_x = 0.0f;
        await(session, tuner, telemetry, hardware, frames, "A: save", 5_000L);
        assertContains(latestFrame(frames), "Estimator Unified adaptive Kalman", "X velocity R / jerk Q",
                "Y velocity R / jerk Q", "Heading velocity R / jerk Q");
        press(session, tuner, telemetry, hardware, Button.A);
        await(session, tuner, telemetry, hardware, frames, "DIRECTIONS <", 2_000L);
    }

    /** JUnit drives and compares localization directly; no validation OpMode phase. */
    private static void validateLocalization(SimLinearOpModeBridge.Session session,
            LocalizationTuner tuner, ApexSimTelemetry telemetry,
            ApexSimulation.Hardware hardware) throws Exception {
        java.lang.reflect.Field field = LocalizationTuner.class.getDeclaredField("context");
        field.setAccessible(true);
        LocalizationTunerContext context = (LocalizationTunerContext) field.get(tuner);
        Pose initial = context.getLocalizer().getPose();
        double physicalX = hardware.drivetrain.position.x, physicalY = hardware.drivetrain.position.y;
        double physicalHeading = hardware.drivetrain.position.theta;
        context.getDrivetrain().moveWithVectors(.35, .15, .2);
        try {
            pump(session, tuner, telemetry, hardware, 1_000L);
            Pose measured = context.getLocalizer().getPose();
            assertEquals("Calibrated X", hardware.drivetrain.position.x - physicalX, measured.getX().getIn() - initial.getX().getIn(), 1.0);
            assertEquals("Calibrated Y", hardware.drivetrain.position.y - physicalY, measured.getY().getIn() - initial.getY().getIn(), 1.0);
            assertEquals("Calibrated heading", 0.0,
                    geometry.Angle.wrap(measured.getHeading().getRad() - initial.getHeading().getRad()
                            - hardware.drivetrain.position.theta + physicalHeading), .1);
        } finally { context.stop(); }
    }

    private static void selectAndBegin(SimLinearOpModeBridge.Session session,
            LocalizationTuner tuner, ApexSimTelemetry telemetry,
            ApexSimulation.Hardware hardware, List<String> frames, String phase) throws Exception {
        press(session, tuner, telemetry, hardware, Button.A);
        await(session, tuner, telemetry, hardware, frames, phase, 2_000L);
        assertContains(latestFrame(frames), "A: begin");
        press(session, tuner, telemetry, hardware, Button.A);
        pump(session, tuner, telemetry, hardware, 60L);
    }

    private static void await(SimLinearOpModeBridge.Session session, LocalizationTuner tuner,
            ApexSimTelemetry telemetry, ApexSimulation.Hardware hardware, List<String> frames,
            String expected, long timeoutMillis) throws Exception {
        long deadline = System.nanoTime() + timeoutMillis * 1_000_000L;
        while (!latestFrame(frames).contains(expected) && System.nanoTime() < deadline) {
            pump(session, tuner, telemetry, hardware, 20L);
        }
        assertTrue("Timed out waiting for '" + expected + "'. Latest telemetry:\n"
                + latestFrame(frames), latestFrame(frames).contains(expected));
    }

    private static void press(SimLinearOpModeBridge.Session session, LocalizationTuner tuner,
            ApexSimTelemetry telemetry, ApexSimulation.Hardware hardware, Button button)
            throws Exception {
        button.set(tuner.gamepad1, true);
        SimLinearOpModeBridge.eventLoopIteration(session, tuner.gamepad1, tuner.gamepad2);
        long consumedDeadline = System.nanoTime() + 2_000_000_000L;
        // The SDK stores one pending edge, not an event queue. Wait for the OpMode to consume
        // this edge before sending another; two quick presses can otherwise collapse under GC.
        while (button.pending(tuner.gamepad1) && System.nanoTime() < consumedDeadline) {
            pump(session, tuner, telemetry, hardware, 20L);
        }
        assertFalse("OpMode did not consume " + button, button.pending(tuner.gamepad1));
        pump(session, tuner, telemetry, hardware, 40L);
        button.set(tuner.gamepad1, false);
        pump(session, tuner, telemetry, hardware, 40L);
    }

    private static void pump(SimLinearOpModeBridge.Session session, LocalizationTuner tuner,
            ApexSimTelemetry telemetry, ApexSimulation.Hardware hardware, long milliseconds)
            throws Exception {
        long deadline = System.nanoTime() + milliseconds * 1_000_000L;
        while (System.nanoTime() < deadline) {
            long now = System.nanoTime();
            double remaining = Math.max(0.0, Math.min(0.05,
                    DeltaTime.seconds(now - hardware.lastPhysicsUpdateNanos)));
            hardware.lastPhysicsUpdateNanos = now;
            while (remaining > 1e-9) {
                double step = Math.min(0.005, remaining);
                stepPhysics(hardware, step);
                remaining -= step;
            }
            SimLinearOpModeBridge.eventLoopIteration(session, tuner.gamepad1, tuner.gamepad2);
            telemetry.update();
            Thread.sleep(5L);
        }
    }

    private static void stepPhysics(ApexSimulation.Hardware hardware, double seconds)
            throws Exception {
        double[] wheelVelocities = new double[hardware.drivetrain.motorNames.length];
        for (int i = 0; i < hardware.drivetrain.motorNames.length; i++) {
            SimMotor motor = (SimMotor) hardware.hardwareMap.get(
                    DcMotorEx.class, hardware.drivetrain.motorNames[i]);
            motor.update(seconds);
            wheelVelocities[i] = motor.getVelocity();
        }
        java.lang.reflect.Method kinematics = hardware.drivetrain.getClass()
                .getDeclaredMethod("forwardKinematics", double[].class);
        kinematics.setAccessible(true);
        MotionVector robotVelocity = (MotionVector) kinematics.invoke(
                hardware.drivetrain, (Object) wheelVelocities);
        hardware.drivetrain.velocity = robotVelocity.toFieldFrame(
                hardware.drivetrain.position.theta);
        hardware.drivetrain.position = hardware.drivetrain.position.step(
                hardware.drivetrain.velocity, seconds);
    }

    private static IMU simulatedImu(ApexSimulation.Hardware hardware) {
        return simulatedImu(hardware, Integer.MAX_VALUE);
    }

    private static IMU simulatedImu(ApexSimulation.Hardware hardware,
                                    int successfulOrientationReads) {
        AtomicInteger reads = new AtomicInteger();
        return (IMU) Proxy.newProxyInstance(LocalizationTunerWorkflowTest.class.getClassLoader(),
                new Class<?>[] {IMU.class}, (proxy, method, args) -> {
                    if (method.getName().equals("getRobotOrientationAsQuaternion")) {
                        if (reads.getAndIncrement() >= successfulOrientationReads) {
                            throw new IllegalStateException("simulated IMU read failure");
                        }
                        double half = hardware.drivetrain.position.theta / 2.0;
                        return new Quaternion((float) Math.cos(half), 0.0f, 0.0f,
                                (float) Math.sin(half), System.nanoTime());
                    }
                    if (method.getName().equals("getDeviceName")) { return "Simulated IMU"; }
                    if (method.getName().equals("getManufacturer")) {
                        return HardwareDevice.Manufacturer.Other;
                    }
                    if (method.getName().equals("getVersion")) { return 1; }
                    if (method.getName().equals("getConnectionInfo")) { return "simulated"; }
                    return defaultValue(method.getReturnType());
                });
    }

    private static void moveSelectionDown(SimLinearOpModeBridge.Session session,
            LocalizationTuner tuner, ApexSimTelemetry telemetry,
            ApexSimulation.Hardware hardware, int count) throws Exception {
        for (int i = 0; i < count; i++) {
            press(session, tuner, telemetry, hardware, Button.DPAD_DOWN);
        }
    }

    private static JSONObject readSavedSetup(File storage) throws Exception {
        File saved = new File(storage, "localization.json");
        assertTrue("localization.json should exist", saved.isFile());
        JSONObject root = new JSONObject(new String(Files.readAllBytes(saved.toPath()),
                StandardCharsets.UTF_8));
        return root.getJSONObject("setups").getJSONObject("DEFAULT");
    }

    private static Servo simulatedServo() {
        final double[] position = {0.0};
        return (Servo) Proxy.newProxyInstance(LocalizationTunerWorkflowTest.class.getClassLoader(),
                new Class<?>[] {Servo.class}, (proxy, method, args) -> {
                    if (method.getName().equals("setPosition")) {
                        position[0] = ((Number) args[0]).doubleValue();
                        return null;
                    }
                    if (method.getName().equals("getPosition")) { return position[0]; }
                    if (method.getName().equals("getManufacturer")) {
                        return HardwareDevice.Manufacturer.Other;
                    }
                    if (method.getName().equals("getDeviceName")) { return "Simulated Servo"; }
                    if (method.getName().equals("getVersion")) { return 1; }
                    if (method.getName().equals("getConnectionInfo")) { return "simulated"; }
                    return defaultValue(method.getReturnType());
                });
    }

    private static void verifyTankMenu() throws Exception {
        ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
        List<String> frames = Collections.synchronizedList(new ArrayList<>());
        ApexSimTelemetry telemetry = new ApexSimTelemetry(frames::add);
        telemetry.setMsTransmissionInterval(0);
        LocalizationTuner tuner = configuredTuner(new TankTuner(), hardware, telemetry);
        SimLinearOpModeBridge.Session session = SimLinearOpModeBridge.initialize(tuner, () -> { });
        try {
            await(session, tuner, telemetry, hardware, frames,
                    "Choose a test", 2_000L);
            moveSelectionDown(session, tuner, telemetry, hardware, 1);
            assertContains(latestFrame(frames), "[READY] FORWARD DISTANCE <");
            moveSelectionDown(session, tuner, telemetry, hardware, 1);
            assertContains(latestFrame(frames), "[N/A] STRAFE DISTANCE <");
            moveSelectionDown(session, tuner, telemetry, hardware, 1);
            assertContains(latestFrame(frames), "[READY] ROTATION <");
            moveSelectionDown(session, tuner, telemetry, hardware, 2);
            SimLinearOpModeBridge.start(session);
            press(session, tuner, telemetry, hardware, Button.A);
            await(session, tuner, telemetry, hardware, frames,
                    "Direction check", 2_000L);
            press(session, tuner, telemetry, hardware, Button.A);
            await(session, tuner, telemetry, hardware, frames, "A: save", 4_000L);
            assertContains(latestFrame(frames), "Forward CORRECT", "Turn CORRECT");
            assertFalse("tank direction check must omit strafe",
                    latestFrame(frames).contains("Strafe"));
        } finally {
            SimLinearOpModeBridge.stop(session);
        }
    }

    private static void verifyDualActuatedMenuSwitch() throws Exception {
        ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
        hardware.hardwareMap.register("driveMode", simulatedServo());
        List<String> frames = Collections.synchronizedList(new ArrayList<>());
        ApexSimTelemetry telemetry = new ApexSimTelemetry(frames::add);
        telemetry.setMsTransmissionInterval(0);
        LocalizationTuner tuner = configuredTuner(new SeparateDualTuner(), hardware, telemetry);
        SimLinearOpModeBridge.Session session = SimLinearOpModeBridge.initialize(tuner, () -> { });
        try {
            await(session, tuner, telemetry, hardware, frames,
                    "Choose a test", 2_000L);
            assertContains(latestFrame(frames), "Drive mode HOLONOMIC",
                    "[READY] DIRECTIONS");
            press(session, tuner, telemetry, hardware, Button.DPAD_RIGHT);
            await(session, tuner, telemetry, hardware, frames, "Drive mode TANK", 2_000L);
            moveSelectionDown(session, tuner, telemetry, hardware, 2);
            assertContains(latestFrame(frames), "[N/A] STRAFE DISTANCE <");
            moveSelectionDown(session, tuner, telemetry, hardware, 1);
            assertContains(latestFrame(frames), "[N/A] ROTATION <");
            press(session, tuner, telemetry, hardware, Button.DPAD_RIGHT);
            await(session, tuner, telemetry, hardware, frames, "Drive mode HOLONOMIC", 2_000L);
            moveSelectionDown(session, tuner, telemetry, hardware, 2);
            assertContains(latestFrame(frames), "[READY] STRAFE DISTANCE <");
        } finally {
            SimLinearOpModeBridge.stop(session);
        }
    }

    private static LocalizationTuner configuredTuner(LocalizationTuner tuner,
            ApexSimulation.Hardware hardware, ApexSimTelemetry telemetry) {
        tuner.hardwareMap = hardware.hardwareMap;
        tuner.telemetry = telemetry;
        tuner.gamepad1 = new Gamepad();
        tuner.gamepad2 = new Gamepad();
        return tuner;
    }

    private static void assertMotorsStopped(ApexSimulation.Hardware hardware) {
        for (String name : hardware.drivetrain.motorNames) {
            SimMotor motor = (SimMotor) hardware.hardwareMap.get(DcMotorEx.class, name);
            assertEquals("drive should be stopped for " + name, 0.0, motor.getPower(), 0.0);
        }
    }

    private static Object defaultValue(Class<?> type) {
        if (!type.isPrimitive()) {
            return type.isArray() ? Array.newInstance(type.getComponentType(), 0) : null;
        }
        if (type == boolean.class) { return false; }
        if (type == char.class) { return '\0'; }
        if (type == byte.class) { return (byte) 0; }
        if (type == short.class) { return (short) 0; }
        if (type == int.class) { return 0; }
        if (type == long.class) { return 0L; }
        if (type == float.class) { return 0.0f; }
        if (type == double.class) { return 0.0; }
        return null;
    }

    private static double parseMeasuredInches(String frame) {
        String marker = "Measured distance ";
        int start = frame.indexOf(marker);
        int end = frame.indexOf(" in", start);
        return Double.parseDouble(frame.substring(start + marker.length(), end));
    }

    private static void enterMinimumPhysicalDistance(SimLinearOpModeBridge.Session session,
            LocalizationTuner tuner, ApexSimTelemetry telemetry,
            ApexSimulation.Hardware hardware, List<String> frames) throws Exception {
        int edits = 0;
        while (parseMeasuredInches(latestFrame(frames)) < 2.0 && edits++ < 12) {
            press(session, tuner, telemetry, hardware, Button.DPAD_UP);
        }
        assertTrue("distance entry should accept a physical measurement of at least two inches",
                parseMeasuredInches(latestFrame(frames)) >= 2.0);
    }

    private static void assertPositiveTuning(JSONObject tuning) throws Exception {
        assertTrue(tuning.getDouble("measurementVariance") > 0.0);
        assertTrue(tuning.getDouble("processVariance") > 0.0);
    }

    private static void assertContains(String actual, String... expected) {
        for (String value : expected) {
            assertTrue("Expected telemetry to contain '" + value + "':\n" + actual,
                    actual.contains(value));
        }
    }

    private static String latestFrame(List<String> frames) {
        return frames.isEmpty() ? "" : frames.get(frames.size() - 1);
    }

    private static void restoreProperty(String name, String value) {
        if (value == null) { System.clearProperty(name); }
        else { System.setProperty(name, value); }
    }

    private enum Button {
        A {
            @Override void set(Gamepad gamepad, boolean pressed) { gamepad.a = pressed; }
        },
        B {
            @Override void set(Gamepad gamepad, boolean pressed) { gamepad.b = pressed; }
        },
        X {
            @Override void set(Gamepad gamepad, boolean pressed) { gamepad.x = pressed; }
        },
        DPAD_UP {
            @Override void set(Gamepad gamepad, boolean pressed) { gamepad.dpad_up = pressed; }
        },
        DPAD_DOWN {
            @Override void set(Gamepad gamepad, boolean pressed) { gamepad.dpad_down = pressed; }
        },
        DPAD_RIGHT {
            @Override void set(Gamepad gamepad, boolean pressed) { gamepad.dpad_right = pressed; }
        };

        abstract void set(Gamepad gamepad, boolean pressed);

        boolean pending(Gamepad gamepad) throws Exception {
            java.lang.reflect.Field changesField = Gamepad.class.getDeclaredField("changes");
            changesField.setAccessible(true);
            Object changes = changesField.get(gamepad);
            String field = name().startsWith("DPAD_")
                    ? "dpad" + name().substring(5, 6) + name().substring(6).toLowerCase(java.util.Locale.US)
                    : name().toLowerCase(java.util.Locale.US);
            java.lang.reflect.Field buttonField = changes.getClass().getDeclaredField(field);
            buttonField.setAccessible(true);
            Object monitor = buttonField.get(changes);
            java.lang.reflect.Field pendingField = monitor.getClass().getDeclaredField("pressNotify");
            pendingField.setAccessible(true);
            return pendingField.getBoolean(monitor);
        }
    }

    private static final class MonitorOnlyTuner extends LocalizationTuner {
        @Override protected ApexConstants createConstants() {
            return new ApexConstants() {
                @Override public BaseDrivetrainConstants<?> drivetrainConstants() {
                    return mecanumConfig();
                }

                @Override public BaseLocalizerConstants<?> localizerConstants() {
                    return new MonitorConfig();
                }
            };
        }
    }

    private static final class PinpointOffsetTuner extends LocalizationTuner {
        @Override protected ApexConstants createConstants() {
            return new ApexConstants() {
                @Override public BaseDrivetrainConstants<?> drivetrainConstants() {
                    return mecanumConfig();
                }

                @Override public BaseLocalizerConstants<?> localizerConstants() {
                    return pinpointConfig().setOffsets(5.0, -3.0, DistUnit.IN);
                }
            };
        }
    }

    private static final class TankTuner extends LocalizationTuner {
        @Override protected ApexConstants createConstants() {
            return new ApexConstants() {
                @Override public BaseDrivetrainConstants<?> drivetrainConstants() {
                    return new Tank.Constants()
                            .setFrontLeftMotor(new Motor(ApexSimulation.FRONT_LEFT_MOTOR))
                            .setFrontRightMotor(new Motor(ApexSimulation.FRONT_RIGHT_MOTOR))
                            .setBackLeftMotor(new Motor(ApexSimulation.BACK_LEFT_MOTOR))
                            .setBackRightMotor(new Motor(ApexSimulation.BACK_RIGHT_MOTOR));
                }

                @Override public BaseLocalizerConstants<?> localizerConstants() {
                    return pinpointConfig();
                }
            };
        }
    }

    private static final class SeparateDualTuner extends LocalizationTuner {
        @Override protected ApexConstants createConstants() {
            return new ApexConstants() {
                @Override public BaseDrivetrainConstants<?> drivetrainConstants() {
                    return new DualActuated.Constants()
                            .setFrontLeftMotor(new Motor(ApexSimulation.FRONT_LEFT_MOTOR))
                            .setFrontRightMotor(new Motor(ApexSimulation.FRONT_RIGHT_MOTOR))
                            .setBackLeftMotor(new Motor(ApexSimulation.BACK_LEFT_MOTOR))
                            .setBackRightMotor(new Motor(ApexSimulation.BACK_RIGHT_MOTOR))
                            .setTransitionSeconds(0.0)
                            .setInitialState(DualActuated.DriveState.HOLONOMIC)
                            .addActuator("driveMode", 1.0, 0.0);
                }

                @Override public BaseLocalizerConstants<?> localizerConstants() {
                    return pinpointConfig();
                }

                @Override public boolean usesSharedLocalizer() { return false; }

                @Override public BaseLocalizerConstants<?> localizerConstants(
                        follower.FollowerConstants.Profile profile) {
                    return profile == follower.FollowerConstants.Profile.TANK
                            ? new MonitorConfig() : pinpointConfig();
                }
            };
        }
    }

    private static Pinpoint.Constants pinpointConfig() {
        return new Pinpoint.Constants().setName(ApexSimulation.PINPOINT)
                .setOffsets(0, 0, DistUnit.IN)
                .setEncoderDirections(Pinpoint.EncoderDirection.FORWARD,
                        Pinpoint.EncoderDirection.FORWARD)
                .setEncoderResolution(Pinpoint.GoBildaPods.goBILDA_4_BAR_POD);
    }

    private static Mecanum.Constants mecanumConfig() {
        return new Mecanum.Constants()
                .setFrontLeftMotor(new Motor(ApexSimulation.FRONT_LEFT_MOTOR))
                .setFrontRightMotor(new Motor(ApexSimulation.FRONT_RIGHT_MOTOR))
                .setBackLeftMotor(new Motor(ApexSimulation.BACK_LEFT_MOTOR))
                .setBackRightMotor(new Motor(ApexSimulation.BACK_RIGHT_MOTOR));
    }

    private static final class MonitorConfig implements BaseLocalizerConstants<MonitorConfig> {
        @Override public BaseLocalizer<?> build(
                com.qualcomm.robotcore.hardware.HardwareMap hardwareMap) {
            return new MonitorLocalizer(this);
        }
    }

    private static final class MonitorLocalizer extends BaseLocalizer<MonitorConfig> {
        MonitorLocalizer(MonitorConfig config) { super(config); }
        @Override public void update() { calculate(UpdateType.BOTH); }
        @Override public void setPose(Pose pose) { resetKinematicEstimate(pose); }
    }
}
