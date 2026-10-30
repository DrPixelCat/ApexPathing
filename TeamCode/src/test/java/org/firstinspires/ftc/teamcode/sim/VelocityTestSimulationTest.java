package org.firstinspires.ftc.teamcode.sim;

import com.qualcomm.robotcore.hardware.Gamepad;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.stream.Collectors;
import org.firstinspires.ftc.teamcode.apexpathing.VelocityTest;
import org.junit.Test;
import storage.ApexStorage;
import static org.junit.Assert.*;

/** Exercises button edges, CSV contents and motor stop using the unchanged simulator plant. */
public class VelocityTestSimulationTest {
    @Test(timeout = 20_000) public void holdRampsAndReleaseSavesOnlyTheCapturedInterval() throws Exception {
        try (Rig rig = new Rig()) {
            rig.opMode.init();
            rig.opMode.gamepad1.a = true;
            rig.loop();
            long start = System.nanoTime(), previous = start;
            while ((System.nanoTime() - start) * 1e-9 < 3.4) {
                Thread.sleep(20);
                long now = System.nanoTime();
                double remaining = (now - previous) * 1e-9;
                previous = now;
                while (remaining > 1e-9) {
                    double dt = Math.min(.005, remaining);
                    ApexSimulation.stepPhysics(rig.hardware, dt);
                    remaining -= dt;
                }
                rig.loop();
            }
            assertTrue("No file I/O during capture", rig.csvFiles().isEmpty());
            rig.opMode.gamepad1.a = false;
            rig.opMode.gamepad1.left_stick_y = -.8f;
            rig.loop();
            rig.assertStopped();
            List<Path> files = rig.csvFiles();
            assertEquals(1, files.size());
            List<String> lines = Files.readAllLines(files.get(0), StandardCharsets.UTF_8);
            assertTrue(lines.size() > 50);
            assertEquals("time_s,applied_forward_power,filtered_forward_velocity_in_s,battery_voltage_v,acceleration_in_s2,acceleration_window_time_s,acceleration_window_velocity_in_s,acceleration_window_power", lines.get(0));
            assertEquals(0, Double.parseDouble(lines.get(1).split(",", -1)[0]), 0);
            double previousTime = -1;
            for (int i = 1; i < lines.size(); i++) {
                String[] row = lines.get(i).split(",", -1);
                assertEquals(8, row.length);
                double time = Double.parseDouble(row[0]);
                assertTrue(time > previousTime);
                double expectedPower = i == 1 ? 0 : Math.min(.9, .1 * previousTime);
                assertEquals(expectedPower, Double.parseDouble(row[1]), .0011);
                previousTime = time;
            }
            String[] last = lines.get(lines.size() - 1).split(",", -1);
            assertTrue(Double.parseDouble(last[2]) > .1);
            java.lang.reflect.Field sensorField = VelocityTest.class.getDeclaredField("localizer");
            sensorField.setAccessible(true);
            localizers.BaseLocalizer<?> sensor = (localizers.BaseLocalizer<?>) sensorField.get(rig.opMode);
            double filtered = sensor.getVel().getVec().rotate(sensor.getPose().getHeading().times(-1)).getX().getIn();
            assertEquals(filtered, Double.parseDouble(last[2]), 1e-9);
            rig.loop();
            assertEquals("Idle/repositioning must not append to the capture", lines,
                    Files.readAllLines(files.get(0), StandardCharsets.UTF_8));
        }
    }

    @Test public void repeatedHoldsStartAtZeroAndStopAlsoSavesPendingSamples() throws Exception {
        try (Rig rig = new Rig()) {
            rig.opMode.init();
            StockSimTestSupport.setSpeed(rig.hardware, 10);
            rig.opMode.gamepad1.a = true;
            rig.loop();
            rig.opMode.gamepad1.a = false;
            rig.loop();
            assertEquals(1, rig.csvFiles().size());
            StockSimTestSupport.setSpeed(rig.hardware, 40);
            rig.opMode.gamepad1.a = true;
            rig.loop();
            rig.opMode.stop();
            rig.assertStopped();
            List<Path> files = rig.csvFiles();
            assertEquals("A distinct file for each hold", 2, files.size());
            for (Path file : files) {
                String[] first = Files.readAllLines(file, StandardCharsets.UTF_8).get(1).split(",", -1);
                assertEquals(0, Double.parseDouble(first[0]), 0);
                assertEquals(0, Double.parseDouble(first[1]), 0);

            }
            rig.opMode.stop();
            assertEquals("STOP must not save the same capture twice", 2, rig.csvFiles().size());
        }
    }

    @Test public void highPowerStepsDeliverSelectedPowerAndSaveOnRelease() throws Exception {
        try (Rig rig = new Rig()) {
            rig.opMode.init();
            for (int selected = 0; selected < 6; selected++) {
                if (selected > 0) {
                    rig.opMode.gamepad1.dpad_right = true; rig.loop();
                    rig.opMode.gamepad1.dpad_right = false; rig.loop();
                }
                if (selected < 3) { continue; }
                double expected = new double[]{.35, .45, .55, .7, .85, 1}[selected];
                rig.opMode.gamepad1.b = true; rig.loop();
                for (org.codeblooded.ftcodesim.hardware.devices.SimMotor motor : StockSimTestSupport.motors(rig.hardware)) {
                    assertEquals(expected, motor.getPower(), 1e-12);
                }
                rig.opMode.gamepad1.b = false; rig.loop();
                rig.assertStopped();
            }
            assertEquals(3, rig.csvFiles().size());
            for (Path file : rig.csvFiles()) { assertTrue(file.getFileName().toString().startsWith("velocity_step_")); }
        }
    }

    @Test public void accelerationRegressionRecoversKnownSlopeAndAlignedWindow() throws Exception {
        VelocityTest op = new VelocityTest();
        java.lang.reflect.Field data = VelocityTest.class.getDeclaredField("samples");
        data.setAccessible(true);
        @SuppressWarnings("unchecked") List<Object[]> rows = (List<Object[]>) data.get(op);
        for (int i = 0; i < 15; i++) {
            double t = i * .02;
            rows.add(new Object[]{t, .85, 3 + 12 * t, 13.0, "", "", "", ""});
        }
        java.lang.reflect.Method fit = VelocityTest.class.getDeclaredMethod("addAcceleration", int.class);
        fit.setAccessible(true); fit.invoke(op, 7);
        assertEquals(12, (double) rows.get(7)[4], 1e-9);
        assertEquals(.14, (double) rows.get(7)[5], 1e-9);
        assertEquals(4.68, (double) rows.get(7)[6], 1e-9);
        assertEquals(.85, (double) rows.get(7)[7], 1e-9);
    }

    private static final class Rig implements AutoCloseable {
        final String previousStorage = System.getProperty(ApexStorage.DIRECTORY_PROPERTY);
        final String previousUnlock = System.getProperty("apex.simulation.unlockTunerPhases");
        final ApexSimulation.Hardware hardware;
        final VelocityTest opMode = new VelocityTest();
        final Path storage;

        Rig() throws Exception {
            Path root = java.nio.file.Paths.get("build", "velocity-test-capture").toAbsolutePath();
            Files.createDirectories(root);
            storage = Files.createTempDirectory(root, "run-");
            System.setProperty(ApexStorage.DIRECTORY_PROPERTY, storage.toString());
            setup.hardware.RobotProfile profile = new setup.hardware.RobotProfile();
            new setup.hardware.SetupStore(storage.toFile()).save(profile);
            hardware = ApexSimulation.createHardware();
            opMode.hardwareMap = hardware.hardwareMap;
            opMode.telemetry = new ApexSimTelemetry(frame -> { });
            opMode.gamepad1 = new Gamepad();
        }

        void loop() throws Exception {
            // The SDK event loop normally updates these edges before calling the OpMode.
            java.lang.reflect.Method update = Gamepad.class.getDeclaredMethod("updateEdgeDetection");
            update.setAccessible(true);
            update.invoke(opMode.gamepad1);
            opMode.loop();
        }

        List<Path> csvFiles() throws Exception {
            try (java.util.stream.Stream<Path> files = Files.list(storage)) {
                return files.filter(path -> path.toString().endsWith(".csv")).sorted().collect(Collectors.toList());
            }
        }

        void assertStopped() {
            for (org.codeblooded.ftcodesim.hardware.devices.SimMotor motor : StockSimTestSupport.motors(hardware)) {
                assertEquals(0, motor.getPower(), 0);
            }
        }

        @Override public void close() {
            opMode.stop();
            if (previousStorage == null) { System.clearProperty(ApexStorage.DIRECTORY_PROPERTY); }
            else { System.setProperty(ApexStorage.DIRECTORY_PROPERTY, previousStorage); }
            if (previousUnlock == null) { System.clearProperty("apex.simulation.unlockTunerPhases"); }
            else { System.setProperty("apex.simulation.unlockTunerPhases", previousUnlock); }
        }
    }
}
