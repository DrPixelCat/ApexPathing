package org.firstinspires.ftc.teamcode.sim;

import util.DeltaTime;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.SimLinearOpModeBridge;
import com.qualcomm.robotcore.hardware.Gamepad;
import storage.ApexStorage;
import follower.Follower;
import follower.FollowerConstants;
import org.firstinspires.ftc.teamcode.apexpathing.FollowerTuner;
import org.junit.Test;
import tuning.follower.TunerContext;
import tuning.follower.tracking.ProfiledTrackingPhase;

import java.io.InputStream;
import java.nio.file.Files;
import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.atomic.AtomicBoolean;

import static org.junit.Assert.assertTrue;

/** Runs the same profiled brake measurement through the tank follower. */
public class TankProfiledTrackingPhaseSimulationTest {
    @Test(timeout = 60_000L)
    public void junitDetectsPoorTankTrackingWithoutBlockingAcquisition() throws Exception {
        TankAutoAccuracySimulationTest environment = new TankAutoAccuracySimulationTest();
        environment.preserveConfiguration();
        String previousUnlock = System.getProperty(FollowerTuner.UNLOCK_PHASES_PROPERTY);
        SimLinearOpModeBridge.Session session = null;
        try {
            java.nio.file.Path directory = Files.createTempDirectory("apex-tank-profiled-tuner-");
            System.setProperty(ApexStorage.DIRECTORY_PROPERTY, directory.toString());
            System.setProperty(ApexSimulation.TANK_SIMULATION_PROPERTY, "true");
            System.setProperty(FollowerTuner.UNLOCK_PHASES_PROPERTY, "true");
            try (InputStream fixture = getClass().getResourceAsStream("/tank-tuned-constants.json")) {
                Files.copy(fixture, directory.resolve("constants.json"));
            }
            FollowerConstants.getInstance().reload();
            ApexSimulation.Hardware hardware = ApexSimulation.createTankHardware();
            List<String> frames = new CopyOnWriteArrayList<>();
            ApexSimTelemetry telemetry = new ApexSimTelemetry(frames::add);
            telemetry.setMsTransmissionInterval(0);
            AtomicBoolean complete = new AtomicBoolean();
            LinearOpMode opMode = new LinearOpMode() {
                @Override public void runOpMode() {
                    TunerContext context = new TunerContext(this);
                    context.setFollower(new Follower(new TankSimulationConstants(), hardwareMap, true));
                    waitForStart();
                    complete.set(new ProfiledTrackingPhase(context).run(this));
                    context.getFollower().stop();
                }
            };
            opMode.hardwareMap = hardware.hardwareMap;
            opMode.telemetry = telemetry;
            opMode.gamepad1 = new Gamepad();
            opMode.gamepad2 = new Gamepad();
            session = SimLinearOpModeBridge.initialize(opMode, () -> { });
            SimLinearOpModeBridge.start(session);
            long deadline = System.nanoTime() + 45_000_000_000L;
            long lastPress = 0L;
            boolean ran = false;
            while (System.nanoTime() < deadline) {
                long now = System.nanoTime();
                String frame = frames.isEmpty() ? "" : frames.get(frames.size() - 1);
                if (frame.contains("Tuning ") && !frame.contains("Press A when positioned")
                        && lastPress != 0L) { ran = true; }
                if (ran && frame.contains("Press A when positioned")) { break; }
                if (frame.contains("Press A") && now - lastPress > 250_000_000L) {
                    lastPress = now;
                }
                opMode.gamepad1.a = now - lastPress < 70_000_000L;
                double remaining = Math.max(0.0, Math.min(0.05,
                        DeltaTime.seconds(now - hardware.lastPhysicsUpdateNanos)));
                hardware.lastPhysicsUpdateNanos = now;
                while (remaining > 1e-9) {
                    double dt = Math.min(0.005, remaining);
                    TankAutoAccuracySimulationTest.stepPhysics(hardware, dt);
                    remaining -= dt;
                }
                SimLinearOpModeBridge.eventLoopIteration(
                        session, opMode.gamepad1, opMode.gamepad2);
                telemetry.update();
                Thread.sleep(5);
            }
            java.nio.file.Path csv;
            try (java.util.stream.Stream<java.nio.file.Path> logs = Files.list(directory)) {
                csv = logs.filter(p -> p.getFileName().toString().startsWith("profiled_tracking_")
                        && p.toString().endsWith(".csv")).findFirst().get();
            }
            AssertionError failure = org.junit.Assert.assertThrows("JUnit must detect the known poor plant response",
                    AssertionError.class, () -> ProfiledTrackingValidationTest.assertLastTrial(csv));
            assertTrue("Must reject tracking quality rather than missing acquisition",
                    failure.getMessage().contains("RMS") || failure.getMessage().contains("peak"));
            assertTrue("Validation telemetry must stay outside the tuner", frames.stream()
                    .noneMatch(f -> f.contains("tracking failed") || f.contains("verify earlier tuning phases")));
        } finally {
            if (session != null) { SimLinearOpModeBridge.stop(session); }
            if (previousUnlock == null) { System.clearProperty(FollowerTuner.UNLOCK_PHASES_PROPERTY); }
            else { System.setProperty(FollowerTuner.UNLOCK_PHASES_PROPERTY, previousUnlock); }
            environment.restoreConfiguration();
        }
    }
}
