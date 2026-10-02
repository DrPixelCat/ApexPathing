package org.firstinspires.ftc.teamcode.sim;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.SimLinearOpModeBridge;
import com.qualcomm.robotcore.hardware.Gamepad;
import core.ApexStorage;
import core.Follower;
import core.FollowerConstants;
import org.firstinspires.ftc.teamcode.apexpathing.FollowerTuner;
import org.junit.Test;
import tuning.follower.TunerContext;
import tuning.follower.phases.ProfiledTrackingPhase;

import java.io.InputStream;
import java.nio.file.Files;
import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.atomic.AtomicBoolean;

import static org.junit.Assert.assertTrue;

/** Runs the same profiled brake measurement through the tank follower. */
public class TankProfiledTrackingPhaseSimulationTest {
    @Test(timeout = 60_000L)
    public void inadequateTankPrerequisitesDoNotMarkProfileTuned() throws Exception {
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
            while (System.nanoTime() < deadline) {
                long now = System.nanoTime();
                String frame = frames.isEmpty() ? "" : frames.get(frames.size() - 1);
                if (frame.contains("verify earlier tuning phases")) { break; }
                if (frame.contains("Press A") && now - lastPress > 250_000_000L) {
                    lastPress = now;
                }
                opMode.gamepad1.a = now - lastPress < 70_000_000L;
                double remaining = Math.max(0.0, Math.min(0.05,
                        (now - hardware.lastPhysicsUpdateNanos) * 1e-9));
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
            String last = frames.isEmpty() ? "" : frames.get(frames.size() - 1);
            assertTrue("Poor tank baseline must point back to prerequisites: " + last,
                    last.contains("verify earlier tuning phases"));
            assertTrue("Poor tank baseline must not complete the phase", !complete.get());
            assertTrue("Poor tank baseline must not be saved as tuned",
                    !FollowerConstants.getInstance().profiledTrackingTuned);
        } finally {
            if (session != null) { SimLinearOpModeBridge.stop(session); }
            if (previousUnlock == null) { System.clearProperty(FollowerTuner.UNLOCK_PHASES_PROPERTY); }
            else { System.setProperty(FollowerTuner.UNLOCK_PHASES_PROPERTY, previousUnlock); }
            environment.restoreConfiguration();
        }
    }
}
