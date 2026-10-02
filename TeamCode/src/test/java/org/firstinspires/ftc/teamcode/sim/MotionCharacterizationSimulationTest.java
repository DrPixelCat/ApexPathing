package org.firstinspires.ftc.teamcode.sim;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.SimLinearOpModeBridge;
import com.qualcomm.robotcore.hardware.Gamepad;
import core.*;
import org.firstinspires.ftc.teamcode.apexpathing.Constants;
import org.junit.Test;
import java.io.File;
import java.nio.file.Files;
import java.util.*;
import java.util.concurrent.atomic.AtomicBoolean;
import tuning.follower.TunerContext;
import tuning.follower.phases.MotionCharacterizationPhase;
import static org.junit.Assert.*;

/** Exercises the real acquisition phase with upstream stock motor/chassis physics. */
public class MotionCharacterizationSimulationTest {
    @Test(timeout = 240_000L) public void calibratesAndValidatesEveryAvailableAxis() throws Exception {
        String previous = System.getProperty(ApexStorage.DIRECTORY_PROPERTY);
        File directory = new File("build/motion-characterization-validation/phase");
        System.setProperty(ApexStorage.DIRECTORY_PROPERTY, directory.getAbsolutePath());
        SimLinearOpModeBridge.Session session = null;
        try {
            ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
            Constants robot = new Constants();
            Follower seed = new Follower(robot, hardware.hardwareMap, true);
            LocalizationConstants localization = LocalizationConstants.empty();
            localization.capture("DEFAULT", robot.localizerConstants(), seed.getLocalizer(),
                    LocalizationConstants.Status.ACCEPTED, LocalizationConstants.Status.ACCEPTED);
            localization.save();
            FollowerConstants c = FollowerConstants.getInstance();
            SimulatorTuning.apply(c); c.motionCharacterization = null;
            List<String> frames = Collections.synchronizedList(new ArrayList<>());
            ApexSimTelemetry telemetry = new ApexSimTelemetry(frames::add);
            telemetry.setMsTransmissionInterval(0);
            AtomicBoolean accepted = new AtomicBoolean();
            LinearOpMode op = new LinearOpMode() {
                @Override public void runOpMode() {
                    TunerContext context = new TunerContext(this);
                    context.setFollower(new Follower(new Constants(), hardwareMap, true));
                    waitForStart(); accepted.set(new MotionCharacterizationPhase(context).run(this));
                }
            };
            op.hardwareMap = hardware.hardwareMap; op.telemetry = telemetry;
            op.gamepad1 = new Gamepad(); op.gamepad2 = new Gamepad();
            session = SimLinearOpModeBridge.initialize(op, () -> { });
            SimLinearOpModeBridge.start(session);
            long deadline = System.nanoTime() + 220_000_000_000L;
            while (!accepted.get() && System.nanoTime() < deadline) {
                String frame = frames.isEmpty() ? "" : frames.get(frames.size() - 1);
                assertFalse(frame, frame.contains("validation failed"));
                if (frame.contains("Press A")) {
                    op.gamepad1.a = true; pump(session, op, telemetry, hardware, 40);
                    op.gamepad1.a = false;
                }
                pump(session, op, telemetry, hardware, 25);
            }
            assertTrue("Characterization never completed: " + frames.get(frames.size() - 1), accepted.get());
            assertNotNull(c.motionCharacterization);
            ApexStorage.saveConstants(c.toJson().toString(4));
            FollowerConstants restored = FollowerConstants.fromJson(new org.json.JSONObject(
                    new String(Files.readAllBytes(ApexStorage.getConstantsFile().toPath()), java.nio.charset.StandardCharsets.UTF_8)));
            for (MotionCharacterization.Axis axis : MotionCharacterization.Axis.values()) {
                for (double sign : new double[]{1, -1}) {
                    MotorPowerModel model = restored.motionCharacterization.model(axis, sign);
                    assertTrue(model.validationRms < .04);
                    System.out.println(axis + " " + sign + " gain " + model.gain0 + ", slope " + model.gainSlope
                            + ", drag " + model.drag0 + ", slope " + model.dragSlope + ", max " + model.maxVelocity
                            + ", RMS " + model.validationRms);
                }
            }
        } finally {
            if (session != null) { SimLinearOpModeBridge.stop(session); }
            if (previous == null) { System.clearProperty(ApexStorage.DIRECTORY_PROPERTY); }
            else { System.setProperty(ApexStorage.DIRECTORY_PROPERTY, previous); }
        }
    }
    private void pump(SimLinearOpModeBridge.Session session, LinearOpMode op,
                      ApexSimTelemetry telemetry, ApexSimulation.Hardware hardware, long millis) throws Exception {
        long end = System.nanoTime() + millis * 1_000_000L;
        while (System.nanoTime() < end) {
            long now = System.nanoTime();
            double remaining = Math.min(.25, Math.max(0, (now - hardware.lastPhysicsUpdateNanos) * 1e-9));
            hardware.lastPhysicsUpdateNanos = now;
            while (remaining > 1e-9) { double step = Math.min(.005, remaining); ApexSimulation.stepPhysics(hardware, step); remaining -= step; }
            SimLinearOpModeBridge.eventLoopIteration(session, op.gamepad1, op.gamepad2);
            telemetry.update(); Thread.sleep(5);
        }
    }
}
