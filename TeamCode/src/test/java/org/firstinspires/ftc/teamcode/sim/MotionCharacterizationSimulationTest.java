package org.firstinspires.ftc.teamcode.sim;

import util.DeltaTime;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.SimLinearOpModeBridge;
import com.qualcomm.robotcore.hardware.Gamepad;
import feedforward.MotionCharacterization;
import feedforward.MotorPowerModel;
import follower.Follower;
import follower.FollowerConstants;
import localizers.LocalizationConstants;
import storage.ApexStorage;
import org.firstinspires.ftc.teamcode.apexpathing.Constants;
import org.junit.Test;
import java.io.File;
import java.nio.file.Files;
import java.util.*;
import java.util.concurrent.atomic.AtomicBoolean;
import tuning.follower.TunerContext;
import tuning.follower.characterization.MotionCharacterizationPhase;
import static org.junit.Assert.*;

/** Exercises the real acquisition phase with upstream stock motor/chassis physics. */
public class MotionCharacterizationSimulationTest {
    @Test(timeout = 240_000L) public void calibratesAndValidatesEveryAvailableAxis() throws Exception {
        characterize(false);
    }
    @Test(timeout = 240_000L) public void tankAlternatesAndOmitsStrafe() throws Exception {
        characterize(true);
    }
    private void characterize(boolean tank) throws Exception {
        java.lang.reflect.Field singleton = FollowerConstants.class.getDeclaredField("instance");
        singleton.setAccessible(true);
        Object originalConstants = singleton.get(null);
        String previousTank = System.getProperty(ApexSimulation.TANK_SIMULATION_PROPERTY);
        String previous = System.getProperty(ApexStorage.DIRECTORY_PROPERTY);
        File directory = new File("build/motion-characterization-validation/phase-" + (tank ? "tank" : "mecanum"));
        System.setProperty(ApexStorage.DIRECTORY_PROPERTY, directory.getAbsolutePath());
        SimLinearOpModeBridge.Session session = null;
        try {
            singleton.set(null, null);
            System.setProperty(ApexSimulation.TANK_SIMULATION_PROPERTY, Boolean.toString(tank));
            ApexSimulation.Hardware hardware = tank ? ApexSimulation.createTankHardware() : ApexSimulation.createHardware();
            config.ApexConstants robot = tank ? new TankSimulationConstants() : new Constants();
            Follower seed = Follower.create(robot, hardware.hardwareMap, true);
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
            java.util.concurrent.atomic.AtomicReference<FollowerConstants> tuned = new java.util.concurrent.atomic.AtomicReference<>();
            LinearOpMode op = new LinearOpMode() {
                @Override public void runOpMode() {
                    TunerContext context = new TunerContext(this);
                    context.setFollower(Follower.create(robot, hardwareMap, true));
                    tuned.set(context.constants);
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
                assertFalse(frame, frame.contains("acquisition failed"));
                if (frame.contains("Press A")) {
                    op.gamepad1.a = true; pump(session, op, telemetry, hardware, 40);
                    op.gamepad1.a = false;
                }
                pump(session, op, telemetry, hardware, 25);
            }
            assertTrue("Characterization never completed: " + frames.get(frames.size() - 1), accepted.get());
            FollowerConstants result = tuned.get();
            assertNotNull(result.motionCharacterization);
            SimLinearOpModeBridge.stop(session); session = null;
            assertRunSequence(directory, tank);
            ApexStorage.saveConstants(result.toJson().toString(4));
            FollowerConstants restored = FollowerConstants.fromJson(new org.json.JSONObject(
                    new String(Files.readAllBytes(ApexStorage.getConstantsFile().toPath()), java.nio.charset.StandardCharsets.UTF_8)));
            for (MotionCharacterization.Axis axis : MotionCharacterization.Axis.values()) {
                if (tank && axis == MotionCharacterization.Axis.STRAFE) {
                    assertNull(restored.motionCharacterization.model(axis, 1)); continue;
                }
                for (double sign : new double[]{1, -1}) {
                    MotorPowerModel model = restored.motionCharacterization.model(axis, sign);
                    validatePhysicalResponse(seed, hardware, axis, sign, model);
                    assertTrue(Double.isFinite(model.validationRms));
                    System.out.println(axis + " " + sign + " gain " + model.gain0 + ", slope " + model.gainSlope
                            + ", drag " + model.drag0 + ", slope " + model.dragSlope + ", max " + model.maxVelocity
                            + ", RMS " + model.validationRms);
                }
            }
        } finally {
            if (session != null) { SimLinearOpModeBridge.stop(session); }
            singleton.set(null, originalConstants);
            if (previousTank == null) { System.clearProperty(ApexSimulation.TANK_SIMULATION_PROPERTY); }
            else { System.setProperty(ApexSimulation.TANK_SIMULATION_PROPERTY, previousTank); }
            if (previous == null) { System.clearProperty(ApexStorage.DIRECTORY_PROPERTY); }
            else { System.setProperty(ApexStorage.DIRECTORY_PROPERTY, previous); }
        }
    }
    private void assertRunSequence(File directory, boolean tank) throws Exception {
        File csv = Arrays.stream(Objects.requireNonNull(directory.listFiles()))
                .filter(f -> f.getName().startsWith("motion_characterization_") && f.getName().endsWith(".csv"))
                .max(Comparator.comparingLong(File::lastModified)).orElseThrow(AssertionError::new);
        List<String> runs = new ArrayList<>();
        for (String row : Files.readAllLines(csv.toPath())) {
            String[] fields = row.split(",", -1);
            if (!fields[0].equals("raw")) continue;
            assertEquals("No held-out motions in the tuner", "false", fields[4]);
            String key = fields[1] + "/" + fields[3] + "/" + fields[2];
            if (runs.isEmpty() || !runs.get(runs.size() - 1).equals(key)) runs.add(key);
        }
        List<String> expected = new ArrayList<>();
        for (MotionCharacterization.Axis axis : MotionCharacterization.Axis.values()) {
            if (tank && axis == MotionCharacterization.Axis.STRAFE) continue;
            for (int run = 0; run < 4; run++) {
                expected.add(axis + "/" + run + "/0"); expected.add(axis + "/" + run + "/1");
            }
        }
        assertEquals("Four acquisition runs per direction, alternating every run", expected, runs);
    }
    /** Independent imposed-speed probes use stock physics, never acquisition residual as validation. */
    private void validatePhysicalResponse(Follower follower, ApexSimulation.Hardware hardware,
                                         MotionCharacterization.Axis axis, double sign, MotorPowerModel model) {
        follower.setPose(geometry.Pose.zero());
        double squared = 0, peak = 0; int count = 0;
        for (double fraction : new double[]{.15, .5, .85}) {
            double speed = fraction * model.maxVelocity;
            for (double power : new double[]{.55, .95}) {
                hardware.drivetrain.velocity = new org.codeblooded.ftcodesim.ascope.boundaries.MotionVector(
                        axis == MotionCharacterization.Axis.FORWARD ? sign * speed : 0,
                        axis == MotionCharacterization.Axis.STRAFE ? sign * speed : 0,
                        axis == MotionCharacterization.Axis.ANGULAR ? sign * speed : 0);
                hardware.drivetrain.updateWheelRollVelocities();
                follower.getDrivetrain().moveWithVectors(
                        axis == MotionCharacterization.Axis.FORWARD ? sign * power : 0,
                        axis == MotionCharacterization.Axis.STRAFE ? sign * power : 0,
                        axis == MotionCharacterization.Axis.ANGULAR ? sign * power : 0);
                ApexSimulation.stepPhysics(hardware, .0001);
                double observed = sign * (axis == MotionCharacterization.Axis.FORWARD ? hardware.drivetrain.velocity.x
                        : axis == MotionCharacterization.Axis.STRAFE ? hardware.drivetrain.velocity.y : hardware.drivetrain.velocity.theta);
                double acceleration = (observed - speed) / .0001;
                double error = model.power(speed, acceleration) - power;
                squared += error * error; peak = Math.max(peak, Math.abs(error)); count++;
                assertTrue(axis + " " + sign + " speed band " + fraction + " power error=" + error, Math.abs(error) <= .05);
            }
        }
        follower.stop();
        assertTrue(axis + " " + sign + " independent RMS", Math.sqrt(squared / count) <= .04);
        assertTrue(axis + " " + sign + " independent peak", peak <= .10);
    }
    private void pump(SimLinearOpModeBridge.Session session, LinearOpMode op,
                      ApexSimTelemetry telemetry, ApexSimulation.Hardware hardware, long millis) throws Exception {
        long end = System.nanoTime() + millis * 1_000_000L;
        while (System.nanoTime() < end) {
            long now = System.nanoTime();
            double remaining = Math.min(.25, Math.max(0, DeltaTime.seconds(now - hardware.lastPhysicsUpdateNanos)));
            hardware.lastPhysicsUpdateNanos = now;
            while (remaining > 1e-9) { double step = Math.min(.005, remaining); ApexSimulation.stepPhysics(hardware, step); remaining -= step; }
            SimLinearOpModeBridge.eventLoopIteration(session, op.gamepad1, op.gamepad2);
            telemetry.update(); Thread.sleep(5);
        }
    }
}
