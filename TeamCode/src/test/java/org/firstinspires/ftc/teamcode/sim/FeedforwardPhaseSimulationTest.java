package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.SimLinearOpModeBridge;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.apexpathing.Constants;
import org.junit.Test;
import java.nio.file.Files;
import java.util.Collections;
import java.util.ArrayList;
import java.util.List;
import storage.ApexStorage;
import follower.Follower;
import localizers.LocalizationConstants;
import controllers.FeedbackController.Coefficients;
import tuning.follower.sysid.FeedforwardTuner;
import tuning.follower.TunerContext;

/** Exercises four-test SysId identification and acceptance through the real tuner UI. */
public class FeedforwardPhaseSimulationTest {
    private java.lang.reflect.Field singleton;
    private Object previousConstants;

    @org.junit.Before public void isolateConstants() throws Exception {
        singleton = follower.FollowerConstants.class.getDeclaredField("instance");
        singleton.setAccessible(true);
        previousConstants = singleton.get(null);
        singleton.set(null, null);
    }

    @org.junit.After public void restoreConstants() throws Exception {
        singleton.set(null, previousConstants);
    }

    @Test
    public void acceptedLocalizationStatusIsAvailableToTuner() throws Exception {
        String previousDirectory = System.getProperty(ApexStorage.DIRECTORY_PROPERTY);
        System.setProperty(ApexStorage.DIRECTORY_PROPERTY,
                Files.createTempDirectory("apex-feedforward-prerequisite-").toString());
        try {
            ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
            Constants robot = new Constants();
            Follower untuned = new Follower(robot, hardware.hardwareMap, true);
            assertFalse(untuned.hasAcceptedKalmanFilterTuning());

            LocalizationConstants localization = LocalizationConstants.empty();
            localization.capture("DEFAULT", robot.localizerConstants(), untuned.getLocalizer(),
                    LocalizationConstants.Status.ACCEPTED,
                    LocalizationConstants.Status.ACCEPTED);
            localization.save();

            Follower tuned = new Follower(robot, hardware.hardwareMap, true);
            assertTrue(tuned.hasAcceptedKalmanFilterTuning());
        } finally {
            if (previousDirectory == null) {
                System.clearProperty(ApexStorage.DIRECTORY_PROPERTY);
            } else {
                System.setProperty(ApexStorage.DIRECTORY_PROPERTY, previousDirectory);
            }
        }
    }

    @Test(timeout = 150000)
    public void linearMecanumSysIdFitsBothAxes() throws Exception {
        runRamp(false);
    }

    @Test(timeout = 150000)
    public void linearMecanumSysIdIdentifiesDifferentMovingFriction() throws Exception {
        runRamp(false, .03);
    }

    @Test(timeout = 150000)
    public void tankSysIdRecoversLowFrictionModel() throws Exception {
        runRamp(true);
    }

    private void runRamp(boolean tank) throws Exception {
        runRamp(tank, .06);
    }

    private void runRamp(boolean tank, double expectedKS) throws Exception {
        String previousDirectory = System.getProperty(ApexStorage.DIRECTORY_PROPERTY);
        System.setProperty(ApexStorage.DIRECTORY_PROPERTY,
                Files.createTempDirectory("apex-feedforward-phase-").toString());
        SimLinearOpModeBridge.Session session = null;
        try {
            ApexSimulation.Hardware hardware = tank ? ApexSimulation.createTankHardware()
                    : ApexSimulation.createHardware();
            List<String> frames = Collections.synchronizedList(new ArrayList<>());
            ApexSimTelemetry telemetry = new ApexSimTelemetry(frames::add);
            telemetry.setMsTransmissionInterval(0);
            java.util.concurrent.atomic.AtomicReference<TunerContext> tunedContext = new java.util.concurrent.atomic.AtomicReference<>();
            LinearOpMode opMode = new LinearOpMode() {
                @Override public void runOpMode() throws InterruptedException {
                    config.ApexConstants robot = tank ? new TankSimulationConstants() : new Constants();
                    Follower seed = new Follower(robot, hardwareMap, true);
                    LocalizationConstants localization = LocalizationConstants.empty();
                    localization.capture("DEFAULT", robot.localizerConstants(), seed.getLocalizer(),
                            LocalizationConstants.Status.ACCEPTED,
                            LocalizationConstants.Status.ACCEPTED);
                    try { localization.save(); }
                    catch (java.io.IOException e) { throw new IllegalStateException(e); }
                    TunerContext context = new TunerContext(this);
                    context.setFollower(new Follower(robot, hardwareMap, true));
                    context.constants.forwardVelLimitIn = 65;
                    context.constants.forwardAccelLimitIn = 148;
                    context.constants.angularVelLimitRad = 7;
                    context.constants.angularAccelLimitRad = 14.5;
                    context.constants.translationalCoeffs =
                            new Coefficients().p(.20).d(.04).s(.24);
                    context.constants.angularCoeffs =
                            new Coefficients().p(2.40).d(.45).s(.24);
                    context.getFollower().setDriveCoefficients(context.constants.translationalCoeffs);
                    context.getFollower().setHeadingCoefficients(context.constants.angularCoeffs);
                    if (tank) {
                        // Seed old gains to verify that SysId replaces the selected axis.
                        context.getFollower().setFeedforwardGains(0.94 / 75, 0.94 / 150,
                                7 * 0.94 / 75, 7 * 0.94 / 150);
                        context.getFollower().setVelocityFeedback(0.02, 0.15);
                    }
                    waitForStart();
                    tunedContext.set(context);
                    if (new FeedforwardTuner(context, FeedforwardTuner.Axis.DRIVE).run(this)) {
                        new FeedforwardTuner(context, FeedforwardTuner.Axis.HEADING).run(this);
                    }

                }
            };
            opMode.hardwareMap = hardware.hardwareMap;
            opMode.telemetry = telemetry;
            opMode.gamepad1 = new Gamepad();
            opMode.gamepad2 = new Gamepad();
            session = SimLinearOpModeBridge.initialize(opMode, () -> {});
            SimLinearOpModeBridge.start(session);
            long deadline = System.nanoTime() + 130_000_000_000L;
            long previous = System.nanoTime();
            long lastPress = 0;
            String frame = "";
            while (System.nanoTime() < deadline) {
                long now = System.nanoTime();
                double remaining = Math.min(.05, (now - previous) * 1e-9);
                previous = now;
                while (remaining > 1e-9) {
                    double dt = Math.min(.005, remaining);
                    step(hardware, dt);
                    remaining -= dt;
                }
                frame = frames.isEmpty() ? "" : frames.get(frames.size() - 1);
                if (frame.contains("validation failed") || frame.contains("HEADING Feedforward SysId constants:")
                        || frame.contains("Fit rejected")) {
                    break;
                }
                if (frame.contains("Press A") && now - lastPress > 250_000_000L) {
                    lastPress = now;
                }
                opMode.gamepad1.a = now - lastPress < 60_000_000L;
                SimLinearOpModeBridge.eventLoopIteration(session, opMode.gamepad1, opMode.gamepad2);
                Thread.sleep(5);
            }
            System.out.println(frame);
            TunerContext context = tunedContext.get();
            assertTrue("Tuner failed to converge: " + frame,
                    frame.contains("HEADING Feedforward SysId constants:"));
            if (tank) {
                org.junit.Assert.assertEquals(0.06,
                        context.constants.translationalFeedforwardKS, 0.005);
                org.junit.Assert.assertEquals(0.06,
                        context.constants.angularFeedforwardKS, 0.005);
                org.junit.Assert.assertEquals(0.94 / 75,
                        context.constants.translationalKV, 0.0005);
                org.junit.Assert.assertEquals(7 * 0.94 / 75,
                        context.constants.angularKV, 0.003);
            } else {
                org.junit.Assert.assertEquals("Drive moving friction", expectedKS,
                        context.constants.translationalFeedforwardKS, 0.005);
                org.junit.Assert.assertEquals("Turn moving friction", expectedKS,
                        context.constants.angularFeedforwardKS, 0.005);
            }
            org.junit.Assert.assertEquals("Drive inertia", .94 / 150, context.constants.translationalKA, .001);
            org.junit.Assert.assertTrue("Heading inertia is identified", context.constants.angularKA > 0);
            if (tank) {
                SimLinearOpModeBridge.stop(session);
                session = null;
                Follower follower = context.getFollower();
                geometry.GeometryFactory factory = new geometry.GeometryFactory(follower)
                        .setDistUnit(geometry.DistUnit.IN).setAngleUnit(geometry.AngleUnit.DEG);
                geometry.Pose start = factory.pose(0, 0, 0);
                follower.setPose(start);
                paths.movements.Path curve = factory.tankPath(start, factory.pose(18, 0),
                                factory.pose(30, 6), factory.pose(36, 18, 45))
                        .interpolateWith(paths.heading.InterpolationStyle.TANGENT_FORWARD).buildToMemory();
                TankPathFollowingSimulationTest.follow(hardware, follower, curve, 15);
            }
        } finally {
            if (session != null) { SimLinearOpModeBridge.stop(session); }
            if (previousDirectory == null) { System.clearProperty(ApexStorage.DIRECTORY_PROPERTY); }
            else { System.setProperty(ApexStorage.DIRECTORY_PROPERTY, previousDirectory); }
        }
    }

    private static void step(ApexSimulation.Hardware hardware, double dt) throws Exception {
        StockSimTestSupport.step(hardware, dt);
    }
}
