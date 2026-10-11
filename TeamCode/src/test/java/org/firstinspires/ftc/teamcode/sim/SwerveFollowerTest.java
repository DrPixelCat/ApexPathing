package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.*;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogInputController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import config.ApexConstants;
import controllers.FeedbackController.Coefficients;
import drivetrains.BaseDrivetrainConstants;
import drivetrains.CoaxialSwerve;
import drivetrains.Motor;
import follower.Follower;
import follower.FollowerConstants;
import follower.drivetrains.MecanumFollower;
import follower.drivetrains.SwerveFollower;
import geometry.GeometryFactory;
import java.lang.reflect.Field;
import java.lang.reflect.Proxy;
import java.nio.file.Files;
import java.util.concurrent.atomic.AtomicLong;
import localizers.BaseLocalizerConstants;
import org.firstinspires.ftc.teamcode.apexpathing.Constants;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import paths.movements.Path;
import storage.ApexStorage;

/** Runs the real swerve drivetrain against simulated motor I/O and fixed steering sensors. */
public class SwerveFollowerTest {
    private Field singleton;
    private Object previous;
    private String previousStorage, previousUnlock;

    @Before public void setup() throws Exception {
        singleton = FollowerConstants.class.getDeclaredField("instance");
        singleton.setAccessible(true);
        previous = singleton.get(null);
        singleton.set(null, SimulatorTuning.snapshot());
        previousStorage = System.getProperty(ApexStorage.DIRECTORY_PROPERTY);
        previousUnlock = System.getProperty("apex.simulation.unlockTunerPhases");
        System.setProperty(ApexStorage.DIRECTORY_PROPERTY,
                Files.createTempDirectory("apex-swerve-follower-").toString());
    }

    @After public void restore() throws Exception {
        singleton.set(null, previous);
        restoreProperty(ApexStorage.DIRECTORY_PROPERTY, previousStorage);
        restoreProperty("apex.simulation.unlockTunerPhases", previousUnlock);
    }

    @Test public void factorySelectsSwerveAndBuildsHardwareOnlyOnce() {
        Fixture fixture = new Fixture();
        AtomicLong clock = new AtomicLong(1_000_000_000L);
        Follower follower = Follower.create(fixture.robot, fixture.hardware.hardwareMap, true, clock::get);
        assertEquals(SwerveFollower.class, follower.getClass());
        assertEquals(1, fixture.drive.builds);
        assertSame(fixture.drive.built, follower.getDrivetrain());
        assertSame(fixture.drive, follower.getDrivetrain().getConstants());
        assertEquals(drivetrains.BaseDrivetrain.DrivetrainType.COAXIAL_SWERVE,
                follower.getConstants().drivetrainType);
    }

    @Test public void specializedConstructorsRejectTheOtherDrivetrain() {
        Fixture fixture = new Fixture();
        assertThrows(IllegalArgumentException.class, () -> new MecanumFollower(
                fixture.robot, fixture.hardware.hardwareMap, true));
        assertThrows(IllegalArgumentException.class, () -> new SwerveFollower(new Constants(),
                ApexSimulation.createHardware().hardwareMap));
        assertThrows(IllegalArgumentException.class, () -> new SwerveFollower(new TankSimulationConstants(),
                ApexSimulation.createTankHardware().hardwareMap, true));
    }

    @Test public void swerveManualStrafeCommandsModulesAndStopClearsSteering() {
        Fixture fixture = new Fixture();
        Follower follower = new SwerveFollower(fixture.robot, fixture.hardware.hardwareMap);
        follower.manual(0, .5, 0);
        assertEquals(.5, follower.getDrivetrain().getLastFlPower(), 1e-9);
        assertEquals(.5, follower.getDrivetrain().getLastFrPower(), 1e-9);
        assertEquals(.5, follower.getDrivetrain().getLastBlPower(), 1e-9);
        assertEquals(.5, follower.getDrivetrain().getLastBrPower(), 1e-9);
        for (double power : fixture.steeringPower) { assertTrue(power > 0); }
        follower.stop();
        assertEquals(0, follower.getDrivetrain().getLastFlPower(), 0);
        assertEquals(0, follower.getDrivetrain().getLastFrPower(), 0);
        for (double power : fixture.steeringPower) { assertEquals(0, power, 0); }
    }

    @Test public void swervePathsRetainPauseRecoveryAndInjectedClockInBothBuildModes() {
        for (Path.BuildMode mode : Path.BuildMode.values()) {
            Fixture fixture = new Fixture();
            AtomicLong clock = new AtomicLong(1_000_000_000L);
            Follower follower = new SwerveFollower(fixture.robot, fixture.hardware.hardwareMap,
                    true, clock::get);
            SimulatorTuning.apply(follower.getConstants());
            follower.getConstants().motionCharacterization = null;
            follower.reset();
            GeometryFactory geometry = new GeometryFactory(follower);
            Path path = geometry.holonomicPath(geometry.pose(0, 0, 0), geometry.pose(40, 0, 0)).quickBuild();
            path.setBuildMode(mode);
            path.precomputeTrajectory();
            follower.follow(path);
            follower.update();
            assertNotNull(follower.getTrajectoryTarget());
            assertTrue(follower.getLastCommandDemand().available);
            assertTrue(follower.getDrivetrain().getLastFlPower() > 0);
            follower.pause();
            assertNull(follower.getTrajectoryTarget());
            for (double power : fixture.steeringPower) { assertEquals(0, power, 0); }
            assertEquals(0, follower.getDrivetrain().getLastFlPower(), 0);
            clock.addAndGet(1_000_000_000L);
            follower.update();
            follower.resume();
            follower.update();
            assertEquals(Follower.MovementOutcome.RUNNING, follower.getMovementOutcome());
            clock.addAndGet(500_000_000L);
            follower.update();
            assertEquals(Follower.MovementOutcome.FAILED, follower.getMovementOutcome());
            assertFalse(follower.isBusy());
            assertEquals(0, follower.getDrivetrain().getLastFlPower(), 0);
            for (double power : fixture.steeringPower) { assertEquals(0, power, 0); }
            follower.clearFailure();
            follower.follow(path);
            follower.stop();
            assertEquals(Follower.MovementOutcome.CANCELLED, follower.getMovementOutcome());
        }
    }

    private static ApexConstants robot(BaseDrivetrainConstants<?> drive) {
        return new ApexConstants() {
            @Override public BaseDrivetrainConstants<?> drivetrainConstants() { return drive; }
            @Override public BaseLocalizerConstants<?> localizerConstants() {
                return new Constants().localizerConstants();
            }
        };
    }

    private static final class Fixture {
        final ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
        final CountingSwerveConstants drive = new CountingSwerveConstants();
        final double[] steeringPower = new double[4];
        final ApexConstants robot = robot(drive);

        Fixture() {
            drive.setFrontLeftMotor(new Motor(ApexSimulation.FRONT_LEFT_MOTOR))
                    .setFrontRightMotor(new Motor(ApexSimulation.FRONT_RIGHT_MOTOR))
                    .setBackLeftMotor(new Motor(ApexSimulation.BACK_LEFT_MOTOR))
                    .setBackRightMotor(new Motor(ApexSimulation.BACK_RIGHT_MOTOR))
                    .setServoNames("steer0", "steer1", "steer2", "steer3")
                    .setEncoderNames("encoder0", "encoder1", "encoder2", "encoder3")
                    .setSteeringCoefficients(new Coefficients().p(.5).d(0).s(0));
            for (int i = 0; i < 4; i++) {
                final int index = i;
                CRServo servo = (CRServo) Proxy.newProxyInstance(CRServo.class.getClassLoader(),
                        new Class<?>[]{CRServo.class}, (proxy, method, args) -> {
                            if (method.getName().equals("setPower")) {
                                steeringPower[index] = (double) args[0];
                                return null;
                            }
                            if (method.getName().equals("getPower")) { return steeringPower[index]; }
                            return deviceValue(proxy, method.getName(), args);
                        });
                AnalogInputController controller = (AnalogInputController) Proxy.newProxyInstance(
                        AnalogInputController.class.getClassLoader(), new Class<?>[]{AnalogInputController.class},
                        (proxy, method, args) -> {
                            if (method.getName().equals("getAnalogInputVoltage")) { return 0.0; }
                            if (method.getName().equals("getMaxAnalogInputVoltage")) { return 3.3; }
                            return deviceValue(proxy, method.getName(), args);
                        });
                hardware.hardwareMap.register("steer" + i, servo);
                hardware.hardwareMap.register("encoder" + i, new AnalogInput(controller, i));
            }
        }
    }

    private static Object deviceValue(Object proxy, String method, Object[] args) {
        switch (method) {
            case "hashCode": return System.identityHashCode(proxy);
            case "equals": return proxy == args[0];
            case "getDeviceName": case "getConnectionInfo": case "toString": return "Follower test steering I/O";
            case "getVersion": return 1;
            case "getManufacturer": return HardwareDevice.Manufacturer.Other;
            default: return null;
        }
    }

    private static final class CountingSwerveConstants extends CoaxialSwerve.Constants {
        int builds;
        CoaxialSwerve built;
        @Override public CoaxialSwerve build(HardwareMap hardwareMap) {
            builds++;
            return built = super.build(hardwareMap);
        }
    }

    private static void restoreProperty(String name, String value) {
        if (value == null) { System.clearProperty(name); } else { System.setProperty(name, value); }
    }
}
