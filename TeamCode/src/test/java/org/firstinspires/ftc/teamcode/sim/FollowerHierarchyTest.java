package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.*;

import com.qualcomm.robotcore.hardware.HardwareMap;
import config.ApexConstants;
import controllers.FeedbackController.Coefficients;
import drivetrains.BaseDrivetrainConstants;
import drivetrains.DualActuated;
import drivetrains.Mecanum;
import drivetrains.Motor;
import follower.drivetrains.DualActuatedFollower;
import follower.Follower;
import follower.FollowerConstants;
import follower.drivetrains.MecanumFollower;
import follower.drivetrains.TankFollower;
import geometry.DistUnit;
import geometry.GeometryFactory;
import geometry.Pose;
import java.lang.reflect.Field;
import java.util.concurrent.atomic.AtomicLong;
import localizers.BaseLocalizerConstants;
import org.firstinspires.ftc.teamcode.apexpathing.Constants;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import paths.movements.Path;

/** Factory, explicit-constructor, and polymorphic behavior checks using simulated hardware I/O. */
public class FollowerHierarchyTest {
    private Field singleton;
    private Object previous;

    @Before public void setup() throws Exception {
        singleton = FollowerConstants.class.getDeclaredField("instance");
        singleton.setAccessible(true);
        previous = singleton.get(null);
        singleton.set(null, SimulatorTuning.snapshot());
    }

    @After public void restore() throws Exception {
        singleton.set(null, previous);
        Follower.setDiagnostics(null);
    }

    @Test public void factoryBuildsHardwareOnceAndUsesTheBuiltDrivetrain() {
        ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
        CountingMecanumConstants drive = new CountingMecanumConstants();
        drive.setFrontLeftMotor(new Motor(ApexSimulation.FRONT_LEFT_MOTOR))
                .setFrontRightMotor(new Motor(ApexSimulation.FRONT_RIGHT_MOTOR))
                .setBackLeftMotor(new Motor(ApexSimulation.BACK_LEFT_MOTOR))
                .setBackRightMotor(new Motor(ApexSimulation.BACK_RIGHT_MOTOR));
        int[] configRequests = {0};
        ApexConstants robot = new ApexConstants() {
            @Override public BaseDrivetrainConstants<?> drivetrainConstants() {
                configRequests[0]++;
                return drive;
            }
            @Override public BaseLocalizerConstants<?> localizerConstants() {
                return new Constants().localizerConstants();
            }
        };

        Follower follower = Follower.create(robot, hardware.hardwareMap);
        assertTrue(follower instanceof MecanumFollower);
        assertEquals(1, configRequests[0]);
        assertEquals(1, drive.builds);
        assertSame(drive.built, follower.getDrivetrain());
        assertSame(drive, follower.getDrivetrain().getConstants());

        Follower tank = Follower.create(new TankSimulationConstants(),
                ApexSimulation.createTankHardware().hardwareMap);
        assertTrue(tank instanceof TankFollower);
    }

    @Test public void dualFactorySelectionDoesNotDependOnTheInitialMode() {
        for (DualActuated.DriveState state : DualActuated.DriveState.values()) {
            ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
            Follower follower = Follower.create(dualRobot(state), hardware.hardwareMap, true);
            assertTrue(follower instanceof DualActuatedFollower);
            assertEquals(state, ((DualActuated) follower.getDrivetrain()).getDriveState());
            assertEquals(state == DualActuated.DriveState.TANK ? FollowerConstants.Profile.TANK
                    : FollowerConstants.Profile.HOLONOMIC, follower.getConstants().getActiveProfile());
            follower.update();
            assertEquals(Follower.MovementOutcome.IDLE, follower.getMovementOutcome());
        }
    }

    @Test public void explicitFollowersAcceptMatchingHardwareAndRejectIncompatibleHardware() {
        assertTrue(new MecanumFollower(new Constants(), ApexSimulation.createHardware().hardwareMap)
                .getDrivetrain().isHolonomic());
        assertFalse(new TankFollower(new TankSimulationConstants(),
                ApexSimulation.createTankHardware().hardwareMap, true).getDrivetrain().isHolonomic());
        assertThrows(IllegalArgumentException.class, () -> new TankFollower(new Constants(),
                ApexSimulation.createHardware().hardwareMap));
        assertThrows(IllegalArgumentException.class, () -> new MecanumFollower(new TankSimulationConstants(),
                ApexSimulation.createTankHardware().hardwareMap));
        assertThrows(IllegalArgumentException.class, () -> new DualActuatedFollower(new Constants(),
                ApexSimulation.createHardware().hardwareMap, true));
        for (DualActuated.DriveState state : DualActuated.DriveState.values()) {
            ApexConstants robot = dualRobot(state);
            assertNotNull(new DualActuatedFollower(robot, ApexSimulation.createHardware().hardwareMap, true));
            assertThrows(IllegalArgumentException.class, () -> new MecanumFollower(robot,
                    ApexSimulation.createHardware().hardwareMap, true));
            assertThrows(IllegalArgumentException.class, () -> new TankFollower(robot,
                    ApexSimulation.createHardware().hardwareMap, true));
        }
    }

    @Test public void externalModeChangesStopAnActiveDualPath() {
        ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
        Follower follower = Follower.create(dualRobot(DualActuated.DriveState.HOLONOMIC),
                hardware.hardwareMap, true);
        SimulatorTuning.apply(follower.getConstants());
        GeometryFactory geometry = new GeometryFactory(follower).setDistUnit(DistUnit.IN);
        Path path = geometry.holonomicPath(geometry.pose(0, 0, 0), geometry.pose(40, 0, 0)).quickBuild();
        follower.follow(path);
        follower.getDrivetrain().moveWithVectors(.4, 0, 0);
        ((DualActuated) follower.getDrivetrain()).activateTractionState();
        assertThrows(IllegalStateException.class, follower::update);
        assertFalse(follower.isBusy());
        assertEquals(Follower.MovementOutcome.CANCELLED, follower.getMovementOutcome());
        assertEquals(0, follower.getDrivetrain().getLastFlPower(), 0);
        assertEquals(0, follower.getDrivetrain().getLastFrPower(), 0);
    }

    @Test public void factoryAndExplicitConstructionPreserveTheInjectedClock() {
        for (boolean explicit : new boolean[]{false, true}) {
            ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
            AtomicLong clock = new AtomicLong(1_000_000_000L);
            Follower follower = explicit
                    ? new MecanumFollower(new Constants(), hardware.hardwareMap, true, clock::get)
                    : Follower.create(new Constants(), hardware.hardwareMap, true, clock::get);
            GeometryFactory geometry = new GeometryFactory(follower).setDistUnit(DistUnit.IN);
            Path path = geometry.holonomicPath(geometry.pose(0, 0, 0), geometry.pose(40, 0, 0)).quickBuild();
            follower.follow(path);
            follower.update();
            assertNotNull(follower.getTrajectoryTarget());
            // No physics steps: the stationary robot must fail exactly at the injected
            // 0.5-second stuck timeout, without waiting for wall-clock time.
            clock.addAndGet(499_000_000L);
            follower.update();
            assertEquals(Follower.MovementOutcome.RUNNING, follower.getMovementOutcome());
            clock.addAndGet(1_000_000L);
            follower.update();
            assertEquals(Follower.MovementOutcome.FAILED, follower.getMovementOutcome());
            assertFalse(follower.isBusy());
            assertEquals(0, follower.getDrivetrain().getLastFlPower(), 0);
            assertThrows(IllegalStateException.class, () -> follower.follow(path));
            follower.clearFailure();
            follower.follow(path);
            follower.stop();
            assertEquals(Follower.MovementOutcome.CANCELLED, follower.getMovementOutcome());
        }
    }

    @Test public void poseHoldingUsesTheConcreteFollowersKinematics() {
        Follower holonomic = new MecanumFollower(new Constants(),
                ApexSimulation.createHardware().hardwareMap, true);
        Follower tank = new TankFollower(new TankSimulationConstants(),
                ApexSimulation.createTankHardware().hardwareMap, true);
        GeometryFactory geometry = new GeometryFactory().setDistUnit(DistUnit.IN);
        for (Follower follower : new Follower[]{holonomic, tank}) {
            follower.setPose(Pose.zero());
            follower.setDriveCoefficients(new Coefficients().p(.1).d(0).s(0));
            follower.setHeadingCoefficients(new Coefficients().p(.1).d(0).s(0));
            follower.setHoldPose(geometry.pose(0, 4, 0));
            follower.update(true);
        }
        assertTrue(Math.abs(holonomic.getDrivetrain().getLastFlPower()) > 0);
        assertEquals(-holonomic.getDrivetrain().getLastFlPower(),
                holonomic.getDrivetrain().getLastFrPower(), 1e-9);
        assertEquals(0, tank.getDrivetrain().getLastFlPower(), 1e-9);
        assertEquals(0, tank.getDrivetrain().getLastFrPower(), 1e-9);
    }

    @Test public void diagnosticsViewStaysLiveAcrossUpdatesPauseAndNewMovements() {
        AtomicLong clock = new AtomicLong(1_000_000_000L);
        Follower follower = Follower.create(new Constants(), ApexSimulation.createHardware().hardwareMap,
                true, clock::get);
        follower.FollowerTelemetry view = follower.getDiagnostics();
        assertFalse(view.isVelocitySampleAvailable());
        assertTrue(Double.isNaN(view.getVelocityGoal()));
        GeometryFactory geometry = new GeometryFactory(follower).setDistUnit(DistUnit.IN);
        Path path = geometry.holonomicPath(geometry.pose(0, 0, 0), geometry.pose(40, 0, 0)).quickBuild();
        follower.setPose(geometry.pose(0, 2, 0));
        follower.follow(path);
        follower.update();
        assertSame(view, follower.getDiagnostics());
        assertTrue(view.isVelocitySampleAvailable());
        assertTrue(Math.abs(view.getCrossTrackErrorIn()) > 1);
        assertEquals(follower.getCrossTrackErrorIn(), view.getCrossTrackErrorIn(), 0);
        assertEquals(follower.getTrackingVelocityTarget(), view.getTrackingVelocityTarget(), 0);
        assertSame(follower.getLastCommandDemand(), view.getLastCommandDemand());
        follower.pause();
        assertFalse(view.isVelocitySampleAvailable());
        follower.resume();
        clock.addAndGet(20_000_000L);
        follower.update();
        assertTrue(view.isVelocitySampleAvailable());
        follower.stop();
        assertEquals(0, view.getCrossTrackErrorIn(), 0);
        assertFalse(view.isVelocitySampleAvailable());
        assertSame(Follower.CommandDemand.ZERO, view.getLastCommandDemand());
        follower.follow(path);
        follower.update();
        assertSame(view, follower.getDiagnostics());
        assertTrue(view.isVelocitySampleAvailable());
        follower.stop();
    }

    @Test public void tankFeedbackSwitchesPreserveFeedforward() {
        ApexSimulation.Hardware hardware = ApexSimulation.createTankHardware();
        Follower follower = new TankFollower(new TankSimulationConstants(), hardware.hardwareMap, true);
        SimulatorTuning.apply(follower.getConstants());
        follower.reset();
        GeometryFactory geometry = new GeometryFactory(follower).setDistUnit(DistUnit.IN);
        Path path = geometry.tankPath(geometry.pose(0, 0, 0), geometry.pose(4, 0, 0)).quickBuild();
        for (int mode = 0; mode < 3; mode++) {
            follower.setPose(geometry.pose(0, 0, 20));
            follower.enableControllers();
            if (mode != 1) { follower.disableDriveController(); }
            if (mode != 0) { follower.disableHeadingController(); }
            follower.follow(path);
            follower.update();
            double left = follower.getDrivetrain().getLastFlPower();
            double right = follower.getDrivetrain().getLastFrPower();
            if (mode == 0) {
                assertEquals(0, follower.getTrackingVelocityFeedback(), 0);
                assertTrue("Heading feedback must still turn", Math.abs(left - right) > .01);
            } else if (mode == 1) {
                assertEquals("Disabling heading must leave pure forward drive", left, right, 1e-9);
                assertTrue(Math.abs(left) > 0);
            } else {
                assertEquals(0, follower.getTrackingVelocityFeedback(), 0);
                assertEquals("A straight reference must not turn with feedback disabled", left, right, 1e-9);
                assertTrue("Disabling feedback must preserve feedforward", Math.abs(left) > 0);
            }
            follower.stop();
        }
    }

    private static ApexConstants dualRobot(DualActuated.DriveState initialState) {
        DualActuated.Constants drive = new DualActuated.Constants()
                .setFrontLeftMotor(new Motor(ApexSimulation.FRONT_LEFT_MOTOR))
                .setFrontRightMotor(new Motor(ApexSimulation.FRONT_RIGHT_MOTOR))
                .setBackLeftMotor(new Motor(ApexSimulation.BACK_LEFT_MOTOR))
                .setBackRightMotor(new Motor(ApexSimulation.BACK_RIGHT_MOTOR))
                .setInitialState(initialState).setTransitionSeconds(0);
        return new ApexConstants() {
            @Override public BaseDrivetrainConstants<?> drivetrainConstants() { return drive; }
            @Override public BaseLocalizerConstants<?> localizerConstants() {
                return new Constants().localizerConstants();
            }
        };
    }

    private static final class CountingMecanumConstants extends Mecanum.Constants {
        int builds;
        Mecanum built;
        @Override public Mecanum build(HardwareMap hardwareMap) {
            builds++;
            return built = super.build(hardwareMap);
        }
    }
}
