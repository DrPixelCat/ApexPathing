package org.firstinspires.ftc.teamcode.sim;


import static org.junit.Assert.*;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import config.ApexConstants;
import controllers.FeedbackController.Coefficients;
import drivetrains.BaseDrivetrain;
import drivetrains.BaseDrivetrainConstants;
import drivetrains.Motor;
import drivetrains.Omni;
import follower.Follower;
import follower.FollowerConstants;
import follower.drivetrains.OmniFollower;
import geometry.Angle;
import geometry.GeometryFactory;
import geometry.Pose;
import java.lang.reflect.Field;
import java.nio.file.Files;
import java.util.concurrent.atomic.AtomicLong;
import localizers.BaseLocalizerConstants;
import org.firstinspires.ftc.teamcode.apexpathing.Constants;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import paths.movements.Path;
import paths.movements.Turn;
import storage.ApexStorage;

/** Real indexed motor I/O with stationary localization; no circular-omni chassis physics model. */
public class OmniFollowerTest {
    private Field singleton;
    private Object previous;
    private String previousStorage;

    @Before public void setup() throws Exception {
        singleton = FollowerConstants.class.getDeclaredField("instance");
        singleton.setAccessible(true);
        previous = singleton.get(null);
        singleton.set(null, SimulatorTuning.snapshot());
        previousStorage = System.getProperty(ApexStorage.DIRECTORY_PROPERTY);
        System.setProperty(ApexStorage.DIRECTORY_PROPERTY,
                Files.createTempDirectory("apex-omni-follower-").toString());
    }

    @After public void restore() throws Exception {
        singleton.set(null, previous);
        if (previousStorage == null) { System.clearProperty(ApexStorage.DIRECTORY_PROPERTY); }
        else { System.setProperty(ApexStorage.DIRECTORY_PROPERTY, previousStorage); }
    }

    @Test public void factoryBuildsEveryMotorOnceForEachWheelCount() {
        for (int count : new int[]{3, 4, 5, 6, 8, 12}) {
            Fixture fixture = new Fixture(count);
            Follower follower = Follower.create(fixture.robot, fixture.hardwareMap, true);
            assertEquals(OmniFollower.class, follower.getClass());
            assertEquals(1, fixture.drive.builds);
            assertEquals(count, fixture.motorBuilds);
            assertSame(fixture.drive.built, follower.getDrivetrain());
            assertEquals(count, follower.getDrivetrain().getMotorCount());
            assertEquals(BaseDrivetrain.DrivetrainType.OMNI, follower.getConstants().drivetrainType);
            assertTrue(follower.getDrivetrain().isHolonomic());
            follower.manual(0, 0, .3);
            for (double power : follower.getDrivetrain().getLastMotorPowers()) {
                assertEquals(.3, power, 1e-12);
            }
            follower.stop();
            assertStopped(follower);
        }
    }

    @Test public void modelSnapshotsMatchPhysicalWheelsAfterConfigurationChanges() {
        for(int count:new int[]{3,5,8}) {
            Fixture fixture=new Fixture(count);
            fixture.drive.setFirstWheelAngle(Angle.fromDeg(17));
            Follower follower=Follower.create(fixture.robot,fixture.hardwareMap,true);
            drivetrains.models.DriveModel model=follower.getDriveModel();
            fixture.drive.setFirstWheelAngle(Angle.fromDeg(90));
            follower.getConstants().drivetrainType=BaseDrivetrain.DrivetrainType.MECANUM;
            GeometryFactory geometry=new GeometryFactory(follower);
            Path path=geometry.holonomicPath(geometry.pose(),geometry.pose(40,0)).quickBuild();
            for(int i=0;i<40;i++) {
                double x=Math.cos(i*.3),y=Math.sin(i*.3);
                double actual=((Omni)follower.getDrivetrain()).translationDemand(x,y);
                assertEquals(actual,model.translationDemand(x,y),1e-12);
                assertEquals(actual,path.createGenerator().getDriveModel().translationDemand(x,y),1e-12);
            }
            assertEquals(1,fixture.drive.builds);
            follower.stop();
        }
    }

    @Test public void explicitOmniConstructionRejectsOtherDriveTypes() {
        Fixture fixture = new Fixture(6);
        new OmniFollower(fixture.robot, fixture.hardwareMap, true).stop();
        assertThrows(IllegalArgumentException.class, () -> new OmniFollower(new Constants(),
                ApexSimulation.createHardware().hardwareMap, true));
        assertThrows(IllegalArgumentException.class, () -> new OmniFollower(new TankSimulationConstants(),
                ApexSimulation.createTankHardware().hardwareMap, true));
    }

    @Test public void onlineAndStoredPathsKeepLifecycleAndDriveAdditionalWheels() {
        for (int count : new int[]{3, 5, 8}) {
            for (Path.BuildMode mode : Path.BuildMode.values()) {
                Fixture fixture = new Fixture(count);
                AtomicLong clock = new AtomicLong(1_000_000_000L);
                Follower follower = new OmniFollower(fixture.robot, fixture.hardwareMap, true, clock::get);
                SimulatorTuning.apply(follower.getConstants());
                follower.getConstants().motionCharacterization = null;
                follower.reset();
                GeometryFactory geometry = new GeometryFactory(follower);
                Path path = geometry.holonomicPath(geometry.pose(0, 0, 0), geometry.pose(40, 0, 0)).quickBuild();
                assertEquals(drivetrains.models.HolonomicDriveModel.Metric.OMNI,
                        ((drivetrains.models.HolonomicDriveModel)path.createGenerator(follower.getConstants()).getDriveModel()).getMetric());
                path.setBuildMode(mode);
                path.precomputeTrajectory();
                follower.follow(path);
                follower.update();
                assertNotNull(follower.getTrajectoryTarget());
                assertTrue(follower.getLastCommandDemand().available);
                assertFalse(follower.isWheelFeedforwardActive());
                assertTrue(follower.getDrivetrain().getMaxMotorPower() > 0);
                if (count > 4) { assertTrue(follower.getDrivetrain().getLastMotorPower(count - 1) > 0); }
                follower.pause();
                assertNull(follower.getTrajectoryTarget());
                assertStopped(follower);
                clock.addAndGet(1_000_000_000L);
                follower.update();
                follower.resume();
                follower.update();
                assertEquals(Follower.MovementOutcome.RUNNING, follower.getMovementOutcome());
                clock.addAndGet(500_000_000L);
                follower.update();
                assertEquals(Follower.MovementOutcome.FAILED, follower.getMovementOutcome());
                assertStopped(follower);
                assertThrows(IllegalStateException.class, () -> follower.follow(path));
                follower.clearFailure();
                follower.follow(path);
                follower.manual(0, 0, .2);
                assertEquals(Follower.MovementOutcome.CANCELLED, follower.getMovementOutcome());
                assertFalse(follower.isBusy());
                follower.stop();
                assertStopped(follower);
            }
        }
    }

    @Test public void turnsAndPoseHoldingUseAllWheelTangents() {
        Fixture fixture = new Fixture(6);
        Follower follower = new OmniFollower(fixture.robot, fixture.hardwareMap, true);
        follower.setPose(Pose.zero());
        follower.setDriveCoefficients(new Coefficients().p(.1).d(0).s(0));
        follower.setHeadingCoefficients(new Coefficients().p(.1).d(0).s(0));
        GeometryFactory geometry = new GeometryFactory(follower);
        follower.setHoldPose(geometry.pose(0, 4, 0));
        follower.update(true);
        for (int i = 0; i < 6; i++) {
            assertEquals(.4 * Math.cos(i * Math.PI / 3), follower.getDrivetrain().getLastMotorPower(i), 1e-9);
        }
        follower.stop();
        for (double target : new double[]{90, -90}) {
            Turn turn = geometry.turn(Pose.zero()).turnTo(Angle.fromDeg(target)).quickBuild();
            follower.follow(turn);
            follower.update();
            for (double power : follower.getDrivetrain().getLastMotorPowers()) {
                assertTrue(power * target > 0);
            }
            follower.stop();
            assertStopped(follower);
        }
    }

    @Test public void customDrivetrainModelReachesStoredAndOnlinePlanningAndExecution() {
        for (Path.BuildMode mode : Path.BuildMode.values()) {
            Fixture fixture=new Fixture(5); fixture.drive.customModel=true;
            AtomicLong clock=new AtomicLong(1_000_000_000L);
            Follower follower=new OmniFollower(fixture.robot,fixture.hardwareMap,true,clock::get);
            SimulatorTuning.apply(follower.getConstants()); follower.getConstants().motionCharacterization=null;
            follower.disableControllers(); follower.setPose(Pose.zero());
            GeometryFactory geometry=new GeometryFactory(follower);
            Path path=geometry.holonomicPath(geometry.pose(),geometry.pose(80,0)).quickBuild();
            path.setBuildMode(mode); path.precomputeTrajectory();
            assertTrue(path.createGenerator().getDriveModel() instanceof ReservedModel);
            assertTrue(follower.getDriveModel() instanceof ReservedModel);
            follower.follow(path); follower.update();
            assertTrue(follower.getTrajectoryTarget().expectedMotorPower>0);
            assertEquals(follower.getTrajectoryTarget().expectedMotorPower,
                    follower.getLastCommandDemand().feedforwardTotal,1e-8);
            assertEquals(follower.getLastCommandDemand().feedforwardTotal
                            *follower.getLastCommandDemand().feedforwardScale,
                    2*follower.getDrivetrain().getMaxMotorPower(),.002);
            follower.stop();
        }
    }
    @Test public void drivetrainDirectionalOverrideSurvivesStoredSnapshotsAndReversal() {
        Fixture fixture=new Fixture(3); fixture.drive.customModel=true;
        Follower follower=new OmniFollower(fixture.robot,fixture.hardwareMap,true);
        follower.getConstants().motionCharacterization=null;
        follower.getConstants().forwardVelLimitIn=60; follower.getConstants().forwardAccelLimitIn=100;
        GeometryFactory geometry=new GeometryFactory(follower);
        Path stored=geometry.holonomicPath(geometry.pose(),geometry.pose(100,0)).buildToMemory();
        assertEquals(15,stored.createGenerator().getDriveModel().directionalLimits(Angle.zero()).maxVelocity,1e-12);
        assertEquals(20,stored.createGenerator().getDriveModel().directionalLimits(Angle.zero()).maxAcceleration,1e-12);
        follower.getConstants().forwardVelLimitIn=120;
        assertEquals(15,stored.reversed().createGenerator().getDriveModel().directionalLimits(Angle.zero()).maxVelocity,1e-12);
        assertEquals(30,follower.getDriveModel().directionalLimits(Angle.zero()).maxVelocity,1e-12);
        follower.stop();
    }

    /** Deliberately reserves twice physical motor demand, proving that runtime uses the provided model. */
    private static final class ReservedModel extends drivetrains.models.DriveModel {
        private final drivetrains.models.HolonomicDriveModel delegate;
        ReservedModel(FollowerConstants settings,drivetrains.models.HolonomicDriveModel delegate) { super(settings); this.delegate=delegate; }
        @Override public drivetrains.models.DriveModel withConstants(FollowerConstants settings) {
            return new ReservedModel(settings,(drivetrains.models.HolonomicDriveModel)delegate.withConstants(settings));
        }
        @Override public boolean isHolonomic() { return true; }
        @Override public double maximumVelocity(geometry.Vector body,double curvature,double prime,double angular) {
            return delegate.maximumVelocity(body,curvature,prime,angular);
        }
        @Override public double maximumTangentialAcceleration(geometry.Vector body) { return delegate.maximumTangentialAcceleration(body); }
        @Override public double translationDemand(double x,double y) { return 2*delegate.translationDemand(x,y); }
        @Override public double commandDemand(double x,double y,double turn) { return 2*delegate.commandDemand(x,y,turn); }
        @Override public PowerEvaluator powerEvaluator(geometry.Vector body,double curvature) { return new PowerEvaluator(body,curvature) {}; }
    }

    private static void assertStopped(Follower follower) {
        assertArrayEquals(new double[follower.getDrivetrain().getMotorCount()],
                follower.getDrivetrain().getLastMotorPowers(), 0);
    }

    private static final class Fixture {
        final StableSimHardwareMap hardwareMap = new StableSimHardwareMap();
        final CountingOmniConstants drive = new CountingOmniConstants();
        int motorBuilds;
        final ApexConstants robot = new ApexConstants() {
            @Override public BaseDrivetrainConstants<?> drivetrainConstants() { return drive; }
            @Override public BaseLocalizerConstants<?> localizerConstants() {
                return new Constants().localizerConstants();
            }
        };

        Fixture(int count) {
            String[] sourceNames = {ApexSimulation.FRONT_LEFT_MOTOR, ApexSimulation.FRONT_RIGHT_MOTOR,
                    ApexSimulation.BACK_LEFT_MOTOR, ApexSimulation.BACK_RIGHT_MOTOR};
            ApexSimulation.Hardware source = null;
            for (int i = 0; i < count; i++) {
                if (i % 4 == 0) {
                    source = ApexSimulation.createHardware();
                    if (i == 0) {
                        hardwareMap.register(ApexSimulation.PINPOINT,
                                source.hardwareMap.get(SimApexPinpoint.class, ApexSimulation.PINPOINT));
                    }
                }
                String name = "omni" + i;
                hardwareMap.register(name, source.hardwareMap.get(DcMotorEx.class, sourceNames[i % 4]));
                drive.addWheel(new Motor(name) {
                    @Override public DcMotorEx build(HardwareMap hardwareMap) {
                        motorBuilds++;
                        return super.build(hardwareMap);
                    }
                });
            }
        }
    }

    private static final class CountingOmniConstants extends Omni.Constants {
        int builds;
        boolean customModel;
        Omni built;
        @Override public Omni build(HardwareMap hardwareMap) {
            builds++;
            if (!customModel) return built=super.build(hardwareMap);
            return built=new Omni(this,hardwareMap) {
                @Override protected drivetrains.models.DirectionalKinematics createDirectionalKinematics(FollowerConstants settings) {
                    return angle -> new drivetrains.models.DirectionalLimits(settings.forwardVelLimitIn/4,settings.forwardAccelLimitIn/5);
                }
                @Override public drivetrains.models.DriveModel createDriveModel(FollowerConstants settings,Path.PathType mode) {
                    drivetrains.models.DriveModel model=super.createDriveModel(settings,mode);
                    return mode==Path.PathType.HOLONOMIC
                            ? new ReservedModel(settings,(drivetrains.models.HolonomicDriveModel)model) : model;
                }
            };
        }
    }
}
