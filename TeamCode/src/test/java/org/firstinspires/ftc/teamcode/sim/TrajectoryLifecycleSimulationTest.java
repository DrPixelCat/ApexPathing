package org.firstinspires.ftc.teamcode.sim;

import follower.Follower;
import follower.FollowerConstants;
import follower.FollowerDiagnostics;
import trajectory.profile.CoupledMotionProfile;
import geometry.AngleUnit;
import geometry.Dist;
import geometry.DistUnit;
import geometry.GeometryFactory;
import org.firstinspires.ftc.teamcode.apexpathing.Constants;
import org.json.JSONObject;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import paths.movements.Path;
import paths.movements.Turn;
import java.lang.reflect.Field;
import static org.junit.Assert.*;

/** Exercises traversal boundaries and failure handling against simulated motor/sensor I/O. */
public class TrajectoryLifecycleSimulationTest {
    private Field singleton;
    private Object previous;
    private FollowerConstants constants;
    @Before public void setup() throws Exception {
        singleton = FollowerConstants.class.getDeclaredField("instance"); singleton.setAccessible(true);
        previous = singleton.get(null);
        constants = FollowerConstants.fromJson(new JSONObject().put("drivetrainType", "MECANUM"));
        singleton.set(null, constants);
    }
    @After public void restore() throws Exception { singleton.set(null, previous); Follower.setDiagnostics(null); }
    private Follower follower(ApexSimulation.Hardware hardware) {
        Follower follower = new Follower(new Constants(), hardware.hardwareMap);
        constants.forwardVelLimitIn = constants.strafeVelLimitIn = 60;
        constants.forwardAccelLimitIn = constants.strafeAccelLimitIn = 100;
        constants.angularVelLimitRad = 4; constants.angularAccelLimitRad = 8;
        constants.translationalKV = .01; constants.translationalKA = .005;
        constants.angularKV = .1; constants.angularKA = .02;
        return follower;
    }
    @Test public void pauseResumeAndRestartResetTargetsInBothModes() throws Exception {
        for (Path.BuildMode mode : Path.BuildMode.values()) {
            ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
            Follower follower = follower(hardware);
            GeometryFactory geometry = new GeometryFactory().setDistUnit(DistUnit.IN).setAngleUnit(AngleUnit.DEG);
            Path path = geometry.holonomicPath(geometry.pose(0,0,0), geometry.pose(40,0,0)).quickBuild();
            path.setBuildMode(mode); path.precomputeTrajectory();
            follower.follow(path); follower.update();
            assertNotNull(follower.getTrajectoryTarget());
            assertTrue(follower.getTrajectoryTarget().tangentialAcceleration() > 0);
            long solves = CoupledMotionProfile.getSolveCount();
            follower.pause(); assertNull(follower.getTrajectoryTarget());
            assertEquals(0, follower.getDrivetrain().getLastFlPower(), 0);
            Thread.sleep(10); follower.resume(); follower.update();
            assertNotNull(follower.getTrajectoryTarget());
            assertEquals(solves + (mode == Path.BuildMode.ONLINE ? 1 : 0), CoupledMotionProfile.getSolveCount());
            follower.stop(); follower.follow(path); assertNull(follower.getTrajectoryTarget());
            follower.update(); assertTrue(follower.getTrajectoryTarget().tangentialAcceleration() > 0);
            follower.stop();
            if (mode == Path.BuildMode.MEMORY) {
                constants.translationalKV += .01;
                long storedSolves = CoupledMotionProfile.getSolveCount();
                follower.follow(path); follower.update();
                assertTrue(follower.isBusy()); assertNull(follower.getTrajectoryFailure());
                assertEquals(storedSolves, CoupledMotionProfile.getSolveCount());
                follower.stop();
            }
        }
    }
    @Test public void solverFailureStopsAndReportsInsteadOfUsingAnotherController() throws Exception {
        ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
        Follower follower = follower(hardware);
        constants.translationalKV = 1; constants.translationalKA = 0;
        GeometryFactory geometry = new GeometryFactory().setDistUnit(DistUnit.IN);
        Path path = geometry.holonomicPath(geometry.pose(0,0), geometry.pose(100,0))
                .setLookaheadDistance(Dist.fromIn(10)).setProfilePowerLimit(.01).quickBuild();
        final String[] failure = {null};
        Follower.setDiagnostics(new FollowerDiagnostics() {
            @Override public void recordPose(geometry.Pose pose) { }
            @Override public void recordCurrentPath(Path current) { }
            @Override public void clearCurrentPath() { }
            @Override public void recordTrajectoryFailure(String message) { failure[0] = message; }
        });
        follower.follow(path);
        StockSimTestSupport.setSpeed(hardware, 20); StockSimTestSupport.step(hardware, .02);
        assertThrows(IllegalStateException.class, follower::update);
        assertFalse(follower.isBusy()); assertNotNull(failure[0]); assertNull(follower.getTrajectoryTarget());
        assertEquals(0, follower.getDrivetrain().getLastFlPower(), 0);
        assertEquals(0, follower.getDrivetrain().getLastFrPower(), 0);
    }

    @Test public void quickTurnsCommandTheFinalHeadingWithoutProfilesOrSolves() throws Exception {
        ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
        Follower follower = follower(hardware);
        GeometryFactory geometry = new GeometryFactory().setDistUnit(DistUnit.IN).setAngleUnit(AngleUnit.DEG);
        // PID turning does not need calibrated profile speed or acceleration limits.
        constants.angularVelLimitRad = constants.angularAccelLimitRad = 0;
        long solves = CoupledMotionProfile.getSolveCount();
        for (double target : new double[]{90, -90}) {
            Turn turn = geometry.turn(geometry.pose(0, 0, 0)).turnTo(geometry.angle(target)).quickBuild();
            assertNull(turn.getFeedforwardLut());
            follower.setPose(geometry.pose(0, 0, 0)); follower.follow(turn); follower.update();
            assertNotNull(follower.getTrajectoryTarget());
            assertEquals(0, follower.getTrajectoryTarget().heading.getShortestAngleTo(geometry.angle(target)).getRad(), 1e-9);
            assertEquals(0, follower.getTrajectoryTarget().angularVelocity, 0);
            assertEquals(0, follower.getTrajectoryTarget().angularAcceleration, 0);
            if (target != 0) {
                assertTrue(Math.abs(follower.getDrivetrain().getLastFlPower()) > 0);
                follower.pause(); follower.resume(); follower.update();
            } else { assertFalse(follower.isBusy()); }
            follower.stop();
        }
        assertEquals(solves, CoupledMotionProfile.getSolveCount());
    }

    @Test public void pausedTurnsKeepUnwrappingMeasuredHeadingInBothModes() throws Exception {
        for (Path.BuildMode mode : Path.BuildMode.values()) {
            ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
            Follower follower = follower(hardware);
            GeometryFactory geometry = new GeometryFactory().setDistUnit(DistUnit.IN).setAngleUnit(AngleUnit.DEG);
            Turn turn = geometry.turn(geometry.pose(0, 0, 0)).turnTo(geometry.angle(160)).quickBuild();
            turn.setBuildMode(mode); turn.precomputeTrajectory();
            follower.follow(turn); follower.update(); follower.pause();
            for (double heading : new double[]{90, 179, -90, -10}) {
                follower.setPose(geometry.pose(0, 0, heading)); follower.update();
                assertNull(follower.getTrajectoryTarget());
            }
            follower.resume(); follower.update();
            assertEquals(Math.toRadians(160), follower.getTrajectoryTarget().angularProgressRad, 1e-6);
            assertEquals(0, follower.getTrajectoryTarget().remainingAngleRad, 1e-6);
            follower.stop();
            follower.setPose(geometry.pose(0, 0, 0)); follower.follow(turn); follower.update();
            assertEquals(0, follower.getTrajectoryTarget().angularProgressRad, 1e-6);
            follower.stop();
        }
    }
}
