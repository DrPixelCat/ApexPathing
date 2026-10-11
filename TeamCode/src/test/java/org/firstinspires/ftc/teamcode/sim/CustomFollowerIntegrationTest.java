package org.firstinspires.ftc.teamcode.sim;

import customization.ExampleCustomFollower;
import customization.PathCustomizationTest;
import follower.Follower;
import follower.FollowerConstants;
import geometry.GeometryFactory;
import geometry.Pose;
import java.lang.reflect.Field;
import java.util.concurrent.atomic.AtomicLong;
import org.firstinspires.ftc.teamcode.apexpathing.Constants;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import paths.builders.HolonomicPathBuilder;
import paths.movements.Path;
import storage.ApexStorage;

import static org.junit.Assert.*;

/** Exercises an external tracker with shared lifecycle services and simulated motor/sensor I/O. */
public class CustomFollowerIntegrationTest {
    private Field singleton;
    private Object previous;
    private String previousStorage;
    private AtomicLong clock;
    private ExampleCustomFollower follower;
    private GeometryFactory geometry;

    @Before public void setup() throws Exception {
        singleton = FollowerConstants.class.getDeclaredField("instance"); singleton.setAccessible(true);
        previous = singleton.get(null);
        previousStorage = System.getProperty(ApexStorage.DIRECTORY_PROPERTY);
        singleton.set(null, SimulatorTuning.snapshot());
        clock = new AtomicLong(1_000_000_000L);
        follower = new ExampleCustomFollower(new Constants(), ApexSimulation.createHardware().hardwareMap, clock::get);
        SimulatorTuning.apply(follower.getConstants());
        geometry = new GeometryFactory(follower);
    }

    @After public void restore() throws Exception {
        singleton.set(null, previous); Follower.setDiagnostics(null);
        if (previousStorage == null) System.clearProperty(ApexStorage.DIRECTORY_PROPERTY);
        else System.setProperty(ApexStorage.DIRECTORY_PROPERTY, previousStorage);
    }

    private HolonomicPathBuilder builder() {
        return geometry.holonomicPath(geometry.pose(0, 0, 0), geometry.pose(40, 0, 0))
                .setCurve(new PathCustomizationTest.Wave())
                .setHeadingInterpolator(new PathCustomizationTest.LinearHeading());
    }

    private Path build(HolonomicPathBuilder builder, Path.BuildMode mode) {
        return mode == Path.BuildMode.ONLINE ? builder.quickBuild() : builder.buildToMemory();
    }

    @Test public void customTrackerUsesBothModesWithCallbacksDiagnosticsAndPauseResume() {
        for (Path.BuildMode mode : Path.BuildMode.values()) {
            int[] callbacks = {0};
            Path path = build(builder().addDistanceCallback(0, () -> callbacks[0]++), mode);
            follower.follow(path); follower.update();
            assertNotNull(follower.getTrajectoryTarget());
            assertTrue(follower.getDiagnostics().isVelocitySampleAvailable());
            assertEquals(1, callbacks[0]);
            assertBoundedOutput();
            clock.addAndGet(20_000_000L); follower.update();
            assertEquals(1, callbacks[0]);
            follower.pause();
            assertStopped();
            assertFalse(follower.getDiagnostics().isVelocitySampleAvailable());
            follower.resume(); clock.addAndGet(20_000_000L); follower.update();
            assertNotNull(follower.getTrajectoryTarget());
            assertEquals(1, callbacks[0]);
            follower.stop(); follower.follow(path); follower.update();
            assertEquals(2, callbacks[0]);
            follower.stop();
        }
    }

    @Test public void callbackStopOrPauseCannotBeOverwrittenByTheCustomCommand() {
        for (boolean pause : new boolean[]{false, true}) {
            follower.follow(builder().addDistanceCallback(0, () -> {
                if (pause) follower.pause(); else follower.stop();
            }).quickBuild());
            follower.update();
            assertStopped();
            assertEquals(pause, follower.isBusy());
            if (!pause) assertEquals(Follower.MovementOutcome.CANCELLED, follower.getMovementOutcome());
            follower.stop();
        }
    }

    @Test public void sharedCompletionRequiresTheSameDwellAndStopsMotorsInBothModes() {
        for (Path.BuildMode mode : Path.BuildMode.values()) {
            Path path = build(builder(), mode);
            follower.setPose(path.getEndPose()); follower.follow(path); follower.update();
            assertEquals(Follower.MovementOutcome.RUNNING, follower.getMovementOutcome());
            clock.addAndGet(99_000_000L); follower.update();
            assertEquals(Follower.MovementOutcome.RUNNING, follower.getMovementOutcome());
            clock.addAndGet(1_000_000L); follower.update();
            assertEquals(Follower.MovementOutcome.COMPLETED, follower.getMovementOutcome());
            assertFalse(follower.isBusy()); assertStopped();
        }
    }

    @Test public void sharedStuckDetectionStopsAndReportsFailure() {
        follower.setPose(Pose.zero()); follower.follow(builder().quickBuild()); follower.update();
        clock.addAndGet(499_000_000L); follower.update();
        assertEquals(Follower.MovementOutcome.RUNNING, follower.getMovementOutcome());
        clock.addAndGet(1_000_000L); follower.update();
        assertEquals(Follower.MovementOutcome.FAILED, follower.getMovementOutcome());
        assertNotNull(follower.getTrajectoryFailure());
        assertFalse(follower.isBusy()); assertStopped();
    }

    @Test public void explicitReferencePowerUsesSharedAllocationAndControllerSwitches() {
        follower.explicitFeedforward = true;
        follower.disableControllers();
        follower.follow(builder().quickBuild()); follower.update();
        assertEquals(.1, follower.getTrackingHeadingFeedforward(), 1e-9);
        assertEquals(0, follower.getCrossTrackCorrection().getMag().getIn(), 1e-9);
        // Heading=0 at the start, so the body command is the supplied forward/turn power.
        assertEquals(.3, follower.getDrivetrain().getMaxMotorPower(), 1e-6);
        assertBoundedOutput();
    }

    private void assertStopped() {
        for (double power : follower.getDrivetrain().getLastMotorPowers()) assertEquals(0, power, 0);
    }

    private void assertBoundedOutput() {
        for (double power : follower.getDrivetrain().getLastMotorPowers()) {
            assertTrue(Double.isFinite(power)); assertTrue(Math.abs(power) <= 1 + 1e-9);
        }
    }
}
