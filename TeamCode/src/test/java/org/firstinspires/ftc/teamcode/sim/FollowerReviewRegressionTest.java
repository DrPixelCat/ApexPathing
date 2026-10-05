package org.firstinspires.ftc.teamcode.sim;

import controllers.FeedbackController;
import follower.Follower;
import follower.FollowerConstants;
import geometry.GeometryFactory;
import org.firstinspires.ftc.teamcode.apexpathing.Constants;
import org.junit.Test;
import storage.ApexStorage;
import java.lang.reflect.Field;
import java.nio.file.Files;
import java.util.concurrent.atomic.AtomicLong;
import static org.junit.Assert.*;

public class FollowerReviewRegressionTest {
    @Test public void pauseAndFailureKeepOutputsStoppedAndSettingsAreIndependent() throws Exception {
        Field singleton = FollowerConstants.class.getDeclaredField("instance");
        singleton.setAccessible(true);
        Object original = singleton.get(null);
        String storage = System.getProperty(ApexStorage.DIRECTORY_PROPERTY);
        try {
            System.setProperty(ApexStorage.DIRECTORY_PROPERTY, Files.createTempDirectory("apex-review-").toString());
            FollowerConstants shared = SimulatorTuning.snapshot();
            singleton.set(null, shared);
            ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
            AtomicLong clock = new AtomicLong(1_000_000_000L);
            Follower follower = new Follower(new Constants(), hardware.hardwareMap, true, clock::get);
            double sharedGain = shared.translationalCoeffs.kP;
            follower.getConstants().translationalCoeffs.p(.2);
            assertEquals(sharedGain, shared.translationalCoeffs.kP, 0);
            GeometryFactory geometry = new GeometryFactory(follower);
            follower.getConstants().translationalCoeffs = new FeedbackController.Coefficients().p(.2).d(0).s(0);
            follower.getConstants().angularCoeffs = new FeedbackController.Coefficients().p(0).d(0).s(0);
            follower.reset();
            follower.setPose(geometry.pose(0, 0, 90));
            follower.setHoldPose(geometry.pose(4, 0, 90));
            follower.update(true);
            assertTrue("Field +X at 90 degrees requires body -Y",
                    follower.getDrivetrain().getLastFlPower() > 0);
            assertTrue(follower.getDrivetrain().getLastFrPower() < 0);
            follower.stop();
            follower.setPose(geometry.pose(0, 0, 0));
            paths.movements.Path path = geometry.holonomicPath(
                    geometry.pose(0, 0, 0), geometry.pose(24, 0, 0)).quickBuild();
            follower.follow(path);
            follower.pause();
            follower.update(true);
            assertEquals(0, follower.getDrivetrain().getLastFlPower(), 0);
            assertEquals(0, follower.getDrivetrain().getLastFrPower(), 0);
            follower.resume();
            for (int tick = 0; tick < 8 && follower.isBusy(); tick++) {
                clock.addAndGet(200_000_000L);
                follower.update();
            }
            assertEquals(Follower.MovementOutcome.FAILED, follower.getMovementOutcome());
            assertFalse(follower.movementSucceeded());
            assertEquals(0, follower.getDrivetrain().getLastFlPower(), 0);
            try {
                follower.follow(path);
                fail("A failed movement needs explicit acknowledgement");
            } catch (IllegalStateException expected) { }
            follower.clearFailure();
            follower.follow(path);
            assertEquals(Follower.MovementOutcome.RUNNING, follower.getMovementOutcome());
            follower.stop();
            assertEquals(Follower.MovementOutcome.CANCELLED, follower.getMovementOutcome());
        } finally {
            singleton.set(null, original);
            if (storage == null) { System.clearProperty(ApexStorage.DIRECTORY_PROPERTY); }
            else { System.setProperty(ApexStorage.DIRECTORY_PROPERTY, storage); }
        }
    }
}
