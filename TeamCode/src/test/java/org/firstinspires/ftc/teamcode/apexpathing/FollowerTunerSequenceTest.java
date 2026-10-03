package org.firstinspires.ftc.teamcode.apexpathing;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class FollowerTunerSequenceTest {
    @Test
    public void advancesThroughEveryTuningPhaseInOrder() {
        FollowerTuner.Phase[] phases = FollowerTuner.Phase.values();

        for (int i = 0; i < phases.length - 1; i++) {
            assertEquals(phases[i + 1], FollowerTuner.nextPhase(phases[i]));
        }
        assertNull(FollowerTuner.nextPhase(phases[phases.length - 1]));
    }

    @Test public void sysIdAxesRunBeforeControllersAndLimits() {
        assertArrayEquals(new FollowerTuner.Phase[] {
                FollowerTuner.Phase.DRIVE_FEEDFORWARD,
                FollowerTuner.Phase.HEADING_FEEDFORWARD,
                FollowerTuner.Phase.MOTION_CHARACTERIZATION,
                FollowerTuner.Phase.HEADING,
                FollowerTuner.Phase.DRIVE,
                FollowerTuner.Phase.LIMITS,
                FollowerTuner.Phase.CENTRIPETAL,
                FollowerTuner.Phase.PROFILED_TRACKING
        }, FollowerTuner.Phase.values());
    }
    @Test
    public void profiledTrackingFollowsCentripetal() {
        assertEquals(FollowerTuner.Phase.PROFILED_TRACKING,
                FollowerTuner.nextPhase(FollowerTuner.Phase.CENTRIPETAL));
        assertNull(FollowerTuner.nextPhase(FollowerTuner.Phase.PROFILED_TRACKING));
    }

    @Test
    public void feedforwardCompletionDoesNotRequireAccelerationOrNonzeroFriction() throws Exception {
        follower.FollowerConstants constants = follower.FollowerConstants.fromJson(
                new org.json.JSONObject().put("drivetrainType", "MECANUM"));
        constants.angularKV = .1;
        constants.translationalKV = .01;
        constants.angularKA = constants.translationalKA = 0;
        constants.angularFeedforwardKS = constants.translationalFeedforwardKS = 0;
        assertTrue(FollowerTuner.Phase.DRIVE_FEEDFORWARD.isTunedPredicate.test(constants));
        constants.translationalKV = 0;
        assertFalse(FollowerTuner.Phase.DRIVE_FEEDFORWARD.isTunedPredicate.test(constants));
    }

    @Test
    public void everyPhaseCanBeSelectedRegardlessOfPriorCompletion() {
        FollowerTuner.Phase[] phases = FollowerTuner.Phase.values();
        boolean[] original = new boolean[phases.length];
        for (int i = 0; i < phases.length; i++) {
            original[i] = phases[i].tuned;
            phases[i].tuned = false;
        }

        try {
            assertTrue(FollowerTuner.phaseAvailable(FollowerTuner.Phase.HEADING));
            assertTrue(FollowerTuner.phaseAvailable(FollowerTuner.Phase.DRIVE_FEEDFORWARD));
            assertTrue(FollowerTuner.phaseAvailable(FollowerTuner.Phase.LIMITS));
            assertTrue(FollowerTuner.phaseAvailable(FollowerTuner.Phase.HEADING_FEEDFORWARD));
            assertTrue(FollowerTuner.phaseAvailable(FollowerTuner.Phase.DRIVE));
            FollowerTuner.Phase.HEADING.tuned = true;
            assertTrue(FollowerTuner.phaseAvailable(FollowerTuner.Phase.HEADING));
            assertTrue(FollowerTuner.phaseAvailable(FollowerTuner.Phase.DRIVE_FEEDFORWARD));
            assertTrue(FollowerTuner.phaseAvailable(FollowerTuner.Phase.LIMITS));
            assertTrue(FollowerTuner.phaseAvailable(FollowerTuner.Phase.HEADING_FEEDFORWARD));
            assertTrue(FollowerTuner.phaseAvailable(FollowerTuner.Phase.DRIVE));
        } finally {
            for (int i = 0; i < phases.length; i++) { phases[i].tuned = original[i]; }
        }
    }
}
