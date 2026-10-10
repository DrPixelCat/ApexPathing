package tuning.follower.tracking;

import follower.FollowerConstants;
import org.json.JSONObject;
import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

/** Checks that automatic selection repeats one course and selects the winner without a validation run. */
public class ProfiledTrackingOptimizerTest {
    private static FollowerConstants seed() throws Exception {
        FollowerConstants constants = FollowerConstants.fromJson(
                new JSONObject().put("drivetrainType", "MECANUM"));
        constants.forwardVelLimitIn = 50.0;
        constants.translationalKV = 0.01;
        constants.velocityFeedbackGain = 0.04;
        constants.brakingPulsePower = 0.4;
        constants.brakingNeutralPowerMargin = 0.02;
        constants.brakingPulseCooldownCycles = 3;
        constants.profileCornerSpeedScale = 0.8;
        return constants;
    }

    @Test public void selectsOnlyMeasuredImprovement() throws Exception {
        FollowerConstants constants = seed();
        ProfiledTrackingOptimizer optimizer = new ProfiledTrackingOptimizer(constants, false, 1.0);
        int trials = 0;
        while (!optimizer.complete() && trials++ < 50) {
            double score = optimizer.candidate().velocityGain > 0.04 ? 8.0 : 10.0;
            optimizer.record(score, 0.0);
        }
        assertTrue(optimizer.complete());
        assertTrue("Expected a measured improvement in velocity gain",
                optimizer.best().velocityGain > 0.04);
        assertEquals(0.4, constants.brakingPulsePower, 0.0);
        optimizer.best().apply(constants, false);
        assertEquals(optimizer.best().pulsePower, constants.brakingPulsePower, 0.0);
        assertEquals(optimizer.best().velocityGain, constants.velocityFeedbackGain, 0.0);
    }

    @Test public void measuredWinnerCompletesWithoutAnotherRun() throws Exception {
        ProfiledTrackingOptimizer optimizer = new ProfiledTrackingOptimizer(seed(), false, 1.0);
        optimizer.record(10.0, 0.0);
        optimizer.record(10.0, 0.0);
        int candidates = 0;
        while (!optimizer.complete()) {
            assertTrue(optimizer.canRejectCurrentCandidate());
            optimizer.record(optimizer.candidate().velocityGain > 0.04 ? 8.0 : 10.0, 0.0);
            assertTrue(++candidates <= optimizer.candidateLimit());
        }
        assertEquals(optimizer.candidateLimit(), candidates);
        assertTrue(optimizer.best().velocityGain > 0.04);
        assertTrue(optimizer.candidate().sameAs(optimizer.best()));
    }

    @Test public void unusableCandidateAdvancesWithoutRetryingIt() throws Exception {
        ProfiledTrackingOptimizer optimizer = new ProfiledTrackingOptimizer(seed(), false, 1.0);
        for (int i = 0; i < 2; i++) {
            optimizer.record(10.0, 0.0);
        }
        assertTrue(optimizer.canRejectCurrentCandidate());
        assertEquals(1, optimizer.candidateNumber());
        optimizer.rejectCurrentCandidate();
        assertEquals(2, optimizer.candidateNumber());
        assertTrue(optimizer.status().contains("CORNER_SPEED_SCALE"));
    }
}
