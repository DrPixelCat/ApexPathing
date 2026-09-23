package controllers;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

import feedforward.MotionParameters;

public class TurnControllerTest {
    @Test
    public void profiledTurnsUseHeadingDerivativeButNotSeparateVelocityFeedback() {
        FeedbackController.Coefficients headingCoefficients =
                new FeedbackController.Coefficients().p(0.1).d(0.8).s(0.0);
        TurnController noVelocityGain = new TurnController(
                headingCoefficients, 0.0, 0.0, 0.0, 0.0);
        TurnController largeVelocityGain = new TurnController(
                headingCoefficients, 0.0, 0.0, 0.0, 0.5);
        MotionParameters targets = new MotionParameters().setAngularVel(1.0);

        double expected = noVelocityGain.calculateProfiled(0.2, 1.0, targets, 0.5);
        assertEquals(expected,
                largeVelocityGain.calculateProfiled(0.2, 1.0, targets, 0.5), 1e-9);
        assertEquals(0.42, expected, 1e-9);
        assertEquals(0.18,
                largeVelocityGain.calculateProfiled(0.2, 1.0, targets, 0.8), 1e-9);
    }

    @Test
    public void stalledProfiledMotionRetainsBreakawayAuthorityDuringDeceleration() {
        assertEquals(0.26375, TurnController.ensureProfiledMotionBreakaway(
                0.20, 3.6, 0.0, 0.24375), 1e-9);
        assertEquals(-0.26375, TurnController.ensureProfiledMotionBreakaway(
                -0.20, -3.6, 0.0, 0.24375), 1e-9);
    }

    @Test
    public void lowSpeedProfileDoesNotStopBeforeEndpointRecovery() {
        assertEquals(0.26375, TurnController.ensureProfiledMotionBreakaway(
                0.04, 0.30, 0.10, 0.24375), 1e-9);
        assertEquals(-0.26375, TurnController.ensureProfiledMotionBreakaway(
                -0.04, -0.30, -0.10, 0.24375), 1e-9);
    }

    @Test
    public void breakawayGuardDoesNotAlterMovingOrStoppedProfileCommands() {
        assertEquals(0.20, TurnController.ensureProfiledMotionBreakaway(
                0.20, 3.6, 0.5, 0.24375), 1e-9);
        assertEquals(0.0, TurnController.ensureProfiledMotionBreakaway(
                0.0, 0.0, 0.0, 0.24375), 1e-9);
        assertEquals(0.40, TurnController.ensureProfiledMotionBreakaway(
                0.40, 3.6, 0.0, 0.24375), 1e-9);
    }

    @Test
    public void breakawayGuardCorrectsACommandOpposingRequestedMotion() {
        assertEquals(0.26375, TurnController.ensureProfiledMotionBreakaway(
                -0.10, 3.6, 0.0, 0.24375), 1e-9);
    }

}
