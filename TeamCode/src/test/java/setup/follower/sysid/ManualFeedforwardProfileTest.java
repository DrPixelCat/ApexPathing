package setup.follower.sysid;

import org.junit.Test;
import static org.junit.Assert.*;

public class ManualFeedforwardProfileTest {
    @Test public void profileHasFixedTimingAndReversesVelocityAndAcceleration() {
        ManualFeedforwardProfile forward = new ManualFeedforwardProfile(24);
        ManualFeedforwardProfile reverse = new ManualFeedforwardProfile(-24);
        assertEquals(12, forward.velocity(.5), 0);
        assertEquals(24, forward.velocity(1.5), 0);
        assertEquals(12, forward.velocity(2.5), 0);
        assertEquals(0, forward.velocity(3), 0);
        for (double t : new double[] {0, .5, 1, 1.5, 2, 2.5, 3}) {
            assertEquals(-forward.velocity(t), reverse.velocity(t), 0);
            assertEquals(-forward.acceleration(t), reverse.acceleration(t), 0);
        }
    }

    @Test public void exactTrackingHasZeroErrorAndBrakingCannotChangeScores() {
        ManualFeedforwardProfile profile = new ManualFeedforwardProfile(24);
        profile.begin(0);
        profile.sample(.4, 9.6);
        profile.sample(1, 24);
        profile.sample(1.3, 24);
        profile.sample(2, 24);
        assertEquals(0, profile.accelerationMeanError(), 1e-12);
        assertEquals(0, profile.cruiseMeanError(), 1e-12);
        profile.sample(2.5, -500);
        profile.sample(3, 500);
        assertEquals(0, profile.accelerationMeanError(), 1e-12);
        assertEquals(0, profile.cruiseMeanError(), 1e-12);
    }

    @Test public void cruiseStartsAtInterpolatedTargetCrossingAndRemainsScoredAfterDroop() {
        for (int direction : new int[] {1, -1}) {
            ManualFeedforwardProfile profile = new ManualFeedforwardProfile(direction * 10);
            profile.begin(0);
            profile.sample(1, direction * 8);
            profile.sample(1.5, direction * 12); // Reaches 10 at 1.25s.
            profile.sample(2, direction * 8);
            assertEquals(-direction, profile.accelerationMeanError(), 1e-12);
            // Error area from 1.25 to 1.5 = 0.25; from 1.5 to 2 = 0.
            assertEquals(direction / 3.0, profile.cruiseMeanError(), 1e-12);
        }
    }

    @Test public void neverReachingCruiseReportsUnavailableInsteadOfZero() {
        ManualFeedforwardProfile profile = new ManualFeedforwardProfile(10);
        profile.begin(0);
        profile.sample(1, 8);
        profile.sample(2, 8);
        profile.sample(3, 0);
        assertEquals(-1, profile.accelerationMeanError(), 1e-12);
        assertTrue(Double.isNaN(profile.cruiseMeanError()));
    }

    @Test public void irregularSamplingSplitsPhaseBoundariesAndWeightsTime() {
        ManualFeedforwardProfile profile = new ManualFeedforwardProfile(10);
        profile.begin(0);
        profile.sample(.5, 6);
        profile.sample(1.5, 18); // Linear measured velocity, including acceleration boundary.
        profile.sample(2.5, 18); // Includes braking, but only cruise through 2 is scored.
        assertEquals(1, profile.accelerationMeanError(), 1e-12);
        assertEquals(6.5, profile.cruiseMeanError(), 1e-12);
    }
}
