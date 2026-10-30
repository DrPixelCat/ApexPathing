package setup.follower.sysid;

import org.junit.Test;
import static org.junit.Assert.*;

public class FeedforwardTunerTest {
    @Test public void commandsSlowForwardReverseAndCounterclockwiseRamps() {
        for (double time : new double[]{0, .02, 2, 5, 12}) {
            double expected = Math.min(.9, .1 * time);
            assertEquals(expected, FeedforwardTuner.trialPower(FeedforwardTuner.Axis.DRIVE, 0, time), 0);
            assertEquals(-expected, FeedforwardTuner.trialPower(FeedforwardTuner.Axis.DRIVE, 1, time), 0);
            assertEquals(expected, FeedforwardTuner.trialPower(FeedforwardTuner.Axis.HEADING, 0, time), 0);
        }
    }
    @Test public void staticBreakawayUsesLastStationaryCommandInEachDirection() {
        assertEquals(.085, FeedforwardTuner.breakawayCrossingPower(
                0, .01, .06, .087, .085, .05), 1e-12);
        assertEquals(.089, FeedforwardTuner.breakawayCrossingPower(
                1, -.02, -.06, -.091, -.089, .05), 1e-12);
        assertTrue(Double.isNaN(FeedforwardTuner.breakawayCrossingPower(
                1, .02, .15, 0.0, 0.0, .05)));
        assertTrue(Double.isNaN(FeedforwardTuner.breakawayCrossingPower(
                0, .01, .06, .01, .02, .05)));
        assertTrue(Double.isNaN(FeedforwardTuner.breakawayCrossingPower(
                2, .01, .06, .087, .085, .05)));
    }
}
