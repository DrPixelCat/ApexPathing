package tuning.follower.sysid;

import org.junit.Test;
import static org.junit.Assert.*;

public class FeedforwardTunerTest {
    @Test public void staticBreakawayUsesCommandedThresholdCrossingInEachDirection() {
        assertEquals(.087, FeedforwardTuner.breakawayCrossingPower(
                0, .01, .06, .087, .05), 1e-12);
        assertEquals(.091, FeedforwardTuner.breakawayCrossingPower(
                1, -.02, -.06, -.091, .05), 1e-12);
        assertTrue(Double.isNaN(FeedforwardTuner.breakawayCrossingPower(
                1, .02, .15, 0.0, .05)));
        assertTrue(Double.isNaN(FeedforwardTuner.breakawayCrossingPower(
                0, .01, .06, .01, .05)));
        assertTrue(Double.isNaN(FeedforwardTuner.breakawayCrossingPower(
                2, .01, .06, .087, .05)));
    }
}
