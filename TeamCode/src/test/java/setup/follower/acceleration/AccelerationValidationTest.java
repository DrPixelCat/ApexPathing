package setup.follower.acceleration;

import org.junit.Test;
import static org.junit.Assert.*;

/** Independent repeatability checks; never part of acquisition or saving. */
public class AccelerationValidationTest {
    @Test public void confirmationAllowsSmallNoiseWithoutAcceptingRealRegression() {
        assertTrue(confirmationAcceptable(.108, .238, 18));
        assertTrue(confirmationAcceptable(0, .01, 1.2));
        org.junit.Assert.assertFalse(
                confirmationAcceptable(.108, 1, 18));
        org.junit.Assert.assertFalse(
                confirmationAcceptable(.01, .1, 1.2));
        org.junit.Assert.assertFalse(
                confirmationAcceptable(.1, Double.NaN, 18));
    }

    static boolean confirmationAcceptable(double bestRms, double confirmedRms, double peak) {
        if (!Double.isFinite(bestRms) || bestRms < 0.0
                || !Double.isFinite(confirmedRms) || confirmedRms < 0.0
                || !Double.isFinite(peak) || peak <= 0.0) { return false; }
        // A relative-only comparison becomes arbitrarily strict as the best score
        // approaches zero. Allow a small, axis-scaled repeatability floor as well.
        return confirmedRms <= bestRms + Math.max(.25 * bestRms, .02 * peak);
    }

}
