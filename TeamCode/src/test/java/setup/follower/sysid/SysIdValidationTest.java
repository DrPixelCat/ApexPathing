package setup.follower.sysid;

import org.junit.Test;
import static org.junit.Assert.*;

/** Prediction-quality gates belong to JUnit, independently of acquisition eligibility. */
public class SysIdValidationTest {
    static void assertFitQuality(SysIdRegression.Fit fit) {
        assertTrue("Fit must be identifiable", fit.valid);
        assertTrue("Power fit R squared", Double.isFinite(fit.rSquared) && fit.rSquared >= .9);
        assertTrue("Every excitation run must have unbiased predictions",
                Double.isFinite(fit.maxRunBias) && fit.maxRunBias <= .05);
    }

    @Test public void predictsIndependentExcitationsInBothDirections() {
        SysIdRegression data = new SysIdRegression();
        for (int run = 0; run < 4; run++) {
            double sign = run % 2 == 0 ? 1 : -1;
            for (int i = 0; i < 40; i++) {
                double v = sign * (2 + .4 * i);
                double a = sign * (run < 2 ? 2 : 20 * Math.exp(-.1 * i));
                data.add(run, .1 * i, v, a, .06 * sign + .012 * v + .006 * a);
            }
        }
        SysIdRegression.Fit fit = data.fit(2);
        assertFitQuality(fit);
        for (double sign : new double[] {-1, 1}) {
            for (double v : new double[] {3.3, 7.7, 13.1}) {
                for (double a : new double[] {-8, 0, 11}) {
                    SysIdRegression.Sample heldOut = new SysIdRegression.Sample(0, 0,
                            sign * v, sign * a, .06 * sign + .012 * sign * v + .006 * sign * a);
                    assertEquals("Independent acceleration/cruise/braking prediction",
                            heldOut.power, fit.predict(heldOut), 1e-10);
                }
            }
        }
    }

    @Test public void rejectsLowQualityAndNonfiniteMetrics() {
        assertThrows(AssertionError.class, () -> assertFitQuality(
                new SysIdRegression.Fit(new double[] {.06, .012, .006}, .89, .01, 0, true)));
        assertThrows(AssertionError.class, () -> assertFitQuality(
                new SysIdRegression.Fit(new double[] {.06, .012, .006}, .99, .01, .051, true)));
        assertThrows(AssertionError.class, () -> assertFitQuality(
                new SysIdRegression.Fit(new double[] {.06, .012, .006}, Double.NaN, .01, 0, true)));
    }
}
