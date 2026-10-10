package trajectory.generators;

import org.junit.Test;
import static org.junit.Assert.*;

/** Independent dense feasibility checks for swerve's signed quadratic power bounds. */
public class NormPowerBoundsTest {
    private static double demand(double[] c, double acceleration) {
        return Math.hypot(c[0] + c[1] * acceleration, c[2] + c[3] * acceleration)
                + Math.abs(c[4] + c[5] * acceleration);
    }

    @Test public void quadraticBoundsMatchConvexPowerAcrossRandomSignedBranches() {
        java.util.Random random = new java.util.Random(4981);
        int analytic = 0;
        for (int i = 0; i < 1000; i++) {
            double[] c = new double[6];
            for (int j = 0; j < c.length; j++) { c[j] = (random.nextDouble() * 2 - 1) * (j % 2 == 0 ? 1 : .03); }
            double min = -100 * random.nextDouble(), max = 100 * random.nextDouble();
            double limit = .1 + random.nextDouble() * 1.5;
            double candidate = BaseProfileGenerator.maximumNormPowerAcceleration(c, min, max, limit);
            if (candidate == Double.NEGATIVE_INFINITY) { continue; }
            analytic++;
            double gridBest = Double.NaN;
            for (int k = 0; k <= 2000; k++) {
                double acceleration = min + (max - min) * k / 2000;
                if (demand(c, acceleration) <= limit) { gridBest = acceleration; }
            }
            if (Double.isNaN(candidate)) { assertTrue("Never discard a feasible signed branch", Double.isNaN(gridBest)); }
            else {
                assertTrue(candidate >= min && candidate <= max);
                assertTrue(demand(c, candidate) <= limit + 1e-10);
                if (Double.isFinite(gridBest)) { assertEquals(gridBest, candidate, (max - min) / 2000 + 1e-8); }
            }
        }
        assertTrue("The analytic path must cover ordinary feasible and infeasible cases", analytic > 900);
    }

    @Test public void degenerateLinearAndZeroTranslationBranchesRemainFeasible() {
        assertEquals(20, BaseProfileGenerator.maximumNormPowerAcceleration(new double[]{0,.01,0,0,0,.02}, -100,100,.6), 1e-10);
        assertEquals(30, BaseProfileGenerator.maximumNormPowerAcceleration(new double[]{0,0,0,0,0,.02}, -100,100,.6), 1e-10);
        assertEquals(100, BaseProfileGenerator.maximumNormPowerAcceleration(new double[6], -100,100,.6), 0);
        assertTrue(Double.isNaN(BaseProfileGenerator.maximumNormPowerAcceleration(new double[]{2,0,0,0,0,0}, -100,100,.6)));
    }
}
