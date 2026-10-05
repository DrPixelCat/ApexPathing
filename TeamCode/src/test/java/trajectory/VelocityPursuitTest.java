package trajectory;

import org.junit.Test;
import static org.junit.Assert.*;

public class VelocityPursuitTest {
    @Test public void convergesFromBothSidesWithoutCrossingConstantGoal() {
        for (double initial : new double[]{12, 20}) {
            VelocityPursuit pursuit = new VelocityPursuit();
            double v = initial, previousError = Math.abs(v - 16);
            for (int tick = 0; tick < 400; tick++) {
                VelocityPursuit.Goal goal = pursuit.goal(0, 100, v, .35,
                        new double[]{0, 100}, new double[]{256, 256}, 2);
                double a = pursuit.damp(goal, 0, v, .35, .02);
                pursuit.applied(a); v += a * .02;
                assertTrue("Convergence must not cross the fixed speed goal", (v - 16) * (initial - 16) >= -1e-6);
                assertTrue("Speed error must decrease", Math.abs(v - 16) <= previousError + 1e-8);
                previousError = Math.abs(v - 16);
            }
            assertEquals(16, v, .001);
        }
    }

    @Test public void cannotSkipASlowCornerToChaseFasterSpeedBeyondIt() {
        VelocityPursuit.Goal goal = new VelocityPursuit().goal(0, 10, 20, .35,
                new double[]{0, 3, 6, 10}, new double[]{400, 25, 400, 400}, 4);
        assertEquals(3, goal.distanceIn, 1e-9);
        assertEquals(5, goal.velocity, 1e-9);
        assertEquals((25 - 400) / 6.0, goal.acceleration, 1e-9);
    }

    @Test public void endpointUsesRemainingDistanceAndInvalidTimingResetsDamping() {
        VelocityPursuit pursuit = new VelocityPursuit();
        VelocityPursuit.Goal stop = pursuit.goal(8, 10, 8, .35,
                new double[]{8, 10}, new double[]{64, 0}, 2);
        assertEquals(10, stop.distanceIn, 1e-9);
        assertEquals(-16, stop.acceleration, 1e-9);
        pursuit.applied(100);
        double afterGap = pursuit.damp(stop, 8, 8, .35, 1);
        assertTrue(afterGap < 0);
        assertTrue(Double.isFinite(pursuit.damp(stop, 8, 8, .35, Double.NaN)));
    }
}
