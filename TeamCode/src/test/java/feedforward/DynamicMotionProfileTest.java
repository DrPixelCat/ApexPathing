package feedforward;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class DynamicMotionProfileTest {
    @Test public void accelerationFallsAsVelocityConsumesPower() {
        double kV = .10, kA = .05, kS = .10;
        DynamicMotionProfile profile = DynamicMotionProfile.forPowerLimitedDistance(
                20.0, 101,
                new DynamicMotionProfile.PowerModel(10.0, 20.0, kV, kA, kS));
        DynamicMotionProfile.State[] states = profile.getStates();

        double firstAcceleration = states[0].acceleration;
        double laterAcceleration = firstAcceleration;
        for (DynamicMotionProfile.State state : states) {
            double power = state.acceleration * kA + state.velocity * kV
                    + (state.velocity > 1e-9 || Math.abs(state.acceleration) > 1e-9 ? kS : 0.0);
            assertTrue("positive power exceeded limit: " + power, power <= 1.0 + 1e-9);
            assertTrue("negative power exceeded limit: " + power, power >= -1.0 - 1e-9);
            if (state.acceleration > 1e-6 && state.velocity > states[1].velocity) {
                laterAcceleration = state.acceleration;
            }
        }
        assertTrue(laterAcceleration < firstAcceleration);
        assertEquals(0.0, states[0].velocity, 0.0);
        assertEquals(0.0, states[states.length - 1].velocity, 0.0);
    }

    @Test public void velocityIsCappedByBackEmfAndStaticPower() {
        DynamicMotionProfile.PowerModel model = new DynamicMotionProfile.PowerModel(
                100.0, 100.0, .20, .05, .10);
        DynamicMotionProfile profile = DynamicMotionProfile.forPowerLimitedDistance(
                100.0, 201, model);

        assertEquals(4.5, model.getVelocityLimit(), 1e-9);
        assertTrue(profile.getPeakVelocity() <= 4.5 + 1e-9);
    }

    @Test public void sampledEnvelopeHonorsLocalCeilingsAndBoundaries() {
        DynamicMotionProfile profile = DynamicMotionProfile.fromVelocityEnvelope(
                new double[]{0, 1, 2, 3, 4}, new double[]{5, 5, 1, 5, 5}, 0, 0,
                constantAcceleration(2), constantAcceleration(2));
        DynamicMotionProfile.State[] states = profile.getStates();

        assertEquals(0.0, states[0].velocity, 0.0);
        assertTrue(states[1].velocity <= 2.0 + 1e-9);
        assertEquals(1.0, states[2].velocity, 1e-9);
        assertEquals(0.0, states[4].velocity, 0.0);
        assertTrue(states[3].timeSeconds > states[2].timeSeconds);
    }

    @Test(expected = IllegalArgumentException.class)
    public void rejectsStaticPowerThatConsumesTheBudget() {
        new DynamicMotionProfile.PowerModel(10, 10, .1, .1, 1.0);
    }

    private static DynamicMotionProfile.Reachability constantAcceleration(
            final double acceleration) {
        return new DynamicMotionProfile.Reachability() {
            @Override public double maxReachableVelocity(
                    double velocity, double ceiling, double distance,
                    double boundaryPosition, int intervalIndex) {
                return Math.min(ceiling,
                        Math.sqrt(velocity * velocity + 2.0 * acceleration * distance));
            }
        };
    }
}
