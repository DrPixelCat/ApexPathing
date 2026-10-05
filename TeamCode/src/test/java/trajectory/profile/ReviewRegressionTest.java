package trajectory.profile;

import org.junit.Test;
import static org.junit.Assert.*;

public class ReviewRegressionTest {
    @Test public void minimumSampleCountProducesRealRestToRestMotion() {
        DynamicMotionProfile profile = DynamicMotionProfile.forPowerLimitedDistance(24, 2,
                new DynamicMotionProfile.PowerModel(20, 10, .01, .01, 0));
        assertTrue(profile.getDurationSeconds() > 0);
        assertTrue(Double.isFinite(profile.getDurationSeconds()));
    }

    @Test public void measuredBrakingRespectsPowerBudget() {
        DynamicMotionProfile.PowerModel model = new DynamicMotionProfile.PowerModel(
                20, 10, .01, .01, 0, 1, (velocity, acceleration) -> .2 * Math.abs(acceleration));
        double velocity = model.backwardReachability().maxReachableVelocity(0, 20, 2, 2, 0);
        assertTrue(.2 * velocity * velocity / 4 <= 1 + 1e-10);
        assertTrue(velocity > 0);
    }
}
