package tuning;

import org.junit.Test;

import controllers.PDSController.PDSCoefficients;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class FeedforwardCalcTest {
    @Test
    public void recoversBidirectionalPlantWithOutliers() {
        FeedforwardCalc fit = new FeedforwardCalc();
        for (int i = 1; i <= 240; i++) {
            double velocity = ((i % 48) - 24) * 0.45;
            if (Math.abs(velocity) < 0.1) velocity = 0.3;
            double acceleration = Math.sin(i * 0.37) * 14.0;
            double power = (0.075 * Math.signum(velocity)) +
                    (0.0115 * velocity) + (0.0032 * acceleration);
            if (i % 53 == 0) power += 0.20;
            fit.add(power, velocity, acceleration, 0.05);
        }

        FeedforwardCalc.Result result = fit.solve();
        assertTrue(result.isUsable());
        assertEquals(0.075, result.kS, 0.004);
        assertEquals(0.0115, result.kV, 0.0008);
        assertEquals(0.0032, result.kA, 0.0005);
    }

    @Test
    public void derivesFiniteConservativeFeedbackGains() {
        FeedforwardCalc.Result result = new FeedforwardCalc.Result(
                0.08, 0.012, 0.0035, 0.95, 100);
        PDSCoefficients gains = result.positionGains(1.0);

        assertTrue(gains.kP > 0.0);
        assertTrue(gains.kD >= 0.0);
        assertEquals(0.08, gains.kS, 1e-9);
        assertTrue(Double.isFinite(result.velocityGain(0.15)));
    }
}
