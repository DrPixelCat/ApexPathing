package tuning;

import geometry.Angle;
import geometry.DistUnit;
import geometry.Pose;
import geometry.Vector;
import java.util.ArrayList;
import java.util.List;
import org.junit.Test;
import static org.junit.Assert.*;

public class FeedforwardRegressionTest {
    @Test public void recoversCoefficientsAcrossFourIndependentExcitationRuns() {
        List<FeedforwardTuner.Observation> samples = new ArrayList<>();
        for (int run = 0; run < 4; run++) {
            for (int i = 1; i <= 60; i++) {
                double v = .3 + i * .4, a = (run < 2 ? .5 : 5) * Math.cos(i * .15);
                samples.add(new FeedforwardTuner.Observation(.08 + .02 * v + .003 * a, v, a, run, i * .02));
            }
        }
        FeedforwardTuner.FitResult fit = FeedforwardTuner.fitFeedforward(samples, .08);
        assertEquals(.02, fit.kV, 1e-8);
        assertEquals(.003, fit.kA, 1e-8);
        assertEquals(1, fit.rSquared, 1e-8);
        assertTrue(fit.crossValidationRmse < 1e-7);
        assertTrue(fit.isValid());
    }

    @Test public void forwardProjectionWorksAfterTurnsAndInReverse() {
        Pose heading = new Pose(Vector.zero(), Angle.fromDeg(90));
        Pose reverseFieldVelocity = new Pose(Vector.of(0, -12, DistUnit.IN), Angle.zero());
        assertEquals(-12, TuningMeasurements.forward(reverseFieldVelocity, heading), 1e-9);
    }
}
