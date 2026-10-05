package tuning.follower.feedback;

import org.junit.Test;
import java.util.concurrent.atomic.AtomicLong;
import static org.junit.Assert.*;

public class ManualResponseMetricsTest {
    @Test public void settlementMustPersistThroughTheEndOfTheTrial() {
        AtomicLong clock = new AtomicLong();
        ManualResponseMetrics metrics = new ManualResponseMetrics(clock::get);
        try {
            metrics.begin("review_regression", 0, 1, .05, .1, .3);
            metrics.sample(1, 0, .3);
            clock.set(300_000_000);
            metrics.sample(1, 0, .3);
            assertEquals(.3, metrics.getSettlingTime(), 1e-12);
            clock.set(400_000_000);
            metrics.sample(.5, 0, .1);
            assertTrue(Double.isNaN(metrics.getSettlingTime()));
            assertEquals(2.0 / 3, metrics.getSaturationFraction(), 1e-12);
        } finally { metrics.finish(); }
    }
}
