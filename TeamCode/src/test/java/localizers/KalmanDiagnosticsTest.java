package localizers;

import java.util.concurrent.atomic.AtomicLong;
import localizers.util.AdaptiveKalmanFilter;
import localizers.util.FilterState;
import org.junit.Test;
import static org.junit.Assert.*;

public class KalmanDiagnosticsTest {
    @Test public void diagnosticsDoNotAdvanceOrChangeTheEstimator() {
        AtomicLong time = new AtomicLong();
        AdaptiveKalmanFilter observed = new AdaptiveKalmanFilter(time::get);
        AdaptiveKalmanFilter control = new AdaptiveKalmanFilter(time::get);
        observed.setAutoTuning(true);
        control.setAutoTuning(true);
        for (int i = 0; i < 150; i++) {
            time.addAndGet(20_000_000L);
            double measurement = 3 * Math.sin(i * .04) + .05 * Math.cos(i * 1.7);
            FilterState actual = observed.update(measurement), expected = control.update(measurement);
            AdaptiveKalmanFilter.Diagnostics a = observed.getDiagnostics(), b = observed.getDiagnostics();
            assertEquals(i + 1, a.samples);
            assertEquals(a.samples, b.samples);
            assertEquals(expected.value(), actual.value(), 0);
            assertEquals(expected.rate(), actual.rate(), 0);
            assertTrue(a.measurementVariance > 0);
            assertTrue(a.processVariance > 0);
            if (i > 0) {
                assertTrue(Double.isFinite(a.innovation));
                assertTrue(Double.isFinite(a.nis));
            }
        }
        observed.setAutoTuning(false);
        assertFalse(observed.getDiagnostics().adapting);
        observed.reset();
        assertEquals(0, observed.getDiagnostics().samples);
        assertFalse(observed.getDiagnostics().initialized);
    }

    @Test public void logsTimingAnomalyAndKeepsTheEffectiveFilterStep() {
        AtomicLong time = new AtomicLong();
        AdaptiveKalmanFilter filter = new AdaptiveKalmanFilter(time::get);
        filter.update(1);
        time.addAndGet(1_000_000_000);
        filter.update(2);
        assertTrue(filter.getDiagnostics().timeAnomaly);
        assertEquals(.02, filter.getDiagnostics().dt, 1e-12);
    }
}
