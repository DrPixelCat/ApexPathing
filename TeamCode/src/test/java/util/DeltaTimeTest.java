package util;

import org.junit.Test;
import static org.junit.Assert.*;

public class DeltaTimeTest {
    @Test public void tracksTimestampsIncludingZeroAndNegativeOrigins() {
        DeltaTime timing = new DeltaTime();
        assertEquals(1e-6, timing.update(-20_000_000L), 0);
        assertEquals(.02, timing.update(0L), 1e-12);
        assertEquals(.03, timing.update(30_000_000L), 1e-12);
    }

    @Test public void resetsAndUsesConfiguredFirstInterval() {
        DeltaTime timing = new DeltaTime(.02);
        assertEquals(.02, timing.update(100L), 0);
        timing.reset();
        assertFalse(timing.isInitialized());
        assertEquals(.02, timing.update(1_000_000_000L), 0);
        timing.reset(0L);
        assertEquals(.05, timing.update(50_000_000L), 1e-12);
    }

    @Test public void previewDoesNotAdvanceAndClampsIntervals() {
        DeltaTime timing = new DeltaTime();
        timing.reset(20_000_000L);
        assertEquals(.001, timing.elapsed(21_000_000L), 1e-12);
        assertEquals(.02, timing.update(40_000_000L), 1e-12);
        assertEquals(.02, timing.update(40_000_000L), 0);
        assertEquals(.02, timing.update(30_000_000L), 0);
    }
    @Test public void clampsSmallAndInvalidInitialIntervals() {
        for (double initial : new double[] {0, -1, 1e-9, Double.NaN, Double.NEGATIVE_INFINITY}) {
            DeltaTime timing = new DeltaTime(initial);
            assertEquals(1e-6, timing.elapsed(0), 0);
            assertEquals(1e-6, timing.update(0), 0);
            assertEquals(1e-6, timing.elapsed(999), 0);
            timing.reset();
            assertEquals(1e-6, timing.update(0), 0);
        }
        assertEquals(1e-6, DeltaTime.seconds(-1), 0);
        assertEquals(1e-6, DeltaTime.seconds(0), 0);
        assertEquals(1e-6, DeltaTime.seconds(999), 0);
        assertEquals(1e-6, DeltaTime.seconds(1000), 0);
        assertEquals(2e-6, DeltaTime.seconds(2000), 0);
    }

    @Test public void timestampValidationRemainsIndependentOfClamp() {
        DeltaTime timing = new DeltaTime();
        assertTrue(timing.isAfterPrevious(0));
        timing.reset(1000);
        assertFalse(timing.isAfterPrevious(999));
        assertFalse(timing.isAfterPrevious(1000));
        assertTrue(timing.isAfterPrevious(1001));
    }
    @Test public void anomaliesUseLastValidIntervalAndRecover() {
        DeltaTime timing = new DeltaTime(.02);
        timing.update(0);
        assertEquals(.03, timing.update(30_000_000L), 1e-12);
        assertFalse(timing.isTimeAnomalyDetected());
        assertEquals(.03, timing.update(230_000_000L), 1e-12);
        assertTrue(timing.isTimeAnomalyDetected());
        assertEquals(.03, timing.update(230_000_000L), 1e-12);
        assertTrue(timing.isTimeAnomalyDetected());
        assertEquals(.03, timing.update(220_000_000L), 1e-12);
        assertTrue(timing.isTimeAnomalyDetected());
        assertEquals(.04, timing.update(270_000_000L), 1e-12);
        assertFalse(timing.isTimeAnomalyDetected());
        timing.reset();
        assertFalse(timing.isTimeAnomalyDetected());
        assertEquals(.02, timing.update(0), 0);
    }

    @Test public void acceptsBoundsAndDetectsShortIntervalsBeforeClamping() {
        DeltaTime timing = new DeltaTime(.02);
        timing.reset(0);
        assertFalse(timing.isAnomaly(1000));
        assertFalse(timing.isAnomaly(100_000_000L));
        assertTrue(timing.isAnomaly(999));
        assertTrue(timing.isAnomaly(100_000_001L));
        assertEquals(.02, timing.elapsed(999), 0);
        assertFalse(timing.isTimeAnomalyDetected());
        assertEquals(.1, new DeltaTime(1).update(0), 0);
    }
}
