package tuning;

import org.junit.Test;

import java.util.concurrent.atomic.AtomicReference;

import static org.junit.Assert.assertEquals;

public class TuneValueTest {
    @Test
    public void appliesFineAndCoarseStepsWithinBounds() {
        AtomicReference<Double> value = new AtomicReference<>(0.5);
        TuneValue parameter = new TuneValue(
                "test", value::get, value::set, 0.1, 0.0, 1.0);

        parameter.adjust(1, 0.1);
        assertEquals(0.51, parameter.get(), 1e-9);
        parameter.adjust(1, 10.0);
        assertEquals(1.0, parameter.get(), 1e-9);
        parameter.adjust(-1, 10.0);
        assertEquals(0.0, parameter.get(), 1e-9);
    }
}
