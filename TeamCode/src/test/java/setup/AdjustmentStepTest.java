package setup;

import org.junit.Test;
import static org.junit.Assert.*;

public class AdjustmentStepTest {
    @Test public void precisionClampsAtBothBoundsAndResetRestoresInitialStep() {
        AdjustmentStep step = new AdjustmentStep(.25, .01, 10, 10);
        for (int i = 0; i < 10; i++) { step.finer(); }
        assertEquals(.01, step.get(), 0);
        assertEquals(-.01, step.delta(-1), 0);
        for (int i = 0; i < 10; i++) { step.coarser(); }
        assertEquals(10, step.get(), 0);
        step.reset();
        assertEquals(.25, step.delta(1), 0);
    }

    @Test public void rotationUsesItsOwnScaleAndTrackingCanChooseItsInitialPrecision() {
        AdjustmentStep rotation = new AdjustmentStep(15, 1, 90, 5);
        rotation.finer();
        assertEquals(3, rotation.get(), 0);
        rotation.finer();
        assertEquals(1, rotation.get(), 0);
        rotation.reset();
        rotation.coarser();
        assertEquals(75, rotation.get(), 0);
        rotation.coarser();
        assertEquals(90, rotation.get(), 0);
        AdjustmentStep gains = new AdjustmentStep(.001, .00001, 1, 10);
        gains.set(.01);
        assertEquals(.01, gains.get(), 0);
        gains.reset();
        assertEquals(.001, gains.get(), 0);
    }

    @Test public void invalidStepsCannotIntroduceNonfiniteOrUnboundedAdjustments() {
        assertThrows(IllegalArgumentException.class, () -> new AdjustmentStep(1, 0, 10, 10));
        assertThrows(IllegalArgumentException.class, () -> new AdjustmentStep(1, .1, 10, 1));
        assertThrows(IllegalArgumentException.class, () -> new AdjustmentStep(Double.NaN, .1, 10, 10));
        AdjustmentStep step = new AdjustmentStep(1, .1, 10, 10);
        assertThrows(IllegalArgumentException.class, () -> step.set(Double.NaN));
        assertThrows(IllegalArgumentException.class, () -> step.set(100));
        assertEquals(1, step.get(), 0);
    }
}
