package follower;

import org.junit.Test;
import geometry.Angle;
import static org.junit.Assert.*;

public class CompletionRegressionTest {
    @Test public void callbacksFireOnceForEachExecutionOfTheSamePath() {
        java.util.concurrent.atomic.AtomicInteger calls = new java.util.concurrent.atomic.AtomicInteger();
        geometry.GeometryFactory geometry = new geometry.GeometryFactory();
        paths.movements.Path path = geometry.holonomicPath(geometry.pose(0, 0, 0), geometry.pose(4, 0, 0))
                .addDistanceCallback(.5, calls::incrementAndGet).quickBuild();
        MovementCallbackRunner runner = new MovementCallbackRunner();
        runner.process(path, .75, Angle.zero());
        runner.process(path, .9, Angle.zero());
        assertEquals(1, calls.get());
        runner.reset();
        runner.process(path, .75, Angle.zero());
        assertEquals(2, calls.get());
    }

    @Test public void leavingTheSettledRegionRestartsCompletionDwell() {
        PathCompletionPolicy policy = new PathCompletionPolicy();
        assertFalse(policy.isComplete(true, true, 1_000_000_000L));
        assertFalse(policy.isComplete(true, false, 1_110_000_000L));
        assertFalse(policy.isComplete(true, true, 1_200_000_000L));
        assertFalse(policy.isComplete(false, true, 1_310_000_000L));
        assertFalse(policy.isComplete(true, true, 1_400_000_000L));
        assertTrue(policy.isComplete(true, true, 1_510_000_000L));
    }
}
