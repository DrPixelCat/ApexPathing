package follower.internal;

import follower.FollowerConstants;

import org.junit.Test;

import paths.geometry.AngleUnit;
import paths.geometry.DistUnit;
import paths.geometry.GeometryFactory;
import paths.geometry.Vector;
import paths.heading.InterpolationStyle;
import paths.movements.Path;
import trajectory.TrajectorySample;
import trajectory.profile.*;
import static org.junit.Assert.*;

public class IndependentPathReferenceTest {
    @Test public void referenceIgnoresMotionErrorsPreservesPauseAndFinishesAtRest() throws Exception {
        GeometryFactory geometry=new GeometryFactory().setDistUnit(DistUnit.IN).setAngleUnit(AngleUnit.DEG);
        Path path=geometry.holonomicPath(geometry.pose(0,0,0),geometry.pose(4,0,0))
                .interpolateWith(InterpolationStyle.CONSTANT_START_HEADING).quickBuild();
        path.setPlanningConstants(FollowerConstants.fromJson(new org.json.JSONObject()
                .put("drivetrainType", "MECANUM")));
        path.setBuildMode(Path.BuildMode.MEMORY);
        path.setFeedforwardLut(new FFLut(new MotionParameters[]{
                new MotionParameters(0,4,0,0,0).setTimeSeconds(0),
                new MotionParameters(4,-4,0,0,2).setTimeSeconds(1),
                new MotionParameters(0,0,0,0,4).setTimeSeconds(2)}));
        FollowerPathState state=new FollowerPathState();state.prepare(path, path.getPlanningConstants());
        state.target(path, Vector.zero(),Vector.zero(),0,1_000_000_000L,true);
        // Exercise the time reference with valid loop intervals; DeltaTime rejects gaps over .1s.
        TrajectorySample moved = null;
        for (int tick = 1; tick <= 5; tick++) moved = state.target(path,
                TrajectorySample.vector(999,-999),TrajectorySample.vector(-50,80),-12,
                1_000_000_000L + tick * 50_000_000L,true);
        assertEquals(.125,moved.progressIn,1e-8);assertEquals(1,moved.tangentialVelocity(),1e-8);
        assertEquals(0,moved.angularVelocity,0);assertFalse(state.independentReferenceFinished());
        state.pause();
        TrajectorySample resumed=state.target(path,Vector.zero(),Vector.zero(),0,10_000_000_000L,true);
        assertEquals(moved.progressIn,resumed.progressIn,1e-8);
        TrajectorySample stopped = resumed;
        for (int tick = 1; tick <= 20; tick++) stopped = state.target(path,
                Vector.zero(),Vector.zero(),0,10_000_000_000L + tick * 100_000_000L,true);
        assertTrue(state.independentReferenceFinished());assertEquals(4,stopped.progressIn,1e-5);
        assertEquals(0,stopped.tangentialVelocity(),0);assertEquals(0,stopped.tangentialAcceleration(),0);
        TrajectorySample future=state.previewReference(path,Vector.zero(),.01);
        assertEquals("Preview past the stop must not predict reversal",0,future.tangentialVelocity(),0);
        assertEquals(0,future.tangentialAcceleration(),0);
        state.reset();assertFalse(state.independentReferenceFinished());
    }
}
