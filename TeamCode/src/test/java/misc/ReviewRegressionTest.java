package geometry;

import org.junit.Test;
import java.util.HashSet;
import static org.junit.Assert.*;

import paths.geometry.Angle;
import paths.geometry.ArcPose;
import paths.geometry.BSpline;
import paths.geometry.Dist;
import paths.geometry.DistUnit;
import paths.geometry.GeometryFactory;
import paths.geometry.Matrix;
import paths.geometry.Pose;
import paths.geometry.Vector;

public class ReviewRegressionTest {
    @Test public void mirroredArcsRetainTheirTypeAndRadius() {
        ArcPose arc = new GeometryFactory().setPoseMirror(GeometryFactory.PoseMirror.X).arcPose(2, 3, 4);
        assertEquals(-3, arc.getY(DistUnit.IN), 0);
        assertEquals(4, arc.getRadius().getIn(), 0);
        assertEquals(arc, arc.copy());
        assertNotEquals(arc, new ArcPose(arc.getVec(), Dist.fromIn(5)));
    }

    @Test public void valueEqualityWorksInHashCollections() {
        HashSet<Pose> poses = new HashSet<>();
        Pose pose = new Pose(Vector.of(1, 2, DistUnit.IN), Angle.fromRad(.5));
        poses.add(pose);
        assertTrue(poses.contains(pose.copy()));
        assertNotEquals(pose, new ArcPose(pose.getVec(), Dist.fromIn(2)));
    }

    @Test public void splineIncludesExactFinalWaypoint() {
        Vector end = Vector.of(9, -3, DistUnit.IN);
        BSpline spline = new BSpline(new Vector[]{Vector.zero(), Vector.of(4, 2, DistUnit.IN), end});
        assertEquals(end.getX().getIn(), spline.getPosition(1).getX().getIn(), 1e-12);
        assertEquals(end.getY().getIn(), spline.getPosition(1).getY().getIn(), 1e-12);
    }

    @Test(expected = IllegalArgumentException.class)
    public void raggedMatricesFailAtConstruction() { Matrix.of(new double[][]{{1, 2}, {3}}); }
}
