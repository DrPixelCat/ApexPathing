package trajectory.generators;

import drivetrains.BaseDrivetrain;
import follower.FollowerConstants;
import geometry.*;
import org.json.JSONObject;
import org.junit.Test;
import paths.builders.PathBuilder;
import paths.constraint.PathConstraint;
import paths.constraint.TranslationalConstraint;
import paths.heading.InterpolationStyle;
import paths.movements.Path;
import trajectory.OnlineTrajectoryPlanner;
import trajectory.TrajectorySample;
import trajectory.profile.CoupledMotionProfile;
import static org.junit.Assert.*;

public class OnlineWindowRobustnessTest {
    private final GeometryFactory g = new GeometryFactory().setDistUnit(DistUnit.IN).setAngleUnit(AngleUnit.DEG);

    @Test public void fullGeometryWindowsSolveOnceAcrossShapesAndDrivetrainModes() throws Exception {
        for (BaseDrivetrain.DrivetrainType type : BaseDrivetrain.DrivetrainType.values()) {
            for (boolean tank : new boolean[]{false, true}) {
                if (type != BaseDrivetrain.DrivetrainType.DUAL_ACTUATED
                        && tank != (type == BaseDrivetrain.DrivetrainType.TANK)) { continue; }
                FollowerConstants c = FollowerConstants.fromJson(new JSONObject().put("drivetrainType", type.name()));
                c.configure(type, type == BaseDrivetrain.DrivetrainType.DUAL_ACTUATED
                        ? tank ? FollowerConstants.Profile.TANK : FollowerConstants.Profile.HOLONOMIC
                        : FollowerConstants.Profile.DEFAULT, true);
                c.forwardVelLimitIn = c.strafeVelLimitIn = 60;
                c.forwardAccelLimitIn = c.strafeAccelLimitIn = 100;
                c.angularVelLimitRad = 8; c.angularAccelLimitRad = 20;
                c.translationalKV = .01; c.translationalKA = .005; c.translationalFeedforwardKS = .03;
                c.angularKV = .1; c.angularKA = .02; c.angularFeedforwardKS = .02;
                for (int shape = 0; shape < 3; shape++) {
                    Pose[] poses = shape == 0 ? new Pose[]{g.pose(0, 0), g.pose(100, 0)}
                            : new Pose[]{g.pose(0, 0), g.arcPose(100, 0, 15), g.pose(100, 100, 90)};
                    PathBuilder<?> builder = tank ? g.tankPath(poses) : g.holonomicPath(poses);
                    builder.setPlanningConstants(c).setLookaheadDistance(Dist.fromIn(20));
                    builder.interpolateWith(InterpolationStyle.TANGENT_FORWARD);
                    if (shape == 2) {
                        builder.addConstraint(new TranslationalConstraint(.45, PathConstraint.Type.VELOCITY, Dist.fromIn(12)));
                    }
                    Path path = builder.quickBuild();
                    OnlineTrajectoryPlanner planner = new OnlineTrajectoryPlanner(path, c, 0);
                    for (int i = 0; i < 20; i++) {
                        double distance = path.getParametricPath().getLengthIn() * (.1 + i * .035);
                        PathPoint point = path.getParametricPath().getPointAtDistance(distance);
                        long before = CoupledMotionProfile.getSolveCount();
                        TrajectorySample sample = planner.sample(point.getLocation(), point.getFirstDerivative().normalize().times(8), 0, .02);
                        assertEquals(type + " shape " + shape + " must never perform a coarse retry", before + 1, CoupledMotionProfile.getSolveCount());
                        assertTrue(Double.isFinite(sample.tangentialAcceleration()));
                        assertEquals(8, sample.tangentialVelocity(), 1e-6);
                    }
                }
            }
        }
    }
}
