package trajectory.generators;


import follower.FollowerConstants;
import feedforward.MotionCharacterization;
import feedforward.MotorPowerModel;

import org.json.JSONObject;
import org.junit.Test;
import paths.builders.HolonomicPathBuilder;
import paths.geometry.Angle;
import paths.geometry.GeometryFactory;
import paths.geometry.PathPoint;
import paths.geometry.Vector;
import paths.heading.InterpolationStyle;
import paths.movements.Path;
import static org.junit.Assert.*;

public class AnalyticPowerBoundsTest {
    @Test public void affineBoundsAgreeWithSignedPowerAcrossLegacyAndAsymmetricCalibratedModels() throws Exception {
        FollowerConstants c = FollowerConstants.fromJson(new JSONObject().put("drivetrainType", "MECANUM"));
        c.translationalKV = .01; c.translationalKA = .004; c.translationalFeedforwardKS = .03;
        c.angularKV = .1; c.angularKA = .02; c.angularFeedforwardKS = .02; c.kCentripetal = .001;
        c.forwardVelLimitIn = c.strafeVelLimitIn = 60; c.forwardAccelLimitIn = c.strafeAccelLimitIn = 100;
        c.angularVelLimitRad = 8; c.angularAccelLimitRad = 20;
        GeometryFactory g = new GeometryFactory();
        Path path = new HolonomicPathBuilder(g.pose(0,0,0), g.arcPose(30,0,7), g.pose(30,40,0))
                .interpolateWith(InterpolationStyle.CONSTANT_START_HEADING).quickBuild();
        int eligible = 0;
        java.util.Random random = new java.util.Random(721);
        for (boolean calibrated : new boolean[]{false, true}) {
            if (calibrated) {
                MotorPowerModel[][] models = new MotorPowerModel[3][2];
                for (MotionCharacterization.Axis axis : MotionCharacterization.Axis.values()) {
                    for (int direction = 0; direction < 2; direction++) {
                        models[axis.ordinal()][direction] = new MotorPowerModel(200 + direction * 30, -1, 20, .1,
                                axis == MotionCharacterization.Axis.ANGULAR ? 10 : 60, 0);
                    }
                }
                c.motionCharacterization = new MotionCharacterization(models, true);
            }
            for (drivetrains.models.DriveModel model : new drivetrains.models.DriveModel[]{
                    drivetrains.models.DriveModels.mecanum(c), drivetrains.models.DriveModels.tank(c),
                    drivetrains.models.DriveModels.swerve(c), drivetrains.models.DriveModels.omni(c),
                    projected(c,3,.3), projected(c,5,1.2), projected(c,8,.7)}) {
                for (int i = 0; i < 250; i++) {
                    PathPoint point = path.getParametricPath().getPointAtDistance(random.nextDouble() * path.getParametricPath().getLengthIn());
                    Angle heading=path.getInterpolator().getHeadingTarg(point.getDistanceToEndIn(),point.getFirstDerivative(),
                            path.getParametricPath().getFirstDerivative(1));
                    Vector body=point.getFirstDerivative().normalize().rotate(heading.times(-1));
                    drivetrains.models.DriveModel.PowerEvaluator evaluator = model.powerEvaluator(body,point.getSignedCurvature());
                    double v = random.nextDouble() * 60, prime = random.nextDouble() * .2 - .1, second = random.nextDouble() * .01 - .005;
                    double min = i % 2 == 0 ? .01 : -100, max = i % 2 == 0 ? 100 : -.01, limit = .4 + random.nextDouble() * .6;
                    double candidate = evaluator.maximumAcceleration(v, prime, second, min, max, limit);
                    if (candidate == Double.NEGATIVE_INFINITY) { continue; }
                    eligible++;
                    double gridBest = Double.NaN;
                    for (int j = 0; j <= 1000; j++) {
                        double a = min + (max - min) * j / 1000;
                        if (evaluator.utilization(v, a, prime * v, second * v * v + prime * a, v) <= limit) { gridBest = a; }
                    }
                    if (Double.isNaN(candidate)) { assertTrue(Double.isNaN(gridBest)); }
                    else {
                        assertTrue(candidate >= min - 1e-9 && candidate <= max + 1e-9);
                        assertTrue(evaluator.utilization(v, candidate, prime * v, second * v * v + prime * candidate, v) <= limit + 1e-10);
                        if (Double.isFinite(gridBest)) { assertTrue(Math.abs(candidate - gridBest) <= (max - min) / 1000 + 1e-8); }
                    }
                }
            }
        }
        assertTrue("Exercise both powered and braking affine branches", eligible > 300);
    }    private static drivetrains.models.DriveModel projected(FollowerConstants c,int n,double offset) {
        double[] x=new double[n],y=new double[n];
        for(int i=0;i<n;i++) { x[i]=-Math.sin(offset+i*2*Math.PI/n); y[i]=Math.cos(offset+i*2*Math.PI/n); }
        return new drivetrains.models.HolonomicDriveModel(c,new drivetrains.models.WheelProjectionModel(x,y,1));
    }

}
