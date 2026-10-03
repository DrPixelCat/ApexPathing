package feedforward.generators;

import core.FollowerConstants;
import core.MotionCharacterization;
import core.MotorPowerModel;
import geometry.*;
import org.json.JSONObject;
import org.junit.Test;
import paths.builders.HolonomicPathBuilder;
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
            for (BaseProfileGenerator generator : new BaseProfileGenerator[]{new MecanumProfileGenerator(c, path), new TankProfileGenerator(c, path)}) {
                for (int i = 0; i < 250; i++) {
                    PathPoint point = path.getParametricPath().getPointAtDistance(random.nextDouble() * path.getParametricPath().getLengthIn());
                    BaseProfileGenerator.PowerEvaluator evaluator = generator.powerEvaluator(path, point);
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
    }
}
