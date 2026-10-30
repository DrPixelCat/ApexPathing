package trajectory;

import follower.FollowerConstants;
import drivetrains.BaseDrivetrain;

import org.json.JSONObject;
import org.junit.Test;
import paths.builders.HolonomicPathBuilder;
import paths.constraint.PathConstraint;
import paths.constraint.TranslationalConstraint;
import paths.geometry.AngleUnit;
import paths.geometry.Dist;
import paths.geometry.DistUnit;
import paths.geometry.GeometryFactory;
import paths.geometry.PathPoint;
import paths.geometry.Vector;
import paths.heading.InterpolationStyle;
import paths.movements.Path;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.Arrays;
import static org.junit.Assert.*;

/** Desktop comparison harness; latency thresholds are deliberately excluded from correctness tests. */
public class OnlinePlanningBenchmarkTest {
    @Test public void benchmarkOnlinePlanningAcrossPathAndScheduleSizes() throws Exception {
        java.lang.reflect.Field singleton = FollowerConstants.class.getDeclaredField("instance");
        singleton.setAccessible(true); Object previous = singleton.get(null);
        FollowerConstants c = FollowerConstants.fromJson(new JSONObject().put("drivetrainType", "MECANUM"));
        c.configure(BaseDrivetrain.DrivetrainType.MECANUM, FollowerConstants.Profile.DEFAULT, true);
        c.forwardVelLimitIn = c.strafeVelLimitIn = 60; c.forwardAccelLimitIn = c.strafeAccelLimitIn = 100;
        c.angularVelLimitRad = 8; c.angularAccelLimitRad = 20;
        c.translationalKV = .01; c.translationalKA = .005; c.translationalFeedforwardKS = .03;
        c.angularKV = .1; c.angularKA = .02; c.angularFeedforwardKS = .02;
        singleton.set(null, c);
        try {
            GeometryFactory g = new GeometryFactory().setDistUnit(DistUnit.IN).setAngleUnit(AngleUnit.DEG);
            StringBuilder csv = new StringBuilder("case,lut_points,window_points,p50_us,p95_us,allocated_bytes_per_query\n");
            for (String name : new String[]{"line_100", "line_1000", "facing_1000", "constraints_256", "curved", "calibrated_1000", "overspeed"}) {
                c.motionCharacterization = null;
                if (name.equals("calibrated_1000")) {
                    feedforward.MotorPowerModel[][] models = new feedforward.MotorPowerModel[3][2];
                    for (feedforward.MotionCharacterization.Axis axis : feedforward.MotionCharacterization.Axis.values()) {
                        for (int direction = 0; direction < 2; direction++) {
                            models[axis.ordinal()][direction] = new feedforward.MotorPowerModel(200 + direction * 30, -1, 20, .1,
                                    axis == feedforward.MotionCharacterization.Axis.ANGULAR ? 10 : 60, 0);
                        }
                    }
                    c.motionCharacterization = new feedforward.MotionCharacterization(models, true);
                }
                boolean shortLine = name.equals("line_100");
                double length = shortLine ? 100 : 1000;
                HolonomicPathBuilder builder = name.equals("curved")
                        ? g.holonomicPath(g.pose(0,0,0), g.arcPose(100,0,15), g.pose(100,100,90))
                            .interpolateWith(InterpolationStyle.TANGENT_FORWARD)
                        : g.holonomicPath(g.pose(0,0,0), g.pose(length,0,0))
                            .interpolateWith(InterpolationStyle.CONSTANT_START_HEADING);
                if (name.equals("facing_1000")) { builder.interpolateWith(InterpolationStyle.FACING_POINT, g.vector(500,100)); }
                if (name.equals("constraints_256")) {
                    for (int i = 0; i < 256; i++) {
                        builder.addConstraint(new TranslationalConstraint(i / 256.0, PathConstraint.Type.VELOCITY, Dist.fromIn(40 + i % 10)));
                    }
                }
                Path path = builder.setLookaheadDistance(Dist.fromIn(20)).quickBuild();
                OnlineTrajectoryPlanner planner = new OnlineTrajectoryPlanner(path, c);
                Vector[] positions = new Vector[10000], velocities = new Vector[10000];
                for (int i = 0; i < positions.length; i++) {
                    PathPoint point = path.getParametricPath().getPointAtDistance(path.getParametricPath().getLengthIn() * .4 + i * .001);
                    positions[i] = point.getLocation(); velocities[i] = point.getFirstDerivative().normalize().times(name.equals("overspeed") ? 80 : 8);
                }
                for (int i = 0; i < 3000; i++) { planner.sample(positions[i], velocities[i], .02); }
                // Load desktop-only management APIs reflectively because tests compile against Android's SDK.
                Object bean = Class.forName("java.lang.management.ManagementFactory").getMethod("getThreadMXBean").invoke(null);
                Class<?> beanType = Class.forName("com.sun.management.ThreadMXBean");
                java.lang.reflect.Method allocation = beanType.getMethod("getThreadAllocatedBytes", long.class);
                long thread = Thread.currentThread().getId();
                long before = (Boolean) beanType.getMethod("isThreadAllocatedMemorySupported").invoke(bean)
                        ? (Long) allocation.invoke(bean, thread) : -1;
                long[] times = new long[positions.length];
                for (int i = 0; i < times.length; i++) {
                    long start = System.nanoTime();
                    TrajectorySample sample = planner.sample(positions[i], velocities[i], .02);
                    times[i] = System.nanoTime() - start;
                    assertTrue(Double.isFinite(sample.tangentialVelocity()));
                }
                long allocated = before >= 0 ? (Long) allocation.invoke(bean, thread) - before : -1;
                Arrays.sort(times);
                csv.append(name).append(',').append(path.getGeneratedPoints().length).append(',').append(planner.getLastSampleCount())
                        .append(',').append(times[times.length / 2 - 1] / 1000.0).append(',').append(times[times.length * 95 / 100 - 1] / 1000.0)
                        .append(',').append(allocated < 0 ? -1 : allocated / (double) times.length).append('\n');
            }
            java.nio.file.Path output = Paths.get(System.getProperty("apex.benchmark.directory", "build"), "quickbuild-benchmark.csv");
            Files.createDirectories(output.getParent()); Files.write(output, csv.toString().getBytes(StandardCharsets.UTF_8));
            System.out.print(csv);
        } finally { singleton.set(null, previous); }
    }
}
