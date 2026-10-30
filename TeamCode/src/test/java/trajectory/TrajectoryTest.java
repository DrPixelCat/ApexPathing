package trajectory;

import follower.FollowerConstants;
import drivetrains.BaseDrivetrain;
import paths.geometry.BSpline;
import paths.geometry.PathPoint;
import trajectory.profile.CoupledMotionProfile;
import trajectory.profile.MotionParameters;
import paths.geometry.AngleUnit;
import paths.geometry.Dist;
import paths.geometry.DistUnit;
import paths.geometry.GeometryFactory;
import paths.geometry.PathSegment;
import paths.geometry.Vector;
import org.json.JSONObject;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import paths.builders.HolonomicPathBuilder;
import paths.constraint.PathConstraint;
import paths.constraint.TranslationalConstraint;
import paths.heading.InterpolationStyle;
import paths.movements.Path;
import paths.movements.Turn;
import java.lang.reflect.Field;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.Arrays;
import static org.junit.Assert.*;

public class TrajectoryTest {
    private Field singleton;
    private Object previous;
    private FollowerConstants constants;
    private final GeometryFactory geometry = new GeometryFactory().setDistUnit(DistUnit.IN).setAngleUnit(AngleUnit.DEG);
    @Before public void setup() throws Exception {
        singleton = FollowerConstants.class.getDeclaredField("instance"); singleton.setAccessible(true);
        previous = singleton.get(null);
        constants = FollowerConstants.fromJson(new JSONObject().put("drivetrainType", "MECANUM"));
        constants.configure(BaseDrivetrain.DrivetrainType.MECANUM, FollowerConstants.Profile.DEFAULT, true);
        constants.forwardVelLimitIn = constants.strafeVelLimitIn = 60;
        constants.forwardAccelLimitIn = constants.strafeAccelLimitIn = 100;
        constants.angularVelLimitRad = 4; constants.angularAccelLimitRad = 8;
        constants.translationalKV = .01; constants.translationalKA = .005; constants.translationalFeedforwardKS = .03;
        constants.angularKV = .1; constants.angularKA = .02; constants.angularFeedforwardKS = .02;
        singleton.set(null, constants);
    }
    @After public void restore() throws Exception { singleton.set(null, previous); }
    private HolonomicPathBuilder line(double x, double y, double length) {
        return geometry.holonomicPath(geometry.pose(x, y, 0), geometry.pose(x + length, y, 0))
                .interpolateWith(InterpolationStyle.CONSTANT_START_HEADING);
    }
    private Vector vec(double x, double y) { return TrajectorySample.vector(x, y); }

    @Test public void quickBuildHasGeometryButNoStoredTrajectoryAndReversalPreservesSettings() {
        Path path = line(0, 0, 100).setLookaheadDistance(Dist.fromIn(14)).quickBuild();
        assertEquals(Path.BuildMode.ONLINE, path.getBuildMode()); assertNull(path.getFeedforwardLut()); assertNull(path.getTrajectoryField());
        Path reversed = path.reversed();
        assertEquals(14, reversed.getLookaheadDistanceIn(), 0); assertNull(reversed.getTrajectoryField());
        assertEquals(0, reversed.getEndPose().getVec().getX().getIn(), 1e-9);
    }
    @Test public void arcLengthLookupUsesPhysicalDistance() {
        Path path = geometry.holonomicPath(geometry.pose(0, 0), geometry.arcPose(30, 0, 7), geometry.pose(30, 40)).quickBuild();
        PathSegment segment = path.getParametricPath();
        for (int i = 0; i <= 20; i++) {
            double distance = segment.getLengthIn() * i / 20;
            double t = segment.getTAtDistance(distance);
            assertEquals(distance, segment.getLengthIn() - segment.getDistanceToEndIn(segment.getPosition(t), t), .04);
        }
    }
    @Test public void solverPreservesMeasuredInitialSpeedAndReportsUncontrollableSpeed() {
        double[] positions = {0, 1, 2}, ceilings = {10, 10, 10};
        CoupledMotionProfile.Transitions rule = (i, q, next) -> q - 2 > next + 1e-9 ? Double.NaN : Math.min(next, q + 2);
        assertEquals(1, CoupledMotionProfile.solve(positions, ceilings, 1.0, 0, rule)[0], 0);
        assertThrows(CoupledMotionProfile.OverspeedException.class, () -> CoupledMotionProfile.solve(positions, ceilings, 10.0, 0, rule));
    }
    @Test public void onlineStartsFromVelocityAndReplansAfterDisturbances() {
        Path path = line(0, 0, 100).setLookaheadDistance(Dist.fromIn(20)).quickBuild();
        OnlineTrajectoryPlanner planner = new OnlineTrajectoryPlanner(path, constants);
        TrajectorySample target = planner.sample(vec(5, 2), vec(20, 0), .02);
        assertTrue(target.tangentialVelocity() >= 20); assertTrue(target.tangentialVelocity() <= 25.001);
        assertEquals(-2, target.toPath.getY().getIn(), 1e-9);
        assertTrue(planner.sample(vec(6, 0), vec(0, 0), .02).tangentialVelocity() < 6);
        assertEquals(20, planner.getHorizonIn(), 0);
    }
    @Test public void overspeedBrakesWithoutReplacingMeasuredVelocityByTheEnvelope() {
        Path path = line(0, 0, 100).setLookaheadDistance(Dist.fromIn(2)).quickBuild();
        TrajectorySample target = new OnlineTrajectoryPlanner(path, constants).sample(vec(0, 0), vec(30, 0), .02);
        assertTrue(target.overspeed); assertTrue(target.tangentialAcceleration() < 0);
        assertTrue(target.tangentialVelocity() > 20); assertTrue(target.tangentialVelocity() < 60);
    }
    @Test public void currentTimeVelocityUsesThePublishedTangentOnCurves() {
        Path path = geometry.holonomicPath(geometry.pose(0, 0), geometry.arcPose(30, 0, 7),
                geometry.arcPose(30, -30, 7), geometry.pose(-30, -30)).quickBuild();
        OnlineTrajectoryPlanner planner = new OnlineTrajectoryPlanner(path, constants);
        for (double distance : new double[]{24.1, 27.7, 30.3, 52.8, 58.4}) {
            PathPoint point = path.getParametricPath().getPointAtDistance(distance);
            Vector tangent = point.getFirstDerivative().normalize();
            Vector normal = PathSegment.calculateLeftNormal(tangent);
            Vector measured = tangent.times(12).plus(normal.times(7));
            TrajectorySample target = planner.sample(point.getLocation().plus(normal.times(3)), measured, .02);
            assertEquals("Current-time target must use the same projection as feedback",
                    measured.dot(target.tangent).getIn(), target.tangentialVelocity(), 1e-9);
        }
    }
    @Test public void futureConstraintIsSeenBeforeCrossingIt() {
        Path path = line(0, 0, 100).setLookaheadDistance(Dist.fromIn(12))
                .addConstraint(new TranslationalConstraint(.3, PathConstraint.Type.VELOCITY, Dist.fromIn(5))).quickBuild();
        TrajectorySample target = new OnlineTrajectoryPlanner(path, constants).sample(vec(28, 0), vec(20, 0), .02);
        assertTrue(target.tangentialAcceleration() < 0);
        assertEquals("Current-time planning must not publish a future braking velocity",20,target.tangentialVelocity(),1e-8);
    }
    @Test public void horizonHasBrakingHeadroomAndRemainsFixed() {
        Path path = line(0, 0, 100).quickBuild();
        OnlineTrajectoryPlanner planner = new OnlineTrajectoryPlanner(path, constants);
        assertTrue(planner.getHorizonIn() >= 1.5 * 60 * 60 / 200 + .05 * 60 - 1e-6);
        double horizon = planner.getHorizonIn(); constants.forwardVelLimitIn = 10;
        planner.sample(vec(0, 0), vec(0, 0), .02); assertEquals(horizon, planner.getHorizonIn(), 0);
    }
    @Test public void rollingSolveIsBoundedByHorizonRatherThanCompletePathLength() {
        OnlineTrajectoryPlanner shortPath = new OnlineTrajectoryPlanner(line(0, 0, 100).setLookaheadDistance(Dist.fromIn(10)).quickBuild(), constants);
        OnlineTrajectoryPlanner longPath = new OnlineTrajectoryPlanner(line(0, 0, 1000).setLookaheadDistance(Dist.fromIn(10)).quickBuild(), constants);
        shortPath.sample(vec(0, 0), vec(0, 0), .02); longPath.sample(vec(0, 0), vec(0, 0), .02);
        assertTrue(longPath.getLastSampleCount() <= 12);
        assertTrue(Math.abs(longPath.getLastSampleCount() - shortPath.getLastSampleCount()) <= 2);
    }
    @Test public void onlineWindowRetainsFullGeometryAndExactConstraintTransitions() {
        class CountingSegment extends PathSegment {
            int reconstructedPoints;
            boolean reconstructedConstraint;
            CountingSegment() {
                super(new BSpline(new Vector[]{vec(0, 0), vec(100, 0)}));
            }
            @Override public PathPoint getPointAtDistance(double distance) {
                reconstructedPoints++;
                if (Math.abs(distance - .31 * getLengthIn()) < 1e-6) { reconstructedConstraint = true; }
                return super.getPointAtDistance(distance);
            }
        }
        Path path = line(0, 0, 100).setLookaheadDistance(Dist.fromIn(20))
                .addConstraint(new TranslationalConstraint(.31, PathConstraint.Type.VELOCITY, Dist.fromIn(5))).quickBuild();
        CountingSegment segment = new CountingSegment();
        path.setParametricPath(segment);
        OnlineTrajectoryPlanner planner = new OnlineTrajectoryPlanner(path, constants);
        for (double progress : new double[]{20, 20.1, 30}) {
            segment.reconstructedPoints = 0;
            segment.reconstructedConstraint = false;
            TrajectorySample target = planner.sample(vec(progress, 0), vec(20, 0), .02);
            int expected = 4; // Exact start, midpoint, end, and the authored transition at 31 inches.
            for (PathPoint point : segment.getPointLUT()) {
                double distance = segment.getLengthIn() - point.getDistanceToEndIn();
                if (distance > progress + 1e-6 && distance < progress + 20 - 1e-6
                        && Math.abs(distance - (progress + 10)) > 1e-9 && Math.abs(distance - 31) > 1e-9) { expected++; }
            }
            assertEquals("Every interior LUT node must be retained", expected, planner.getLastSampleCount());
            assertTrue("Only boundaries, midpoint and the first constraint need reconstruction", segment.reconstructedPoints <= 4);
            if (progress == 20) { assertTrue("The authored transition must be sampled at its exact distance", segment.reconstructedConstraint); }
            else { assertFalse("The authored transition geometry should be reused", segment.reconstructedConstraint); }
            if (progress == 30) { assertTrue(target.tangentialAcceleration() < 0); }
        }
    }
    @Test public void onlineCachesRefreshAfterConstraintHeadingAndPhysicalLimitChanges() {
        TranslationalConstraint limit = new TranslationalConstraint(.1, PathConstraint.Type.VELOCITY, Dist.fromIn(5));
        Path path = line(0, 0, 100).setLookaheadDistance(Dist.fromIn(20)).addConstraint(limit).quickBuild();
        OnlineTrajectoryPlanner planner = new OnlineTrajectoryPlanner(path, constants);
        assertTrue(planner.sample(vec(30, 0), vec(20, 0), .02).overspeed);
        limit.setS(.9);
        assertFalse(planner.sample(vec(30, 0), vec(20, 0), .02).overspeed);
        path.addConstraint(new TranslationalConstraint(.2, PathConstraint.Type.VELOCITY, Dist.fromIn(7)));
        assertTrue(planner.sample(vec(30, 0), vec(20, 0), .02).overspeed);
        Path curve = geometry.holonomicPath(geometry.pose(0, 0, 0), geometry.pose(100, 0, 90))
                .interpolateWith(InterpolationStyle.SMOOTH_START_TO_END).setLookaheadDistance(Dist.fromIn(20)).quickBuild();
        OnlineTrajectoryPlanner cached = new OnlineTrajectoryPlanner(curve, constants);
        cached.sample(vec(30, 0), vec(20, 0), .02);
        curve.getInterpolator().setPathLength(120);
        constants.angularVelLimitRad = .3; constants.angularAccelLimitRad = 1;
        TrajectorySample changed = cached.sample(vec(30, 0), vec(20, 0), .02);
        TrajectorySample fresh = new OnlineTrajectoryPlanner(curve, constants).sample(vec(30, 0), vec(20, 0), .02);
        assertEquals(fresh.angularVelocity, changed.angularVelocity, 1e-10);
        assertEquals(fresh.tangentialAcceleration(), changed.tangentialAcceleration(), 1e-10);
    }
    @Test public void customMutableConstraintsRemainLiveWithoutCacheInvalidationHooks() {
        double[] value = {5};
        TranslationalConstraint live = new TranslationalConstraint(.1, PathConstraint.Type.VELOCITY, Dist.fromIn(5)) {
            @Override public double getValueIn() { return value[0]; }
        };
        Path path = line(0, 0, 100).setLookaheadDistance(Dist.fromIn(20)).addConstraint(live).quickBuild();
        OnlineTrajectoryPlanner planner = new OnlineTrajectoryPlanner(path, constants);
        assertTrue(planner.sample(vec(30, 0), vec(20, 0), .02).overspeed);
        value[0] = 40;
        assertFalse(planner.sample(vec(30, 0), vec(20, 0), .02).overspeed);
    }
    @Test public void storedFieldSupportsTranslatedCoordinatesAndBilinearToPath() {
        Path path = line(-220, 320, 40).buildToMemory();
        TrajectorySample sample = path.getTrajectoryField().sample(vec(-201.25, 323.4));
        assertEquals(18.75, sample.progressIn, 1e-7); assertEquals(0, sample.toPath.getX().getIn(), 1e-7);
        assertEquals(-3.4, sample.toPath.getY().getIn(), 1e-7); assertFalse(sample.outsideGrid);
        assertEquals(Path.BuildMode.MEMORY, path.reversed().getBuildMode());
    }
    @Test public void gridEdgesCornersAndOutsidePositionsReanchorThePathPoint() {
        Path path = line(-10, -10, 40).setMemoryGridPadding(Dist.fromIn(2)).buildToMemory();
        TrajectoryField2D field = path.getTrajectoryField();
        double[][] points = {{field.getMinXIn(), field.getMinYIn()}, {field.getMaxXIn(), field.getMaxYIn()},
                {-100, 80}, {100, -80}, {0, 100}, {0, -100}};
        for (double[] p : points) {
            TrajectorySample target = field.sample(vec(p[0], p[1]));
            assertTrue(target.velocity.isFinite()); assertTrue(target.acceleration.isFinite());
            assertEquals(-10 - p[1], target.toPath.getY().getIn(), 1e-7);
            assertEquals(target.positionOnPath.getX().getIn(), p[0] + target.toPath.getX().getIn(), 1e-9);
            assertEquals(-10, target.positionOnPath.getY().getIn(), 1e-7);
        }
        assertTrue(field.sample(vec(-100, 80)).outsideGrid);
    }
    @Test public void storedLaunchAndTerminalBrakingSurviveInterpolation() {
        Path path = line(0, 0, 40).buildToMemory();
        TrajectoryField2D field = path.getTrajectoryField();
        assertTrue(field.sample(vec(0, 0)).tangentialAcceleration() > 0);
        TrajectorySample beforeEnd = field.sample(vec(39.8, 0));
        assertTrue(beforeEnd.tangentialAcceleration() < 0);
        assertEquals(beforeEnd.tangentialVelocity() * beforeEnd.tangentialVelocity(),
                -2 * beforeEnd.tangentialAcceleration() * beforeEnd.remainingIn, 1e-6);
        assertEquals(0, field.sample(vec(40, 0)).tangentialVelocity(), 0);
        assertTrue(line(0, 0, 2).buildToMemory().getTrajectoryField().sample(vec(0, 0)).tangentialAcceleration() > 0);
    }
    @Test public void storedTrajectoryUsesSnapshotAndRebuildsForPathChanges() {
        Path path = line(0, 0, 40).buildToMemory();
        TrajectoryField2D field = path.getTrajectoryField();
        TrajectorySample original = field.sample(vec(20, 0));
        long solves = CoupledMotionProfile.getSolveCount();
        constants.velocityFeedbackGain = .2; constants.angularCoeffs.kP = 3;
        constants.forwardVelLimitIn = 30;
        assertSame(field, path.getTrajectoryField());
        assertEquals(original.tangentialVelocity(), path.getTrajectoryField().sample(vec(20, 0)).tangentialVelocity(), 0);
        path.setProfilePowerLimit(.5);
        path.precomputeTrajectory();
        assertNotSame(field, path.getTrajectoryField());
        assertTrue(path.getTrajectoryField().sample(vec(20, 0)).tangentialVelocity() < original.tangentialVelocity());
        assertTrue(CoupledMotionProfile.getSolveCount() > solves);
    }
    @Test public void accelerationVectorIncludesSignedNormalAcceleration() {
        Path path = geometry.holonomicPath(geometry.pose(0, 0, 0), geometry.arcPose(30, 0, 7), geometry.pose(30, 40, 90))
                .interpolateWith(InterpolationStyle.TANGENT_FORWARD).buildToMemory();
        double s = path.getParametricPath().getLengthIn() / 2;
        Vector position = path.getParametricPath().getPosition(path.getParametricPath().getTAtDistance(s));
        TrajectorySample target = path.getTrajectoryField().sample(position);
        assertTrue(target.normalAcceleration().getMag().getIn() > 0);
        assertTrue(target.normalAcceleration().dot(PathSegment.calculateLeftNormal(target.tangent)).getIn() > 0);
    }
    @Test public void storedHeadingInterpolationPreservesWrap() {
        Path path = geometry.holonomicPath(geometry.pose(-100, -100, 170), geometry.pose(-60, -100, -170))
                .interpolateWith(InterpolationStyle.SMOOTH_START_TO_END).buildToMemory();
        assertTrue(Math.abs(path.getTrajectoryField().sample(vec(-80, -100)).heading.getRad()) > Math.toRadians(160));
    }

    @Test public void tangentHeadingStaysAlignedWithTheInterpolatedVelocityDirection() {
        Path path = geometry.holonomicPath(geometry.pose(0, 0, 0), geometry.arcPose(30, 0, 7), geometry.pose(30, 40, 90))
                .interpolateWith(InterpolationStyle.TANGENT_FORWARD).buildToMemory();
        for (double s = 20; s < 40; s += .2) {
            Vector point = path.getParametricPath().getPointAtDistance(s).getLocation();
            TrajectorySample target = path.getTrajectoryField().sample(point.plus(vec(.2, .3)));
            assertEquals(0, target.heading.getShortestAngleTo(target.tangent.getTheta()).getRad(), 1e-9);
        }
    }
    @Test public void quickTurnsHaveNoProfileAndStoredTurnsUseAngularDisplacement() {
        long solves = trajectory.profile.CoupledMotionProfile.getSolveCount();
        Turn turn = geometry.turn(geometry.pose(0, 0, 170)).turnTo(geometry.angle(-170)).quickBuild();
        assertNull(turn.getFeedforwardLut());
        assertNull(turn.reversed().getFeedforwardLut());
        assertEquals(solves, trajectory.profile.CoupledMotionProfile.getSolveCount());
        Turn stored = geometry.turn(geometry.pose(0, 0, 170)).turnTo(geometry.angle(-170)).buildToMemory();
        MotionParameters start = stored.getFeedforwardLut().getAngularKinematicParams(0);
        assertTrue(start.getAngularAccel() > 0);
        assertEquals(0, stored.getFeedforwardLut().getAngularKinematicParams(Math.toRadians(20)).getAngularVel(), 1e-9);
        assertEquals(Path.BuildMode.MEMORY, stored.reversed().getBuildMode());
        trajectory.profile.FFLut original = stored.getFeedforwardLut();
        constants.angularAccelLimitRad = 4; stored.precomputeTrajectory();
        assertSame(original, stored.getFeedforwardLut());
    }
    @Test public void rejectInvalidLookupAndBuilderSettings() {
        assertThrows(IllegalArgumentException.class, () -> line(0, 0, 40).setLookaheadDistance(Dist.fromIn(0)));
        assertThrows(IllegalArgumentException.class, () -> line(0, 0, 40).setMemoryGridSpacing(Dist.fromIn(-1)));
        assertThrows(IllegalArgumentException.class, () -> line(0, 0, 40).setMemoryGridPadding(Dist.fromIn(-1)));
        assertThrows(IllegalArgumentException.class, () -> line(0, 0, 40).quickBuild().getParametricPath().getTAtDistance(Double.NaN));
    }

    @Test public void pathAngularOverspeedRecoversWithoutClippingTheInitialRate() {
        OnlineTrajectoryPlanner planner = new OnlineTrajectoryPlanner(line(0, 0, 100).quickBuild(), constants);
        TrajectorySample target = planner.sample(vec(10, 0), vec(0, 0), 5, .02);
        assertTrue(target.overspeed);
        assertTrue(target.angularVelocity > constants.angularVelLimitRad);
        assertEquals("Recovery keeps the measured current rate and supplies braking acceleration",5,target.angularVelocity,1e-9);
        assertTrue(target.angularAcceleration < 0);
    }

    @Test public void storedInteriorMotionUsesBilinearNeighborWeights() {
        TrajectoryField2D field = line(0, 0, 100).buildToMemory().getTrajectoryField();
        double x = field.getMinXIn() + 32, y = field.getMinYIn() + 14;
        TrajectorySample a = field.sample(vec(x, y)), b = field.sample(vec(x + 1, y));
        TrajectorySample c = field.sample(vec(x, y + 1)), d = field.sample(vec(x + 1, y + 1));
        TrajectorySample interpolated = field.sample(vec(x + .3, y + .6));
        double expectedVelocity = .4 * (.7 * a.velocity.getX().getIn() + .3 * b.velocity.getX().getIn())
                + .6 * (.7 * c.velocity.getX().getIn() + .3 * d.velocity.getX().getIn());
        double expectedAcceleration = .4 * (.7 * a.acceleration.getX().getIn() + .3 * b.acceleration.getX().getIn())
                + .6 * (.7 * c.acceleration.getX().getIn() + .3 * d.acceleration.getX().getIn());
        assertEquals(expectedVelocity, interpolated.velocity.getX().getIn(), 1e-8);
        assertEquals(expectedAcceleration, interpolated.acceleration.getX().getIn(), 1e-8);
    }
    @Test public void benchmarkInitializationMemoryAndQueries() throws Exception {
        long start = System.nanoTime(); Path online = line(0, 0, 100).setLookaheadDistance(Dist.fromIn(20)).quickBuild();
        long onlineBuild = System.nanoTime() - start;
        start = System.nanoTime(); Path memory = line(0, 0, 100).buildToMemory(); long memoryBuild = System.nanoTime() - start;
        OnlineTrajectoryPlanner planner = new OnlineTrajectoryPlanner(online, constants);
        for (int i = 0; i < 30; i++) { planner.sample(vec(20, 0), vec(20, 0), .02); memory.getTrajectoryField().sample(vec(20, 0)); }
        long[] rolling = new long[200], lookup = new long[200];
        for (int i = 0; i < 200; i++) {
            start = System.nanoTime(); planner.sample(vec(20 + i * .01, 0), vec(20, 0), .02); rolling[i] = System.nanoTime() - start;
            start = System.nanoTime(); memory.getTrajectoryField().sample(vec(20 + i * .01, 0)); lookup[i] = System.nanoTime() - start;
        }
        Arrays.sort(rolling); Arrays.sort(lookup);
        java.nio.file.Path output = java.nio.file.Paths.get("build/trajectory-benchmark.csv"); Files.createDirectories(output.getParent());
        String report = "online_build_ms,memory_build_ms,field_bytes,online_p95_ms,lookup_p95_ms,window_samples\n"
                + onlineBuild / 1e6 + "," + memoryBuild / 1e6 + "," + memory.getTrajectoryField().getStorageBytes()
                + "," + rolling[189] / 1e6 + "," + lookup[189] / 1e6 + "," + planner.getLastSampleCount() + "\n";
        Files.write(output, report.getBytes(StandardCharsets.UTF_8));
        assertTrue(memory.getTrajectoryField().getStorageBytes() < 1024 * 1024);
        assertTrue(planner.getLastSampleCount() < online.getGeneratedPoints().length);
    }
    @Test public void cornerTargetsAndRecoveryStayInsidePowerAndAngularLimits() throws Exception {
        Path path = geometry.holonomicPath(geometry.pose(0, 0, 0), geometry.arcPose(30, 0, 7), geometry.pose(30, -30, -90))
                .interpolateWith(InterpolationStyle.TANGENT_FORWARD).quickBuild();
        OnlineTrajectoryPlanner planner = new OnlineTrajectoryPlanner(path, constants);
        for (double s : new double[]{20, 23, 26, 28, 30, 31}) {
            Vector position = path.getParametricPath().getPointAtDistance(s).getLocation();
            Vector tangent = path.getParametricPath().getPointAtDistance(s).getFirstDerivative().normalize();
            for (double speed : new double[]{0, 5, 10, 20, 30}) {
                TrajectorySample target = planner.sample(position, tangent.times(speed), 0, .02);
                assertTrue(Math.abs(target.angularAcceleration) <= constants.angularAccelLimitRad + 1e-5);
                assertTrue(Math.abs(target.angularVelocity) <= constants.angularVelLimitRad + 1e-5);
                Vector body = target.tangent.rotate(target.heading.times(-1));
                double normalAcceleration = target.normalAcceleration().dot(
                        Vector.of(-target.tangent.getY().getIn(), target.tangent.getX().getIn(), DistUnit.IN)).getIn();
                feedforward.MotionPowerCalculator.Output power = drivetrains.models.DriveModels.mecanum(constants)
                        .powerCalculator().calculate(body, target.tangentialVelocity(), target.tangentialAcceleration(),
                                target.angularVelocity, target.angularAcceleration, normalAcceleration);
                assertTrue(power.demand <= 1 + 1e-5);
            }
        }
    }

    @Test public void endpointOverspeedAndWrongDirectionRecoverWithoutInventingRest() {
        Path path = line(0, 0, 20).quickBuild();
        OnlineTrajectoryPlanner planner = new OnlineTrajectoryPlanner(path, constants);
        TrajectorySample atEnd = planner.sample(vec(20, 0), vec(30, 0), 0, .02);
        assertTrue(atEnd.overspeed); assertTrue(atEnd.tangentialVelocity() >= 25 - 1e-6);
        assertTrue(atEnd.tangentialAcceleration() < 0);
        TrajectorySample backwards = planner.sample(vec(10, 0), vec(-.1, 0), 0, .02);
        assertTrue(backwards.overspeed); assertEquals("Recovery must not preview an invented stop",-.1, backwards.tangentialVelocity(), 1e-6);
        assertTrue(backwards.tangentialAcceleration() > 0);
    }

    @Test public void storedQueriesPerformNoSolvesAndOnlineQueriesSolveOnlyTheirWindows() {
        Path stored = line(0, 0, 100).buildToMemory();
        long initial = CoupledMotionProfile.getSolveCount();
        for (int i = 0; i < 100; i++) { stored.getTrajectoryField().sample(vec(i, 3)); }
        assertEquals(initial, CoupledMotionProfile.getSolveCount());
        OnlineTrajectoryPlanner online = new OnlineTrajectoryPlanner(line(0, 0, 1000).setLookaheadDistance(Dist.fromIn(12)).quickBuild(), constants);
        for (int i = 0; i < 10; i++) { online.sample(vec(i, 3), vec(10, 0), .02); }
        assertEquals(initial + 10, CoupledMotionProfile.getSolveCount());
        assertTrue(online.getLastSampleCount() < 15);
    }

    @Test public void coarseFieldDoesNotBlendAcrossAFutureVelocityReduction() {
        Path path = line(0, 0, 100).setMemoryGridSpacing(Dist.fromIn(10))
                .addConstraint(new TranslationalConstraint(.31, PathConstraint.Type.VELOCITY, Dist.fromIn(5))).buildToMemory();
        for (double x : new double[]{31, 31.2, 34, 39, 41}) {
            assertTrue(path.getTrajectoryField().sample(vec(x, 0)).tangentialVelocity() <= 5 + 1e-6);
        }
    }

    @Test public void generatedFieldKeepsItsOriginalGeometrySnapshot() {
        Path path = line(0, 0, 40).buildToMemory();
        TrajectoryField2D oldField = path.getTrajectoryField();
        Path shifted = line(100, 100, 40).quickBuild();
        path.setParametricPath(shifted.getParametricPath()); path.setInterpolator(shifted.getInterpolator());
        assertEquals(0, oldField.sample(vec(20, 0)).toPath.getMag().getIn(), 1e-9);
    }

    @Test public void storedHeadingSnapshotSurvivesInterpolatorEdits() {
        Path path = geometry.holonomicPath(geometry.pose(0, 0, 0), geometry.pose(40, 0, 90))
                .interpolateWith(InterpolationStyle.SMOOTH_START_TO_END).buildToMemory();
        TrajectoryField2D field = path.getTrajectoryField();
        double heading = field.sample(vec(20, 0)).heading.getRad();
        path.getInterpolator().setPathLength(80);
        assertEquals(heading, field.sample(vec(20, 0)).heading.getRad(), 1e-9);
    }

    @Test public void storedBrakingEnvelopeDoesNotRetainMutableGenerationRows() {
        Path path = line(0, 0, 40).quickBuild();
        MotionParameters[] states = {new MotionParameters(0, 10, 0, 0, 0),
                new MotionParameters(20, -10, 0, 0, 20), new MotionParameters(0, 0, 0, 0, 40)};
        TrajectoryField2D field = new TrajectoryField2D(path, new trajectory.profile.FFLut(states), 1, 12);
        double speed = field.sample(vec(30, 0)).tangentialVelocity();
        states[1].setTangentialVel(100);
        assertEquals(speed, field.sample(vec(30, 0)).tangentialVelocity(), 1e-9);
    }

    @Test public void gridIncludesSplineExtremaBetweenArcLengthNodes() {
        Path path = geometry.holonomicPath(geometry.pose(-1, -1, 0), geometry.pose(0, .7, 0), geometry.pose(1, -1, 0))
                .setMemoryGridSpacing(Dist.fromIn(.1)).setMemoryGridPadding(Dist.fromIn(0)).buildToMemory();
        TrajectoryField2D field = path.getTrajectoryField();
        for (int i = 0; i <= 200; i++) {
            Vector point = path.getParametricPath().getPosition(i / 200.0);
            assertTrue(point.getX().getIn() >= field.getMinXIn() - 1e-9);
            assertTrue(point.getY().getIn() >= field.getMinYIn() - 1e-9);
            assertTrue(point.getX().getIn() <= field.getMaxXIn() + 1e-9);
            assertTrue(point.getY().getIn() <= field.getMaxYIn() + 1e-9);
        }
    }

}
