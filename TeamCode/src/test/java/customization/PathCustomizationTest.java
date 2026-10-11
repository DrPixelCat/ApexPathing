package customization;

import drivetrains.models.DriveModel;
import follower.FollowerConstants;
import geometry.Angle;
import geometry.DistUnit;
import geometry.GeometryFactory;
import geometry.ParametricSegment;
import geometry.Vector;
import org.json.JSONObject;
import org.junit.Before;
import org.junit.Test;
import paths.builders.HolonomicPathBuilder;
import paths.constraint.PathConstraint;
import paths.heading.HeadingInterpolator;
import paths.heading.InterpolationStyle;
import paths.heading.TankInterpolator;
import paths.movements.Path;
import trajectory.OnlineTrajectoryPlanner;
import trajectory.TrajectorySample;
import trajectory.generators.PathProfileGenerator;
import trajectory.profile.MotionParameters;

import static org.junit.Assert.*;

/** Uses only extension APIs available to robot code, outside Apex implementation packages. */
public class PathCustomizationTest {
    private FollowerConstants settings;
    private final GeometryFactory geometry = new GeometryFactory();

    @Before public void setup() throws Exception {
        settings = FollowerConstants.fromJson(new JSONObject().put("drivetrainType", "MECANUM"));
        settings.forwardVelLimitIn = settings.strafeVelLimitIn = 60;
        settings.forwardAccelLimitIn = settings.strafeAccelLimitIn = 100;
        settings.angularVelLimitRad = 4;
        settings.angularAccelLimitRad = 8;
        settings.translationalKV = .01;
        settings.translationalKA = .005;
        settings.angularKV = .1;
        settings.angularKA = .02;
    }

    public static class Wave implements ParametricSegment {
        @Override public Vector getPosition(double t) { return vector(40 * t, 5 * Math.sin(Math.PI * t)); }
        @Override public Vector getFirstDerivative(double t) { return vector(40, 5 * Math.PI * Math.cos(Math.PI * t)); }
        @Override public Vector getSecondDerivative(double t) { return vector(0, -5 * Math.PI * Math.PI * Math.sin(Math.PI * t)); }
    }

    /** Heading argument is remaining inches; derivatives are with respect to forward progress. */
    public static class LinearHeading implements HeadingInterpolator {
        double turn = .5;
        private double length;
        @Override public void setPathLength(double length) { this.length = length; }
        @Override public Angle getHeadingTarg(double remaining, Vector tangent, Vector end) {
            return Angle.fromRad(turn * (1 - remaining / length));
        }
        @Override public double getHeadingFirstDerivative(double remaining, double curvature, Vector end) {
            return turn / length;
        }
        @Override public double getHeadingSecondDerivative(double remaining, double derivative, Vector end) { return 0; }
        @Override public HeadingInterpolator copyForTrajectory() {
            LinearHeading copy = new LinearHeading(); copy.turn = turn; copy.length = length; return copy;
        }
    }

    private static class Limit implements PathConstraint {
        double progress, value;
        final Domain domain;
        final Type type;
        Limit(double progress, Domain domain, Type type, double value) {
            this.progress = progress; this.domain = domain; this.type = type; this.value = value;
        }
        @Override public double getS() { return progress; }
        @Override public void setS(double progress) { this.progress = progress; }
        @Override public Domain getDomain() { return domain; }
        @Override public Type getType() { return type; }
        @Override public double getLimit() { return value; }
        @Override public PathConstraint copyForTrajectory() { return new Limit(progress, domain, type, value); }
    }

    private HolonomicPathBuilder builder() {
        return geometry.holonomicPath(geometry.pose(0, 0, 0), geometry.pose(40, 0, 0))
                .setPlanningConstants(settings).setCurve(new Wave()).setHeadingInterpolator(new LinearHeading());
    }

    private Path build(HolonomicPathBuilder builder, Path.BuildMode mode) {
        return mode == Path.BuildMode.ONLINE ? builder.quickBuild() : builder.buildToMemory();
    }

    @Test public void curvesAndHeadingWorkInBothModesAndReverseWithDetachedHeading() {
        for (Path.BuildMode mode : Path.BuildMode.values()) {
            LinearHeading source = new LinearHeading();
            Path path = build(builder().setHeadingInterpolator(source), mode);
            source.turn = 2; // Builder took a detached copy, before generating either profile.
            assertEquals(5, path.getParametricPath().getPosition(.5).getY().getIn(), 1e-9);
            assertEquals(.5, path.getEndPose().getHeading().getRad(), 1e-9);
            TrajectorySample target = mode == Path.BuildMode.ONLINE
                    ? new OnlineTrajectoryPlanner(path, settings).sample(vector(20, 5), vector(5, 0), 0, .02)
                    : path.getTrajectoryField().sample(vector(20, 5));
            assertEquals(.25, target.heading.getRad(), .01);
            assertTrue(target.angularVelocity > 0);
            assertEquals(target.tangentialVelocity() * .5 / path.getParametricPath().getLengthIn(),
                    target.angularVelocity, .02);
            Path reversed = path.reversed();
            assertEquals(0, reversed.getEndPose().getHeading().getRad(), 1e-9);
            assertEquals(40, reversed.getParametricPath().getPosition(0).getX().getIn(), 1e-9);
            assertEquals(5, reversed.getParametricPath().getPosition(.5).getY().getIn(), 1e-9);
            assertTrue(reversed.getInterpolator().getHeadingFirstDerivative(20, 0, vector(-1, 0)) < 0);
            ((LinearHeading) path.getInterpolator()).turn = 1;
            assertEquals(.5, reversed.getInterpolator().getHeadingTarg(reversed.getParametricPath().getLengthIn(),
                    vector(-1, 0), vector(-1, 0)).getRad(), 1e-9);
            if (mode == Path.BuildMode.MEMORY) {
                assertEquals(.25, path.getTrajectoryField().sample(vector(20, 5)).heading.getRad(), .01);
            }
        }
    }

    @Test public void unrelatedConstraintTypesLimitBothModesAndReverseAsCopies() {
        for (Path.BuildMode mode : Path.BuildMode.values()) {
            Limit speed = new Limit(0, PathConstraint.Domain.TRANSLATIONAL, PathConstraint.Type.VELOCITY, 12);
            Limit acceleration = new Limit(0, PathConstraint.Domain.TRANSLATIONAL, PathConstraint.Type.ACCELERATION, 7);
            Limit omega = new Limit(0, PathConstraint.Domain.ANGULAR, PathConstraint.Type.VELOCITY, .1);
            Limit alpha = new Limit(0, PathConstraint.Domain.ANGULAR, PathConstraint.Type.ACCELERATION, .09);
            Path path = build(builder().addConstraint(speed).addConstraint(acceleration).addConstraint(omega).addConstraint(alpha), mode);
            PathProfileGenerator liveGenerator = path.createGenerator();
            assertLimits(liveGenerator.generateRestToRest(), 12, 7, .1, .09);
            TrajectorySample sample = mode == Path.BuildMode.ONLINE
                    ? new OnlineTrajectoryPlanner(path, settings).sample(vector(20, 5), Vector.zero(), 0, .02)
                    : path.getTrajectoryField().sample(vector(20, 5));
            assertTrue(sample.tangentialVelocity() <= 12.001);
            assertTrue(Math.abs(sample.angularVelocity) <= .101);
            Path reverse = path.reversed();
            speed.value = 3; // Unobservable custom mutation must be read live online.
            assertLimits(liveGenerator.generateRestToRest(), 3, 7, .1, .09);
            assertLimits(reverse.createGenerator().generateRestToRest(), 12, 7, .1, .09);
            assertNotSame(speed, reverse.getConstraints()[0]);
            if (mode == Path.BuildMode.MEMORY)
                assertEquals(sample.tangentialVelocity(), path.getTrajectoryField().sample(vector(20, 5)).tangentialVelocity(), 0);
        }
    }

    @Test public void customStepScheduleReversesAtTheSameFieldLocations() {
        Path path = builder()
                .addConstraint(new Limit(0, PathConstraint.Domain.TRANSLATIONAL, PathConstraint.Type.VELOCITY, 12))
                .addConstraint(new Limit(.4, PathConstraint.Domain.TRANSLATIONAL, PathConstraint.Type.VELOCITY, 5))
                .quickBuild();
        Path reverse = path.reversed();
        double length = path.getParametricPath().getLengthIn();
        for (MotionParameters state : reverse.createGenerator().generateRestToRest()) {
            if (state.getProgression() <= .6 * length) assertTrue(state.getTangentialVel() <= 5.001);
        }
        assertEquals(12, reverse.getQuickVelocityLimit(.7, 60), 0);
        assertEquals(5, reverse.getQuickVelocityLimit(.3, 60), 0);
    }

    @Test public void generatorFactoryIsUsedForEveryEntryPointOnlineMemoryAndReversal() {
        for (Path.BuildMode mode : Path.BuildMode.values()) {
            int[] calls = new int[3]; // creation, complete profile, online window
            Path path = build(builder().setProfileGeneratorFactory((p, c, model) -> {
                calls[0]++;
                return new CountingGenerator(c, p, model, calls);
            }), mode);
            int afterBuild = calls[0];
            assertEquals(mode == Path.BuildMode.MEMORY ? 1 : 0, afterBuild);
            assertTrue(path.createGenerator() instanceof CountingGenerator);
            assertTrue(path.createGenerator(settings) instanceof CountingGenerator);
            assertTrue(path.createGenerator(settings, path.getDriveModel(settings)) instanceof CountingGenerator);
            assertEquals(afterBuild + 3, calls[0]);
            new OnlineTrajectoryPlanner(path, settings).sample(vector(10, 4), Vector.zero(), 0, .02);
            assertEquals(1, calls[2]);
            int profiles = calls[1];
            Path reverse = path.reversed();
            assertEquals(profiles + (mode == Path.BuildMode.MEMORY ? 1 : 0), calls[1]);
            assertTrue(reverse.createGenerator() instanceof CountingGenerator);
            path.setProfileGeneratorFactory((p, c, model) -> null);
            assertThrows(NullPointerException.class, path::createGenerator);
            if (mode == Path.BuildMode.MEMORY) assertThrows(NullPointerException.class, path::getTrajectoryField);
        }
    }

    private static class CountingGenerator extends PathProfileGenerator {
        final int[] calls;
        CountingGenerator(FollowerConstants c, Path p, DriveModel model, int[] calls) {
            super(c, p, model); this.calls = calls;
        }
        @Override public MotionParameters[] generateRestToRest() { calls[1]++; return super.generateRestToRest(); }
        @Override public WindowTarget generateWindow(double start, double horizon, double speed, double omega,
                double dt, double preview) {
            calls[2]++; return super.generateWindow(start, horizon, speed, omega, dt, preview);
        }
    }

    @Test public void invalidGeometryAndNonTangentTankHeadingsAreRejected() {
        assertThrows(IllegalArgumentException.class, () -> geometry.holonomicPath(geometry.pose(0, 0),
                geometry.pose(20, 0), geometry.pose(40, 0)).setCurve(new Wave()));
        assertThrows(IllegalArgumentException.class, () -> geometry.holonomicPath(geometry.pose(0, 0),
                geometry.pose(41, 0)).setCurve(new Wave()).quickBuild());
        assertThrows(IllegalArgumentException.class, () -> builder().setCurve(new Wave() {
            @Override public Vector getSecondDerivative(double t) { return vector(Double.NaN, 0); }
        }).quickBuild());
        paths.builders.TankPathBuilder tank = geometry.tankPath(geometry.pose(0, 0), geometry.pose(40, 0));
        assertThrows(IllegalArgumentException.class, () -> tank.setHeadingInterpolator(new LinearHeading()));
        Path path = tank.setCurve(new Wave()).setPlanningConstants(settings)
                .setHeadingInterpolator(new TankInterpolator(InterpolationStyle.TANGENT_FORWARD)).quickBuild();
        assertTrue(path.getInterpolator().supportsTank());
        assertTrue(path.reversed().getInterpolator().supportsTank());
        assertEquals(new Wave().getFirstDerivative(1).getTheta().getRad(), path.getEndPose().getHeading().getRad(), 1e-9);
    }

    private static void assertLimits(MotionParameters[] states, double v, double a, double w, double alpha) {
        assertTrue(states.length > 2);
        double peak = 0;
        for (MotionParameters state : states) {
            peak = Math.max(peak, state.getTangentialVel());
            assertTrue(state.getTangentialVel() <= v + .001);
            assertTrue(Math.abs(state.getTangentialAccel()) <= a + .001);
            assertTrue(Math.abs(state.getAngularVel()) <= w + .001);
            assertTrue(Math.abs(state.getAngularAccel()) <= alpha + .001);
        }
        assertTrue(peak > 0);
    }

    private static Vector vector(double x, double y) { return Vector.of(x, y, DistUnit.IN); }
}
