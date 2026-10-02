package feedforward.generators;

import core.ApexStorage;
import core.FollowerConstants;
import core.MotionCharacterization;
import core.MotorPowerModel;
import drivetrains.BaseDrivetrain.DrivetrainType;
import feedforward.MotionParameters;
import geometry.AngleUnit;
import geometry.Dist;
import geometry.DistUnit;
import geometry.GeometryFactory;
import org.junit.Before;
import org.junit.Test;
import paths.builders.HolonomicPathBuilder;
import paths.builders.TankPathBuilder;
import paths.constraint.PathConstraint.Type;
import paths.constraint.TranslationalConstraint;
import paths.heading.InterpolationStyle;
import paths.movements.Path;

import java.io.File;

import static org.junit.Assert.*;

public class ProfilePowerLimitTest {
    private FollowerConstants constants;
    private GeometryFactory geometry;

    @Before public void setup() {
        System.setProperty(ApexStorage.DIRECTORY_PROPERTY,
                new File("build/profile-power-limit-tests").getAbsolutePath());
        constants = FollowerConstants.getInstance();
        constants.configure(DrivetrainType.MECANUM, FollowerConstants.Profile.DEFAULT, true);
        constants.motionCharacterization = null;
        constants.forwardVelLimitIn = 100;
        constants.strafeVelLimitIn = 100;
        constants.forwardAccelLimitIn = 100;
        constants.strafeAccelLimitIn = 100;
        constants.angularVelLimitRad = 5;
        constants.angularAccelLimitRad = 10;
        constants.translationalKV = .01;
        constants.translationalKA = .005;
        constants.translationalFeedforwardKS = 0;
        constants.angularKV = .1;
        constants.angularKA = .02;
        constants.angularFeedforwardKS = 0;
        constants.kCentripetal = 0;
        constants.profileVelocityPowerCoefficient = 0;
        constants.profileCornerSpeedScale = 1;
        geometry = new GeometryFactory().setDistUnit(DistUnit.IN).setAngleUnit(AngleUnit.DEG);
    }

    private HolonomicPathBuilder builder() {
        return new HolonomicPathBuilder(geometry.pose(0, 0, 0), geometry.pose(500, 0, 0))
                .interpolateWith(InterpolationStyle.CONSTANT_START_HEADING);
    }

    @Test public void builderCeilingChangesProfileAndSurvivesReversal() {
        Path lower = builder().setProfilePowerLimit(.6).buildToMemory();
        Path full = builder().setProfilePowerLimit(1).buildToMemory();
        assertEquals(.6, lower.getProfilePowerLimit(), 0);
        assertTrue(peakVelocity(lower) < peakVelocity(full) - 20);
        assertWithinBudget(new MecanumProfileGenerator(constants, lower), .6);
        double peak = peakVelocity(lower);
        lower.precomputeTrajectory();
        assertEquals(peak, peakVelocity(lower), 1e-6);
        Path reverse = lower.reversed();
        assertEquals(.6, reverse.getProfilePowerLimit(), 0);
        assertWithinBudget(new MecanumProfileGenerator(constants, reverse), .6);
    }

    @Test public void explicitGeneratorCeilingOverridesPathWithoutChangingIt() {
        Path path = builder().setProfilePowerLimit(.4).quickBuild();
        MecanumProfileGenerator generator = new MecanumProfileGenerator(constants, path, .7);
        assertEquals(.7, generator.getProfilePowerLimit(), 0);
        assertWithinBudget(generator, .7);
        assertEquals(.4, path.getProfilePowerLimit(), 0);
        assertEquals(.4, new MecanumProfileGenerator(constants, path).getProfilePowerLimit(), 0);
    }

    @Test public void lowPowerCeilingLimitsBothAccelerationAndBraking() {
        Path path = builder().setProfilePowerLimit(.2).quickBuild();
        MotionParameters[] profile = new MecanumProfileGenerator(constants, path).generate();
        boolean brakes = false;
        for (MotionParameters point : profile) {
            assertTrue(point.getMotorPower() <= .201001);
            if (point.getTangentialAccel() < -1) { brakes = true; }
        }
        assertTrue(brakes);
        assertTrue(profile[0].getTangentialAccel() < constants.forwardAccelLimitIn);
    }

    @Test public void boostedBoundariesKeepTheirSteadyPowerBudgetAndReportTerminalPower() {
        Path path = builder().setProfilePowerLimit(.6).quickBuild();
        path.useBoostedAccel();
        MecanumProfileGenerator generator = new MecanumProfileGenerator(constants, path);
        MotionParameters[] profile = generator.generate();
        assertEquals(60, profile[0].getTangentialVel(), 1e-5);
        assertEquals(60, profile[profile.length - 1].getTangentialVel(), 1e-5);
        assertEquals(.6, profile[profile.length - 1].getMotorPower(), 1e-7);
        assertTrue(generator.getLastDebugReport().converged);
    }

    @Test public void defaultBudgetsRemainUnchanged() {
        Path path = builder().quickBuild();
        assertNull(path.getProfilePowerLimit());
        assertEquals(1, new MecanumProfileGenerator(constants, path).getProfilePowerLimit(), 0);
        constants.motionCharacterization = characterization(1);
        assertEquals(.88, new MecanumProfileGenerator(constants, path).getProfilePowerLimit(), 0);
        constants.motionCharacterization = characterization(.8);
        assertEquals(.704, new MecanumProfileGenerator(constants, path).getProfilePowerLimit(), 1e-12);
    }

    @Test public void characterizedCeilingsUseRequestedPowerAboveAndBelowDefault() {
        constants.motionCharacterization = characterization(1);
        Path defaultPath = builder().buildToMemory();
        Path full = builder().setProfilePowerLimit(1).buildToMemory();
        Path lower = builder().setProfilePowerLimit(.6).buildToMemory();
        assertTrue("Full power must escape the old 0.88 measured velocity ceiling",
                peakVelocity(full) > peakVelocity(defaultPath) + 6);
        assertTrue(peakVelocity(defaultPath) > peakVelocity(lower) + 20);
        assertWithinBudget(new MecanumProfileGenerator(constants, lower), .6);
        assertWithinBudget(new MecanumProfileGenerator(constants, full), 1);
    }

    @Test public void calibratedAvailablePowerStillBoundsExplicitCeiling() {
        constants.motionCharacterization = characterization(.8);
        Path path = builder().setProfilePowerLimit(1).buildToMemory();
        MecanumProfileGenerator generator = new MecanumProfileGenerator(constants, path);
        assertEquals(.8, generator.getProfilePowerLimit(), 0);
        assertWithinBudget(generator, .8);
        assertTrue(peakVelocity(path) <= 80.001);
        assertEquals(80, constants.motionCharacterization.velocityLimit(MotionCharacterization.Axis.FORWARD, 1), 1e-5);
    }

    @Test public void powerCeilingDoesNotRequireFullPowerWhenKinematicsBind() {
        Path path = builder().setProfilePowerLimit(.9)
                .addConstraint(new TranslationalConstraint(0, Type.VELOCITY, Dist.fromIn(15)))
                .addConstraint(new TranslationalConstraint(0, Type.ACCELERATION, Dist.fromIn(5)))
                .buildToMemory();
        MotionParameters[] profile = new MecanumProfileGenerator(constants, path).generate();
        double maxPower = 0;
        for (MotionParameters point : profile) {
            assertTrue(point.getTangentialVel() <= 15 + 1e-6);
            assertTrue(Math.abs(point.getTangentialAccel()) <= 5 + 1e-6);
            maxPower = Math.max(maxPower, point.getMotorPower());
        }
        assertTrue(maxPower < .2);
    }

    @Test public void sharedPathApiAlsoHonorsTankSwerveAndTankMemoryFields() {
        Path holonomic = builder().setProfilePowerLimit(.6).quickBuild();
        assertWithinBudget(new SwerveProfileGenerator(constants, holonomic), .6);
        Path tank = new TankPathBuilder(geometry.pose(0, 0, 0), geometry.pose(500, 0, 0))
                .setProfilePowerLimit(.6).buildToMemory();
        assertWithinBudget(new TankProfileGenerator(constants, tank), .6);
        Path quick = new TankPathBuilder(geometry.pose(0, 0, 0), geometry.pose(500, 0, 0))
                .setProfilePowerLimit(.4).quickBuild();
        assertNull(quick.getFeedforwardLut());
        assertEquals(Path.BuildMode.ONLINE, quick.getBuildMode());
        trajectory.OnlineTrajectoryPlanner planner = new trajectory.OnlineTrajectoryPlanner(quick, constants);
        assertTrue(planner.sample(geometry.vector(0, 0), geometry.vector(40, 0), .02).tangentialVelocity() <= 40 + 1e-6);
        assertEquals(.5, new SwerveProfileGenerator(constants, holonomic, .5).getProfilePowerLimit(), 0);
        assertEquals(.5, new TankProfileGenerator(constants, tank, .5).getProfilePowerLimit(), 0);
    }

    @Test public void rejectsInvalidPowerLimitsAtPublicEntryPoints() {
        Path path = builder().quickBuild();
        for (double invalid : new double[] {0, -.1, 1.01, Double.NaN,
                Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY}) {
            assertThrows(IllegalArgumentException.class, () -> builder().setProfilePowerLimit(invalid));
            assertThrows(IllegalArgumentException.class, () -> path.setProfilePowerLimit(invalid));
            assertThrows(IllegalArgumentException.class, () -> new MecanumProfileGenerator(constants, path, invalid));
            assertThrows(IllegalArgumentException.class, () -> characterization(1).velocityLimit(MotionCharacterization.Axis.FORWARD, invalid));
        }
    }

    private MotionCharacterization characterization(double available) {
        MotorPowerModel[][] models = new MotorPowerModel[3][2];
        for (MotionCharacterization.Axis axis : MotionCharacterization.Axis.values()) {
            MotorPowerModel model = axis == MotionCharacterization.Axis.ANGULAR
                    ? new MotorPowerModel(10, 0, 0, 1, 5, 0)
                    : new MotorPowerModel(100, 0, 0, 1, 95, 0);
            models[axis.ordinal()][0] = model;
            models[axis.ordinal()][1] = model;
        }
        return new MotionCharacterization(models, true, available);
    }

    private double peakVelocity(Path path) {
        double max = 0;
        for (geometry.PathPoint point : path.getGeneratedPoints()) {
            double distance = path.getParametricPath().getLengthIn() - point.getDistanceToEndIn();
            max = Math.max(max, path.getFeedforwardLut().getFFParams(distance).getTangentialVel());
        }
        return max;
    }

    private void assertWithinBudget(BaseProfileGenerator generator, double budget) {
        MotionParameters[] profile = generator.generate();
        double maxPower = 0;
        for (MotionParameters point : profile) {
            assertTrue(Double.isFinite(point.getMotorPower()));
            assertTrue(point.getMotorPower() <= budget + .001001);
            maxPower = Math.max(maxPower, point.getMotorPower());
        }
        assertTrue("Long path should reach the configured power budget", maxPower >= budget - .01);
        assertEquals(0, profile[0].getTangentialVel(), 0);
        assertEquals(0, profile[profile.length - 1].getTangentialVel(), 0);
        assertTrue(generator.getLastDebugReport().converged);
    }
}
