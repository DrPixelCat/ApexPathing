package feedforward;

import paths.geometry.Angle;
import paths.geometry.AngleUnit;
import paths.geometry.DistUnit;
import paths.geometry.GeometryFactory;
import paths.geometry.Pose;
import trajectory.generators.PathProfileGenerator;

import follower.FollowerConstants;
import storage.ApexStorage;

import trajectory.profile.MotionParameters;

import org.json.JSONObject;
import org.junit.Test;
import paths.heading.InterpolationStyle;
import paths.movements.Path;
import static org.junit.Assert.*;

public class MotionCharacterizationTest {
    private String previousDirectory;
    @org.junit.Before public void desktopStorage() {
        previousDirectory = System.getProperty(ApexStorage.DIRECTORY_PROPERTY);
        System.setProperty(ApexStorage.DIRECTORY_PROPERTY, new java.io.File("build/characterization-unit").getAbsolutePath());
    }
    @org.junit.After public void restoreStorage() {
        if (previousDirectory == null) { System.clearProperty(ApexStorage.DIRECTORY_PROPERTY); }
        else { System.setProperty(ApexStorage.DIRECTORY_PROPERTY, previousDirectory); }
    }
    private MotionCharacterization calibration(boolean holonomic) {
        MotorPowerModel[][] models = new MotorPowerModel[3][2];
        for (MotionCharacterization.Axis axis : MotionCharacterization.Axis.values()) {
            if (axis == MotionCharacterization.Axis.STRAFE && !holonomic) { continue; }
            for (int i = 0; i < 2; i++) {
                models[axis.ordinal()][i] = new MotorPowerModel(183, -2.3076923, 49, 0, 53, 0);
            }
        }
        return new MotionCharacterization(models, holonomic);
    }
    private FollowerConstants constants() throws Exception {
        FollowerConstants c = FollowerConstants.fromJson(new JSONObject().put("drivetrainType", "MECANUM"));
        c.translationalKV = .0082; c.translationalKA = .0053; c.translationalFeedforwardKS = .238;
        c.angularKV = .0082; c.angularKA = .0053; c.angularFeedforwardKS = .238;
        c.forwardVelLimitIn = c.strafeVelLimitIn = 60;
        c.forwardAccelLimitIn = c.strafeAccelLimitIn = 95;
        c.angularVelLimitRad = 10; c.angularAccelLimitRad = 20;
        c.profileCornerSpeedScale = 1; c.motionCharacterization = calibration(true);
        return c;
    }
    @Test public void correctsHighSpeedHoldingAndAccelerationWhilePreservingBrakingFallback() throws Exception {
        FollowerConstants c = constants();
        double hold = c.characterizedPower(MotionCharacterization.Axis.FORWARD, 50, 0, .648);
        assertEquals(49 / (183 - 2.3076923 * 50), hold, 1e-7);
        assertEquals(-hold, c.characterizedPower(MotionCharacterization.Axis.FORWARD, -50, 0, -.648), 1e-7);
        assertEquals(.2, c.characterizedPower(MotionCharacterization.Axis.FORWARD, 50, -10, .2), 0);
        assertEquals(.3, c.characterizedPower(MotionCharacterization.Axis.FORWARD, 60, 10, .3), 0);
    }
    @Test public void survivesSerializationAndDualProfileSwitching() throws Exception {
        FollowerConstants c = FollowerConstants.fromJson(new JSONObject().put("drivetrainType", "DUAL_ACTUATED"));
        c.configure(drivetrains.BaseDrivetrain.DrivetrainType.DUAL_ACTUATED, FollowerConstants.Profile.TANK, true);
        c.motionCharacterization = calibration(false);
        c.selectProfile(FollowerConstants.Profile.HOLONOMIC);
        assertNull(c.motionCharacterization);
        c.motionCharacterization = calibration(true);
        FollowerConstants copy = FollowerConstants.fromJson(c.toJson());
        copy.selectProfile(FollowerConstants.Profile.TANK);
        assertNotNull(copy.motionCharacterization);
        assertNull(copy.motionCharacterization.model(MotionCharacterization.Axis.STRAFE, 1));
        copy.selectProfile(FollowerConstants.Profile.HOLONOMIC);
        assertNotNull(copy.motionCharacterization.model(MotionCharacterization.Axis.STRAFE, 1));
    }
    @Test public void optionalWheelModelSurvivesCharacterizationPersistence() {
        MotionCharacterization original=calibration(true);
        MotorPowerModel[][] axes=new MotorPowerModel[3][2];
        for(MotionCharacterization.Axis axis:MotionCharacterization.Axis.values()) {
            axes[axis.ordinal()][0]=original.model(axis,1);axes[axis.ordinal()][1]=original.model(axis,-1);
        }
        MecanumWheelModel wheels=new MecanumWheelModel(100,1,3,2,8,16,2,3,40,.01);
        MotionCharacterization restored=MotionCharacterization.fromJson(new MotionCharacterization(axes,true,.8,wheels).toJson(),true);
        assertEquals(3,restored.mecanumWheelModel.effectiveRadius,0);assertEquals(.8,restored.availableMotorPower,0);
        assertNull(MotionCharacterization.fromJson(original.toJson(),true).mecanumWheelModel);
        assertThrows(IllegalArgumentException.class,()->new MotionCharacterization(axes,false,1,wheels));
    }
    @Test public void straightProfilesRespectMeasuredAuthorityAndConfiguredLimits() throws Exception {
        FollowerConstants c = constants();
        GeometryFactory geometry = new GeometryFactory().setDistUnit(DistUnit.IN).setAngleUnit(AngleUnit.DEG);
        Path path = geometry.holonomicPath(geometry.pose(0, 0, 0), geometry.pose(120, 0, 0))
                .interpolateWith(InterpolationStyle.CONSTANT_START_HEADING).quickBuild();
        PathProfileGenerator generator = new PathProfileGenerator(c, path, drivetrains.models.DriveModels.mecanum(c));
        MotionParameters[] states = generator.generate();
        assertTrue(generator.getLastDebugReport().converged);
        boolean highSpeedRamp = false;
        for (MotionParameters state : states) {
            double v = state.getTangentialVel(), a = state.getTangentialAccel();
            assertTrue(v <= c.calibratedVelocityLimit(MotionCharacterization.Axis.FORWARD, c.forwardVelLimitIn) + .001);
            assertTrue(Math.abs(a) <= c.forwardAccelLimitIn + .001);
            assertTrue(state.getMotorPower() <= c.profilePowerLimit() + .002);
            if (a > 0 && v > 35) {
                highSpeedRamp = true;
                assertTrue(c.motionCharacterization.model(MotionCharacterization.Axis.FORWARD, 1)
                        .power(v, a) <= c.profilePowerLimit() + .002);
            }
        }
        assertTrue("Expected a ramp through the high-speed region", highSpeedRamp);
        assertEquals(0, states[0].getTangentialVel(), 0);
        assertEquals(0, states[states.length - 1].getTangentialVel(), 0);
    }
    @Test public void malformedCharacterizationsFailInsteadOfSupplyingUnboundedPower() throws Exception {
        assertThrows(IllegalArgumentException.class, () -> new MotorPowerModel(1, -1, 0, 0, 2, 0));
        assertThrows(IllegalArgumentException.class, () -> MotionCharacterization.fromJson(new JSONObject().put("version", 99), true));
    }
    @Test public void calibratedPowerLimitReservesAuthorityUnderLowerDrivetrainPowerCaps() throws Exception {
        MotorPowerModel[][] models = new MotorPowerModel[3][2];
        for (int axis = 0; axis < 3; axis++) {
            for (int dir = 0; dir < 2; dir++) { models[axis][dir] = new MotorPowerModel(183, -2.3076923, 49, 0, 53, 0); }
        }
        MotionCharacterization c = new MotionCharacterization(models, true, .7);
        assertEquals(.616, c.profilePowerLimit(), 1e-9);
        double ceiling = c.velocityLimit(MotionCharacterization.Axis.FORWARD);
        assertEquals(.616, c.model(MotionCharacterization.Axis.FORWARD, 1).holdingPower(ceiling), 1e-6);
        assertEquals(.7, MotionCharacterization.fromJson(c.toJson(), true).availableMotorPower, 0);
    }
    @Test public void translationAndTurningShareTheMeasuredPowerBudget() throws Exception {
        FollowerConstants c = constants();
        GeometryFactory geometry = new GeometryFactory().setDistUnit(DistUnit.IN).setAngleUnit(AngleUnit.DEG);
        Path path = geometry.holonomicPath(geometry.pose(0, 0, 0), geometry.pose(70, 30, 90))
                .interpolateWith(InterpolationStyle.SMOOTH_START_TO_END).quickBuild();
        PathProfileGenerator generator = new PathProfileGenerator(c, path, drivetrains.models.DriveModels.mecanum(c));
        for (MotionParameters state : generator.generate()) {
            assertTrue(state.getMotorPower() <= c.profilePowerLimit() + .002);
            assertTrue(Math.abs(state.getAngularVel()) <= c.angularVelLimitRad + .001);
            assertTrue(Math.abs(state.getAngularAccel()) <= c.angularAccelLimitRad + .001);
        }
        assertTrue(generator.getLastDebugReport().converged);
    }
    @Test public void tankAndSwerveProfilesAlsoRetainTheirKinematicAndPowerCaps() throws Exception {
        FollowerConstants c = constants();
        GeometryFactory g = new GeometryFactory().setDistUnit(DistUnit.IN).setAngleUnit(AngleUnit.DEG);
        Path tank = g.tankPath(g.pose(0, 0, 0), g.pose(80, 0, 0))
                .interpolateWith(InterpolationStyle.TANGENT_FORWARD).quickBuild();
        Path holonomic = g.holonomicPath(g.pose(0, 0, 0), g.pose(80, 20, 45))
                .interpolateWith(InterpolationStyle.SMOOTH_START_TO_END).quickBuild();
        for (trajectory.generators.PathProfileGenerator generator : new trajectory.generators.PathProfileGenerator[]{
                new PathProfileGenerator(c, tank, drivetrains.models.DriveModels.tank(c)), new PathProfileGenerator(c, holonomic, drivetrains.models.DriveModels.swerve(c))}) {
            MotionParameters[] states = generator.generate();
            assertTrue(generator.getLastDebugReport().converged);
            for (MotionParameters s : states) {
                assertTrue(s.getMotorPower() <= c.profilePowerLimit() + .002);
                assertTrue(Math.abs(s.getTangentialAccel()) <= c.forwardAccelLimitIn + .001);
                assertTrue(Math.abs(s.getAngularAccel()) <= c.angularAccelLimitRad + .001);
            }
            assertEquals(0, states[states.length - 1].getTangentialVel(), 0);
        }
    }
    @Test public void pointTurnsUseTheSameModelInBothDirections() throws Exception {
        FollowerConstants c = constants();
        for (double direction : new double[]{1, -1}) {
            paths.movements.Turn turn = new paths.movements.Turn(Pose.zero(), Angle.fromRad(direction * 2));
            trajectory.profile.FFLut lut = new trajectory.generators.TurnProfileGenerator(10, 20, c).generate(turn);
            for (int i = 0; i <= 100; i++) {
                MotionParameters s = lut.getFFParams(i * .02);
                double v = s.getAngularVel(), a = s.getAngularAccel();
                double legacy = c.angularKV * v + c.angularKA * a + direction * c.angularFeedforwardKS;
                assertTrue(Math.abs(c.characterizedPower(MotionCharacterization.Axis.ANGULAR, v, a, legacy))
                        <= c.profilePowerLimit() + .002);
                assertTrue(Math.abs(a) <= 20 + .001);
            }
            assertEquals(0, lut.getFFParams(2).getAngularVel(), 0);
        }
    }
}
