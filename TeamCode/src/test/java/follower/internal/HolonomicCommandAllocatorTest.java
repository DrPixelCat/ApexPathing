package follower.internal;


import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import drivetrains.Mecanum;
import drivetrains.Omni;
import drivetrains.Motor;
import paths.geometry.Angle;
import paths.geometry.Dist;
import paths.geometry.DistUnit;
import paths.geometry.Vector;
import org.junit.Test;
import static org.junit.Assert.assertEquals;

public class HolonomicCommandAllocatorTest {
    private static HolonomicCommandAllocator allocatorFor(drivetrains.BaseDrivetrain<?> drivetrain) {
        try {
            follower.FollowerConstants settings=follower.FollowerConstants.fromJson(
                    new org.json.JSONObject().put("drivetrainType",drivetrain.getDrivetrainType().name()));
            return new HolonomicCommandAllocator(drivetrain,null,drivetrains.models.DriveModels.forDrivetrain(
                    drivetrain,settings,paths.movements.Path.PathType.HOLONOMIC));
        } catch (org.json.JSONException exception) { throw new AssertionError(exception); }
    }

    private Omni omni(int count, double offset) {
        Omni.Constants constants = new Omni.Constants().setFirstWheelAngle(Angle.fromDeg(offset));
        for (int i = 0; i < count; i++) {
            constants.addWheel(new Motor("omni" + i) {
                @Override public DcMotorEx build(HardwareMap map) { return null; }
            });
        }
        return constants.build(null);
    }

    @Test public void omniAllocationUsesItsMountingOffsetAndWheelCount() {
        Vector zero = Vector.zero();
        for (double offset : new double[]{0, 30}) {
            Omni drivetrain = omni(3, offset);
            HolonomicCommandAllocator omni = allocatorFor(drivetrain);
            HolonomicCommandAllocator.Result result = omni.allocate(zero, zero,
                    Vector.of(1, 0, DistUnit.IN), Angle.zero(), 0, 0, 2, 0, 0, 0);
            double expected = offset == 0 ? 2 / Math.sqrt(3) : 1;
            assertEquals(expected, result.drive.getX().getIn(), 1e-12);
        }
    }

    @Test public void omniAllocationBoundsEveryWheelWithMixedCorrectionsTranslationAndRotation() {
        Vector zero = Vector.zero();
        for (int count : new int[]{3, 4, 5, 6, 8, 17}) {
            for (double offset : new double[]{0, 17, 90}) {
                HolonomicCommandAllocator omni = allocatorFor(omni(count, offset));
                for (int heading = 0; heading < 360; heading += 45) {
                    for (int direction = 0; direction < 360; direction += 30) {
                        Vector tangent = Vector.of(Math.cos(Math.toRadians(direction)),
                                Math.sin(Math.toRadians(direction)), DistUnit.IN);
                        HolonomicCommandAllocator.Result result = omni.allocate(tangent.times(.3), zero,
                                tangent, Angle.fromDeg(heading), .2, -.15, 1.4, .5, .2, .8);
                        for (int i = 0; i < count; i++) {
                            double angle = Math.toRadians(offset) + 2 * Math.PI * i / count;
                            double power = -Math.sin(angle) * result.drive.getX().getIn()
                                    + Math.cos(angle) * result.drive.getY().getIn() + result.turn;
                            org.junit.Assert.assertTrue(Math.abs(power) <= 1 + 1e-12);
                        }
                        org.junit.Assert.assertTrue(result.demand.velocityScale >= 0 && result.demand.velocityScale <= 1);
                    }
                }
            }
        }
    }

    @Test public void threeWheelOmniKeepsSignedTranslationRotationCancellation() {
        HolonomicCommandAllocator allocator = allocatorFor(omni(3,90));
        Vector zero = Vector.zero();
        // Wheel translations are {-1, .5, .5}; +.25 turn reduces the peak to .75.
        HolonomicCommandAllocator.Result result = allocator.allocate(zero,zero,
                Vector.of(1,0,DistUnit.IN),Angle.zero(),0,0,1,0,0,.25);
        assertEquals(1,result.demand.feedforwardScale,1e-12);
        assertEquals(.75,result.demand.feedforwardTotal,1e-12);
        assertEquals(1,result.drive.getX().getIn(),1e-12);
        assertEquals(.25,result.turn,1e-12);
    }

    private controllers.DriveController positionController() {
        return new controllers.DriveController(Dist.fromIn(60), Dist.fromIn(30),
                new controllers.FeedbackController.Coefficients(), false);
    }

    @Test public void positionHoldBoundsOmniWheelsUsingTheirActualGeometry() throws Exception {
        follower.FollowerConstants settings = follower.FollowerConstants.fromJson(
                new org.json.JSONObject().put("drivetrainType", "OMNI"));
        for (int count : new int[]{3, 4, 5, 8, 17}) for (double offset : new double[]{0, 17, 90}) {
            Omni drivetrain = omni(count, offset);
            HolonomicCommandAllocator allocator = new HolonomicCommandAllocator(drivetrain, positionController(),
                    drivetrains.models.DriveModels.forDrivetrain(drivetrain, settings, paths.movements.Path.PathType.HOLONOMIC));
            for (int direction = 0; direction < 360; direction += 15) {
                controllers.DriveController.AllocatedCommand result = allocator.allocatePositionHold(
                        Vector.of(2, 3, DistUnit.IN), Angle.fromDeg(direction), .6);
                Vector body = result.getRobotCommand();
                for (int wheel = 0; wheel < count; wheel++) {
                    double translation = drivetrain.getWheelProjectionModel().wheelPower(wheel,
                            body.getX().getIn(), body.getY().getIn(), 0);
                    org.junit.Assert.assertTrue(Math.abs(translation) <= .6 + 1e-12);
                    org.junit.Assert.assertTrue(Math.abs(translation + .4) <= 1 + 1e-12);
                }
                assertEquals(.6, result.getPowerDemand(), 1e-12);
            }
        }
    }

    @Test public void modelPositionHoldPreservesLegacyMecanumPenaltyAndBudget() throws Exception {
        follower.FollowerConstants settings = follower.FollowerConstants.fromJson(
                new org.json.JSONObject().put("drivetrainType", "MECANUM"));
        settings.forwardVelLimitIn = 60; settings.strafeVelLimitIn = 30;
        controllers.DriveController controller = positionController();
        Mecanum drivetrain = new Mecanum(new Mecanum.Constants().setFrontLeftMotor(unusedMotor)
                .setFrontRightMotor(unusedMotor), null);
        HolonomicCommandAllocator allocator = new HolonomicCommandAllocator(drivetrain,
                controller, drivetrains.models.DriveModels.mecanum(settings));
        for (double budget : new double[]{0, .3, 1}) for (int heading = 0; heading < 360; heading += 15) {
            Vector command = Vector.of(.2, .3, DistUnit.IN);
            controllers.DriveController.AllocatedCommand expected = controller.allocateMecanum(command, Angle.fromDeg(heading), budget);
            controllers.DriveController.AllocatedCommand actual = allocator.allocatePositionHold(command, Angle.fromDeg(heading), budget);
            assertEquals(expected.getRobotCommand().getX().getIn(), actual.getRobotCommand().getX().getIn(), 1e-12);
            assertEquals(expected.getRobotCommand().getY().getIn(), actual.getRobotCommand().getY().getIn(), 1e-12);
            assertEquals(expected.getPowerDemand(), actual.getPowerDemand(), 1e-12);
        }
    }

    @Test public void customPositionModelControlsAllocationWithoutDrivetrainTypeDispatch() throws Exception {
        follower.FollowerConstants settings = follower.FollowerConstants.fromJson(
                new org.json.JSONObject().put("drivetrainType", "MECANUM"));
        Mecanum drivetrain = new Mecanum(new Mecanum.Constants().setFrontLeftMotor(unusedMotor)
                .setFrontRightMotor(unusedMotor), null);
        HolonomicCommandAllocator allocator = new HolonomicCommandAllocator(drivetrain, positionController(),
                new WeightedPositionModel(settings));
        controllers.DriveController.AllocatedCommand command = allocator.allocatePositionHold(
                Vector.of(1, 1, DistUnit.IN), Angle.zero(), .9);
        assertEquals(.2, command.getRobotCommand().getX().getIn(), 1e-12);
        assertEquals(.1, command.getRobotCommand().getY().getIn(), 1e-12);
        assertEquals(.9, command.getPowerDemand(), 1e-12);
    }

    private static final class WeightedPositionModel extends drivetrains.models.DriveModel {
        WeightedPositionModel(follower.FollowerConstants settings) { super(settings); }
        @Override public drivetrains.models.DriveModel withConstants(follower.FollowerConstants settings) {
            return new WeightedPositionModel(settings);
        }
        @Override public boolean isHolonomic() { return true; }
        @Override public double maximumVelocity(Vector body, double curvature, double prime, double angular) { return 60; }
        @Override public double maximumTangentialAcceleration(Vector body) { return 100; }
        @Override public PowerEvaluator powerEvaluator(Vector body, double curvature) { return new PowerEvaluator(body, curvature) { }; }
        @Override public double translationDemand(double x, double y) { return 4 * Math.abs(x) + Math.abs(y); }
        @Override public Vector positionHoldCommand(Vector body) {
            return Vector.of(2 * body.getX().getIn(), body.getY().getIn(), DistUnit.IN);
        }
    }

    @Test public void nativeWheelCompensationSurvivesAllocationIncludingFourthMode() {
        Vector zero=Vector.zero(), tangent=Vector.of(1,0,DistUnit.IN);
        double[] wheels={.3,.1,-.2,.4};
        HolonomicCommandAllocator.Result result=allocator.allocateWheels(zero,tangent,Angle.zero(),0,0,wheels,0,0,zero);
        org.junit.Assert.assertArrayEquals(wheels,result.wheelPowers,1e-12);
        assertEquals(.4,result.demand.feedforwardTotal,1e-12);
        double[] overloaded={1.2,.4,-.8,1.6};
        result=allocator.allocateWheels(zero,tangent,Angle.zero(),0,.2,overloaded,0,0,zero);
        assertEquals(.5,result.demand.feedforwardScale,1e-12);
        org.junit.Assert.assertArrayEquals(new double[]{.4,.4,-.6,1},result.wheelPowers,1e-12);
    }
    private final Motor unusedMotor = new Motor() {
        @Override public DcMotorEx build(HardwareMap hardwareMap) { return null; }
    };
    private final HolonomicCommandAllocator allocator = allocatorFor(
            new Mecanum(new Mecanum.Constants().setFrontLeftMotor(unusedMotor)
                    .setFrontRightMotor(unusedMotor), null));

    private HolonomicCommandAllocator.Result allocate(double correction, double drive,
                                                       double turn, double velocity) {
        Vector zero = Vector.of(0, 0, DistUnit.IN);
        return allocator.allocate(zero, zero, Vector.of(1, 0, DistUnit.IN),
                Angle.fromRad(0), 0, correction, drive, velocity, 0, turn);
    }

    @Test public void saturatedFeedforwardPreservesTranslationAndBrakingRatio() {
        HolonomicCommandAllocator.Result result = allocate(0.2, 0.8, -0.4, 0.5);
        assertEquals(0.8 * 2 / 3, result.drive.getX().getIn(), 1e-12);
        assertEquals(0.2 - 0.4 * 2 / 3, result.turn, 1e-12);
        assertEquals(1, result.demand.correctionScale, 1e-12);
        assertEquals(2.0 / 3, result.demand.feedforwardScale, 1e-12);
        assertEquals(0, result.demand.velocityScale, 1e-12);
    }

    @Test public void unsaturatedCommandsRetainTheirFullValues() {
        HolonomicCommandAllocator.Result result = allocate(0.1, 0.2, -0.1, 0.15);
        assertEquals(0.35, result.drive.getX().getIn(), 1e-12);
        assertEquals(0, result.turn, 1e-12);
        assertEquals(1, result.demand.feedforwardScale, 1e-12);
        assertEquals(1, result.demand.velocityScale, 1e-12);
    }

    @Test public void correctionCanStillUseTheEntireBudget() {
        HolonomicCommandAllocator.Result result = allocate(2, 0.8, -0.4, 0.5);
        assertEquals(0, result.drive.getX().getIn(), 1e-12);
        assertEquals(1, result.turn, 1e-12);
        assertEquals(0.5, result.demand.correctionScale, 1e-12);
        assertEquals(0, result.demand.feedforwardScale, 1e-12);
    }
    @Test public void independentStrafeFeedforwardSharesTranslationAndTurningBudget() {
        Vector zero = Vector.of(0, 0, DistUnit.IN);
        HolonomicCommandAllocator.Result result = allocator.allocate(zero, zero,
                Vector.of(1, 0, DistUnit.IN), Angle.fromRad(0), 0, .1, .6, 0, 0, .2,
                Vector.of(0, .4, DistUnit.IN));
        assertEquals(.75, result.demand.feedforwardScale, 1e-9);
        assertEquals(.45, result.drive.getX().getIn(), 1e-9);
        assertEquals(.3, result.drive.getY().getIn(), 1e-9);
        assertEquals(.25, result.turn, 1e-9);
    }

    @Test public void signedTangentAndNormalFeedforwardCombineBeforeChargingMotorBudget() {
        Vector zero = Vector.of(0, 0, DistUnit.IN);
        double component = Math.sqrt(.5);
        Vector tangent = Vector.of(component, component, DistUnit.IN);
        Vector normal = Vector.of(-component, component, DistUnit.IN).times(.4);
        HolonomicCommandAllocator.Result result = allocator.allocate(zero, zero, tangent,
                Angle.zero(), 0, 0, .6, 0, 0, .1, zero, normal);
        assertEquals(1, result.demand.feedforwardScale, 0);
        assertEquals(.2 * component, result.drive.getX().getIn(), 1e-12);
        assertEquals(component, result.drive.getY().getIn(), 1e-12);
        assertEquals(1.2 * component + .1, result.demand.feedforwardTotal, 1e-12);
        assertEquals(.4, result.demand.centripetal, 1e-12);
    }
}
