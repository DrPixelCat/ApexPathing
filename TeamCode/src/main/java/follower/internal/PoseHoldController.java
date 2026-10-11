package follower.internal;


import controllers.DriveController;
import controllers.FeedbackController;
import drivetrains.BaseDrivetrain;
import geometry.AngleUnit;
import geometry.Dist;
import geometry.DistUnit;
import geometry.Pose;
import geometry.Vector;

/** Shared pose-holding math; the concrete follower selects the active kinematics. */
final class PoseHoldController {
    private final BaseDrivetrain<?> drivetrain;
    private final DriveController driveController;
    private final FeedbackController headingController;

    PoseHoldController(BaseDrivetrain<?> drivetrain, DriveController driveController,
                       FeedbackController headingController) {
        this.drivetrain = drivetrain;
        this.driveController = driveController;
        this.headingController = headingController;
    }

    void update(Pose target, Pose current, boolean holonomic,
                boolean headingEnabled, boolean driveEnabled) {
        double angularResponse = headingEnabled ? headingController.p().d().s().calculate(
                target.getHeading(AngleUnit.RAD) - current.getHeading(AngleUnit.RAD)).get() : 0.0;
        Vector translationalResponse;
        if (!driveEnabled) {
            translationalResponse = Vector.zero();
        } else if (holonomic) {
            translationalResponse = driveController.calculatePointToPoint(
                    target.getVec(), current.getVec());
        } else {
            Vector globalError = target.getVec().minus(current.getVec());
            Vector localError = globalError.rotate(current.getHeading().times(-1.0));
            translationalResponse = new Vector(Dist.fromIn(driveController.calculateEndDistance(
                    localError.getX(DistUnit.IN))), Dist.zero());
        }
        if (holonomic) {
            translationalResponse = translationalResponse.rotate(current.getHeading().times(-1));
        }
        drivetrain.moveWithVectors(translationalResponse.getX().getIn(),
                translationalResponse.getY().getIn(), angularResponse);
    }
}
