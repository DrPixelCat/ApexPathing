package followers;

import controllers.PDSController;
import drivetrains.Drivetrain;
import geometry.Angle;
import localizers.Localizer;
import followers.constants.P2PFollowerConstants;

import geometry.Pose;
import geometry.Vector;
import util.DistUnit;

/**
 * Simple point-to-point follower
 * @author Sohum Arora 22985 Paraducks
 * @author Dylan B. 18597 RoboClovers - Delta
 */
public class P2PFollower extends Follower {
    private final P2PFollowerConstants constants;

    private final PDSController axialController;
    private final PDSController strafeController;
    private final PDSController headingController;

    /**
     * Constructor for the P2PFollower
     * @param drivetrain the mecanum drivetrain class to control
     * @param localizer the Pinpoint localizer to get pose estimates from
     */
    public P2PFollower(P2PFollowerConstants constants, Drivetrain drivetrain, Localizer localizer) {
        super(drivetrain, localizer);
        this.constants = constants;
        this.axialController = constants.axialController;
        this.strafeController = constants.strafeController;
        this.headingController = constants.headingController;
    }

    /**
     * Set the target pose for the robot to move to
     * @param targetPose the new target pose
     */
    public void setTargetPose(Pose targetPose) {
        // Use the unexposed method from the Follower class (converts target pose to inches and radians)
        super.setTargetPose(targetPose);
        this.axialController.reset();
        this.axialController.setTarget(this.targetPose.getX().getIn());
        this.strafeController.reset();
        this.strafeController.setTarget(this.targetPose.getY().getIn());
        this.headingController.reset();
        this.headingController.setTarget(this.targetPose.getHeading().getRad());
    }

    public boolean axialAtTarget() { return constants.axialController.isAtTarget(); }
    public boolean strafeAtTarget() { return constants.strafeController.isAtTarget(); }
    public boolean headingAtTarget() { return constants.headingController.isAtTarget(); }

    @Override
    public void update() {
        localizer.update();
        Pose pose = localizer.getPose();
        double currentX = pose.getX().getIn();
        double currentY = pose.getY().getIn();
        double currentHeading = pose.getHeading().getRad();

        if (!isBusy) {
            return; // No need to calculate anything if we're not busy
        }

        if (axialController.isAtTarget() && strafeController.isAtTarget() && headingController.isAtTarget()) {
            if (!this.holdingPose) {
                isBusy = false;
                drivetrain.stop();
                return;
            }
        }

        // Rotate backwards to convert from field to robot centric (CCW rotation = positive)
        Vector translational = Vector.of(
                axialController.calculate(currentX),
                strafeController.calculate(currentY),
                DistUnit.IN // Not actually inches, but the values will stay the same with inches
        ).rotate(Angle.fromRad(-currentHeading));
        double turn = headingController.calculate(currentHeading);
        drivetrain.drive(translational.getX().getIn(), translational.getY().getIn(), turn);
    }
}