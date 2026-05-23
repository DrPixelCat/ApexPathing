package followers;

import controllers.PDFLController;
import drivetrains.Drivetrain;
import localizers.Localizer;
import followers.constants.P2PFollowerConstants;

import util.Pose;
import util.Vector;

/**
 * Simple point-to-point follower
 * @author Sohum Arora 22985 Paraducks
 * @author Dylan B. 18597 RoboClovers - Delta
 */
public class P2PFollower extends Follower {
    private final P2PFollowerConstants constants;

    private final PDFLController axialController;
    private final PDFLController strafeController;
    private final PDFLController headingController;

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
        this.axialController.setTarget(targetPose.getX());
        this.strafeController.reset();
        this.strafeController.setTarget(targetPose.getY());
        this.headingController.reset();
        this.headingController.setTarget(targetPose.getHeading());
    }

    public boolean axialAtTarget() { return constants.axialController.isAtTarget(); }

    public boolean strafeAtTarget() { return constants.strafeController.isAtTarget(); }

    public boolean headingAtTarget() { return constants.headingController.isAtTarget(); }

    @Override
    public void update() {
        localizer.update();
        Pose pose = localizer.getPose();
        double currentX = pose.getXComponent().getIn();
        double currentY = pose.getYComponent().getIn();
        double currentHeading = pose.getHeadingComponent().getRad();

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

        // Note: powers are clipped to max powers defined in constants
        Vector translational = new Vector(
                axialController.calculate(currentX),
                -strafeController.calculate(currentY)
        ).rotated(-currentHeading); // Rotate to the robot's frame of reference
        double turn = -headingController.calculate(currentHeading);

        drivetrain.drive(-translational.getX(), translational.getY(), turn);
    }
}