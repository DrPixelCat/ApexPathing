package followers;

import controllers.PDSController;
import drivetrains.Drivetrain;
import followers.constants.BSplineFollowerConstants;
import localizers.Localizer;

// New Architecture Imports
import paths.movements.FollowerMovement;
import paths.movements.Path;
import paths.movements.Turn;
import paths.geometry.PathSegment;
import paths.heading.HeadingInterpolator;

import util.Angle;
import util.Pose;
import util.Vector;

/**
 * MovementFollower class, capable of following FollowerMovements made with Builders
 * Important: Ensure your BSplineFollower constants are fully configured
 * before attempting to use this follower {@link BSplineFollowerConstants}
 * @author Sohum Arora 22985 Paraducks
 */
public class MovementFollower extends Follower {
    private static final double pi2 = 2 * Math.PI;
    private final BSplineFollowerConstants constants;

    // PDS Controllers for closed-loop feedback
    private final PDSController translationController;
    private final PDSController headingController;

    // Architecture Change: Singular active movement. Throws exception if overridden prematurely.
    private FollowerMovement currentMovement = null;

    private long holdStartTimeNs = 0;
    private boolean holdTimerInitialized = false;
    private long pauseStartNs = 0;
    private boolean wasHoldingPosePrevFrame = false;

    /**
     * MovementFollower constructor
     * @param constants - Your BSplineFollowerConstants (ensure configured)
     */
    public MovementFollower(BSplineFollowerConstants constants, Drivetrain drivetrain, Localizer localizer) {
        super(drivetrain, localizer);
        this.constants = constants;

        // Initialize controllers with PDS coefficients from constants
        this.translationController = new PDSController(constants.translationCoeffs);
        this.headingController = new PDSController(constants.headingCoeffs);

        // Mark heading controller as angular so it handles angle normalization
        this.headingController.setAngularController();
    }

    /**
     * Retrieves the active constants instance driving this follower.
     * Useful for live tuning via dashboards.
     */
    public BSplineFollowerConstants getConstants() {
        return this.constants;
    }

    /**
     * Sets the movement to be followed.
     * @param movement is the movement to be followed
     * @throws IllegalStateException if the follower is already busy executing a movement.
     */
    public void follow(FollowerMovement movement) {
        if (this.isBusy || this.currentMovement != null) {
            throw new IllegalStateException("Cannot follow a new movement while the follower is currently busy! Check !follower.isBusy() in your state machine before calling follow().");
        }

        // Standard call: Start executing immediately
        this.currentMovement = movement;
        this.isBusy = true;
        this.holdingPose = false;
        this.holdTimerInitialized = false;
        this.wasHoldingPosePrevFrame = false;

        // Reset controllers right before starting a new path to prevent derivative kick
        translationController.reset();
        headingController.reset();
    }

    @Override
    public void update() {
        if (holdingPose && targetPose != null) {
            if (!wasHoldingPosePrevFrame) {
                pauseStartNs = System.nanoTime();
                wasHoldingPosePrevFrame = true;
            }
            holdPose();
            return;
        }

        if (wasHoldingPosePrevFrame) {
            long pauseDurationNs = System.nanoTime() - pauseStartNs;
            if (holdTimerInitialized && holdStartTimeNs > 0) {
                holdStartTimeNs += pauseDurationNs;
            }
            wasHoldingPosePrevFrame = false;
        }

        if (!isBusy || currentMovement == null) {
            drivetrain.stop();
            return;
        }

        Pose current = getPose();

        // Turn logic
        if (currentMovement instanceof Turn) {
            Turn turn = (Turn) currentMovement;

            double targetHeading = turn.getEndPose().getHeading();
            double currentHeading = current.getHeading();
            double headingError = getShortestAngularDistance(currentHeading, targetHeading);

            if (Math.abs(headingError) < constants.headingTolerance) {
                this.isBusy = false;
                this.currentMovement = null;
                this.breakFollowing();
                return;
            }

            Vector targetPoseVec = turn.getStartPose().toVec();
            Vector error = targetPoseVec.subtract(current.toVec());

            double errorMag = error.getMagnitude();
            double translationPower = translationController.calculateFromError(errorMag);
            Vector feedback = errorMag > 0 ? error.normalize().multiply(translationPower) : new Vector(0, 0);

            double turnPower = headingController.calculateFromError(headingError);

            // Pass the calculated feedback instead of 0, 0
            drive(feedback.getX(), feedback.getY(), turnPower, currentHeading);

        } else if (currentMovement instanceof Path) {
            Path pathSegmentMove = (Path) currentMovement;
            PathSegment segment = pathSegmentMove.getParametricPath();
            HeadingInterpolator interpolator = pathSegmentMove.getInterpolator();

            if (segment == null || interpolator == null) {
                stop();
                return;
            }

            double t = segment.getBestT(current.toVec());

            Vector targetPoseVec = segment.getPosition(t);
            Vector targetVel = segment.getFirstDerivative(t);

            Vector error = targetPoseVec.subtract(current.toVec());

            double errorMag = error.getMagnitude();
            double translationPower = translationController.calculateFromError(errorMag);
            Vector feedback = errorMag > 0 ? error.normalize().multiply(translationPower) : new Vector(0, 0);

            Vector feedforward = targetVel.multiply(constants.velocityFF);
            Vector drivePower = feedback.add(feedforward);

            double driveX = drivePower.getX();
            double driveY = drivePower.getY();

            Angle targetAngle = interpolator.getHeading(t, targetVel);
            double targetHeading = targetAngle.getRad();
            double currentHeading = current.getHeading();

            double headingError = getShortestAngularDistance(currentHeading, targetHeading);
            double turnPower = headingController.calculateFromError(headingError);

            double distance = segment.getDistanceToEnd_in(targetPoseVec, t);
            if (t >= constants.tTolerance && distance < constants.distanceTolerance) {
                Vector finalPosition = segment.getPosition(1.0);
                this.setTargetPose(new Pose(finalPosition.getX(), finalPosition.getY(), targetHeading));
                this.holdingPose = true;
                this.isBusy = false;
                this.currentMovement = null;
                this.breakFollowing();
                return;
            }

            drive(driveX, driveY, turnPower, currentHeading);
        }
    }

    private void holdPose() {
        Pose currentPose = getPose();

        Vector error = targetPose.toVec().subtract(currentPose.toVec());
        double errorMag = error.getMagnitude();
        double headingError = getShortestAngularDistance(currentPose.getHeading(), targetPose.getHeading());

        if (errorMag < constants.distanceTolerance && Math.abs(headingError) < constants.headingTolerance) {
            drivetrain.stop();
            return;
        }

        double translationPower = translationController.calculateFromError(errorMag);
        Vector feedback = errorMag > 0 ? error.normalize().multiply(translationPower) : new Vector(0, 0);

        double turnPower = headingController.calculateFromError(headingError);

        drive(feedback.getX(), feedback.getY(), turnPower, currentPose.getHeading());
    }

    private double getShortestAngularDistance(double currentRad, double targetRad) {
        double diff = (targetRad - currentRad) % (pi2);
        if (diff > Math.PI) diff -= pi2;
        else if (diff < -Math.PI) diff += pi2;
        return diff;
    }

    @Override
    public void stop() {
        super.stop();
        this.holdTimerInitialized = false;
        this.wasHoldingPosePrevFrame = false;
        this.currentMovement = null;
    }
}