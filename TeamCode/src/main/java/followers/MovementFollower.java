package followers;

import controllers.PDSController;
import drivetrains.Drivetrain;
import followers.constants.BSplineFollowerConstants;
import localizers.Localizer;

import paths.movements.FollowerMovement;
import paths.movements.Path;
import paths.movements.Turn;
import geometry.PathSegment;
import paths.heading.HeadingInterpolator;

import geometry.Angle;
import geometry.Vector;
import geometry.Pose;

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
    public BSplineFollowerConstants getConstants() { return this.constants; }

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

            double targetHeading = turn.getEndPose().getHeading().getRad();
            double currentHeading = current.getHeading().getRad();
            double headingError = getShortestAngularDistance(currentHeading, targetHeading);

            if (Math.abs(headingError) < constants.headingTolerance) {
                this.isBusy = false;
                this.currentMovement = null;
                this.breakFollowing();
                return;
            }

            Vector targetPoseVec = turn.getStartPose().getPos();
            Vector error = targetPoseVec.minus(current.getPos());

            double errorMag = error.getMag().getIn();
            double translationPower = translationController.calculateFromError(errorMag);
            Vector feedback = errorMag > 0 ? error.normalize().times(translationPower) : Vector.zero();

            double turnPower = headingController.calculateFromError(headingError);

            // Pass the calculated feedback instead of 0, 0
            drive(feedback.getX().getIn(), feedback.getY().getIn(), turnPower, currentHeading);

        } else if (currentMovement instanceof Path) {
            Path pathSegmentMove = (Path) currentMovement;
            PathSegment segment = pathSegmentMove.getParametricPath();
            HeadingInterpolator interpolator = pathSegmentMove.getInterpolator();

            if (segment == null || interpolator == null) {
                stop();
                return;
            }

            double t = segment.getBestT(current.getPos());

            Vector targetPoseVec = segment.getPosition(t);
            Vector targetVel = segment.getFirstDerivative(t);

            Vector error = targetPoseVec.minus(current.getPos());

            double errorMag = error.getMag().getIn();
            double translationPower = translationController.calculateFromError(errorMag);
            Vector feedback = errorMag > 0 ? error.normalize().times(translationPower) : Vector.zero();

            Vector feedforward = targetVel.times(constants.velocityFF);
            Vector drivePower = feedback.plus(feedforward);

            double driveX = drivePower.getX().getIn();
            double driveY = drivePower.getY().getIn();

            Angle targetAngle = interpolator.getHeading(t, targetVel);
            double targetHeading = targetAngle.getRad();
            double currentHeading = current.getHeading().getRad();

            double headingError = getShortestAngularDistance(currentHeading, targetHeading);
            double turnPower = headingController.calculateFromError(headingError);

            double distance = segment.getDistanceToEnd_in(targetPoseVec, t);
            if (t >= constants.tTolerance && distance < constants.distanceTolerance) {
                Vector finalPosition = segment.getPosition(1.0);
                this.setTargetPose(new Pose(finalPosition, Angle.fromRad(targetHeading)));
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

        Vector error = targetPose.getPos().minus(currentPose.getPos());
        double errorMag = error.getMag().getIn();
        double headingError = getShortestAngularDistance(currentPose.getHeading().getRad(), targetPose.getHeading().getRad());

        if (errorMag < constants.distanceTolerance && Math.abs(headingError) < constants.headingTolerance) {
            drivetrain.stop();
            return;
        }

        double translationPower = translationController.calculateFromError(errorMag);
        Vector feedback = errorMag > 0 ? error.normalize().times(translationPower) : Vector.zero();

        double turnPower = headingController.calculateFromError(headingError);

        drive(feedback.getX().getIn(), feedback.getY().getIn(), turnPower, currentPose.getHeading().getRad());
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