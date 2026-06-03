package followers;

import com.qualcomm.robotcore.util.Range;

import controllers.PDFLController;
import controllers.PDSController;
import drivetrains.BaseDrivetrain;
import followers.constants.BSplineFollowerConstants;
import localizers.BaseLocalizer;

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
    private final BSplineFollowerConstants constants;

    // PDS Controllers for closed-loop feedback
    private final PDSController lateralController;
    private final PDSController velocityController;
    private final PDSController driveController;
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
    public MovementFollower(BSplineFollowerConstants constants, BaseDrivetrain<?> drivetrain, BaseLocalizer<?> localizer) {
        super(drivetrain, localizer);
        this.constants = constants;

        // Initialize controllers with PDS coefficients from constants
        this.driveController = new PDSController(constants.driveCoeffs);
        this.lateralController = new PDSController(constants.translationCoeffs);
        this.headingController = new PDSController(constants.headingCoeffs);

        // Mark heading controller as angular so it handles angle normalization
        this.headingController.setAngularController();
        this.velocityController = new PDSController(constants.velocityCoeffs);
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
        lateralController.reset();
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

            Angle targetHeading = turn.getEndPose().getHeading();
            Angle currentHeading = current.getHeading();
            double headingError = targetHeading.getShortestAngularDifferenceTo(currentHeading).getRad();

            if (Math.abs(headingError) < constants.headingTolerance) {
                this.isBusy = false;
                this.currentMovement = null;
                this.breakFollowing();
                return;
            }

            Vector targetPoseVec = turn.getStartPose().getPos();
            Vector error = targetPoseVec.minus(current.getPos());

            double errorMag = error.getMag().getIn();
            double translationPower = lateralController.calculateFromError(errorMag);
            Vector feedback = errorMag > 0 ? error.normalize().times(translationPower) : Vector.zero();

            double turnPower = headingController.calculateFromError(headingError);

            // Pass the calculated feedback instead of 0, 0
            drive(feedback.getX().getIn(), feedback.getY().getIn(), turnPower, currentHeading.getRad());

            // region Curve movement

        } else if (currentMovement instanceof Path) {
            Path pathSegmentMove = (Path) currentMovement;
            PathSegment segment = pathSegmentMove.getParametricPath();
            HeadingInterpolator interpolator = pathSegmentMove.getInterpolator();

            if (segment == null || interpolator == null) {
                stop();
                return;
            }

            // region 1. READS: Extract all state, geometry, and error values
            double t = segment.getBestT(current.getPos());
            Angle currentHeading = current.getHeading();

            Vector robotVelocity = localizer.getVel().getPos();
            Vector targetPoseVec = segment.getPosition(t);

            // Raw derivatives needed ONLY for calculating the radius of curvature
            Vector velVec = segment.getFirstDerivative(t);
            Vector accelVec = segment.getSecondDerivative(t);

            Vector normal = PathSegment.calculateArcNormal(velVec, accelVec);

            // The normalized tangent dictates pure geometric direction, independent of speed
            Vector unitTangent = velVec.normalize();

            double distanceRemaining = segment.getDistanceToEnd_in(targetPoseVec, t);
            double distanceTravelled = segment.getLength_in() - distanceRemaining;
            double pathProgression = distanceTravelled / segment.getLength_in();

            Angle targetHeading = interpolator.getHeading(pathProgression, velVec);

            Vector fieldError = targetPoseVec.minus(current.getPos());
            double headingError = targetHeading.getShortestAngularDifferenceTo(currentHeading).getRad();

            // region 2. CALCULATIONS: Apply physics, scaling, and PID math
            double availableMotorPower = 1.0;

            //  region --- 1. Turn Power Allocation (Highest Priority) ---
            double turnPower = headingController.calculateFromError(headingError);
            availableMotorPower -= Math.abs(turnPower);

            // region --- 2. Combined Lateral Power (Centripetal + Feedback) ---
            double radius = PathSegment.calculateRadiusOfCurvature(velVec, accelVec);
            double robotVelMag = robotVelocity.getMag().getIn();

            // Required inward acceleration based on ACTUAL current speed
            double requiredLateralAccel = (robotVelMag * robotVelMag) / radius;
            double centripetalMag = requiredLateralAccel / constants.getMaxLateralAccel();

            // Project field error onto the normal vector to isolate lateral drift
            double crossTrackError = fieldError.dot(normal).getIn();
            double lateralFeedbackMag = lateralController.calculateFromError(crossTrackError);

            // Mathematically combine centripetal feedforward and corrective feedback
            double netLateralMag = centripetalMag + lateralFeedbackMag;
            netLateralMag = Range.clip(netLateralMag, -availableMotorPower, availableMotorPower);

            Vector lateralDriveVec = normal.times(netLateralMag);
            availableMotorPower -= Math.abs(netLateralMag);

            // region --- 3. Tangent/Along-Track Power Allocation ---
            double desiredVelocity = constants.velocityLimit.getIn();

            // Cap the desired speed if the upcoming curve is too sharp
            if (radius != Double.POSITIVE_INFINITY) {
                double maxSafeVelocity = Math.sqrt(constants.getMaxLateralAccel() * radius);
                if (desiredVelocity > maxSafeVelocity) {
                    desiredVelocity = maxSafeVelocity;
                }
            }

            // Project field error onto the tangent vector to isolate forward/backward error
            double alongTrackError = fieldError.dot(unitTangent).getIn();

            // Calculate deceleration if current speed exceeds the safe requested speed
            double requiredAccel = 0.0;
            if (desiredVelocity < robotVelMag) {
                requiredAccel = desiredVelocity - robotVelMag; // Yields a negative value for braking
            }

            // Feedforward: kV scales the target speed, kA (with negative accel) pulls power back to brake
            double feedforward = (constants.kV * desiredVelocity) + (constants.kA * requiredAccel) + driveController.getCoefficients().kS;

            double tangentFeedbackMag;
            if (t < 1.0) {
                tangentFeedbackMag = driveController.calculateFromError(alongTrackError);
            } else {
                // Infinite line fallback for end of path
                Vector endPos = segment.getPosition(1.0);
                Vector endTangent = segment.getFirstDerivative(1.0).normalize();
                Vector endToRobot = current.getPos().minus(endPos);

                double distancePastEnd = endToRobot.dot(endTangent).getIn();
                feedforward = 0.0;
                tangentFeedbackMag = driveController.calculateFromError(-distancePastEnd);
            }

            // Calculate the velocity feedback and safely append the Feedforward and kS
            double velocityFeedback = velocityController.calculateFromError(desiredVelocity - robotVelMag)
                    + feedforward
                    + driveController.getCoefficients().kS;

            double positionFeedback = tangentFeedbackMag;

            // Cap the spatial position request with the velocity ceiling
            double totalTangentPower = Math.min(velocityFeedback, positionFeedback);
            totalTangentPower = Range.clip(totalTangentPower, -availableMotorPower, availableMotorPower);

            // Apply the power scalar to the purely directional unit tangent
            Vector tangentDriveVec = unitTangent.times(totalTangentPower);

            // --- Final Blending ---
            Vector driveOutput = tangentDriveVec.plus(lateralDriveVec);

            double driveX = driveOutput.getX().getIn();
            double driveY = driveOutput.getY().getIn();

            // region 3. WRITES: Actuate motors or advance state machine
            if (t >= constants.tTolerance && distanceRemaining < constants.distanceTolerance) {
                Vector finalPosition = currentMovement.getEndPose().getPos();
                this.setTargetPose(new Pose(finalPosition, targetHeading));
                this.holdingPose = true;
                this.isBusy = false;
                this.currentMovement = null;
                this.breakFollowing();
                return;
            }

            drive(driveX, driveY, turnPower, currentHeading.getRad());
        }
    }

    private void holdPose() {
        Pose currentPose = getPose();

        Vector error = targetPose.getPos().minus(currentPose.getPos());
        double errorMag = error.getMag().getIn();
        double headingError =  targetPose.getHeading().getShortestAngularDifferenceTo(currentPose.getHeading()).getRad();

        if (errorMag < constants.distanceTolerance && Math.abs(headingError) < constants.headingTolerance) {
            drivetrain.stop();
            return;
        }
        double translationPower = lateralController.calculateFromError(errorMag);
        Vector feedback = errorMag > 0 ? error.normalize().times(translationPower) : Vector.zero();

        double turnPower = headingController.calculateFromError(headingError);

        drive(feedback.getX().getIn(), feedback.getY().getIn(), turnPower, currentPose.getHeading().getRad());
    }
    private double calculateNewAvailablePower(double availablePower, double newRequest) {
        return Range.clip(availablePower - Math.abs(newRequest), 0.0, 1.0);
    }

    @Override
    public void stop() {
        super.stop();
        this.holdTimerInitialized = false;
        this.wasHoldingPosePrevFrame = false;
        this.currentMovement = null;
    }
}