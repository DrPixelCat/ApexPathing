package follower.internal;

import follower.Follower;
import follower.FollowerConstants;


import com.qualcomm.robotcore.util.Range;

import controllers.DriveController;
import controllers.FeedbackController;
import drivetrains.BaseDrivetrain;
import trajectory.profile.MotionParameters;
import geometry.Angle;
import geometry.PathSegment;
import geometry.Pose;
import geometry.Vector;
import localizers.BaseLocalizer;
import paths.movements.Path;

/**
 * Tracks online and stored vector targets for differential drives using Ramsete feedback.
 *
 * @author Joel H
 */
final class TankPathFollower implements PathExecutor {
    private static final double INCHES_TO_METRES = 0.0254;
    // Conventional Ramsete tuning assumes SI: b has units rad^2/m^2 and zeta has units 1/rad.
    // Short FTC paths need faster lateral convergence than the conventional b=2 default.
    private static final double RAMSETE_B = 8.0;
    private static final double RAMSETE_ZETA = 0.7;

    private final BaseDrivetrain<?> drivetrain;
    private final DriveController driveController;
    private final FollowerConstants constants;
    private final java.util.function.LongSupplier monotonicClock;
    private final double distanceTolerance;
    private final double headingTolerance;
    private final FeedbackController endpointHeadingController;
    private final feedforward.MotionPowerCalculator powerCalculator;
    private final feedforward.MotionPowerCalculator.Output referencePower=new feedforward.MotionPowerCalculator.Output();

    TankPathFollower(BaseDrivetrain<?> drivetrain, DriveController driveController,
                     FollowerConstants constants, double distanceTolerance,
                     double headingTolerance,java.util.function.LongSupplier monotonicClock,drivetrains.models.DriveModel model) {
        this.drivetrain = drivetrain;
        this.driveController = driveController;
        this.constants = constants;
        this.monotonicClock=monotonicClock;
        this.distanceTolerance = distanceTolerance;
        this.headingTolerance = headingTolerance;
        endpointHeadingController = new FeedbackController(constants.angularCoeffs, monotonicClock);
        endpointHeadingController.setAngularController();
        powerCalculator=model.powerCalculator();
    }

    @Override
    public Follower.MovementOutcome update(Path path, PathSegment segment, Pose current, BaseLocalizer<?> localizer,
                   MovementCallbackRunner callbacks, FollowerPathState state,
                   boolean headingEnabled, boolean driveEnabled) {
        double translationalKV = constants.translationalKV;
        double velocityFeedbackGain = constants.velocityFeedbackGain;
        double angularVelocityFeedbackGain = constants.angularVelocityFeedbackGain;

        // region 1. Path Information
        // 1.1 Robot pose and path geometry
        Vector currentPosition = current.getVec();
        Angle currentHeading = current.getHeading();
        Vector robotVelocity = localizer.getVel().getVec();
        trajectory.TrajectorySample sample = state.target(path, currentPosition, robotVelocity, localizer.getVel().getHeading().getRad(), monotonicClock.getAsLong());
        state.t = sample.pathT;
        Vector closestPathPoint = sample.positionOnPath;
        state.closestPathPoint = closestPathPoint;
        double distanceRemaining = sample.remainingIn;
        double pathProgress = segment.getLengthIn() > 1e-9 ? sample.progressIn / segment.getLengthIn() : 1;
        callbacks.process(path, Range.clip(pathProgress, 0.0, 1.0), currentHeading);
        Vector pathDerivative = sample.tangent;
        Angle headingTarget = sample.heading;
        MotionParameters targets = new MotionParameters(sample.tangentialVelocity(), sample.tangentialAcceleration(),
                sample.angularVelocity, sample.angularAcceleration, sample.progressIn);

        // endregion

        // region 2. Algorithm
        // 2.1 Signed motion targets
        double driveSign = Math.cos(headingTarget.getRad() - pathDerivative.getTheta().getRad()) < 0
                ? -1.0 : 1.0;
        double targetVelocity = driveSign * (state.t < 1.0
                ? targets.getTangentialVel() : 0.0);
        double targetAcceleration = driveSign * targets.getTangentialAccel();
        double targetAngularVelocity = targets.getAngularVel();
        double targetAngularAcceleration = targets.getAngularAccel();

        // 2.2 Tracking error in the robot frame
        Vector globalError = sample.toPath;
        Vector tangent = pathDerivative.normalize();
        state.crossTrackError = globalError.getY().getIn() * tangent.getX().getIn()
                - globalError.getX().getIn() * tangent.getY().getIn();
        Vector localError = globalError.rotate(Angle.fromRad(-currentHeading.getRad()));
        double forwardError = localError.getX().getIn();
        double lateralError = localError.getY().getIn();
        double headingError = currentHeading.getShortestAngleTo(headingTarget).getRad();

        // 2.3 Ramsete pose feedback
        double targetVelocityMetres = targetVelocity * INCHES_TO_METRES;
        double forwardErrorMetres = forwardError * INCHES_TO_METRES;
        double lateralErrorMetres = lateralError * INCHES_TO_METRES;
        double k = 2.0 * RAMSETE_ZETA * Math.sqrt(
                targetAngularVelocity * targetAngularVelocity
                        + RAMSETE_B * targetVelocityMetres * targetVelocityMetres);
        double sinc = Math.abs(headingError) < 1e-6
                ? 1.0 : Math.sin(headingError) / headingError;

        // Ramsete returns a linear correction in m/s and an angular correction in rad/s. Convert
        // only the linear correction back to Apex's inch-based feedforward-gain convention.
        double forwardCorrectionInches = k * forwardErrorMetres / INCHES_TO_METRES;
        double correctedVelocity = targetVelocity + (driveEnabled
                ? targetVelocity * (Math.cos(headingError) - 1) + forwardCorrectionInches : 0);
        double correctedAngularVelocity = targetAngularVelocity + (headingEnabled
                ? k * headingError + RAMSETE_B * targetVelocityMetres * sinc * lateralErrorMetres : 0);
        double currentAngularVelocity = localizer.getVel().getHeading().getRad();

        // 2.4 Measured progress and tracking state
        double actualForwardVelocity = robotVelocity
                .rotate(Angle.fromRad(-currentHeading.getRad())).getX().getIn();
        state.trackingVelocityTarget = correctedVelocity;
        state.trackingMeasuredVelocity = actualForwardVelocity;
        state.trackingProgressVelocity = actualForwardVelocity * driveSign;
        state.trackingAngularVelocityTarget = correctedAngularVelocity;
        state.trackingEndpointBlend = 0.0;
        state.velocitySampleAvailable = true;

        // 2.5 Velocity feedback and motion feedforward
        double forwardVelocityFeedback = driveEnabled ? velocityFeedbackGain * (correctedVelocity - actualForwardVelocity) : 0;
        double turnVelocityFeedback = headingEnabled ? angularVelocityFeedbackGain * (correctedAngularVelocity - currentAngularVelocity) : 0;
        // Preserve the generated acceleration in both travel directions. Ramsete pose
        // correction remains in correctedVelocity; velocity feedback uses the same power gain.
        double motionSign = MotionFeedforward.profiledStaticSign(
                correctedVelocity, targetAcceleration, forwardVelocityFeedback, translationalKV);
        powerCalculator.calculate(1,0,correctedVelocity,targetAcceleration,correctedAngularVelocity,targetAngularAcceleration,
                0,false,motionSign,referencePower);
        double forwardFeedforward=referencePower.forward;

        // 2.6 Blend into endpoint capture
        double endError = driveSign * FollowerMath.pathEndpointTangentError(
                path.getEndPose().getVec(), currentPosition,
                segment.getFirstDerivative(1.0).normalize());
        double endpointPower = driveController.calculateEndDistance(
                endError, -actualForwardVelocity);
        endpointPower = FollowerMath.ensureEndpointBreakawayPower(
                endpointPower, endError, actualForwardVelocity,
                constants.translationalCoeffs.kS,
                PathCompletionPolicy.distanceTolerance(distanceTolerance), distanceRemaining);
        double endpointBlend = FollowerMath.endpointCaptureBlend(distanceRemaining);
        if (!driveEnabled) { endpointPower = 0; }
        endpointHeadingController.setCoefficients(constants.angularCoeffs);
        double endpointHeadingError = currentHeading
                .getShortestAngleTo(path.getEndPose().getHeading()).getRad();
        double endpointTurn = endpointHeadingController.p().d(-currentAngularVelocity).s()
                .calculate(endpointHeadingError).get();
        if (!headingEnabled) { endpointTurn = 0; }
        state.trackingEndpointBlend = endpointBlend;
        state.trackingFeedforward = forwardFeedforward;
        state.trackingVelocityFeedback = forwardVelocityFeedback;
        double profilePower = forwardFeedforward + forwardVelocityFeedback;
        forwardFeedforward = forwardFeedforward * (1.0 - endpointBlend)
                + endpointPower * endpointBlend;
        forwardVelocityFeedback *= 1.0 - endpointBlend;
        forwardFeedforward += forwardVelocityFeedback;
        // Endpoint capture must not dilute an
        // already stronger braking command while the robot is above its speed target.
        if (endpointBlend > 0.0 && profilePower * actualForwardVelocity < 0.0
                && driveSign * actualForwardVelocity > driveSign * correctedVelocity) {
            forwardFeedforward = driveSign * Math.min(driveSign * forwardFeedforward,
                    driveSign * profilePower);
        }

        // 2.7 Angular feedforward for Ramsete's corrected chassis speed
        double turnFeedforward=referencePower.heading;

        // 2.8 Completion and stuck detection
        double endpointDistance = currentPosition.distanceTo(path.getEndPose().getVec()).getIn();
        Follower.MovementOutcome outcome = state.completionOutcome(endpointDistance, endpointHeadingError,
                robotVelocity, currentAngularVelocity, distanceTolerance, headingTolerance, monotonicClock.getAsLong());
        if (outcome != Follower.MovementOutcome.RUNNING) return outcome;

        // endregion

        // region 3. Send powers
        double turnPower = (turnFeedforward + turnVelocityFeedback) * (1.0 - endpointBlend)
                + endpointTurn * endpointBlend;
        double powerLimit = drivetrain.getConstants().maxPower;
        // Preserve steering authority under saturation. Common wheel normalization would
        // reduce the requested turn whenever translation consumes the shared power budget.
        // For a tank, max(|forward-turn|, |forward+turn|) = |forward| + |turn|.
        turnPower = Range.clip(turnPower, -powerLimit, powerLimit);
        double forwardBudget = Math.max(0.0, powerLimit - Math.abs(turnPower));
        drivetrain.drive(Range.clip(forwardFeedforward, -forwardBudget, forwardBudget),
                0.0, turnPower);
        // endregion

        return Follower.MovementOutcome.RUNNING;
    }
}
