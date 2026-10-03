package follower;

import feedforward.HolonomicFeedforward;

import com.qualcomm.robotcore.util.Range;
import controllers.DriveController;
import controllers.FeedbackController;
import drivetrains.BaseDrivetrain;
import geometry.Angle;
import geometry.PathSegment;
import geometry.Pose;
import geometry.Vector;
import localizers.BaseLocalizer;
import paths.movements.Path;
import trajectory.TrajectorySample;

/** Shared holonomic control for online and stored vector targets. */
final class HolonomicPathFollower {
    private final BaseDrivetrain<?> drivetrain;
    private final DriveController driveController;
    private final FeedbackController headingController;
    private final HolonomicCommandAllocator allocator;
    private final FollowerConstants constants;
    private final double distanceTolerance, headingTolerance;

    HolonomicPathFollower(BaseDrivetrain<?> drivetrain, DriveController driveController,
                          FeedbackController headingController, HolonomicCommandAllocator allocator,
                          FollowerConstants constants, double distanceTolerance, double headingTolerance) {
        this.drivetrain = drivetrain; this.driveController = driveController;
        this.headingController = headingController; this.allocator = allocator;
        this.constants = constants; this.distanceTolerance = distanceTolerance; this.headingTolerance = headingTolerance;
    }

    boolean update(Path path, PathSegment segment, Pose current, BaseLocalizer<?> localizer,
                   MovementCallbackRunner callbacks, FollowerPathState state,
                   boolean headingControllerEnabled, boolean driveControllerEnabled,
                   double translationalKV, double translationalKA, double angularKV, double angularKA,
                   double centripetalGain, double velocityFeedbackGain, double angularVelocityFeedbackGain) {
        Vector position = current.getVec(), velocity = localizer.getVel().getVec();
        Angle heading = current.getHeading();
        TrajectorySample target = state.target(path, position, velocity, localizer.getVel().getHeading().getRad(), System.nanoTime());
        state.t = target.pathT; state.closestPathPoint = target.positionOnPath;
        Vector tangent = target.tangent;
        if (tangent.getMag().getIn() < 1e-9) { tangent = path.getEndPose().getVec().minus(position).normalize(); }
        if (tangent.getMag().getIn() < 1e-9) { tangent = TrajectorySample.vector(1, 0); }
        Vector normal = PathSegment.calculateLeftNormal(tangent);
        Vector endTangent = segment.getFirstDerivative(1).normalize();
        if (endTangent.getMag().getIn() < 1e-9) { endTangent = tangent; }
        double remaining = target.remainingIn;
        double measured = velocity.dot(tangent).getIn();
        double progressVelocity = state.progressEstimator.update(target.progressIn, measured, System.nanoTime());
        double targetVelocity = target.velocity.dot(tangent).getIn();
        double targetAcceleration = target.acceleration.dot(tangent).getIn();
        // Heading follows spatial progress. Preserve the existing rate compensation when
        // translation lags the reference, and publish exactly the angular target consumed.
        double headingScale = target.overspeed ? 1 : MotionFeedforward.timeScale(targetVelocity, progressVelocity);
        target = target.withAngularMotion(target.angularVelocity * headingScale,
                target.angularAcceleration * headingScale * headingScale);
        state.trajectorySample = target;
        double angularVelocity = localizer.getVel().getHeading().getRad();
        double endpointError = Follower.pathEndpointTangentError(path.getEndPose().getVec(), position, endTangent);
        double endpointHeadingError = heading.getShortestAngleTo(path.getEndPose().getHeading()).getRad();
        boolean applyStatic = Math.abs(targetVelocity) < .1 * constants.forwardVelLimitIn;
        callbacks.process(path, segment.getLengthIn() > 1e-9 ? Range.clip(target.progressIn / segment.getLengthIn(), 0, 1) : 1, heading);

        state.crossTrackNormal = normal;
        state.pathNormal = PathSegment.calculateArcNormal(segment.getFirstDerivative(state.t), segment.getSecondDerivative(state.t));
        state.crossTrackError = target.toPath.dot(normal).getIn();
        state.centripetalError = target.toPath.dot(state.pathNormal).getIn();
        driveController.setCrossTrackStaticCompensationEnabled(applyStatic);
        double lateralPower = driveControllerEnabled ? driveController.calculateCrossTrack(state.crossTrackError) : 0;
        if (driveControllerEnabled) {
            lateralPower = Follower.ensureEndpointBreakawayPower(lateralPower, state.crossTrackError,
                    velocity.dot(normal).getIn(), constants.translationalCoeffs.kS,
                    FeedbackController.LINEAR_STATIC_DEADBAND, remaining);
        }
        state.crossTrackCorrection = normal.times(lateralPower);

        double headingError = heading.getShortestAngleTo(target.heading).getRad();
        double headingFeedback = headingControllerEnabled ? Follower.calculatePathHeadingFeedback(
                headingController, headingError, target.angularVelocity, angularVelocity, applyStatic) : 0;
        double headingVelocityFeedback = headingControllerEnabled
                ? angularVelocityFeedbackGain * (target.angularVelocity - angularVelocity) : 0;
        if (remaining < Follower.PROFILED_ENDPOINT_CAPTURE_DISTANCE_INCHES) {
            headingFeedback = Follower.ensureAngularEndpointBreakawayPower(headingFeedback, endpointHeadingError,
                    angularVelocity, applyStatic ? constants.angularCoeffs.kS : 0,
                    PathCompletionPolicy.headingTolerance(headingTolerance));
        }
        double headingFeedforward = HolonomicFeedforward.heading(constants, target.angularVelocity, target.angularAcceleration);

        Vector velocityError = target.velocity.minus(velocity);
        double tangentVelocityFeedback = velocityFeedbackGain * velocityError.dot(tangent).getIn();
        Vector normalVelocityFeedback = normal.times(velocityFeedbackGain * velocityError.dot(normal).getIn());
        double motionSign = MotionFeedforward.profiledStaticSign(targetVelocity, targetAcceleration,
                tangentVelocityFeedback, translationalKV);
        Vector bodyDirection = tangent.rotate(target.heading.times(-1));
        Vector translationPower = HolonomicFeedforward.translation(constants, bodyDirection, targetVelocity,
                targetAcceleration, translationalKV, motionSign).rotate(target.heading);
        double tangentFeedforward = translationPower.dot(tangent).getIn();
        Vector normalFeedforward = translationPower.minus(tangent.times(tangentFeedforward));
        Vector centripetalFeedforward = target.normalAcceleration().times(centripetalGain);
        double centripetalMagnitude = centripetalFeedforward.getMag().getIn();
        if (centripetalMagnitude > Follower.MAX_CENTRIPETAL_POWER) {
            centripetalFeedforward = centripetalFeedforward.times(Follower.MAX_CENTRIPETAL_POWER / centripetalMagnitude);
        }
        state.centripetalCorrection = centripetalFeedforward;

        double endpointPower = driveController.calculateEndDistance(endpointError, -measured);
        endpointPower = Follower.ensureEndpointBreakawayPower(endpointPower, endpointError, measured,
                constants.translationalCoeffs.kS, PathCompletionPolicy.distanceTolerance(distanceTolerance), remaining);
        double blend = Follower.endpointCaptureBlend(remaining);
        double priorTangent = tangentFeedforward + tangentVelocityFeedback;
        double tangentFeedback = endpointPower * blend;
        tangentFeedforward *= 1 - blend; tangentVelocityFeedback *= 1 - blend;
        normalFeedforward = normalFeedforward.times(1 - blend);
        normalVelocityFeedback = normalVelocityFeedback.times(1 - blend);
        centripetalFeedforward = centripetalFeedforward.times(1 - blend);
        if (blend > 0 && priorTangent * measured < 0 && measured > targetVelocity
                && tangentFeedback + tangentFeedforward + tangentVelocityFeedback > priorTangent) {
            tangentFeedback += priorTangent - (tangentFeedback + tangentFeedforward + tangentVelocityFeedback);
        }
        if (tangentFeedforward * tangentVelocityFeedback < 0) {
            double net = tangentFeedforward + tangentVelocityFeedback;
            if (net * tangentFeedforward >= 0) { tangentFeedforward = net; tangentVelocityFeedback = 0; }
            else { tangentFeedforward = 0; tangentVelocityFeedback = net; }
        }
        if (measured * tangentVelocityFeedback < 0) { tangentFeedback += tangentVelocityFeedback; tangentVelocityFeedback = 0; }

        state.trackingVelocityTarget = targetVelocity; state.trackingMeasuredVelocity = measured;
        state.trackingProgressVelocity = progressVelocity; state.trackingAngularVelocityTarget = target.angularVelocity;
        state.trackingFeedforward = translationPower.dot(tangent).getIn();
        state.trackingVelocityFeedback = velocityFeedbackGain * velocityError.dot(tangent).getIn();
        state.trackingEndpointBlend = blend; state.velocitySampleAvailable = true;
        HolonomicCommandAllocator.Result command = allocator.allocate(state.crossTrackCorrection, Vector.zero(), tangent,
                heading, tangentFeedback, headingFeedback, tangentFeedforward, tangentVelocityFeedback,
                headingVelocityFeedback, headingFeedforward, normalFeedforward, centripetalFeedforward, normalVelocityFeedback);
        state.lastCommandDemand = command.demand;
        double endpointDistance = position.distanceTo(path.getEndPose().getVec()).getIn();
        boolean inside = PathCompletionPolicy.isInsideTolerance(endpointDistance, endpointHeadingError, distanceTolerance, headingTolerance);
        boolean settled = PathCompletionPolicy.isVelocitySettled(velocity.getMagSq().getIn(), angularVelocity);
        if (state.completion.isComplete(inside, settled, System.nanoTime())) { return true; }
        if (state.stuckWatchdog.update(Follower.stuckForSpeed(velocity.getMag().getIn()), System.nanoTime())) { return true; }
        drivetrain.drive(command.drive.getX().getIn(), command.drive.getY().getIn(), command.turn);
        return false;
    }
}
