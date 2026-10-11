package follower.internal;

import follower.Follower;
import follower.FollowerConstants;

import feedforward.MotionPowerCalculator;
import feedforward.MecanumWheelModel;

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

/**
 * Shared holonomic control for online and stored vector targets.
 *
 * @author Joel H
 */
final class HolonomicPathFollower implements PathExecutor {
    private final BaseDrivetrain<?> drivetrain;
    private final DriveController driveController;
    private final FeedbackController headingController;
    private final HolonomicCommandAllocator allocator;
    private final FollowerConstants constants;
    private final drivetrains.models.DriveModel driveModel;
    private final MotionPowerCalculator powerCalculator;
    private final MotionPowerCalculator.Output referencePower=new MotionPowerCalculator.Output();
    private final double distanceTolerance, headingTolerance;
    private final java.util.function.LongSupplier monotonicClock;
    private feedforward.CenteredBrakingFeedforward brakingFeedforward;
    private final MecanumWheelFeedforward wheelFeedforward = new MecanumWheelFeedforward();

    void setBrakingFeedforward(feedforward.CenteredBrakingFeedforward model) {
        brakingFeedforward = model;
        resetBrakingFeedforward();
    }

    void resetBrakingFeedforward() {
        if (brakingFeedforward != null) { brakingFeedforward.reset(); }
        wheelFeedforward.reset();
    }

    HolonomicPathFollower(BaseDrivetrain<?> drivetrain, DriveController driveController,
                          FeedbackController headingController, HolonomicCommandAllocator allocator,
                          FollowerConstants constants, double distanceTolerance, double headingTolerance,
                          java.util.function.LongSupplier monotonicClock,drivetrains.models.DriveModel model) {
        this.drivetrain = drivetrain; this.driveController = driveController;
        this.headingController = headingController; this.allocator = allocator;
        this.constants = constants; this.distanceTolerance = distanceTolerance; this.headingTolerance = headingTolerance;
        this.monotonicClock=monotonicClock;
        driveModel=model; powerCalculator=model.powerCalculator();
    }

    @Override
    public Follower.MovementOutcome update(Path path, PathSegment segment, Pose current, BaseLocalizer<?> localizer,
                   MovementCallbackRunner callbacks, FollowerPathState state,
                   boolean headingControllerEnabled, boolean driveControllerEnabled) {
        double translationalKV = constants.translationalKV, translationalKA = constants.translationalKA;
        double velocityFeedbackGain = constants.velocityFeedbackGain;
        double angularVelocityFeedbackGain = constants.angularVelocityFeedbackGain;
        Vector position = current.getVec(), velocity = localizer.getVel().getVec();
        long now=monotonicClock.getAsLong();
        Angle heading = current.getHeading();
        MecanumWheelModel wheelModel = driveModel.wheelModel();
        boolean independentReference = wheelModel != null && !driveControllerEnabled && !headingControllerEnabled;
        boolean feedbackEnabled = driveControllerEnabled || headingControllerEnabled;
        TrajectorySample target = state.target(path, position, velocity, localizer.getVel().getHeading().getRad(), now,
                independentReference);
        state.t = target.pathT; state.closestPathPoint = target.positionOnPath;
        Vector tangent = trackingTangent(target, path, position);
        Vector normal = PathSegment.calculateLeftNormal(tangent);
        Vector endTangent = segment.getFirstDerivative(1).normalize();
        if (endTangent.getMag().getIn() < 1e-9) { endTangent = tangent; }
        double remaining = target.remainingIn;
        double measured = velocity.dot(tangent).getIn();
        double progressVelocity = state.progressEstimator.update(target.progressIn, measured, now);
        double targetVelocity = target.velocity.dot(tangent).getIn();
        double targetAcceleration = target.acceleration.dot(tangent).getIn();
        // Heading follows spatial progress. With feedback disabled, consume the planned
        // angular demand directly rather than compensating for measured translation lag.
        double headingScale = !driveControllerEnabled || target.overspeed ? 1
                : MotionFeedforward.timeScale(targetVelocity, progressVelocity);
        target = target.withAngularMotion(target.angularVelocity * headingScale,
                target.angularAcceleration * headingScale * headingScale);
        state.trajectorySample = target;
        double angularVelocity = localizer.getVel().getHeading().getRad();
        double endpointError = FollowerMath.pathEndpointTangentError(path.getEndPose().getVec(), position, endTangent);
        double endpointHeadingError = heading.getShortestAngleTo(path.getEndPose().getHeading()).getRad();
        boolean applyStatic = Math.abs(targetVelocity) < .1 * constants.forwardVelLimitIn;
        callbacks.process(path, segment.getLengthIn() > 1e-9 ? Range.clip(target.progressIn / segment.getLengthIn(), 0, 1) : 1, heading);

        updateCrossTrackCorrection(target, segment, velocity, normal, applyStatic, driveControllerEnabled, state);
        double headingFeedback = calculateHeadingFeedback(target, heading, angularVelocity,
                endpointHeadingError, applyStatic, headingControllerEnabled);
        double headingVelocityFeedback = headingControllerEnabled
                ? angularVelocityFeedbackGain * (target.angularVelocity - angularVelocity) : 0;
        state.wheelFeedforwardActive = wheelModel != null;

        double activeVelocityGain = driveControllerEnabled ? velocityFeedbackGain : 0;
        double feedbackTargetVelocity = targetVelocity;
        double accelerationPerPower = wheelModel != null ? wheelModel.gain
                : translationalKA > 0 ? 1 / translationalKA : 0;
        boolean velocityPursuit=driveControllerEnabled && state.onlinePlanner!=null
                && state.onlinePlanner.isVelocityPursuitEnabled();
        if (velocityPursuit && Double.isFinite(state.onlinePlanner.getVelocityGoal()))
            feedbackTargetVelocity = state.onlinePlanner.getVelocityGoal();
        boolean onlineVelocityReference = driveControllerEnabled && path.getBuildMode() == Path.BuildMode.ONLINE && !velocityPursuit;
        if (onlineVelocityReference) {
            // Normal online following applies P feedback to the current sensor-time velocity error.
            feedbackTargetVelocity = state.velocityReference.update(now,
                    localizer.isLastMeasurementValid() ? localizer.getLastMeasurementTimeNanos() : -1,
                    measured, targetAcceleration);
            activeVelocityGain = state.velocityReference.powerGain(activeVelocityGain, accelerationPerPower);
        } else state.velocityReference.reset();
        Vector velocityError = target.velocity.minus(velocity)
                .plus(tangent.times(feedbackTargetVelocity - targetVelocity));
        double rawVelocityError = velocityError.dot(tangent).getIn();
        double tangentVelocityFeedback = velocityPursuit ? 0 : activeVelocityGain * rawVelocityError;
        Vector normalVelocityFeedback = normal.times(activeVelocityGain * velocityError.dot(normal).getIn());
        double motionSign = MotionFeedforward.profiledStaticSign(targetVelocity, targetAcceleration,
                tangentVelocityFeedback, translationalKV);
        Vector bodyDirection = tangent.rotate(target.heading.times(-1));
        powerCalculator.calculate(bodyDirection.getX().getIn(),bodyDirection.getY().getIn(),targetVelocity,targetAcceleration,
                target.angularVelocity,target.angularAcceleration,target.normalAcceleration().dot(normal).getIn(),false,motionSign,referencePower);
        Vector bodyPower=referencePower.translation();
        double headingFeedforward=referencePower.heading;
        feedforward.CenteredBrakingFeedforward.Result braking = brakingFeedforward == null || wheelModel != null ? null
                : brakingFeedforward.calculate(path, target, bodyDirection, bodyPower);
        state.centeredBrakingActive = braking != null;
        state.centeredBrakingMeanPower = braking == null ? 0
                : braking.meanBodyPower.rotate(target.heading).dot(tangent).getIn();
        if (braking != null) { bodyPower = braking.pulseBodyPower; }
        Vector translationPower = bodyPower.rotate(target.heading);
        double tangentFeedforward = translationPower.dot(tangent).getIn();
        Vector normalFeedforward = translationPower.minus(tangent.times(tangentFeedforward));
        Vector centripetalFeedforward=referencePower.normal().rotate(target.heading);
        if (braking != null) {
            // Publish the new unallocated pulse prediction, including signed curvature
            // and heading demand, before feedback, endpoint blending or saturation.
            target = target.withExpectedMotorPower(driveModel.commandDemand(
                    bodyPower.plus(centripetalFeedforward.rotate(target.heading.times(-1))), headingFeedforward));
            state.trajectorySample = target;
        }
        state.centripetalCorrection = centripetalFeedforward;
        if (wheelModel != null) {
            // Use the measured heading only to transform the planned field motion into
            // wheel coordinates. No pose or velocity error is added to this demand.
            double lead = wheelModel.commandPreviewSeconds;
            TrajectorySample forceReference=independentReference?state.previewReference(path,position,lead):target;
            if (!wheelFeedforward.prepare(wheelModel, target, forceReference, heading,
                    localizer.getRawVel().getVec(), localizer.getRawVel().getHeading().getRad(),
                    independentReference, feedbackEnabled)) {
                wheelModel = null;
                state.wheelFeedforwardActive = false;
            } else {
                double[] wheelPower = wheelFeedforward.powers();
                double x=(wheelPower[0]+wheelPower[1]+wheelPower[2]+wheelPower[3])/4;
                double y=(-wheelPower[0]+wheelPower[1]+wheelPower[2]-wheelPower[3])/4;
                headingFeedforward=(-wheelPower[0]+wheelPower[1]-wheelPower[2]+wheelPower[3])/4;
                translationPower=TrajectorySample.vector(x,y).rotate(heading);
                tangentFeedforward=translationPower.dot(tangent).getIn();
                normalFeedforward=translationPower.minus(tangent.times(tangentFeedforward));
                centripetalFeedforward=Vector.zero(); state.centripetalCorrection=Vector.zero();
                double peak=0;for(double power:wheelPower) peak=Math.max(peak,Math.abs(power));
                target=target.withExpectedMotorPower(peak);state.trajectorySample=target;
            }
        }
        state.trackingHeadingFeedforward=headingFeedforward;

        double endpointPower = driveControllerEnabled ? driveController.calculateEndDistance(endpointError, -measured) : 0;
        if (driveControllerEnabled) { endpointPower = FollowerMath.ensureEndpointBreakawayPower(endpointPower, endpointError, measured,
                constants.translationalCoeffs.kS, PathCompletionPolicy.distanceTolerance(distanceTolerance), remaining);
        }
        double blend = driveControllerEnabled
                ? FollowerMath.endpointCaptureBlend(remaining) : 0;
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
        if (wheelModel == null && tangentFeedforward * tangentVelocityFeedback < 0) {
            double net = tangentFeedforward + tangentVelocityFeedback;
            if (net * tangentFeedforward >= 0) { tangentFeedforward = net; tangentVelocityFeedback = 0; }
            else { tangentFeedforward = 0; tangentVelocityFeedback = net; }
        }
        boolean velocityBraking=measured*tangentVelocityFeedback<0;
        if (velocityBraking) { tangentFeedback += tangentVelocityFeedback; tangentVelocityFeedback = 0; }

        state.trackingVelocityTarget = feedbackTargetVelocity; state.trackingMeasuredVelocity = measured;
        state.trackingProgressVelocity = progressVelocity; state.trackingAngularVelocityTarget = target.angularVelocity;
        state.trackingFeedforward = translationPower.dot(tangent).getIn();
        state.trackingVelocityFeedback = velocityPursuit ? 0 : activeVelocityGain * rawVelocityError;
        state.trackingEndpointBlend = blend; state.velocitySampleAvailable = true;
        HolonomicCommandAllocator.Result command;
        double forceScale=1;
        if (wheelModel != null) {
            double[] wheelPower = wheelFeedforward.powers();
            for(int i=0;i<4;i++) wheelPower[i]*=1-blend;
            command=allocator.allocateWheels(state.crossTrackCorrection,tangent,heading,tangentFeedback,headingFeedback,
                    wheelPower,tangentVelocityFeedback,headingVelocityFeedback,normalVelocityFeedback);
            if(feedbackEnabled) {
                Vector correction = state.crossTrackCorrection.plus(tangent.times(tangentFeedback))
                        .times(command.demand.correctionScale)
                        .plus(tangent.times(tangentVelocityFeedback).plus(normalVelocityFeedback).times(command.demand.velocityScale))
                        .rotate(heading.times(-1)).times(wheelModel.gain);
                double angularCorrection=(headingFeedback*command.demand.correctionScale
                        +headingVelocityFeedback*command.demand.velocityScale)*wheelModel.gain/wheelModel.effectiveRadius;
                double fade=(1-blend)*command.demand.feedforwardScale;
                forceScale = wheelFeedforward.applyFeedback(wheelModel, correction, angularCorrection,
                        fade, target.angularAcceleration, command.wheelPowers);
            }
        } else command = allocator.allocate(state.crossTrackCorrection, Vector.zero(), tangent,
                heading, tangentFeedback, headingFeedback, tangentFeedforward, tangentVelocityFeedback,
                headingVelocityFeedback, headingFeedforward, normalFeedforward, centripetalFeedforward, normalVelocityFeedback);
        if(onlineVelocityReference) state.velocityReference.condition((1-blend)*forceScale*
                (velocityBraking?command.demand.correctionScale:command.demand.velocityScale));
        state.lastCommandDemand = command.demand;
        Follower.MovementOutcome outcome = completionOutcome(path, position, velocity, angularVelocity,
                endpointHeadingError, independentReference, now, state);
        if (outcome != Follower.MovementOutcome.RUNNING) return outcome;
        if (command.wheelPowers != null) drivetrain.setPowers(command.wheelPowers[0],command.wheelPowers[1],
                command.wheelPowers[2],command.wheelPowers[3]);
        else drivetrain.drive(command.drive.getX().getIn(), command.drive.getY().getIn(), command.turn);
        return Follower.MovementOutcome.RUNNING;
    }

    private static Vector trackingTangent(TrajectorySample target, Path path, Vector position) {
        Vector tangent = target.tangent;
        if (tangent.getMag().getIn() < 1e-9) tangent = path.getEndPose().getVec().minus(position).normalize();
        return tangent.getMag().getIn() < 1e-9 ? TrajectorySample.vector(1, 0) : tangent;
    }

    private void updateCrossTrackCorrection(TrajectorySample target, PathSegment segment, Vector velocity,
            Vector normal, boolean applyStatic, boolean driveControllerEnabled, FollowerPathState state) {
        state.crossTrackNormal = normal;
        state.pathNormal = PathSegment.calculateArcNormal(segment.getFirstDerivative(state.t), segment.getSecondDerivative(state.t));
        state.crossTrackError = target.toPath.dot(normal).getIn();
        state.centripetalError = target.toPath.dot(state.pathNormal).getIn();
        driveController.setCrossTrackStaticCompensationEnabled(applyStatic);
        double lateralPower = driveControllerEnabled ? driveController.calculateCrossTrack(state.crossTrackError) : 0;
        if (driveControllerEnabled) {
            lateralPower = FollowerMath.ensureEndpointBreakawayPower(lateralPower, state.crossTrackError,
                    velocity.dot(normal).getIn(), constants.translationalCoeffs.kS,
                    FeedbackController.LINEAR_STATIC_DEADBAND, target.remainingIn);
        }
        state.crossTrackCorrection = normal.times(lateralPower);
    }

    private double calculateHeadingFeedback(TrajectorySample target, Angle heading, double angularVelocity,
            double endpointHeadingError, boolean applyStatic, boolean enabled) {
        if (!enabled) return 0;
        double error = heading.getShortestAngleTo(target.heading).getRad();
        double power = FollowerMath.calculatePathHeadingFeedback(headingController, error,
                target.angularVelocity, angularVelocity, applyStatic);
        return target.remainingIn < FollowerMath.PROFILED_ENDPOINT_CAPTURE_DISTANCE_INCHES
                ? FollowerMath.ensureAngularEndpointBreakawayPower(power, endpointHeadingError, angularVelocity,
                    applyStatic ? constants.angularCoeffs.kS : 0, PathCompletionPolicy.headingTolerance(headingTolerance))
                : power;
    }

    private Follower.MovementOutcome completionOutcome(Path path, Vector position, Vector velocity,
            double angularVelocity, double endpointHeadingError, boolean independentReference,
            long now, FollowerPathState state) {
        double endpointDistance = position.distanceTo(path.getEndPose().getVec()).getIn();
        if (independentReference) {
            // Endpoint capture normally permits an early motor stop at 8 in/s.
            // An isolated feedforward test must execute its entire braking schedule.
            if (state.independentReferenceFinished() && velocity.getMag().getIn()<.5 && Math.abs(angularVelocity)<.1) return Follower.MovementOutcome.COMPLETED;
        } else {
            return state.completionOutcome(endpointDistance, endpointHeadingError, velocity, angularVelocity,
                    distanceTolerance, headingTolerance, now);
        }
        return Follower.MovementOutcome.RUNNING;
    }
}
