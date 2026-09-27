package core;

import com.qualcomm.robotcore.util.Range;

import controllers.DriveController;
import controllers.FeedbackController;
import drivetrains.BaseDrivetrain;
import feedforward.MotionParameters;
import geometry.Angle;
import geometry.PathSegment;
import geometry.Pose;
import geometry.Vector;
import localizers.BaseLocalizer;
import paths.movements.Path;

/** Executes quick and profiled paths for holonomic drivetrains. */
final class HolonomicPathFollower {
    private static final double MIN_PROFILE_VELOCITY_IN_PER_SECOND = 6.0;

    private final BaseDrivetrain<?> drivetrain;
    private final DriveController driveController;
    private final FeedbackController headingController;
    private final FeedbackController translationFeedforwardController;
    private final FeedbackController headingFeedforwardController;
    private final FeedbackController translationVelocityController;
    private final FeedbackController headingVelocityController;
    private final FeedbackController centripetalController;
    private final HolonomicCommandAllocator allocator;
    private final FollowerConstants constants;
    private final double distanceTolerance;
    private final double headingTolerance;

    HolonomicPathFollower(BaseDrivetrain<?> drivetrain, DriveController driveController,
                          FeedbackController headingController, HolonomicCommandAllocator allocator,
                          FollowerConstants constants, double distanceTolerance,
                          double headingTolerance) {
        this.drivetrain = drivetrain;
        this.driveController = driveController;
        this.headingController = headingController;
        this.allocator = allocator;
        this.constants = constants;
        this.distanceTolerance = distanceTolerance;
        this.headingTolerance = headingTolerance;
        translationFeedforwardController = new FeedbackController(
                new FeedbackController.Coefficients().vel(0.0).accel(0.0));
        headingFeedforwardController = new FeedbackController(
                new FeedbackController.Coefficients().vel(0.0).accel(0.0));
        translationVelocityController = new FeedbackController(
                new FeedbackController.Coefficients().p(0.0));
        headingVelocityController = new FeedbackController(
                new FeedbackController.Coefficients().p(0.0));
        centripetalController = new FeedbackController(
                new FeedbackController.Coefficients().p(0.0));
    }

    boolean update(Path path, PathSegment segment, Pose current, BaseLocalizer<?> localizer,
                   MovementCallbackRunner callbacks, FollowerPathState state,
                   boolean headingControllerEnabled, boolean driveControllerEnabled,
                   double translationalKV, double translationalKA,
                   double angularKV, double angularKA, double centripetalGain,
                   double velocityFeedbackGain, double angularVelocityFeedbackGain) {
        translationFeedforwardController.getCoefficients()
                .vel(translationalKV).accel(translationalKA);
        headingFeedforwardController.getCoefficients().vel(angularKV).accel(angularKA);
        translationVelocityController.getCoefficients().p(velocityFeedbackGain);
        headingVelocityController.getCoefficients().p(angularVelocityFeedbackGain);

        // region 1. Path Information
        // 1.1 Robot pose and path geometry
        Vector currentPosition = current.getVec();
        Angle currentHeading = current.getHeading();
        state.t = segment.getBestT(currentPosition);
        state.closestPathPoint = segment.getPosition(state.t);

        Vector firstDerivative = segment.getFirstDerivative(state.t);
        Vector secondDerivative = segment.getSecondDerivative(state.t);
        Vector unitTangent = firstDerivative.normalize();
        Vector lateralNormal = PathSegment.calculateLeftNormal(firstDerivative);
        Vector principalNormal = PathSegment.calculateArcNormal(firstDerivative, secondDerivative);
        state.crossTrackNormal = lateralNormal;
        state.pathNormal = principalNormal;
        Vector endTangent = segment.getFirstDerivative(1.0).normalize();
        double distanceRemaining = segment.getDistanceToEndIn(state.closestPathPoint, state.t);
        double signedEndpointError = Follower.pathEndpointTangentError(
                path.getEndPose().getVec(), currentPosition, endTangent);
        double distanceTraveled = path.getParametricPath().getLengthIn() - distanceRemaining;
        double curvature = segment.getSignedCurvature(state.t);

        // 1.2 Progress callbacks
        double pathProgress = 1.0 - distanceRemaining / segment.getLengthIn();
        callbacks.process(path, Range.clip(pathProgress, 0.0, 1.0), currentHeading);

        // 1.3 Motion targets
        boolean profiled = path.isProfiled();
        MotionParameters targets = profiled
                ? path.getFeedforwardLut().getFFParams(distanceTraveled) : null;
        Vector robotVelocity = localizer.getVel().getVec();
        double quickProgress = Range.clip(
                1.0 - distanceRemaining / path.getParametricPath().getLengthIn(), 0.0, 1.0);
        double commandedForwardVelocity = profiled
                ? targets.getTangentialVel()
                : path.getQuickVelocityLimit(quickProgress, constants.forwardVelLimitIn);
        boolean applyCorrectiveStatic = Math.abs(commandedForwardVelocity)
                < 0.10 * constants.forwardVelLimitIn;

        // 1.4 Measured progress and tracking state
        allocator.activeDriveModel();
        double measuredTangentialVelocity = robotVelocity.dot(unitTangent).getIn();
        double progressVelocity = state.progressEstimator.update(
                distanceTraveled, measuredTangentialVelocity, System.nanoTime());
        double robotTangentialVelocity = measuredTangentialVelocity;
        state.trackingMeasuredVelocity = measuredTangentialVelocity;
        state.trackingProgressVelocity = progressVelocity;
        state.trackingVelocityTarget = commandedForwardVelocity;
        state.trackingFeedforward = 0.0;
        state.trackingVelocityFeedback = 0.0;
        state.trackingEndpointBlend = 0.0;
        state.velocitySampleAvailable = true;

        Angle headingTarget = path.getInterpolator().getHeadingTarg(
                distanceRemaining, firstDerivative, endTangent);
        // endregion

        // region 2. Algorithm
        // 2.1 Heading feedforward and feedback
        double targetAngularVelocity = 0.0;
        double headingFeedforward = 0.0;
        if (profiled) {
            double headingTimeScale = MotionFeedforward.timeScale(
                    targets.getTangentialVel(), progressVelocity);
            targetAngularVelocity = targets.getAngularVel() * headingTimeScale;
            double targetAngularAcceleration = targets.getAngularAccel()
                    * headingTimeScale * headingTimeScale;
            double movingFriction = Math.abs(targetAngularVelocity) > 1e-6
                    ? Math.signum(targetAngularVelocity) * constants.angularFeedforwardKS : 0.0;
            headingFeedforward = headingFeedforwardController.vel(targetAngularVelocity)
                    .accel(targetAngularAcceleration).f(movingFriction)
                    .calculate(0.0).get();
        }

        double currentAngularVelocity = localizer.getVel().getHeading().getRad();
        state.trackingAngularVelocityTarget = targetAngularVelocity;
        double headingError = currentHeading.getShortestAngleTo(headingTarget).getRad();
        double headingTargetRate = profiled ? targetAngularVelocity
                : path.getInterpolator().getHeadingFirstDerivative(
                        distanceRemaining, curvature, endTangent) * progressVelocity;
        double headingFeedback = headingControllerEnabled
                ? Follower.calculatePathHeadingFeedback(
                        headingController, headingError, headingTargetRate,
                        currentAngularVelocity, applyCorrectiveStatic)
                : 0.0;
        double headingVelocityFeedback = headingControllerEnabled
                && Math.abs(targetAngularVelocity) > 1e-6
                ? headingVelocityController.p()
                        .calculate(targetAngularVelocity - currentAngularVelocity).get() : 0.0;
        double endpointHeadingError = currentHeading
                .getShortestAngleTo(path.getEndPose().getHeading()).getRad();
        if (distanceRemaining < Follower.PROFILED_ENDPOINT_CAPTURE_DISTANCE_INCHES) {
            headingFeedback = Follower.ensureAngularEndpointBreakawayPower(
                    headingFeedback, endpointHeadingError, currentAngularVelocity,
                    applyCorrectiveStatic ? constants.angularCoeffs.kS : 0.0,
                    PathCompletionPolicy.headingTolerance(headingTolerance));
        }

        // 2.2 Cross-track and centripetal corrections
        Vector positionalError = state.closestPathPoint.minus(currentPosition);
        state.crossTrackError = positionalError.dot(lateralNormal).getIn();
        state.centripetalError = positionalError.dot(principalNormal).getIn();
        driveController.setCrossTrackStaticCompensationEnabled(applyCorrectiveStatic);
        double lateralFeedback = driveControllerEnabled
                ? driveController.calculateCrossTrack(state.crossTrackError) : 0.0;
        if (driveControllerEnabled) {
            double measuredLateralVelocity = robotVelocity.dot(lateralNormal).getIn();
            lateralFeedback = Follower.ensureEndpointBreakawayPower(
                    lateralFeedback, state.crossTrackError, measuredLateralVelocity,
                    constants.translationalCoeffs.kS,
                    FeedbackController.LINEAR_STATIC_DEADBAND,
                    distanceRemaining);
        }
        state.crossTrackCorrection = lateralNormal.times(lateralFeedback);
        centripetalController.getCoefficients().p(centripetalGain);
        double centripetalResponse = centripetalController.p()
                .calculate(robotTangentialVelocity * robotTangentialVelocity
                        * Math.abs(curvature)).get();
        state.centripetalCorrection = principalNormal.times(
                Math.min(centripetalResponse, Follower.MAX_CENTRIPETAL_POWER));

        // 2.3 Tangential feedforward and feedback
        double tangentFeedforward;
        double tangentFeedback = 0.0;
        double tangentVelocityFeedback = 0.0;
        if (state.t < 1.0) {
            if (profiled) {
                double velocityTarget = Math.max(
                        targets.getTangentialVel(), MIN_PROFILE_VELOCITY_IN_PER_SECOND);
                double accelerationTarget = MotionFeedforward.scaleBrakingAcceleration(
                        velocityTarget, targets.getTangentialAccel(), robotTangentialVelocity);
                double motionSign = MotionFeedforward.motionSign(
                        velocityTarget, accelerationTarget);
                double feedforward = translationFeedforwardController.vel(velocityTarget)
                        .accel(accelerationTarget)
                        .f(motionSign * constants.translationalFeedforwardKS)
                        .calculate(0.0).get();
                tangentVelocityFeedback = translationVelocityController.p()
                        .calculate(velocityTarget - robotTangentialVelocity).get();
                double cruisePower = motionSign * constants.translationalFeedforwardKS
                        + constants.translationalKV * velocityTarget;
                boolean densityFeedback = targets.getTangentialAccel() < 0.0
                        && constants.brakingResponse
                        == FollowerConstants.BrakingResponse.PULSE_DENSITY_FEEDBACK;
                boolean toleranceBrake = constants.brakingResponse
                        == FollowerConstants.BrakingResponse.TOLERANCE_BRAKE;
                boolean belowBrakingBoundary = targets.getTangentialAccel() < 0
                        && feedforward <= 0.0;
                if (toleranceBrake) {
                    double overspeed = robotTangentialVelocity - velocityTarget;
                    double pulseTolerance = targets.getTangentialAccel() < -5.0
                            && Math.abs(curvature) > 0.005
                            ? constants.brakingCornerToleranceIn
                            : constants.brakingOverspeedToleranceIn;
                    if (state.brakingPulseCooldown > 0) {
                        state.brakingPulseCooldown--;
                    }
                    if (overspeed > pulseTolerance
                            && state.brakingPulseCooldown == 0) {
                        // One opposing feedforward cycle. Keep position and
                        // heading corrections ahead of this brake request.
                        feedforward = -constants.brakingPulsePower;
                        tangentVelocityFeedback = 0.0;
                        state.brakingPulseCooldown =
                                Math.max(0, constants.brakingPulseCooldownCycles) + 1;
                    } else if (targets.getTangentialAccel() < 0.0) {
                        double netDrive = cruisePower + constants.brakingVelocityFeedbackGain
                                * (velocityTarget - robotTangentialVelocity);
                        if (targets.getTangentialAccel() < -5.0) {
                            // During sustained braking, feedback may relax the
                            // brake but must not command recovery acceleration.
                            double neutralPower = constants.translationalFeedforwardKS
                                    + constants.translationalKV
                                    * Math.max(0.0, robotTangentialVelocity);
                            netDrive = Math.min(netDrive,
                                    neutralPower - constants.brakingNeutralPowerMargin);
                        }
                        // The near-flat velocity floor needs enough drive to
                        // keep rolling through a corner after deceleration.
                        feedforward = Math.max(0.001, netDrive);
                        tangentVelocityFeedback = 0.0;
                    }
                } else if (densityFeedback) {
                    // In this simulator, zero power is a distinct, strong BRAKE regime.
                    // Distribute brake cycles instead of remaining in that regime for
                    // several cycles after the continuous command crosses zero.
                    double brakingMagnitude = constants.brakingRegenerationRatePerSecond
                            * Math.max(0.0, robotTangentialVelocity)
                            + constants.brakingNaturalDecelerationIn;
                    double velocityError = velocityTarget - robotTangentialVelocity;
                    double requestedDeceleration = -targets.getTangentialAccel()
                            - constants.brakingDensityFeedbackRatePerSecond * velocityError;
                    if (brakingMagnitude > 0.0 && Double.isFinite(brakingMagnitude)) {
                        double fraction = Range.clip(
                                requestedDeceleration / brakingMagnitude, 0.0, 1.0);
                        state.brakingPulseCredit += fraction;
                        boolean brakeCycle = state.brakingPulseCredit >= 1.0;
                        if (brakeCycle) state.brakingPulseCredit -= 1.0;
                        feedforward = brakeCycle ? 0.0 : cruisePower;
                        tangentVelocityFeedback = 0.0;
                    } else {
                        // An uncalibrated selector must preserve ordinary braking.
                        state.brakingPulseCredit = 0.5;
                    }
                } else if (targets.getTangentialAccel() < 0 && !belowBrakingBoundary
                        && constants.brakingKAScaleAboveBoundary != 1.0) {
                    // Keep this sweep on the original positive-power side of the boundary.
                    feedforward = Math.max(0.001, cruisePower
                            + constants.brakingKAScaleAboveBoundary
                            * constants.translationalKA * accelerationTarget);
                }
                if (belowBrakingBoundary && !densityFeedback && !toleranceBrake) {
                    switch (constants.brakingResponse) {
                        case VELOCITY_FEEDBACK:
                            feedforward = cruisePower;
                            tangentVelocityFeedback = constants.brakingVelocityFeedbackGain
                                    * (velocityTarget - robotTangentialVelocity);
                            break;
                        case PULSE_DENSITY:
                            double brakingMagnitude = constants.brakingRegenerationRatePerSecond
                                    * velocityTarget + constants.brakingNaturalDecelerationIn;
                            if (brakingMagnitude > 0.0) {
                                double fraction = Range.clip(
                                        -targets.getTangentialAccel() / brakingMagnitude, 0.0, 1.0);
                                state.brakingPulseCredit += fraction;
                                boolean brakeCycle = state.brakingPulseCredit >= 1.0;
                                if (brakeCycle) state.brakingPulseCredit -= 1.0;
                                feedforward = brakeCycle ? 0.0 : cruisePower;
                                tangentVelocityFeedback = 0.0;
                            }
                            break;
                        case COAST:
                            // Exact zero invokes FTCodeSim's strong BRAKE branch.
                            feedforward = 0.001;
                            tangentVelocityFeedback = 0.0;
                            break;
                        default:
                            break;
                    }
                } else if (!densityFeedback) {
                    state.brakingPulseCredit = 0.5;
                }
                if (!toleranceBrake) {
                    state.brakingPulseCooldown = 0;
                }
                state.trackingVelocityTarget = velocityTarget;
                state.trackingFeedforward = feedforward;
                state.trackingVelocityFeedback = tangentVelocityFeedback;
                tangentFeedforward = feedforward;

                // Let endpoint braking override the boosted profile when needed.
                if (path.isAccelBoosted()) {
                    double decelPower = driveController.calculateEndDistance(
                            distanceRemaining, -measuredTangentialVelocity);
                    if (decelPower <= tangentFeedforward + tangentVelocityFeedback) {
                        tangentFeedback = decelPower;
                        tangentVelocityFeedback = 0.0;
                        tangentFeedforward = 0.0;
                    }
                }
            } else {
                // Quick paths choose between cruising power and endpoint braking.
                double endDistanceError = distanceRemaining
                        < Follower.PROFILED_ENDPOINT_CAPTURE_DISTANCE_INCHES
                        ? signedEndpointError : distanceRemaining;
                double decelPower = driveController.calculateEndDistance(
                        endDistanceError, -measuredTangentialVelocity);
                double velocityError = commandedForwardVelocity - robotTangentialVelocity;
                tangentVelocityFeedback = translationVelocityController.p()
                        .calculate(velocityError).get();
                double propulsionStatic = Math.abs(robotTangentialVelocity)
                        < Follower.ENDPOINT_STALLED_VELOCITY_IN_PER_SECOND
                        ? Math.max(constants.translationalFeedforwardKS,
                                constants.translationalCoeffs.kS)
                        : constants.translationalFeedforwardKS;
                double feedforwardPower = translationFeedforwardController
                        .vel(commandedForwardVelocity)
                        .f(Math.signum(commandedForwardVelocity) * propulsionStatic)
                        .calculate(0.0).get();
                double accelerationPower = feedforwardPower + tangentVelocityFeedback;
                if (decelPower <= accelerationPower) {
                    tangentFeedback = decelPower;
                    tangentVelocityFeedback = 0.0;
                    tangentFeedforward = 0.0;
                } else {
                    tangentFeedforward = feedforwardPower;
                }
            }
        } else {
            tangentFeedback = profiled ? 0.0 : driveController.calculateEndDistance(
                    signedEndpointError, -measuredTangentialVelocity);
            tangentFeedforward = 0.0;
        }

        // 2.4 Blend into endpoint capture
        if (profiled) {
            double endpointPower = driveController.calculateEndDistance(
                    signedEndpointError, -measuredTangentialVelocity);
            endpointPower = Follower.ensureEndpointBreakawayPower(
                    endpointPower, signedEndpointError, robotTangentialVelocity,
                    constants.translationalCoeffs.kS,
                    PathCompletionPolicy.distanceTolerance(distanceTolerance), distanceRemaining);
            double endpointBlend = Follower.endpointCaptureBlend(distanceRemaining);
            state.trackingEndpointBlend = endpointBlend;
            tangentFeedback = tangentFeedback * (1.0 - endpointBlend)
                    + endpointPower * endpointBlend;
            tangentVelocityFeedback *= 1.0 - endpointBlend;
            tangentFeedforward *= 1.0 - endpointBlend;
        }

        // 2.5 Reconcile propulsion and braking
        if (tangentFeedforward * tangentVelocityFeedback < 0.0) {
            double netTangentPower = tangentFeedforward + tangentVelocityFeedback;
            if (netTangentPower * tangentFeedforward >= 0.0) {
                tangentFeedforward = netTangentPower;
                tangentVelocityFeedback = 0.0;
            } else {
                tangentFeedforward = 0.0;
                tangentVelocityFeedback = netTangentPower;
            }
        }
        if (robotTangentialVelocity * tangentVelocityFeedback < 0.0) {
            tangentFeedback += tangentVelocityFeedback;
            tangentVelocityFeedback = 0.0;
        }

        // 2.6 Allocate available motor power
        HolonomicCommandAllocator.Result command = allocator.allocate(
                state.crossTrackCorrection, state.centripetalCorrection, unitTangent,
                currentHeading, tangentFeedback, headingFeedback, tangentFeedforward,
                tangentVelocityFeedback, headingVelocityFeedback, headingFeedforward);
        state.lastCommandDemand = command.demand;

        // 2.7 Completion and stuck detection
        double endpointDistance = currentPosition.distanceTo(path.getEndPose().getVec()).getIn();
        boolean insideTolerance = PathCompletionPolicy.isInsideTolerance(
                endpointDistance, endpointHeadingError, distanceTolerance, headingTolerance);
        boolean velocitySettled = PathCompletionPolicy.isVelocitySettled(
                robotVelocity.getMagSq().getIn(), currentAngularVelocity);
        if (state.completion.isComplete(insideTolerance, velocitySettled, System.nanoTime())) {
            return true;
        }
        // End the movement if the robot remains stuck.
        if (state.stuckWatchdog.update(
                Follower.stuckForSpeed(robotVelocity.getMag().getIn()), System.nanoTime())) {
            return true;
        }

        // endregion

        // region 3. Send powers
        drivetrain.drive(command.drive.getX().getIn(), command.drive.getY().getIn(), command.turn);
        // endregion

        return false;
    }
}
