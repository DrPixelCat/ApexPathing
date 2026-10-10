package follower;

import util.DeltaTime;

import controllers.DriveController;
import controllers.DriveController.AllocatedCommand;
import controllers.TurnController;
import drivetrains.BaseDrivetrain;
import trajectory.profile.MotionParameters;
import geometry.Angle;
import geometry.Pose;
import geometry.Vector;
import localizers.BaseLocalizer;
import paths.movements.Path;
import paths.movements.Turn;
import trajectory.TrajectorySample;

/**
 * Executes PID quick turns and stored profiles, with continuous heading progress.
 *
 * @author Joel H
 */
final class TurnExecutor {
    private static final double SETTLED_ANGULAR_VELOCITY = .10;
    private final BaseDrivetrain<?> drivetrain;
    private final DriveController driveController;
    private final TurnController turnController;
    private final HolonomicCommandAllocator allocator;
    private final FollowerConstants constants;
    private final double headingTolerance, distanceTolerance;
    private Angle targetHeading, previousHeading;
    private Vector targetPosition;
    private double direction, totalDisplacement, progress;
    private final DeltaTime timing = new DeltaTime(.02);
    private double stalledSeconds;
    private final java.util.function.LongSupplier monotonicClock;

    TurnExecutor(BaseDrivetrain<?> drivetrain, DriveController driveController,
                 TurnController turnController, HolonomicCommandAllocator allocator,
                 FollowerConstants constants, double headingTolerance, double distanceTolerance,
                 java.util.function.LongSupplier monotonicClock) {
        this.drivetrain = drivetrain; this.driveController = driveController; this.turnController = turnController;
        this.allocator = allocator; this.constants = constants;
        this.headingTolerance = headingTolerance; this.distanceTolerance = distanceTolerance;
        this.monotonicClock=monotonicClock;
    }

    void start(Turn turn, long now) {
        targetHeading = turn.getEndPose().getHeading(); targetPosition = turn.getStartPose().getVec();
        previousHeading = turn.getStartPose().getHeading();
        double signedTurn = previousHeading.getShortestAngleTo(targetHeading).getRad();
        direction = Math.signum(signedTurn); totalDisplacement = Math.abs(signedTurn); progress = 0;
        timing.reset();
        stalledSeconds = 0;
    }

    Follower.MovementOutcome update(Turn turn, Pose current, BaseLocalizer<?> localizer,
                   MovementCallbackRunner callbacks, FollowerPathState state, boolean driveControllerEnabled, boolean headingControllerEnabled) {
        long now = monotonicClock.getAsLong();
        double dt = Math.max(0, Math.min(.05, timing.update(now)));

        Angle heading = current.getHeading();
        observeHeading(heading);
        double headingError = heading.getShortestAngleTo(targetHeading).getRad();
        double angularVelocity = localizer.getVel().getHeading().getRad();
        double tolerance = PathCompletionPolicy.headingTolerance(headingTolerance);
        callbacks.process(turn, -1, heading);
        // Compensate one control interval of motion before the new power takes effect.
        // This is measured heading progress plus a bounded rate preview, never wall time.
        double sampleProgress = Math.min(totalDisplacement, Math.max(0, progress)
                + Math.max(0, direction * angularVelocity) * dt);
        MotionParameters targets;
        boolean stored = turn.getBuildMode() == Path.BuildMode.MEMORY;
        if (stored) {
            targets = turn.getFeedforwardLut().getAngularKinematicParams(sampleProgress);
        } else { targets = new MotionParameters(); }
        double boundedProgress = Math.max(0, Math.min(totalDisplacement, progress));
        Angle referenceHeading = stored
                ? turn.getStartPose().getHeading().plus(Angle.fromRad(direction * boundedProgress))
                : targetHeading;
        // Once captured inside tolerance, damp the remaining rate without commanding a
        // small reverse correction. Completion still requires the existing settled rate.
        if (stored && progress >= totalDisplacement && Math.abs(headingError) < tolerance) { referenceHeading = heading; }
        double referenceError = heading.getShortestAngleTo(referenceHeading).getRad();
        state.trajectorySample = new TrajectorySample(Vector.zero(), Vector.zero(), targetPosition.minus(current.getVec()),
                Vector.zero(), targetPosition, 0, 0,
                totalDisplacement > 1e-9 ? boundedProgress / totalDisplacement : 1, referenceHeading,
                targets.getAngularVel(), targets.getAngularAccel(), false, false, boundedProgress, Math.max(0, totalDisplacement - progress));
        state.trackingAngularVelocityTarget = targets.getAngularVel();
        if (Math.abs(headingError) <= tolerance && Math.abs(angularVelocity) < SETTLED_ANGULAR_VELOCITY) { return Follower.MovementOutcome.COMPLETED; }
        double turnPower = stored
                ? turnController.calculateProfiled(referenceError, direction, targets, angularVelocity, constants)
                : turnController.calculateQuick(headingError, angularVelocity, tolerance);
        turnPower = Follower.ensureAngularEndpointBreakawayPower(turnPower, headingError, angularVelocity,
                constants.angularCoeffs.kS, tolerance);
        if (!stored) {
            if (Math.abs(headingError) > tolerance && Math.abs(angularVelocity) < .05) {
                stalledSeconds += dt;
                // Preserve the existing stall-only breakaway ramp for unprofiled turns.
                turnPower = Math.max(-1, Math.min(1, turnPower
                        + Math.copySign(Math.min(.25, stalledSeconds * .25), headingError)));
            } else { stalledSeconds = 0; }
        }
        if (!headingControllerEnabled) {
            turnPower = stored ? constants.characterizedPower(feedforward.MotionCharacterization.Axis.ANGULAR,
                    targets.getAngularVel(), targets.getAngularAccel(), constants.angularKV * targets.getAngularVel()
                            + constants.angularKA * targets.getAngularAccel()
                            + constants.angularFeedforwardKS * MotionFeedforward.motionSign(targets.getAngularVel(), targets.getAngularAccel())) : 0;
        }
        Vector error = targetPosition.minus(current.getVec());
        if (driveControllerEnabled && drivetrain.isHolonomic() && error.getMag().getIn() > distanceTolerance) {
            Vector fieldFeedback = driveController.calculatePointToPoint(targetPosition, current.getVec());
            AllocatedCommand hold = allocator.allocatePositionHold(fieldFeedback, heading, 1 - Math.abs(turnPower));
            Vector command = hold.getRobotCommand();
            drivetrain.drive(command.getX().getIn(), command.getY().getIn(), turnPower);
        } else { drivetrain.drive(0, 0, turnPower); }
        return Follower.MovementOutcome.RUNNING;
    }
    void observeHeading(Angle heading) {
        if (previousHeading != null) {
            progress += direction * previousHeading.getShortestAngleTo(heading).getRad();
            previousHeading = heading;
        }
    }
    void pause() { timing.reset(); stalledSeconds = 0; }
    void resume(long now) { timing.reset(); }
    void markTransitionTick(long now) { timing.reset(); }
    void reset() { targetHeading = null; targetPosition = null; previousHeading = null; stalledSeconds = 0; timing.reset(); progress = 0; }
}
