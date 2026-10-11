package follower.internal;

import controllers.DriveController;
import controllers.FeedbackController;
import controllers.TurnController;
import drivetrains.BaseDrivetrain;
import feedforward.CenteredBrakingFeedforward;
import follower.Follower;
import follower.FollowerConstants;
import geometry.Angle;
import geometry.PathSegment;
import geometry.Pose;
import geometry.Vector;
import java.util.function.LongSupplier;
import localizers.BaseLocalizer;
import paths.movements.Path;
import paths.movements.Turn;
import trajectory.TrajectorySample;

/**
 * Internal bridge between the public lifecycle and package-private tracking engines.
 * Owns their mutable state; robot code should use Follower and its drivetrain subclasses.
 */
public final class FollowerRuntime {
    public interface PathController {
        Follower.MovementOutcome update(Path path, PathSegment segment, Pose pose,
                BaseLocalizer<?> localizer, boolean headingEnabled, boolean driveEnabled);
        void reset();
        void setBrakingFeedforward(CenteredBrakingFeedforward model);
    }

    private final BaseDrivetrain<?> drivetrain;
    private final DriveController driveController;
    private final FeedbackController headingController;
    private final FollowerConstants constants;
    private final double distanceTolerance, headingTolerance;
    private final LongSupplier clock;
    private final MovementCallbackRunner callbackRunner = new MovementCallbackRunner();
    private final FollowerPathState pathState = new FollowerPathState();
    private final drivetrains.models.DriveModel holonomicModel, tankModel;
    private final HolonomicCommandAllocator allocator;
    private final TurnExecutor turnExecutor;
    private final PoseHoldController poseHoldController;

    public FollowerRuntime(BaseDrivetrain<?> drivetrain, DriveController driveController,
            FeedbackController headingController, TurnController turnController,
            FollowerConstants constants, double distanceTolerance, double headingTolerance,
            LongSupplier clock) {
        this.drivetrain = drivetrain;
        this.driveController = driveController;
        this.headingController = headingController;
        this.constants = constants;
        this.distanceTolerance = distanceTolerance;
        this.headingTolerance = headingTolerance;
        this.clock = clock;
        holonomicModel=drivetrains.models.DriveModels.forDrivetrain(drivetrain,constants,Path.PathType.HOLONOMIC);
        tankModel=drivetrains.models.DriveModels.forDrivetrain(drivetrain,constants,Path.PathType.TANK);
        allocator = new HolonomicCommandAllocator(drivetrain,driveController,holonomicModel);
        turnExecutor = new TurnExecutor(drivetrain, driveController, turnController,
                allocator, constants, headingTolerance, distanceTolerance, clock);
        poseHoldController = new PoseHoldController(drivetrain, driveController, headingController);
    }

    public PathController createHolonomicController() {
        HolonomicPathFollower executor = new HolonomicPathFollower(drivetrain, driveController,
                headingController, allocator, constants, distanceTolerance, headingTolerance, clock,holonomicModel);
        return controller(executor, executor);
    }

    public PathController createTankController() {
        return controller(new TankPathFollower(drivetrain, driveController, constants,
                distanceTolerance, headingTolerance, clock,tankModel), null);
    }

    private PathController controller(PathExecutor executor, HolonomicPathFollower holonomic) {
        return new PathController() {
            @Override public Follower.MovementOutcome update(Path path, PathSegment segment, Pose pose,
                    BaseLocalizer<?> localizer, boolean headingEnabled, boolean driveEnabled) {
                return executor.update(path, segment, pose, localizer, callbackRunner, pathState,
                        headingEnabled, driveEnabled);
            }
            @Override public void reset() {
                if (holonomic != null) { holonomic.resetBrakingFeedforward(); }
            }
            @Override public void setBrakingFeedforward(CenteredBrakingFeedforward model) {
                if (holonomic != null) { holonomic.setBrakingFeedforward(model); }
            }
        };
    }

    public Follower.MovementOutcome updateTurn(Turn turn, Pose pose, BaseLocalizer<?> localizer,
            boolean driveEnabled, boolean headingEnabled) {
        return turnExecutor.update(turn, pose, localizer, callbackRunner, pathState,
                driveEnabled, headingEnabled);
    }

    public void holdPose(Pose target, Pose current, boolean holonomic,
            boolean headingEnabled, boolean driveEnabled) {
        poseHoldController.update(target, current, holonomic, headingEnabled, driveEnabled);
    }

    public TrajectorySample samplePathTarget(Path path, Pose pose, BaseLocalizer<?> localizer) {
        return pathState.target(path, pose.getVec(), localizer.getVel().getVec(),
                localizer.getVel().getHeading().getRad(), clock.getAsLong());
    }

    /** Body-axis reference power; custom trackers may instead supply their own reference to executeHolonomicPath. */
    public Follower.MovementOutcome executeHolonomicPath(Path path, TrajectorySample target, Pose pose,
            BaseLocalizer<?> localizer, Vector correction, double headingCorrection,
            boolean headingEnabled, boolean driveEnabled) {
        Vector tangent = target.tangent;
        Vector body = tangent.rotate(target.heading.times(-1));
        feedforward.MotionPowerCalculator.Output power = holonomicModel.powerCalculator().calculate(body,
                target.tangentialVelocity(), target.tangentialAcceleration(), target.angularVelocity,
                target.angularAcceleration, target.normalAcceleration().dot(PathSegment.calculateLeftNormal(tangent)).getIn());
        return executeHolonomicPath(path, target, pose, localizer, correction, headingCorrection,
                power.translation().plus(power.normal()).rotate(target.heading), power.heading, headingEnabled, driveEnabled);
    }

    public Follower.MovementOutcome executeHolonomicPath(Path path, TrajectorySample target, Pose pose,
            BaseLocalizer<?> localizer, Vector correction, double headingCorrection,
            Vector feedforward, double headingFeedforward, boolean headingEnabled, boolean driveEnabled) {
        if (!correction.isFinite() || !feedforward.isFinite() || !Double.isFinite(headingCorrection)
                || !Double.isFinite(headingFeedforward)) throw new IllegalArgumentException("Path commands must be finite");
        if (!driveEnabled) correction = Vector.zero();
        if (!headingEnabled) headingCorrection = 0;
        Vector tangent = target.tangent, normal = PathSegment.calculateLeftNormal(tangent);
        double alongCorrection = correction.dot(tangent).getIn();
        Vector crossCorrection = correction.minus(tangent.times(alongCorrection));
        double alongFeedforward = feedforward.dot(tangent).getIn();
        HolonomicCommandAllocator.Result command = allocator.allocate(crossCorrection, Vector.zero(), tangent,
                pose.getHeading(), alongCorrection, headingCorrection, alongFeedforward, 0, 0, headingFeedforward,
                feedforward.minus(tangent.times(alongFeedforward)));
        long now = clock.getAsLong();
        pathState.trajectorySample = target;
        pathState.t = target.pathT; pathState.closestPathPoint = target.positionOnPath;
        pathState.crossTrackNormal = normal;
        pathState.pathNormal = PathSegment.calculateArcNormal(path.getParametricPath().getFirstDerivative(target.pathT),
                path.getParametricPath().getSecondDerivative(target.pathT));
        pathState.crossTrackError = target.toPath.dot(normal).getIn();
        pathState.centripetalError = target.toPath.dot(pathState.pathNormal).getIn();
        pathState.crossTrackCorrection = crossCorrection;
        pathState.centripetalCorrection = Vector.zero();
        pathState.trackingVelocityTarget = target.tangentialVelocity();
        pathState.trackingMeasuredVelocity = localizer.getVel().getVec().dot(tangent).getIn();
        pathState.trackingProgressVelocity = pathState.progressEstimator.update(target.progressIn, pathState.trackingMeasuredVelocity, now);
        pathState.trackingAngularVelocityTarget = target.angularVelocity;
        pathState.trackingFeedforward = alongFeedforward; pathState.trackingHeadingFeedforward = headingFeedforward;
        pathState.trackingVelocityFeedback = pathState.trackingEndpointBlend = 0;
        pathState.velocitySampleAvailable = true;
        pathState.wheelFeedforwardActive = pathState.centeredBrakingActive = false;
        pathState.centeredBrakingMeanPower = 0;
        pathState.lastCommandDemand = command.demand;
        double length = path.getParametricPath().getLengthIn();
        callbackRunner.process(path, length > 1e-9 ? Math.max(0, Math.min(1, target.progressIn / length)) : 1, pose.getHeading());
        // A callback may stop or pause the movement; preserve the stopped output in that case.
        if (pathState.trajectorySample != target) return Follower.MovementOutcome.RUNNING;
        Follower.MovementOutcome outcome = pathState.completionOutcome(
                pose.getVec().distanceTo(path.getEndPose().getVec()).getIn(),
                pose.getHeading().getShortestAngleTo(path.getEndPose().getHeading()).getRad(),
                localizer.getVel().getVec(), localizer.getVel().getHeading().getRad(), distanceTolerance, headingTolerance, now);
        if (outcome == Follower.MovementOutcome.RUNNING)
            drivetrain.drive(command.drive.getX().getIn(), command.drive.getY().getIn(), command.turn);
        return outcome;
    }

    public void startTurn(Turn turn, long now) { turnExecutor.start(turn, now); }
    public void observeHeading(Angle heading) { turnExecutor.observeHeading(heading); }
    public void markTransitionTick(long now) { turnExecutor.markTransitionTick(now); }
    public void resetTurn() { turnExecutor.reset(); }
    public void pauseTurn() { turnExecutor.pause(); }
    public void resumeTurn(long now) { turnExecutor.resume(now); }
    public void resetPath() { pathState.reset(); }
    public void preparePath(Path path) { pathState.prepare(path,constants,path.getPathType()==Path.PathType.TANK?tankModel:holonomicModel); }
    public drivetrains.models.DriveModel getDriveModel(FollowerConstants.Profile profile) {
        boolean tank=profile==FollowerConstants.Profile.TANK || (profile==FollowerConstants.Profile.DEFAULT && !drivetrain.isHolonomic());
        return (tank?tankModel:holonomicModel).withConstants(constants.forProfile(profile));
    }
    public void pausePath() { pathState.pause(); }
    public void resetCallbacks() { callbackRunner.reset(); }
    public void clearCommandDemand() { pathState.lastCommandDemand = Follower.CommandDemand.ZERO; }

    public follower.FollowerTelemetry getDiagnostics() { return pathState; }
    public TrajectorySample getTrajectoryTarget() { return pathState.trajectorySample; }
}
