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
