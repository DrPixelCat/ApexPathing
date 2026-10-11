package follower;

import config.ApexConstants;
import follower.drivetrains.FollowerFactory;
import follower.internal.FollowerMath;
import follower.internal.FollowerRuntime;
import follower.internal.FollowerRuntime.PathController;
import localizers.LocalizerSet;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import controllers.DriveController;
import controllers.FeedbackController;
import controllers.FeedbackController.Coefficients;
import controllers.TurnController;
import drivetrains.BaseDrivetrain;
import drivetrains.BaseDrivetrainConstants;
import drivetrains.DualActuated;
import drivetrains.Mecanum;
import geometry.AngleUnit;
import geometry.Dist;
import geometry.PathSegment;
import geometry.Pose;
import geometry.Vector;
import localizers.BaseLocalizer;
import paths.movements.FollowerMovement;
import paths.movements.Path;
import paths.movements.Turn;

/**
 * Shared movement lifecycle for all Apex Pathing followers. Concrete subclasses select path
 * tracking and pose holding while this class owns hardware, turns, tuning, and diagnostics.
 * Use {@link #create(ApexConstants, HardwareMap)} to select a follower from robot hardware.
 *
 * @author Sohum Arora - 22985 Paraducks
 * @author Joel H
 * @author Dylan B. - 18597 RoboClovers - Delta
 * @author Xander Haemel - 31616 404 Not Found
 */
public abstract class Follower {
    private static final FollowerDiagnostics NO_DIAGNOSTICS = new FollowerDiagnostics() {
        @Override public void recordPose(Pose pose) { }
        @Override public void recordCurrentPath(Path path) { }
        @Override public void clearCurrentPath() { }
    };
    private static volatile FollowerDiagnostics diagnostics = NO_DIAGNOSTICS;

    private final FollowerConstants constants;
    private final BaseDrivetrain<?> drivetrain;
    private BaseLocalizer<?> localizer;
    private final java.util.function.LongSupplier monotonicClock;
    private final LocalizerSet localizerSet;

    private final FeedbackController headingController;
    private final TurnController turnController;
    private final DriveController driveController;
    private final FollowerRuntime runtime;
    private final FollowerTelemetry telemetry;

    private FollowerMovement currentMovement;
    private PathSegment segment;
    private Pose lastPose;
    private boolean paused;
    private String trajectoryFailure;
    public enum MovementOutcome { IDLE, RUNNING, COMPLETED, CANCELLED, FAILED }
    private MovementOutcome movementOutcome = MovementOutcome.IDLE;

    public MovementOutcome getMovementOutcome() { return movementOutcome; }
    public boolean movementSucceeded() { return movementOutcome == MovementOutcome.COMPLETED; }

    /** Acknowledge a failure before deliberately starting another movement. */
    public void clearFailure() {
        if (isBusy()) { throw new IllegalStateException("Cannot clear a running movement"); }
        trajectoryFailure = null;
        movementOutcome = MovementOutcome.IDLE;
    }
    private trajectory.TrajectorySample latestTrajectoryTarget;
    private boolean headingControllerEnabled = true;
    private boolean driveControllerEnabled = true;

    /** Raw controller demand from the most recent holonomic update, before normalization. */
    public static final class CommandDemand {
        public static final CommandDemand ZERO =
                new CommandDemand(false, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        public final boolean available;
        public final double crossTrack;
        public final double tangentCorrection;
        public final double headingCorrection;
        public final double centripetal;
        public final double forwardVelocity;
        public final double headingVelocity;
        public final double driveFeedforward;
        public final double headingFeedforward;
        public final double correctiveTotal;
        public final double velocityTotal;
        public final double feedforwardTotal;
        public final double total;
        /** Applied fractions of the requested stages; feedforward uses one coupled scale. */
        public final double correctionScale;
        public final double feedforwardScale;
        public final double velocityScale;

        public CommandDemand(boolean available, double crossTrack, double tangentCorrection,
                      double headingCorrection, double centripetal,
                      double forwardVelocity, double headingVelocity,
                      double driveFeedforward, double headingFeedforward,
                      double correctiveTotal, double velocityTotal,
                      double feedforwardTotal, double total) {
            this(available, crossTrack, tangentCorrection, headingCorrection, centripetal,
                    forwardVelocity, headingVelocity, driveFeedforward, headingFeedforward,
                    correctiveTotal, velocityTotal, feedforwardTotal, total, 1.0, 1.0, 1.0);
        }

        public CommandDemand(boolean available, double crossTrack, double tangentCorrection,
                      double headingCorrection, double centripetal,
                      double forwardVelocity, double headingVelocity,
                      double driveFeedforward, double headingFeedforward,
                      double correctiveTotal, double velocityTotal,
                      double feedforwardTotal, double total, double correctionScale,
                      double feedforwardScale, double velocityScale) {
            this.correctionScale = correctionScale;
            this.feedforwardScale = feedforwardScale;
            this.velocityScale = velocityScale;
            this.available = available;
            this.crossTrack = crossTrack;
            this.tangentCorrection = tangentCorrection;
            this.headingCorrection = headingCorrection;
            this.centripetal = centripetal;
            this.forwardVelocity = forwardVelocity;
            this.headingVelocity = headingVelocity;
            this.driveFeedforward = driveFeedforward;
            this.headingFeedforward = headingFeedforward;
            this.correctiveTotal = correctiveTotal;
            this.velocityTotal = velocityTotal;
            this.feedforwardTotal = feedforwardTotal;
            this.total = total;
        }
    }

    /** Builds the hardware once and selects the follower matching its drivetrain capabilities. */
    public static Follower create(ApexConstants constants, HardwareMap hardwareMap) {
        return create(constants, hardwareMap, false);
    }

    public static Follower create(ApexConstants constants, HardwareMap hardwareMap, boolean tuningMode) {
        return create(constants, hardwareMap, tuningMode, System::nanoTime);
    }

    /** Simulation may supply a monotonic clock matching its sensor acquisition timestamps. */
    public static Follower create(ApexConstants constants, HardwareMap hardwareMap, boolean tuningMode,
                                  java.util.function.LongSupplier monotonicClock) {
        return FollowerFactory.create(constants, hardwareMap, tuningMode, monotonicClock);
    }

    /** Initializes shared state using hardware already built and validated by a concrete follower. */
    protected Follower(ApexConstants constants, HardwareMap hardwareMap, boolean tuningMode,
                       java.util.function.LongSupplier monotonicClock, BaseDrivetrain<?> drivetrain) {
        this.monotonicClock=java.util.Objects.requireNonNull(monotonicClock);
        this.drivetrain = java.util.Objects.requireNonNull(drivetrain);
        BaseDrivetrainConstants<?> drivetrainConstants = drivetrain.getConstants();
        this.constants = FollowerConstants.getInstance().copy();
        this.constants.configure(drivetrain.getDrivetrainType(),
                drivetrain.isHolonomic() ? FollowerConstants.Profile.HOLONOMIC
                        : FollowerConstants.Profile.TANK, tuningMode);
        localizerSet = new LocalizerSet(constants, hardwareMap, drivetrain.getDrivetrainType(),
                drivetrain.isHolonomic() ? FollowerConstants.Profile.HOLONOMIC
                        : FollowerConstants.Profile.TANK);
        localizer = localizerSet.getLocalizer();
        double headingTolerance = drivetrainConstants.headingTolerance.getRad();
        double distanceTolerance = drivetrainConstants.distanceTolerance.getIn();


        headingController = new FeedbackController(this.constants.angularCoeffs,monotonicClock);
        headingController.setAngularController();
        turnController = new TurnController(this.constants.angularCoeffs, this.constants.angularKV, this.constants.angularKA,
                this.constants.angularFeedforwardKS, monotonicClock);
        driveController = new DriveController(
                Dist.fromIn(this.constants.forwardVelLimitIn),
                Dist.fromIn(this.constants.strafeVelLimitIn),
                this.constants.translationalCoeffs,
                !tuningMode && (drivetrain instanceof Mecanum || drivetrain instanceof DualActuated),monotonicClock);

        runtime = new FollowerRuntime(drivetrain, driveController, headingController,
                turnController, this.constants, distanceTolerance, headingTolerance, monotonicClock);
        telemetry = runtime.getDiagnostics();
    }

    /** Must be called continuously during the active OpMode loop. */
    public final void update() {
        update(false);
    }

    /** Runs one follower tick and optionally holds the last commanded pose while idle. */
    public final void update(boolean holdPose) {
        synchronizeDriveProfile();
        if (isDriveTransitioning()) {
            drivetrain.stop();
            localizer.update();
            runtime.markTransitionTick(monotonicClock.getAsLong());
            return;
        }

        localizer.update();
        diagnostics.recordPose(localizer.getPose());
        if (!localizer.isLastMeasurementValid()) {
            drivetrain.stop();
            runtime.markTransitionTick(monotonicClock.getAsLong());
            runtime.pausePath();
            return;
        }
        if (paused) {
            if (currentMovement instanceof Turn) { runtime.observeHeading(getPose().getHeading()); }
            drivetrain.stop();
            return;
        }
        if (currentMovement == null) {
            if (holdPose && lastPose != null) {
                holdPose(lastPose);
            }
            return;
        }

        Pose current = getPose();
        runtime.clearCommandDemand();
        MovementOutcome outcome;
        try {
            if (currentMovement instanceof Turn) {
                outcome = runtime.updateTurn((Turn) currentMovement, current, localizer,
                        driveControllerEnabled, headingControllerEnabled);
            } else if (segment == null) {
                outcome = MovementOutcome.COMPLETED;
            } else {
                outcome = updatePath((Path) currentMovement);
            }

        } catch (RuntimeException exception) {
            latestTrajectoryTarget = null; trajectoryFailure = exception.getMessage();
            stop();
            movementOutcome = MovementOutcome.FAILED;
            diagnostics.recordTrajectoryFailure(trajectoryFailure);
            throw exception;
        }
        if (runtime.getTrajectoryTarget() != null) { latestTrajectoryTarget = runtime.getTrajectoryTarget(); diagnostics.recordTrajectoryTarget(latestTrajectoryTarget); }
        if (outcome != MovementOutcome.RUNNING) {
            stop();
            movementOutcome = outcome;
            if (outcome == MovementOutcome.FAILED) {
                trajectoryFailure = "Movement stalled before reaching its endpoint";
                diagnostics.recordTrajectoryFailure(trajectoryFailure);
            }
        }
    }

    /** Computes one path tick; the base class processes completion and failures. */
    protected abstract MovementOutcome updatePath(Path path);

    protected abstract void holdPose(Pose target);

    protected abstract void resetPathControllers();

    /** Optional drivetrain hooks, called only after construction. */
    protected void prepareMovement(FollowerMovement movement) { }
    protected void synchronizeDriveProfile() { }
    protected boolean isDriveTransitioning() { return false; }

    protected final FollowerMovement getCurrentMovement() { return currentMovement; }

    protected final PathController createHolonomicPathFollower() {
        return runtime.createHolonomicController();
    }

    protected final PathController createTankPathFollower() {
        return runtime.createTankController();
    }

    protected final MovementOutcome executePath(Path path, PathController controller) {
        return controller.update(path, segment, getPose(), localizer,
                headingControllerEnabled, driveControllerEnabled);
    }

    protected final void holdPose(Pose target, boolean holonomic) {
        runtime.holdPose(target, getPose(), holonomic,
                headingControllerEnabled, driveControllerEnabled);
    }

    /** Starts following the given movement. */
    public final void follow(FollowerMovement movement) {
        java.util.Objects.requireNonNull(movement, "Movement");
        if (movementOutcome == MovementOutcome.FAILED) {
            throw new IllegalStateException("Previous movement failed: " + trajectoryFailure
                    + ". Call clearFailure() before retrying.");
        }
        if (isBusy()) {
            throw new IllegalStateException(
                    "Cannot execute a new movement while another movement is still in progress. "
                            + "Tip: use follower.isBusy() to check if the follower is currently "
                            + "executing a movement before starting a new one.");
        }

        try {
            prepareMovement(movement);

            currentMovement = movement;
            movementOutcome = MovementOutcome.RUNNING;
            runtime.resetPath();
            trajectoryFailure = null; latestTrajectoryTarget = null;
            if (movement instanceof Path) { runtime.preparePath((Path) movement); }
            movement.setStarted(true);
            movement.setEnded(false);
            lastPose = movement.getEndPose();
            if (movement instanceof Turn) {
                diagnostics.clearCurrentPath();
                runtime.startTurn((Turn) movement, monotonicClock.getAsLong());
            } else if (movement instanceof Path) {
                segment = ((Path) movement).getParametricPath();
                diagnostics.recordCurrentPath((Path) movement);
            }

            synchronizeDriveProfile();
            resetControllers();
            paused = false;
            runtime.resetCallbacks();
        } catch (RuntimeException exception) {
            latestTrajectoryTarget = null;
            trajectoryFailure = exception.getMessage();
            stop();
            movementOutcome = MovementOutcome.FAILED;
            diagnostics.recordTrajectoryFailure(trajectoryFailure);
            throw exception;
        }
    }

    /** Instantly stops the drivetrain and ends any ongoing movement. */
    public final void stop() {
        if (movementOutcome == MovementOutcome.RUNNING) { movementOutcome = MovementOutcome.CANCELLED; }
        if (currentMovement != null) {
            currentMovement.setEnded(true);
        }
        currentMovement = null;
        paused = false;
        segment = null;
        runtime.resetTurn();
        runtime.resetPath();
        diagnostics.clearCurrentPath();
        drivetrain.stop();
    }

    public final void pause() {
        paused = true; latestTrajectoryTarget = null;
        runtime.pauseTurn();
        runtime.pausePath();
        resetControllers();
        drivetrain.stop();
    }

    public final void resume() {
        if (paused) {
            paused = false;
            runtime.resumeTurn(monotonicClock.getAsLong());
            runtime.pausePath();
            resetControllers();
        }
    }

    public static void setDiagnostics(FollowerDiagnostics observer) {
        diagnostics = observer == null ? NO_DIAGNOSTICS : observer;
    }

    public void manual(double x, double y, double turn) {
        if (isBusy()) {
            stop();
        }
        drivetrain.drive(x, y, turn, getPose().getHeading(AngleUnit.RAD));
    }

    public void manual(Gamepad gamepad) {
        manual(-gamepad.left_stick_y, -gamepad.left_stick_x, -gamepad.right_stick_x);
    }

    public Pose getPose() { return localizer.getPose(); }
    public Pose getVelocity() { return localizer.getVel(); }
    public Pose getRawVelocity() { return localizer.getRawVel(); }
    public Pose getAcceleration() { return localizer.getAccel(); }
    public boolean isBusy() { return currentMovement != null; }

    public boolean isSlowed() {
        return FollowerMath.pathStatusActive(currentMovement, paused) && telemetry.isVelocitySampleAvailable()
                && FollowerMath.slowedForVelocities(telemetry.getTrackingVelocityTarget(),
                        telemetry.getTrackingMeasuredVelocity());
    }

    public boolean isStuck() {
        return FollowerMath.pathStatusActive(currentMovement, paused)
                && FollowerMath.stuckForSpeed(localizer.getVel().getVec().getMag().getIn());
    }

    public void setPose(Pose pose) { localizer.setPose(pose); }
    public void setHoldPose(Pose pose) { lastPose = pose; }

    public boolean hasAcceptedKalmanFilterTuning() {
        return localizer.getVelocityFilterMode() == BaseLocalizer.VelocityFilterMode.ADAPTIVE_KALMAN
                && localizerSet.hasAcceptedFilterCalibration();
    }

    /** Stable read-only view of the latest tracking values; existing getters remain supported. */
    public final FollowerTelemetry getDiagnostics() { return telemetry; }

    public double getTrackingVelocityTarget() { return telemetry.getTrackingVelocityTarget(); }
    /** Future speed goal and its path location; NaN when pursuit is inactive or recovering overspeed. */
    public double getVelocityGoalDistanceIn() { return telemetry.getVelocityGoalDistanceIn(); }
    public double getVelocityGoal() { return telemetry.getVelocityGoal(); }
    public double getCurrentSpeedCeiling() { return telemetry.getCurrentSpeedCeiling(); }
    public trajectory.TrajectorySample getTrajectoryTarget() { return latestTrajectoryTarget; }
    public String getTrajectoryFailure() { return trajectoryFailure; }
    public double getTrackingMeasuredVelocity() { return telemetry.getTrackingMeasuredVelocity(); }
    public double getTrackingProgressVelocity() { return telemetry.getTrackingProgressVelocity(); }
    public double getTrackingFeedforward() { return telemetry.getTrackingFeedforward(); }
    public double getTrackingHeadingFeedforward() { return telemetry.getTrackingHeadingFeedforward(); }
    public boolean isWheelFeedforwardActive() { return telemetry.isWheelFeedforwardActive(); }
    public double getTrackingVelocityFeedback() { return telemetry.getTrackingVelocityFeedback(); }
    public double getTrackingAngularVelocityTarget() { return telemetry.getTrackingAngularVelocityTarget(); }
    public double getTrackingEndpointBlend() { return telemetry.getTrackingEndpointBlend(); }
    public double getBestT() { return telemetry.getBestT(); }
    public double getCrossTrackErrorIn() { return telemetry.getCrossTrackErrorIn(); }
    public double getCentripetalErrorIn() { return telemetry.getCentripetalErrorIn(); }
    public CommandDemand getLastCommandDemand() { return telemetry.getLastCommandDemand(); }
    public Vector getClosestPathPoint() { return telemetry.getClosestPathPoint(); }
    public Vector getCrossTrackNormal() { return telemetry.getCrossTrackNormal(); }
    public Vector getPathNormal() { return telemetry.getPathNormal(); }
    public Vector getCrossTrackCorrection() { return telemetry.getCrossTrackCorrection(); }
    public Vector getCentripetalCorrection() { return telemetry.getCentripetalCorrection(); }

    public boolean isHeadingControllerEnabled() { return headingControllerEnabled; }
    public boolean isDriveControllerEnabled() { return driveControllerEnabled; }

    public void disableHeadingController() { headingControllerEnabled = false; }
    /** Install a holonomic braking model. Has no effect on tank-only followers. */
    public void setBrakingFeedforward(feedforward.CenteredBrakingFeedforward model) {
    }
    public boolean isCenteredBrakingActive() { return telemetry.isCenteredBrakingActive(); }
    public double getCenteredBrakingMeanPower() { return telemetry.getCenteredBrakingMeanPower(); }
    public void disableDriveController() { driveControllerEnabled = false; }
    /** Calibrated mecanum paths then play an independent rest-to-rest time reference,
     * through the planned stop, without measured-motion replanning or endpoint capture. */
    public void disableControllers() { disableHeadingController(); disableDriveController(); }
    public void enableHeadingController() { headingControllerEnabled = true; }
    public void enableDriveController() { driveControllerEnabled = true; }
    public void enableControllers() { enableHeadingController(); enableDriveController(); }

    public final void reset() {
        headingController.setCoefficients(constants.angularCoeffs);
        turnController.setCoefficients(constants.angularCoeffs);
        turnController.setMotionGains(this.constants.angularKV, this.constants.angularKA, constants.angularFeedforwardKS,
                this.constants.angularVelocityFeedbackGain);
        driveController.setCoefficients(constants.translationalCoeffs);
        driveController.setVelocityLimits(Dist.fromIn(constants.forwardVelLimitIn),
                Dist.fromIn(constants.strafeVelLimitIn), false);
        resetControllers();
    }

    private void resetControllers() {
        headingController.reset();
        turnController.reset();
        driveController.reset();
        resetPathControllers();
    }

    /** Refreshes shared controllers and localization when a subclass changes drive profile. */
    protected final void selectDriveProfile(FollowerConstants.Profile profile) {
        if (constants.getActiveProfile() != profile) {
            try {
                constants.selectProfile(profile);
            } catch (RuntimeException exception) {
                drivetrain.stop();
                throw exception;
            }
            reset();
        }
        localizerSet.select(profile);
        localizer = localizerSet.getLocalizer();
    }

    public void setHeadingCoefficients(Coefficients coefficients) {
        headingController.setCoefficients(coefficients);
        turnController.setCoefficients(coefficients);
    }

    public void setDriveCoefficients(Coefficients coefficients) {
        driveController.setCoefficients(coefficients);
    }

    public void setCentripetal(double kCentripetal) { this.constants.kCentripetal = kCentripetal; }

    public void setVelocityFeedback(double velocityFeedbackGain,
                                    double angularVelocityFeedbackGain) {
        this.constants.velocityFeedbackGain = velocityFeedbackGain;
        this.constants.angularVelocityFeedbackGain = angularVelocityFeedbackGain;
        turnController.setMotionGains(
                this.constants.angularKV,
                this.constants.angularKA,
                this.constants.angularVelocityFeedbackGain);
    }

    public void setFeedforwardGains(double translationalKV, double translationalKA,
                                    double angularKV, double angularKA) {
        this.constants.translationalKV = translationalKV;
        this.constants.translationalKA = translationalKA;
        this.constants.angularKV = angularKV;
        this.constants.angularKA = angularKA;
        turnController.setMotionGains(
                this.constants.angularKV,
                this.constants.angularKA,
                this.constants.angularFeedforwardKS,
                this.constants.angularVelocityFeedbackGain);
    }

    public BaseLocalizer<?> getLocalizer() { return localizer; }
    public BaseDrivetrain<?> getDrivetrain() { return drivetrain; }
    public FollowerConstants getConstants() { return constants; }
    public drivetrains.models.DriveModel getDriveModel() {
        return getDriveModel(drivetrain.isHolonomic() ? FollowerConstants.Profile.HOLONOMIC : FollowerConstants.Profile.TANK);
    }
    /** Hardware-free model snapshot for either profile; this never actuates the drivetrain. */
    public drivetrains.models.DriveModel getDriveModel(FollowerConstants.Profile profile) {
        return runtime.getDriveModel(profile);
    }

}
