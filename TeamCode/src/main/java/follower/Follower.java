package follower;

import config.ApexConstants;
import feedforward.HolonomicFeedforward;
import localizers.LocalizerSet;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

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
import geometry.DistUnit;
import geometry.PathSegment;
import geometry.Pose;
import geometry.Vector;
import localizers.BaseLocalizer;
import paths.movements.FollowerMovement;
import paths.movements.Path;
import paths.movements.Turn;

/**
 * Apex Pathing's public movement coordinator. Execution algorithms live in dedicated collaborators
 * while this class retains lifecycle, hardware, tuning, and diagnostic APIs.
 *
 * @author Sohum Arora - 22985 Paraducks
 * @author Joel H
 * @author Dylan B. - 18597 RoboClovers - Delta
 * @author Xander Haemel - 31616 404 Not Found
 */
public class Follower {
    private static final FollowerDiagnostics NO_DIAGNOSTICS = new FollowerDiagnostics() {
        @Override public void recordPose(Pose pose) { }
        @Override public void recordCurrentPath(Path path) { }
        @Override public void clearCurrentPath() { }
    };
    private static volatile FollowerDiagnostics diagnostics = NO_DIAGNOSTICS;

    static final double PROFILED_ENDPOINT_CAPTURE_DISTANCE_INCHES = 8.0;
    static final double ENDPOINT_STALLED_VELOCITY_IN_PER_SECOND = 0.25;
    private static final double PROFILED_HEADING_CAPTURE_RADIANS = Math.toRadians(10.0);
    static final double MAX_CENTRIPETAL_POWER = HolonomicFeedforward.MAX_CENTRIPETAL_POWER;
    private static final double SLOWED_VELOCITY_DELTA_IN_PER_SECOND = 6.0;
    private static final double STUCK_LINEAR_VELOCITY_IN_PER_SECOND = 0.5;

    private final FollowerConstants constants;
    private final BaseDrivetrain<?> drivetrain;
    private BaseLocalizer<?> localizer;
    private final java.util.function.LongSupplier monotonicClock;
    private final LocalizerSet localizerSet;
    private final double headingTolerance;
    private final double distanceTolerance;

    private final FeedbackController headingController;
    private final TurnController turnController;
    private final DriveController driveController;
    private final MovementCallbackRunner callbackRunner = new MovementCallbackRunner();
    private final FollowerPathState pathState = new FollowerPathState();
    private final HolonomicCommandAllocator commandAllocator;
    private final TurnExecutor turnExecutor;
    private final HolonomicPathFollower holonomicPathFollower;
    private final TankPathFollower tankPathFollower;

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
        static final CommandDemand ZERO =
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

        CommandDemand(boolean available, double crossTrack, double tangentCorrection,
                      double headingCorrection, double centripetal,
                      double forwardVelocity, double headingVelocity,
                      double driveFeedforward, double headingFeedforward,
                      double correctiveTotal, double velocityTotal,
                      double feedforwardTotal, double total) {
            this(available, crossTrack, tangentCorrection, headingCorrection, centripetal,
                    forwardVelocity, headingVelocity, driveFeedforward, headingFeedforward,
                    correctiveTotal, velocityTotal, feedforwardTotal, total, 1.0, 1.0, 1.0);
        }

        CommandDemand(boolean available, double crossTrack, double tangentCorrection,
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

    /** Compatibility wrapper retained for package tests; timing logic lives in its own class. */
    static final class StuckWatchdog {
        private final FollowerStuckWatchdog delegate = new FollowerStuckWatchdog();
        boolean update(boolean stuck, long nowNanos) { return delegate.update(stuck, nowNanos); }
        void reset() { delegate.reset(); }
    }

    /** Constructs the drivetrain, localizer, and follower from the given {@link ApexConstants}. */
    public Follower(ApexConstants constants, HardwareMap hardwareMap) {
        this(constants, hardwareMap, false);
    }

    public Follower(ApexConstants constants, HardwareMap hardwareMap, boolean tuningMode) {
        this(constants,hardwareMap,tuningMode,System::nanoTime);
    }

    /** Simulation may supply a monotonic clock matching its sensor acquisition timestamps. */
    public Follower(ApexConstants constants, HardwareMap hardwareMap, boolean tuningMode,
                    java.util.function.LongSupplier monotonicClock) {
        this.monotonicClock=java.util.Objects.requireNonNull(monotonicClock);
        BaseDrivetrainConstants<?> drivetrainConstants = constants.drivetrainConstants();
        drivetrain = drivetrainConstants.build(hardwareMap);
        this.constants = FollowerConstants.getInstance().copy();
        this.constants.configure(drivetrain.getDrivetrainType(),
                drivetrain.isHolonomic() ? FollowerConstants.Profile.HOLONOMIC
                        : FollowerConstants.Profile.TANK, tuningMode);
        localizerSet = new LocalizerSet(constants, hardwareMap, drivetrain.getDrivetrainType(),
                drivetrain.isHolonomic() ? FollowerConstants.Profile.HOLONOMIC
                        : FollowerConstants.Profile.TANK);
        localizer = localizerSet.getLocalizer();
        headingTolerance = drivetrainConstants.headingTolerance.getRad();
        distanceTolerance = drivetrainConstants.distanceTolerance.getIn();


        headingController = new FeedbackController(this.constants.angularCoeffs,monotonicClock);
        headingController.setAngularController();
        turnController = new TurnController(this.constants.angularCoeffs, this.constants.angularKV, this.constants.angularKA,
                this.constants.angularFeedforwardKS, monotonicClock);
        driveController = new DriveController(
                Dist.fromIn(this.constants.forwardVelLimitIn),
                Dist.fromIn(this.constants.strafeVelLimitIn),
                this.constants.translationalCoeffs,
                !tuningMode && (drivetrain instanceof Mecanum || drivetrain instanceof DualActuated),monotonicClock);

        commandAllocator = new HolonomicCommandAllocator(drivetrain, driveController);
        turnExecutor = new TurnExecutor(drivetrain, driveController, turnController,
                commandAllocator, this.constants, headingTolerance, distanceTolerance,monotonicClock);
        holonomicPathFollower = new HolonomicPathFollower(
                drivetrain, driveController, headingController, commandAllocator, this.constants,
                distanceTolerance, headingTolerance,monotonicClock);
        tankPathFollower = new TankPathFollower(
                drivetrain, driveController, this.constants, distanceTolerance, headingTolerance,monotonicClock);
    }

    /** Must be called continuously during the active OpMode loop. */
    public void update() {
        update(false);
    }

    /** Runs one follower tick and optionally holds the last commanded pose while idle. */
    public void update(boolean holdPose) {
        synchronizeDriveProfile();
        if (drivetrain instanceof DualActuated && ((DualActuated) drivetrain).isTransitioning()) {
            drivetrain.stop();
            localizer.update();
            turnExecutor.markTransitionTick(monotonicClock.getAsLong());
            return;
        }

        localizer.update();
        diagnostics.recordPose(localizer.getPose());
        if (!localizer.isLastMeasurementValid()) {
            drivetrain.stop();
            turnExecutor.markTransitionTick(monotonicClock.getAsLong());
            pathState.pause();
            return;
        }
        if (paused) {
            if (currentMovement instanceof Turn) { turnExecutor.observeHeading(getPose().getHeading()); }
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
        pathState.lastCommandDemand = CommandDemand.ZERO;
        MovementOutcome outcome;
        try {
            if (currentMovement instanceof Turn) {
                outcome = turnExecutor.update((Turn) currentMovement, current, localizer,
                        callbackRunner, pathState, driveControllerEnabled, headingControllerEnabled);
            } else if (segment == null) {
                outcome = MovementOutcome.COMPLETED;
            } else if (drivetrain.isHolonomic()) {
                outcome = holonomicPathFollower.update(
                        (Path) currentMovement, segment, current, localizer,
                        callbackRunner, pathState, headingControllerEnabled, driveControllerEnabled,
                        this.constants.translationalKV, this.constants.translationalKA, this.constants.angularKV, this.constants.angularKA, this.constants.kCentripetal,
                        this.constants.velocityFeedbackGain, this.constants.angularVelocityFeedbackGain);
            } else {
                outcome = tankPathFollower.update(
                        (Path) currentMovement, segment, current, localizer,
                        callbackRunner, pathState, this.constants.translationalKV, this.constants.translationalKA,
                        this.constants.angularKV, this.constants.angularKA, this.constants.velocityFeedbackGain, this.constants.angularVelocityFeedbackGain,
                        driveControllerEnabled, headingControllerEnabled);
            }

        } catch (RuntimeException exception) {
            latestTrajectoryTarget = null; trajectoryFailure = exception.getMessage();
            stop();
            movementOutcome = MovementOutcome.FAILED;
            diagnostics.recordTrajectoryFailure(trajectoryFailure);
            throw exception;
        }
        if (pathState.trajectorySample != null) { latestTrajectoryTarget = pathState.trajectorySample; diagnostics.recordTrajectoryTarget(latestTrajectoryTarget); }
        if (outcome != MovementOutcome.RUNNING) {
            stop();
            movementOutcome = outcome;
            if (outcome == MovementOutcome.FAILED) {
                trajectoryFailure = "Movement stalled before reaching its endpoint";
                diagnostics.recordTrajectoryFailure(trajectoryFailure);
            }
        }
    }

    private void holdPose(Pose target) {
        double angularResponse = headingControllerEnabled ? headingController.p().d().s().calculate(
                target.getHeading(AngleUnit.RAD) - getPose().getHeading(AngleUnit.RAD)).get() : 0.0;
        Vector translationalResponse;
        if (!driveControllerEnabled) {
            translationalResponse = Vector.zero();
        } else if (drivetrain.isHolonomic()) {
            translationalResponse = driveController.calculatePointToPoint(
                    target.getVec(), getPose().getVec());
        } else {
            Vector globalError = target.getVec().minus(getPose().getVec());
            Vector localError = globalError.rotate(getPose().getHeading().times(-1.0));
            translationalResponse = new Vector(
                    Dist.fromIn(driveController.calculateEndDistance(
                            localError.getX(DistUnit.IN))), Dist.zero());
        }
        if (drivetrain.isHolonomic()) {
            translationalResponse = translationalResponse.rotate(getPose().getHeading().times(-1));
        }
        drivetrain.moveWithVectors(translationalResponse.getX().getIn(),
                translationalResponse.getY().getIn(), angularResponse);
    }

    /** Starts following the given movement. */
    public void follow(FollowerMovement movement) {
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
            if (drivetrain instanceof DualActuated) {
                FollowerConstants.Profile intended = movement instanceof Path
                        ? ((Path) movement).getPathType() == Path.PathType.TANK
                                ? FollowerConstants.Profile.TANK : FollowerConstants.Profile.HOLONOMIC
                        : movement instanceof Turn && ((Turn) movement).getDriveProfile() != null
                                ? ((Turn) movement).getDriveProfile()
                                : drivetrain.isHolonomic() ? FollowerConstants.Profile.HOLONOMIC
                                        : FollowerConstants.Profile.TANK;
                constants.forProfile(intended);
                if (intended == FollowerConstants.Profile.TANK) {
                    ((DualActuated) drivetrain).activateTractionState();
                } else {
                    ((DualActuated) drivetrain).activateHolonomicState();
                }
                synchronizeDriveProfile();
            }

            currentMovement = movement;
            movementOutcome = MovementOutcome.RUNNING;
            pathState.reset();
            trajectoryFailure = null; latestTrajectoryTarget = null;
            if (movement instanceof Path) { pathState.prepare((Path) movement, this.constants); }
            movement.setStarted(true);
            movement.setEnded(false);
            lastPose = movement.getEndPose();
            if (movement instanceof Turn) {
                diagnostics.clearCurrentPath();
                turnExecutor.start((Turn) movement, monotonicClock.getAsLong());
            } else if (movement instanceof Path) {
                segment = ((Path) movement).getParametricPath();
                diagnostics.recordCurrentPath((Path) movement);
            }

            synchronizeDriveProfile();
            resetControllers();
            paused = false;
            callbackRunner.reset();
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
    public void stop() {
        if (movementOutcome == MovementOutcome.RUNNING) { movementOutcome = MovementOutcome.CANCELLED; }
        if (currentMovement != null) {
            currentMovement.setEnded(true);
        }
        currentMovement = null;
        paused = false;
        segment = null;
        turnExecutor.reset();
        pathState.reset();
        diagnostics.clearCurrentPath();
        drivetrain.stop();
    }

    public void pause() {
        paused = true; latestTrajectoryTarget = null;
        turnExecutor.pause();
        pathState.pause();
        resetControllers();
        drivetrain.stop();
    }

    public void resume() {
        if (paused) {
            paused = false;
            turnExecutor.resume(monotonicClock.getAsLong());
            pathState.pause();
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
        return pathStatusActive(currentMovement, paused) && pathState.velocitySampleAvailable
                && slowedForVelocities(pathState.trackingVelocityTarget,
                        pathState.trackingMeasuredVelocity);
    }

    public boolean isStuck() {
        return pathStatusActive(currentMovement, paused)
                && stuckForSpeed(localizer.getVel().getVec().getMag().getIn());
    }

    public void setPose(Pose pose) { localizer.setPose(pose); }
    public void setHoldPose(Pose pose) { lastPose = pose; }

    public boolean hasAcceptedKalmanFilterTuning() {
        return localizer.getVelocityFilterMode() == BaseLocalizer.VelocityFilterMode.ADAPTIVE_KALMAN
                && localizerSet.hasAcceptedFilterCalibration();
    }

    public double getTrackingVelocityTarget() { return pathState.trackingVelocityTarget; }
    /** Future speed goal and its path location; NaN when pursuit is inactive or recovering overspeed. */
    public double getVelocityGoalDistanceIn() { return pathState.onlinePlanner == null ? Double.NaN
            : pathState.onlinePlanner.getVelocityGoalDistanceIn(); }
    public double getVelocityGoal() { return pathState.onlinePlanner == null ? Double.NaN
            : pathState.onlinePlanner.getVelocityGoal(); }
    public double getCurrentSpeedCeiling() { return pathState.onlinePlanner == null ? Double.NaN
            : pathState.onlinePlanner.getCurrentSpeedCeiling(); }
    public trajectory.TrajectorySample getTrajectoryTarget() { return latestTrajectoryTarget; }
    public String getTrajectoryFailure() { return trajectoryFailure; }
    public double getTrackingMeasuredVelocity() { return pathState.trackingMeasuredVelocity; }
    public double getTrackingProgressVelocity() { return pathState.trackingProgressVelocity; }
    public double getTrackingFeedforward() { return pathState.trackingFeedforward; }
    public double getTrackingHeadingFeedforward() { return pathState.trackingHeadingFeedforward; }
    public boolean isWheelFeedforwardActive() { return pathState.wheelFeedforwardActive; }
    public double getTrackingVelocityFeedback() { return pathState.trackingVelocityFeedback; }
    public double getTrackingAngularVelocityTarget() {
        return pathState.trackingAngularVelocityTarget;
    }
    public double getTrackingEndpointBlend() { return pathState.trackingEndpointBlend; }
    public double getBestT() { return pathState.t; }
    public double getCrossTrackErrorIn() { return pathState.crossTrackError; }
    public double getCentripetalErrorIn() { return pathState.centripetalError; }
    public CommandDemand getLastCommandDemand() { return pathState.lastCommandDemand; }
    public Vector getClosestPathPoint() { return pathState.closestPathPoint; }
    public Vector getCrossTrackNormal() { return pathState.crossTrackNormal; }
    public Vector getPathNormal() { return pathState.pathNormal; }
    public Vector getCrossTrackCorrection() { return pathState.crossTrackCorrection; }
    public Vector getCentripetalCorrection() { return pathState.centripetalCorrection; }

    public boolean isHeadingControllerEnabled() { return headingControllerEnabled; }
    public boolean isDriveControllerEnabled() { return driveControllerEnabled; }

    public void disableHeadingController() { headingControllerEnabled = false; }
    /** Install an explicitly calibrated forward-axis braking model for this follower only. */
    public void setBrakingFeedforward(feedforward.CenteredBrakingFeedforward model) {
        holonomicPathFollower.setBrakingFeedforward(model);
    }
    public boolean isCenteredBrakingActive() { return pathState.centeredBrakingActive; }
    public double getCenteredBrakingMeanPower() { return pathState.centeredBrakingMeanPower; }
    public void disableDriveController() { driveControllerEnabled = false; }
    /** Calibrated mecanum paths then play an independent rest-to-rest time reference,
     * through the planned stop, without measured-motion replanning or endpoint capture. */
    public void disableControllers() { disableHeadingController(); disableDriveController(); }
    public void enableHeadingController() { headingControllerEnabled = true; }
    public void enableDriveController() { driveControllerEnabled = true; }
    public void enableControllers() { enableHeadingController(); enableDriveController(); }

    public void reset() {
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
        holonomicPathFollower.resetBrakingFeedforward();
    }

    private void synchronizeDriveProfile() {
        if (!(drivetrain instanceof DualActuated)) {
            return;
        }
        FollowerConstants.Profile profile = drivetrain.isHolonomic()
                ? FollowerConstants.Profile.HOLONOMIC : FollowerConstants.Profile.TANK;
        if (currentMovement instanceof Path
                && ((((Path) currentMovement).getPathType() == Path.PathType.TANK)
                        != (profile == FollowerConstants.Profile.TANK))) {
            stop();
            throw new IllegalStateException("Drivetrain mode changed during an active path");
        }
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

    static boolean pathStatusActive(FollowerMovement movement, boolean paused) {
        return movement instanceof Path && !paused;
    }

    static boolean slowedForVelocities(double targetVelocity, double measuredVelocity) {
        return Double.isFinite(targetVelocity) && Double.isFinite(measuredVelocity)
                && targetVelocity - measuredVelocity >= SLOWED_VELOCITY_DELTA_IN_PER_SECOND;
    }

    static boolean stuckForSpeed(double speed) {
        return Double.isFinite(speed) && speed >= 0.0
                && speed <= STUCK_LINEAR_VELOCITY_IN_PER_SECOND;
    }

    static boolean pathInsideCompletionTolerance(double endpointDistance,
                                                 double endpointHeadingError,
                                                 double configuredDistanceTolerance,
                                                 double configuredHeadingTolerance) {
        return PathCompletionPolicy.isInsideTolerance(endpointDistance, endpointHeadingError,
                configuredDistanceTolerance, configuredHeadingTolerance);
    }

    static boolean pathVelocitySettled(double linearVelocitySq, double angularVelocity) {
        return PathCompletionPolicy.isVelocitySettled(linearVelocitySq, angularVelocity);
    }

    static double commandNormalizationScale(double x, double y, double turn,
                                            boolean anisotropic) {
        return HolonomicCommandAllocator.normalizationScale(x, y, turn, anisotropic);
    }

    static double pathEndpointTangentError(Vector endpoint, Vector current, Vector endTangent) {
        return endpoint.minus(current).dot(endTangent).getIn();
    }

    static double blendProfiledEndpointPower(double profilePower, double endpointPower,
                                              double pathDistanceRemaining) {
        double blend = endpointCaptureBlend(pathDistanceRemaining);
        return profilePower * (1.0 - blend) + endpointPower * blend;
    }

    static double endpointCaptureBlend(double pathDistanceRemaining) {
        return Range.clip((PROFILED_ENDPOINT_CAPTURE_DISTANCE_INCHES - pathDistanceRemaining)
                / PROFILED_ENDPOINT_CAPTURE_DISTANCE_INCHES, 0.0, 1.0);
    }

    static double ensureEndpointBreakawayPower(double requestedPower, double endpointError,
                                               double tangentialVelocity, double staticGain,
                                               double positionTolerance,
                                               double pathDistanceRemaining) {
        boolean inEndpointCapture =
                pathDistanceRemaining < PROFILED_ENDPOINT_CAPTURE_DISTANCE_INCHES;
        boolean outsideTolerance = Math.abs(endpointError) > positionTolerance;
        boolean stalled = Math.abs(tangentialVelocity)
                < ENDPOINT_STALLED_VELOCITY_IN_PER_SECOND;
        if (!inEndpointCapture || !outsideTolerance || !stalled) {
            return requestedPower;
        }
        double minimumPower = Math.min(1.0, Math.abs(staticGain));
        if (Math.abs(requestedPower) >= minimumPower) {
            return requestedPower;
        }
        return Range.clip(requestedPower + Math.copySign(minimumPower, endpointError), -1.0, 1.0);
    }

    static double ensureAngularEndpointBreakawayPower(double requestedPower, double headingError,
                                                      double angularVelocity, double staticGain,
                                                      double headingTolerance) {
        boolean inEndpointCapture = Math.abs(headingError) < PROFILED_HEADING_CAPTURE_RADIANS;
        boolean outsideTolerance = Math.abs(headingError) > headingTolerance;
        boolean stalled = Math.abs(angularVelocity) < 0.05;
        if (!inEndpointCapture || !outsideTolerance || !stalled) {
            return requestedPower;
        }
        double minimumPower = Math.min(1.0, Math.abs(staticGain));
        if (Math.abs(requestedPower) >= minimumPower) {
            return requestedPower;
        }
        return Range.clip(requestedPower + Math.copySign(minimumPower, headingError), -1.0, 1.0);
    }

    /** Do not repeat a brake pulse while the filtered estimate lags a raw speed drop. */
    static boolean shouldPulseBrake(double filteredSpeed, double rawSpeed,
                                    double targetSpeed, double tolerance, int cooldown,
                                    boolean awaitingResponse) {
        return cooldown == 0 && Double.isFinite(filteredSpeed)
                && filteredSpeed - targetSpeed > tolerance
                && (!awaitingResponse || Double.isFinite(rawSpeed)
                    && rawSpeed >= targetSpeed);
    }

    static Vector calculateCentripetalCorrection(Vector principalNormal,
                                                 double tangentialVelocity,
                                                 double signedCurvature, double gain) {
        double magnitude = gain * tangentialVelocity * tangentialVelocity * Math.abs(signedCurvature);
        return principalNormal.times(Math.min(magnitude, MAX_CENTRIPETAL_POWER));
    }

    static double calculatePathHeadingFeedback(FeedbackController controller, double headingError,
                                               double targetRate, double measuredRate,
                                               boolean applyStatic) {
        return controller.p().d(targetRate - measuredRate)
                .s(applyStatic ? Math.signum(headingError) : 0.0)
                .calculate(headingError).get();
    }

    static double feedforwardMotionSign(double targetVelocity, double targetAcceleration) {
        return MotionFeedforward.motionSign(targetVelocity, targetAcceleration);
    }

    static double headingFeedforwardTimeScale(double targetTangentialVelocity,
                                              double measuredTangentialVelocity) {
        return MotionFeedforward.timeScale(targetTangentialVelocity, measuredTangentialVelocity);
    }

    static double scaleBrakingAcceleration(double targetVelocity, double targetAcceleration,
                                           double measuredVelocity) {
        return MotionFeedforward.scaleBrakingAcceleration(
                targetVelocity, targetAcceleration, measuredVelocity);
    }

    static double calculateTranslationFeedforward(double targetVelocity,
                                                  double targetAcceleration,
                                                  double kV, double kA, double kS) {
        double motionSign = MotionFeedforward.motionSign(targetVelocity, targetAcceleration);
        return kV * targetVelocity + kA * targetAcceleration + motionSign * kS;
    }
}
