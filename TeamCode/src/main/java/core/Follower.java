package core;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import controllers.MecanumDriveController;
import controllers.PDSController;
import drivetrains.BaseDrivetrain;
import drivetrains.BaseDrivetrainConstants;
import drivetrains.CoaxialSwerve;
import drivetrains.DualActuated;
import drivetrains.Tank;
import feedforward.MotionParameters;
import geometry.Angle;
import geometry.Dist;
import geometry.PathSegment;
import geometry.Pose;
import geometry.Vector;
import localizers.BaseLocalizer;
import paths.callbacks.Callback;
import paths.movements.FollowerMovement;
import paths.movements.Path;
import paths.movements.Turn;

/**
 * Apex Pathing main Follower class. Handles the execution of generated paths and turns using
 * kinematic feedforward and feedback controllers.
 *
 * @author Sohum Arora 22985 Paraducks
 * @author DrPixelCat
 * @author Dylan B. 18597 RoboClovers - Delta
 * @author Xander Haemel 31616 404 Not Found
 */
public class Follower {
    private final FollowerConstants constants;
    private final BaseDrivetrain<?> drivetrain;
    private final BaseLocalizer<?> localizer;

    private final double headingTol; // Radians
    private final double distanceTol; // Inches

    private double lastS = -1.0;
    private long lastNano = -1;
    private Angle lastHeading = null; // Tracks heading between ticks for angular callback sweeps

    private final PDSController headingController;
    private final MecanumDriveController mecanumDriveController;
    private final double velocityFeedbackGain;

    private FollowerMovement currentMovement = null;
    private boolean paused = false;

    PathSegment segment;
    Angle targetHeading;
    Vector targetTurnPoseVec;

    /** Constructs the drivetrain, localizer, and follower from the given {@link ApexConstants}. */
    public Follower(ApexConstants constants, HardwareMap hardwareMap) {
        BaseDrivetrainConstants<?> drivetrainConstants = constants.drivetrainConstants();

        this.drivetrain = drivetrainConstants.build(hardwareMap);
        this.localizer = constants.localizerConstants().build(hardwareMap);
        this.constants = new FollowerConstants();

        this.headingTol = drivetrainConstants.headingTolerance.getRad();
        this.distanceTol = drivetrainConstants.distanceTolerance.getIn();

        this.headingController = new PDSController(this.constants.headingCoeffs);
        this.headingController.setAngularController();

        mecanumDriveController = new MecanumDriveController(
                this.constants.forwardVelocityLimit,
                this.constants.strafeVelocityLimit,
                this.constants.translationalCoeffs,
                Dist.fromIn(0.25) // TODO: remove this hardcode ??
        );
        velocityFeedbackGain = this.constants.velocityFeedbackGain;
    }

    // region Callbacks

    /**
     * Evaluates all callbacks attached to the current movement and executes them if their
     * conditions are met.
     * * @param t The current geometric progression percentage [0.0, 1.0]. Pass -1.0 for turns.
     *
     * @param currentHeading The robot's current field orientation.
     */
    private void processCallbacks(double t, Angle currentHeading) {
        Callback[] callbacks = null;
        if (currentMovement instanceof Path) {
            callbacks = ((Path) currentMovement).getCallbacks();
        } else if (currentMovement instanceof Turn) {
            callbacks = ((Turn) currentMovement).getCallbacks();
        }

        if (callbacks != null) {
            for (Callback cb : callbacks) {
                if (cb.isTriggered()) continue;

                boolean shouldTrigger = false;

                if (cb.getType() == Callback.CallbackType.DISTANCE) {
                    if (t >= cb.getS() && t >= 0.0) {
                        shouldTrigger = true;
                    }
                } else if (cb.getType() == Callback.CallbackType.ANGLE) {
                    double error =
                            Math.abs(currentHeading.getShortestAngleTo(cb.getTheta()).getRad());

                    // Trigger if resting within 1 degree of target
                    if (error < Math.toRadians(1.0)) {
                        shouldTrigger = true;
                    }
                    // Trigger if the target angle was swept past between the last tick and this
                    // tick
                    else if (lastHeading != null) {
                        double tickSweep = lastHeading.getShortestAngleTo(currentHeading).getRad();
                        double targetSweep = lastHeading.getShortestAngleTo(cb.getTheta()).getRad();

                        // If sweeps are in the same direction AND the tick sweep is larger, it
                        // was crossed
                        if (Math.signum(tickSweep) == Math.signum(targetSweep) && Math.abs(targetSweep) <= Math.abs(tickSweep)) {
                            shouldTrigger = true;
                        }
                    }
                }

                if (shouldTrigger) {
                    cb.getAction().run();
                    cb.setTriggered(true);
                }
            }
        }
        lastHeading = currentHeading;
    }

    // region General methods
    /**
     * The main execution loop of the follower.
     * Must be called continuously during the active OpMode loop to drive the robot along the path.
     */
    public void update() {
        // Exit early if nothing is running
        if (currentMovement == null || paused) {
            return;
        }

        Pose current = getPose();
        Vector currentPos = current.getVec();
        Angle currentHeading = current.getHeading();

        double headingError = targetHeading.getShortestAngleTo(currentHeading).getRad();
        double headingFeedforward = headingController.calculateFromError(headingError);
        long currentNano = System.nanoTime();

        // Calculate delta time for velocity feedback
        double deltaT_seconds = (lastNano != -1) ? (currentNano - lastNano) / 1e9 : 0.0;
        lastNano = currentNano;

        // region Turn Execution
        if (currentMovement instanceof Turn) {
            Turn turn = (Turn) currentMovement;

            // Process angular callbacks
            processCallbacks(-1.0, currentHeading);

            // Require both positional accuracy and low angular velocity to prevent momentum
            // overshoot
            double currentAngularVel = localizer.getVel().getHeading().getRad();
            if (Math.abs(headingError) < headingTol && Math.abs(currentAngularVel) < 0.05) {
                this.stop();
                return;
            }

            // Calculate heading power with optional profile feedforward
            double headingFF = 0.0;
            if (turn.getFeedforwardLut() != null) {
                // Query profile using absolute angular distance remaining
                MotionParameters turnTargets = turn.getFeedforwardLut()
                        .getFeedforwardParams(Math.abs(headingError));
                headingFF = (turnTargets.getAngularVel() * constants.angularKV) +
                        (turnTargets.getAngularAccel() * constants.angularKA);
                if (Math.abs(turnTargets.getAngularVel()) > 1e-6) {
                    headingFF += Math.signum(turnTargets.getAngularVel()) *
                            constants.headingCoeffs.kS;
                }
            }

            double headingFeedback = headingController.calculateFromError(headingError);
            double totalTurnPower = Range.clip(headingFeedback + headingFF, -1.0, 1.0);

            Vector error = targetTurnPoseVec.minus(currentPos);
            double errorMag = error.getMag().getIn();

            // Hold xy position actively while turning
            if (errorMag > 0) {
                Vector feedback = mecanumDriveController.calculatePointToPoint(
                        targetTurnPoseVec,
                        currentPos,
                        currentHeading
                );
                drivetrain.drive(feedback.getX().getIn(), feedback.getY().getIn(), totalTurnPower);
            } else {
                drivetrain.drive(0, 0, totalTurnPower);
            }

        } else if (segment == null) {
            this.stop();
            // region Holonomic Following
        } else if (drivetrain.isHolonomic()) {
            // Retrieve path geometry at closest point
            double t = segment.getBestT(currentPos);

            // Process scheduled distance and angular callbacks
            processCallbacks(t, currentHeading);

            Vector targetPoseVec = segment.getPosition(t);
            double s = segment.getDistanceToEndIn(targetPoseVec, t);
            Vector velVec = segment.getFirstDerivative(t);
            Vector accelVec = segment.getSecondDerivative(t);
            Vector normal = PathSegment.calculateArcNormal(velVec, accelVec);
            Vector unitTangent = velVec.normalize();
            Vector endTangent = segment.getFirstDerivative(1.0).normalize();

            Vector robotVel = localizer.getVel().getVec();
            double distanceRemaining = segment.getDistanceToEndIn(targetPoseVec, t);
            double kappa = segment.getSignedCurvature(t);
            double dKappa = segment.getCurvatureDerivative(t);

            Path path = (Path) currentMovement;
            boolean isProfiled = path.isProfiled();
            MotionParameters targets = isProfiled ?
                    path.getFeedforwardLut().getFeedforwardParams(s) : null;

            boolean isMecanum =
                    !(drivetrain instanceof CoaxialSwerve) && !(drivetrain instanceof Tank);
            boolean isSwerve = drivetrain instanceof CoaxialSwerve;

            double robotTangentialVel = (deltaT_seconds > 1e-6 && lastS >= 0.0) ?
                    (lastS - s) / deltaT_seconds : 0.0;
            lastS = s;

            // Calculate heading power allocation
            Angle headingTarg = path.getInterpolator().getHeadingTarg(s, velVec, endTangent);
            double fPrime = path.getInterpolator().getHeadingFirstDerivative(s, kappa, endTangent);
            double fDoublePrime = path.getInterpolator().getHeadingSecondDerivative(s, dKappa,
                    endTangent);

            double headingFF = 0.0;
            if (isProfiled) {
                double omegaTarget = fPrime * robotTangentialVel;
                double alphaTarget =
                        (fDoublePrime * (robotTangentialVel * robotTangentialVel)) + (fPrime * targets.getTangentialAccel());

                headingFF = (omegaTarget * constants.angularKV) + (alphaTarget * constants.angularKA);
                if (Math.abs(omegaTarget) > 1e-6) {
                    headingFF += Math.signum(omegaTarget) * constants.headingCoeffs.kS;
                }
            }

            double headingFeedback =
                    headingController.calculateFromError(headingTarg.getRad() - currentHeading.getRad());
            double turnPow = Range.clip(headingFeedback + headingFF, -1.0, 1.0);
            double availableMotorPower = 1.0 - Math.abs(turnPow);

            // Calculate lateral cross track power allocation
            Vector positionalError = targetPoseVec.minus(currentPos);
            double crossTrackError = positionalError.dot(normal).getIn();
            double lateralFeedbackMag =
                    mecanumDriveController.pds.calculateFromError(crossTrackError);

            double requiredLateralAccel = (robotTangentialVel * robotTangentialVel) * kappa;
            double centripetalMag = requiredLateralAccel * constants.Kcentripetal;

            double netLateralMag = Range.clip(centripetalMag + lateralFeedbackMag,
                    -availableMotorPower, availableMotorPower);
            Vector rawLateralDriveVec = normal.times(netLateralMag);

            // Calculate tangential forward power allocation
            double tangentBudget;
            if (isSwerve) {
                tangentBudget =
                        Math.sqrt((availableMotorPower * availableMotorPower) - (netLateralMag * netLateralMag));
            } else {
                tangentBudget = availableMotorPower - Math.abs(netLateralMag);
            }

            double totalTangentPower;
            if (t < 1.0) {
                if (isProfiled) {
                    double feedforward = (constants.translationalKV * targets.getTangentialVel()) +
                            (constants.translationalKA * targets.getTangentialAccel()) +
                            (Math.signum(targets.getTangentialVel()) * constants.translationalCoeffs.kS);

                    totalTangentPower =
                            ((targets.getTangentialVel() - robotTangentialVel) * velocityFeedbackGain) + feedforward; //TODO: Verify p only feedback performance, compare to SquID
                    if (path.isAccelBoosted()) {
                        totalTangentPower = Math.min(
                                totalTangentPower,
                                mecanumDriveController.pds.calculateFromError(distanceRemaining));
                    }
                } else {
                    double decelPower =
                            mecanumDriveController.pds.calculateFromError(distanceRemaining);
                    double percentage = 1.0 - (s / path.getParametricPath().getLengthIn());
                    double percentageClipped = Math.min(Math.max(percentage, 0.0), 1.0);
                    double maxVel = path.getQuickVelocityLimit(percentageClipped,
                            constants.forwardVelocityLimit.getIn());
                    double velError = maxVel - robotTangentialVel;
                    double accelPower = (maxVel * constants.translationalKV)
                            + (Math.signum(maxVel) * constants.translationalCoeffs.kS)
                            + (velError * velocityFeedbackGain);
                    totalTangentPower = Math.min(accelPower, decelPower);
                }
            } else {
                // Apply reverse feedback if robot drifts past the final point
                double distancePastEnd = currentPos.minus(targetPoseVec).dot(endTangent).getIn();
                totalTangentPower = mecanumDriveController.pds.calculateFromError(-distancePastEnd);
            }

            totalTangentPower = Range.clip(totalTangentPower, -tangentBudget, tangentBudget);
            Vector rawTangentDriveVec = unitTangent.times(totalTangentPower);

            // Combine and filter output
            Vector rawTranslationalOutput = rawTangentDriveVec.plus(rawLateralDriveVec);
            Vector finalDriveOutput;

            if (isMecanum) {
                finalDriveOutput =
                        mecanumDriveController.applyMecanumCorrections(rawTranslationalOutput,
                                currentHeading);
            } else {
                finalDriveOutput = rawTranslationalOutput;
            }

            // Check stop condition and drive hardware
            if (distanceRemaining < distanceTol && robotVel.getMagSq().getIn() < 25) {
                stop();
                return;
            }

            drivetrain.drive(finalDriveOutput.getX().getIn(), finalDriveOutput.getY().getIn(),
                    turnPow);
            // region Tank Following
        } else {
            // Process tank driving via Ramsete controller
            double t = segment.getBestT(currentPos);

            // Process scheduled distance and angular callbacks
            processCallbacks(t, currentHeading);

            Vector targetPoseVec = segment.getPosition(t);
            double s = segment.getDistanceToEndIn(targetPoseVec, t);
            Vector velVec = segment.getFirstDerivative(t);
            Vector robotVel = localizer.getVel().getVec();

            Path path = (Path) currentMovement;
            Angle headingTarg = path.getInterpolator().getHeadingTarg(s, velVec,
                    segment.getFirstDerivative(1.0));
            MotionParameters targets = path.getFeedforwardLut().getFeedforwardParams(s);

            double v_d = targets.getTangentialVel();
            double a_d = targets.getTangentialAccel();
            double omega_d = targets.getAngularVel();
            double alpha_d = targets.getAngularAccel();

            // Transform global error to robot local frame
            Vector globalError = targetPoseVec.minus(currentPos);
            Vector localError = globalError.rotate(Angle.fromRad(-currentHeading.getRad()));

            double e_x = localError.getX().getIn();
            double e_y = localError.getY().getIn();
            double e_theta = headingTarg.getShortestAngleTo(currentHeading).getRad();

            // Calculate non linear Ramsete gains
            double b = 2.0;
            double zeta = 0.7;
            double k = 2.0 * zeta * Math.sqrt(Math.pow(omega_d, 2) + b * Math.pow(v_d, 2));
            double sinc = (Math.abs(e_theta) < 1e-6) ? 1.0 : (Math.sin(e_theta) / e_theta);

            double v_cmd = v_d * Math.cos(e_theta) + k * e_x;
            double w_cmd = omega_d + k * e_theta + b * v_d * sinc * e_y;

            // Convert velocity commands to motor power using feedforward constants
            double totalTangentPower =
                    (v_cmd * constants.translationalKV) + (a_d * constants.translationalKA) + (Math.signum(v_cmd) * constants.translationalCoeffs.kS);
            double turnPow = (w_cmd * constants.angularKV) + (alpha_d * constants.angularKA);
            turnPow += Math.signum(turnPow) * constants.headingCoeffs.kS;

            double availableMotorPower = 1.0;
            turnPow = Range.clip(turnPow, -availableMotorPower, availableMotorPower);
            availableMotorPower -= Math.abs(turnPow);
            totalTangentPower = Range.clip(totalTangentPower, -availableMotorPower,
                    availableMotorPower);

            if (s < distanceTol && robotVel.getMagSq().getIn() < 16) {
                stop();
                return;
            }

            drivetrain.drive(totalTangentPower, 0.0, turnPow);
        }
    }

    //region Initialize Sequence

    /**
     * Starts following the given movement.
     *
     * @param movement The movement object to be executed.
     * @throws IllegalStateException if the follower is already busy executing a movement.
     */
    public void follow(FollowerMovement movement) {
        if (isBusy()) {
            throw new IllegalStateException(
                    "Cannot execute a new movement while another movement is still in progress " +
                            "Tip use follower.isBusy() to check if the follower is currently " +
                            "executing a movement before starting a new one."
            );
        }

        this.currentMovement = movement;
        this.currentMovement.setStarted(true);
        this.currentMovement.setEnded(false);
        this.targetHeading = movement.getEndPose().getHeading();

        if (movement instanceof Turn) {
            Turn turn = (Turn) currentMovement;
            this.targetTurnPoseVec = turn.getStartPose().getVec();
        } else if (movement instanceof Path) {
            Path pathSegmentMove = (Path) currentMovement;
            this.segment = pathSegmentMove.getParametricPath();
            if (drivetrain instanceof DualActuated) {
                if (pathSegmentMove.getPathType() == Path.PathType.HOLONOMIC) {
                    ((DualActuated) drivetrain).activateHolonomicState();
                } else {
                    ((DualActuated) drivetrain).activateTractionState();
                }
            }
        }

        headingController.reset();
        mecanumDriveController.pds.reset();

        // Reset sweeping tracker for angular callbacks so it doesn't instantly trigger on path
        // start
        lastHeading = null;
    }

    // region Public Methods

    /** Instantly stops the drivetrain and ends any ongoing movement. */
    public void stop() {
        if (this.currentMovement != null) {
            this.currentMovement.setEnded(true);
        }

        this.currentMovement = null;
        this.segment = null;
        this.targetHeading = null;
        this.targetTurnPoseVec = null;

        this.drivetrain.stop();
    }

    /** Halts the current movement temporarily without clearing the target state */
    public void pause() {
        this.paused = true;
        this.drivetrain.stop();
    }

    /** Resumes a paused movement from the robots current location. */
    public void resume() {
        if (this.paused) {
            this.paused = false;
        }
    }

    /**
     * Drives the robot using joystick inputs adjusted for field centric control.
     * Stops any active autonomous movement.
     * Drives the robot using the provided  inputs. The joystick inputs are adjusted for
     * field-centric or robot-centric control based on the constants. This method  will stop the
     * current movement if one is in progress, as manual control takes priority over following a path.
     *
     * @param x left/right joystick input (positive for right, negative for left)
     * @param y forward/backward joystick input (positive for forward, negative for backward)
     * @param turn rotation joystick input (positive for clockwise, negative for counterclockwise)
     */
    public void teleOpDrive(double x, double y, double turn) {
        if (isBusy()) { stop(); }
        drivetrain.drive(x, y, turn, this.getPose().getHeading().getRad());
    }

    /**
     * Drives the robot using standard gamepad inputs. The left stick controls translation (x and y),
     * and the right stick controls rotation (turn). The joystick inputs are adjusted for
     * field-centric or robot-centric control based on the constants. This method will stop the
     * current movement if one is in progress, as manual control takes priority over following a path.
     *
     * @param gamepad the gamepad to read inputs from
     */
    public void teleOpDrive(Gamepad gamepad) {
        teleOpDrive(-gamepad.left_stick_x, -gamepad.left_stick_y, -gamepad.right_stick_x);
    }
    // endregion

    /**
     * Retrieves the robots current pose estimate from the localizer.
     *
     * @return The current global position and heading.
     */
    public Pose getPose() { return localizer.getPose(); }

    /**
     * Checks if the follower is currently executing a movement.
     *
     * @return true if a movement is in progress false otherwise.
     */
    public boolean isBusy() { return currentMovement != null; }

    /**
     * Forcibly overrides the localizers current pose estimate.
     *
     * @param pose The new global position and heading.
     */
    public void setPose(Pose pose) { localizer.setPose(pose); }

    /**
     * Retrieves the robots current velocity estimate from the localizer.
     *
     * @return The current velocity expressed in robot local frame.
     */
    public Pose getVelocity() { return localizer.getVel(); }

    /**
     * Retrieves the robots current acceleration estimate from the localizer.
     *
     * @return The current acceleration expressed in robot local frame.
     */
    public Pose getAcceleration() { return localizer.getAccel(); }

    /**
     * DO NOT USE THIS METHOD UNLESS YOU KNOW WHAT YOU ARE DOING.
     * It is intended for internal use only.
     */
    public BaseLocalizer<?> getLocalizer() { return localizer; }

    /**
     * DO NOT USE THIS METHOD UNLESS YOU KNOW WHAT YOU ARE DOING.
     * It is intended for internal use only.
     */
    public BaseDrivetrain<?> getDrivetrain() { return drivetrain; }

    public FollowerConstants getConstants() { return constants; }
    // endregion
}