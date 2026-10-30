package controllers;

import paths.geometry.Angle;
import paths.geometry.Dist;
import paths.geometry.Vector;

/**
 * Shared translational controller and holonomic power allocator.
 *
 * <p>Field-space path corrections are converted to robot space here. Mecanum allocation
 * additionally applies the robot-relative strafe penalty and measures power using mecanum wheel
 * demand {@code |forward| + |strafe|}. Isotropic allocation is used by swerve and other holonomic
 * drives.
 *
 * @author Joel H
 */
public class DriveController {
    private static final double EPSILON = 1e-9;

    /** One robot-centric command together with the translation budget it consumes. */
    public static class AllocatedCommand {
        private final Vector robotCommand;
        private final double powerDemand;

        private AllocatedCommand(Vector robotCommand, double powerDemand) {
            this.robotCommand = robotCommand;
            this.powerDemand = powerDemand;
        }

        public Vector getRobotCommand() { return robotCommand; }

        public double getPowerDemand() { return powerDemand; }
    }

    private double strafePenaltyRatio;
    private final FeedbackController crossTrackController;
    private final FeedbackController endDistanceController;
    private final FeedbackController turnPositionController;
    private boolean crossTrackStaticCompensationEnabled = true;

    public DriveController(Dist maxForwardVelocity, Dist maxStrafeVelocity,
                           FeedbackController.Coefficients coefficients,
                           boolean requireMecanumLimits) {
        this(maxForwardVelocity,maxStrafeVelocity,coefficients,requireMecanumLimits,System::nanoTime);
    }

    public DriveController(Dist maxForwardVelocity, Dist maxStrafeVelocity,
                           FeedbackController.Coefficients coefficients, boolean requireMecanumLimits,
                           java.util.function.LongSupplier clock) {
        crossTrackController = new FeedbackController(coefficients,clock);
        endDistanceController = new FeedbackController(coefficients,clock);
        turnPositionController = new FeedbackController(coefficients,clock);
        setVelocityLimits(maxForwardVelocity, maxStrafeVelocity, requireMecanumLimits);
    }

    public void setVelocityLimits(Dist maxForwardVelocity, Dist maxStrafeVelocity,
                                  boolean requireMecanumLimits) {
        double forwardVelocity = maxForwardVelocity.getIn();
        double strafeVelocity = maxStrafeVelocity.getIn();
        boolean invalidLimits = !Double.isFinite(forwardVelocity) || forwardVelocity <= 0.0 ||
                !Double.isFinite(strafeVelocity) || strafeVelocity <= 0.0;

        // TODO: Re-enable this once we figure out a good way to not require mecanum limits when its untuned
        /*
        if (requireMecanumLimits && invalidLimits) {
            throw new IllegalArgumentException(
                    "Mecanum forward and strafe velocity limits must both be positive."
            );
        }
        */

        strafePenaltyRatio = invalidLimits ? 1.0 : forwardVelocity / strafeVelocity;
    }

    public void setCoefficients(FeedbackController.Coefficients coefficients) {
        crossTrackController.setCoefficients(coefficients);
        endDistanceController.setCoefficients(coefficients);
        turnPositionController.setCoefficients(coefficients);
        reset();
    }

    /** Returns an unallocated field-centric correction toward a fixed position. */
    public Vector calculatePointToPoint(Vector targetPos, Vector currentPos) {
        Vector fieldError = targetPos.minus(currentPos);

        double basePower = turnPositionController.p().d().s()
                .calculate(fieldError.getMag().getIn()).get();
        return Vector.fromPolar(Dist.fromIn(basePower), fieldError.getTheta());
    }

    public double calculateCrossTrack(double error) {
        return crossTrackController.p().d()
                .s(crossTrackStaticCompensationEnabled ? Math.signum(error) : 0.0)
                .calculate(error).get();
    }

    /** Enables or disables static-friction compensation for cross-track correction. */
    public void setCrossTrackStaticCompensationEnabled(boolean enabled) {
        crossTrackStaticCompensationEnabled = enabled;
    }

    public double calculateEndDistance(double error) {
        return endDistanceController.p().d().s().calculate(error).get();
    }

    /**
     * Calculates endpoint power using a measured error rate instead of differentiating projected
     * path distance. For a fixed endpoint, pass the negative measured tangential velocity.
     */
    public double calculateEndDistance(double error, double errorRate) {
        return endDistanceController.p().d(errorRate).s().calculate(error).get();
    }

    /** Allocate a position correction using the drivetrain's command shaping and motor demand. */
    public AllocatedCommand allocatePositionHold(Vector fieldCommand, Angle currentHeading,
                                                  double availablePower, drivetrains.models.DriveModel model) {
        Vector command = model.positionHoldCommand(fieldToRobotCentric(fieldCommand, currentHeading));
        return limitByDemand(command, model.commandDemand(command, 0), availablePower);
    }

    /** Legacy mecanum allocation; follower code uses the drivetrain's physical model. */
    public AllocatedCommand allocateMecanum(Vector fieldCommand, Angle currentHeading,
                                             double availablePower) {
        Vector robotCommand = fieldToRobotCentric(fieldCommand, currentHeading);
        Vector penalized = applyMecanumDirectionalPenalty(robotCommand);
        return limitByDemand(penalized, mecanumWheelDemand(penalized), availablePower);
    }

    /** Allocates one field-centric stage for an isotropic holonomic drivetrain such as swerve. */
    public AllocatedCommand allocateIsotropic(Vector fieldCommand, Angle currentHeading,
                                               double availablePower) {
        Vector robotCommand = fieldToRobotCentric(fieldCommand, currentHeading);
        return limitByDemand(robotCommand, robotCommand.getMag().getIn(), availablePower);
    }

    public static Vector fieldToRobotCentric(Vector fieldVector, Angle currentHeading) {
        return fieldVector.rotate(currentHeading.times(-1.0));
    }

    private Vector applyMecanumDirectionalPenalty(Vector robotCommand) {
        return new Vector(
                robotCommand.getX(),
                Dist.fromIn(robotCommand.getY().getIn() * strafePenaltyRatio)
        );
    }

    private static double mecanumWheelDemand(Vector robotCommand) {
        return Math.abs(robotCommand.getX().getIn()) + Math.abs(robotCommand.getY().getIn());
    }

    private static AllocatedCommand limitByDemand(Vector command, double demand, double available) {
        double budget = Math.max(0.0, available);
        if (demand <= EPSILON || budget <= EPSILON) {
            return new AllocatedCommand(Vector.zero(), 0.0);
        }
        if (demand > budget) {
            return new AllocatedCommand(command.times(budget / demand), budget);
        }
        return new AllocatedCommand(command, demand);
    }

    public void reset() {
        crossTrackController.reset();
        endDistanceController.reset();
        turnPositionController.reset();
    }
}
