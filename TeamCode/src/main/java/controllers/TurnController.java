package controllers;

import feedforward.MotionParameters;

/**
 * Heading feedback for quick turns and feedback/feedforward for stored point turns.
 *
 * <p>Quick turns target the final heading directly. Stored turns track angular velocity
 * and acceleration sampled from their precomputed profile by measured heading progress.
 *
 * @author DrPixelCat - 7842 alum
 */
public class TurnController {
    private static final double EPSILON = 1e-6;
    // Preserve breakaway authority through the low-speed end of profile deceleration. Waiting
    // until nearly zero lets static friction stop the robot for a loop before recovery restarts it.
    private static final double LOW_SPEED_ANGULAR_VELOCITY = 0.25;
    private static final double BREAKAWAY_RESERVE = 0.02;
    private final FeedbackController headingController;
    private final FeedbackController motionController;
    private double angularFeedforwardKS;

    private boolean quickEndpointCapture;

    public TurnController(FeedbackController.Coefficients headingCoefficients,
                          double angularKV, double angularKA, double angularFeedforwardKS,
                          double ignoredAngularVelocityFeedbackGain) {
        headingController = new FeedbackController(headingCoefficients);
        headingController.setAngularController();
        motionController = new FeedbackController(new FeedbackController.Coefficients()
                .vel(angularKV).accel(angularKA));

        this.angularFeedforwardKS = angularFeedforwardKS;
    }

    public void setCoefficients(FeedbackController.Coefficients coefficients) {
        headingController.setCoefficients(coefficients);
        reset();
    }

    public void setMotionGains(double angularKV, double angularKA,
                               double angularVelocityFeedbackGain) {
        setMotionGains(angularKV, angularKA, angularFeedforwardKS,
                angularVelocityFeedbackGain);
    }

    /** Updates dynamic and moving-friction feedforward gains. */
    public void setMotionGains(double angularKV, double angularKA,
                               double angularFeedforwardKS,
                               double ignoredAngularVelocityFeedbackGain) {
        this.angularFeedforwardKS = angularFeedforwardKS;
        motionController.getCoefficients().vel(angularKV).accel(angularKA);
        reset();
    }

    /** Uses complete heading feedback for an unprofiled turn. */
    public double calculateQuick(double headingError) {
        return headingController.p().d().s().calculate(headingError).get();
    }

    /** Latches out static compensation after a quick turn reaches its endpoint band. */
    public double calculateQuick(double headingError, double measuredAngularVelocity,
                                 double endpointCaptureHeading) {
        if (Math.abs(headingError) <= endpointCaptureHeading) {
            quickEndpointCapture = true;
        }
        return headingController.p().d(-measuredAngularVelocity)
                .s(quickEndpointCapture ? 0.0 : Math.signum(headingError))
                .calculate(headingError).get();
    }

    /**
     * Calculates a profiled turn command from heading-progress targets and measured angular velocity.
     */
    public double calculateProfiled(double headingError, double intendedDirection,
                                    MotionParameters targets, double measuredAngularVelocity) {
        return calculateProfiled(headingError, intendedDirection, targets, measuredAngularVelocity, null);
    }

    public double calculateProfiled(double headingError, double intendedDirection,
                                    MotionParameters targets, double measuredAngularVelocity,
                                    core.FollowerConstants constants) {
        double targetVelocity = targets.getAngularVel();
        double targetAcceleration = targets.getAngularAccel();

        double motionSign = 0.0;
        if (Math.abs(targetVelocity) > EPSILON) {
            motionSign = Math.signum(targetVelocity);
        } else if (Math.abs(targetAcceleration) > EPSILON) {
            motionSign = Math.signum(targetAcceleration);
        }

        double feedforward = motionController.vel(targetVelocity).accel(targetAcceleration)
                .f(angularFeedforwardKS * motionSign).calculate(headingError).get();
        if (constants != null) {
            feedforward = constants.characterizedPower(core.MotionCharacterization.Axis.ANGULAR,
                    targetVelocity, targetAcceleration, feedforward);
        }
        double errorRate = targetVelocity - measuredAngularVelocity;
        boolean stoppedReference = Math.abs(targetVelocity) <= EPSILON &&
                Math.abs(measuredAngularVelocity) < LOW_SPEED_ANGULAR_VELOCITY;
        double positionPower = headingController.p().d(errorRate)
                .s(stoppedReference ? Math.signum(headingError) : 0.0)
                .calculate(headingError).get();
        double requestedPower = feedforward + positionPower;
        return clip(ensureProfiledMotionBreakaway(
                requestedPower,
                targetVelocity,
                measuredAngularVelocity,
                headingController.getCoefficients().kS
        ));
    }

    /**
     * Restores enough authority to restart a profiled turn if deceleration feedforward cancels
     * the velocity/static terms below breakaway while the profile still requests motion.
     */
    static double ensureProfiledMotionBreakaway(double requestedPower, double targetVelocity,
                                                 double measuredVelocity, double staticGain) {
        if (Math.abs(targetVelocity) <= EPSILON ||
                Math.abs(measuredVelocity) >= LOW_SPEED_ANGULAR_VELOCITY) {
            return requestedPower;
        }

        double minimumPower = Math.min(1.0, Math.abs(staticGain) + BREAKAWAY_RESERVE);
        if (Math.abs(requestedPower) >= minimumPower &&
                Math.signum(requestedPower) == Math.signum(targetVelocity)) {
            return requestedPower;
        }
        return Math.copySign(minimumPower, targetVelocity);
    }

    public void reset() {
        quickEndpointCapture = false;
        headingController.reset();
        motionController.reset();
    }

    private static double clip(double power) { return Math.max(-1.0, Math.min(1.0, power)); }
}
