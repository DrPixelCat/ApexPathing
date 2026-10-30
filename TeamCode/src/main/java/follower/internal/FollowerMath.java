package follower.internal;

import com.qualcomm.robotcore.util.Range;
import controllers.FeedbackController;
import feedforward.MotionPowerCalculator;
import paths.geometry.Vector;
import paths.movements.FollowerMovement;
import paths.movements.Path;

/** Shared calculations for the follower engines; not part of the robot-facing API. */
public final class FollowerMath {
    private FollowerMath() { }

    static final double PROFILED_ENDPOINT_CAPTURE_DISTANCE_INCHES = 8.0;
    static final double ENDPOINT_STALLED_VELOCITY_IN_PER_SECOND = 0.25;
    private static final double PROFILED_HEADING_CAPTURE_RADIANS = Math.toRadians(10.0);
    static final double MAX_CENTRIPETAL_POWER = MotionPowerCalculator.MAX_CENTRIPETAL_POWER;
    private static final double SLOWED_VELOCITY_DELTA_IN_PER_SECOND = 6.0;
    private static final double STUCK_LINEAR_VELOCITY_IN_PER_SECOND = 0.5;

    public static boolean pathStatusActive(FollowerMovement movement, boolean paused) {
        return movement instanceof Path && !paused;
    }

    public static boolean slowedForVelocities(double targetVelocity, double measuredVelocity) {
        return Double.isFinite(targetVelocity) && Double.isFinite(measuredVelocity)
                && targetVelocity - measuredVelocity >= SLOWED_VELOCITY_DELTA_IN_PER_SECOND;
    }

    public static boolean stuckForSpeed(double speed) {
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
