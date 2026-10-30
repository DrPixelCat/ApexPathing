package follower;

import paths.geometry.Vector;

/**
 * Read-only live tracking telemetry owned by one follower. Read after update() on the OpMode thread.
 * The same view is reused across movements; retain individual values when a historical sample is needed.
 * This is separate from FollowerDiagnostics, the optional visualization callback observer.
 */
public interface FollowerTelemetry {
    double getTrackingVelocityTarget();
    /** Future pursuit goal; NaN when pursuit is inactive or recovering overspeed. */
    double getVelocityGoalDistanceIn();
    double getVelocityGoal();
    double getCurrentSpeedCeiling();
    double getTrackingMeasuredVelocity();
    double getTrackingProgressVelocity();
    double getTrackingFeedforward();
    double getTrackingHeadingFeedforward();
    boolean isWheelFeedforwardActive();
    double getTrackingVelocityFeedback();
    double getTrackingAngularVelocityTarget();
    double getTrackingEndpointBlend();
    double getBestT();
    double getCrossTrackErrorIn();
    double getCentripetalErrorIn();
    Follower.CommandDemand getLastCommandDemand();
    Vector getClosestPathPoint();
    Vector getCrossTrackNormal();
    Vector getPathNormal();
    Vector getCrossTrackCorrection();
    Vector getCentripetalCorrection();
    boolean isCenteredBrakingActive();
    double getCenteredBrakingMeanPower();
    boolean isVelocitySampleAvailable();
}
