package followers.constants;

import com.bylazar.configurables.annotations.Configurable;
import drivetrains.Drivetrain;
import followers.BSplineFollower;
import localizers.Localizer;

/**
 * B-Spline path follower constants class.
 */
@Configurable
public class BSplineFollowerConstants extends FollowerConstants {
    // Keep them as flat, static primitives so Dashboard/Panels can read them live!
    public static double translationP = 0.1;
    public static double headingP = 0.4;
    public static double velocityFF = 0.01;

    // Tolerances
    public static double headingTolerance = Math.toRadians(1.0);
    public static double distanceTolerance = 0.5;
    public static double tTolerance = 0.95;

    public BSplineFollowerConstants() {}

    @Override
    public BSplineFollower build(Drivetrain drivetrain, Localizer localizer) {
        return new BSplineFollower(this, drivetrain, localizer);
    }

    // Keep instance methods normal so they don't accidentally stomp on static values globally
    public BSplineFollowerConstants setTranslationP(double translationP) {
        BSplineFollowerConstants.translationP = translationP;
        return this;
    }

    public BSplineFollowerConstants setHeadingP(double headingP) {
        BSplineFollowerConstants.headingP = headingP;
        return this;
    }

    public BSplineFollowerConstants setVelocityFF(double velocityFF) {
        BSplineFollowerConstants.velocityFF = velocityFF;
        return this;
    }
}