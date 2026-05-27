package followers.constants;

import controllers.PDSController;
import drivetrains.Drivetrain;
import followers.BSplineFollower;
import localizers.Localizer;

/**
 * B-Spline path follower constants class.
 * Configured via the BSplineTuner OpMode for live dashboard tuning.
 * * @author Sohum Arora - 22985 Paraducks
 */
public class BSplineFollowerConstants extends FollowerConstants {

    // PDS Coefficients instead of flat primitives!
    public PDSController.PDSCoefficients translationCoeffs = new PDSController.PDSCoefficients(0.1, 0.0, 0.0, 0.0);
    public PDSController.PDSCoefficients headingCoeffs = new PDSController.PDSCoefficients(0.4, 0.0, 0.0, 0.0);
    public double velocityFF = 0.01;

    // Tolerances
    public double headingTolerance = Math.toRadians(1.0);
    public double distanceTolerance = 0.5;
    public double tTolerance = 0.95;

    public BSplineFollowerConstants() {}

    @Override
    public BSplineFollower build(Drivetrain drivetrain, Localizer localizer) {
        return new BSplineFollower(this, drivetrain, localizer);
    }

    // --- Builder Setters for Constants.java ---

    public BSplineFollowerConstants setTranslationCoeffs(PDSController.PDSCoefficients translationCoeffs) {
        this.translationCoeffs = translationCoeffs;
        return this;
    }

    public BSplineFollowerConstants setHeadingCoeffs(PDSController.PDSCoefficients headingCoeffs) {
        this.headingCoeffs = headingCoeffs;
        return this;
    }

    public BSplineFollowerConstants setVelocityFF(double velocityFF) {
        this.velocityFF = velocityFF;
        return this;
    }

    public BSplineFollowerConstants setHeadingTolerance(double headingTolerance) {
        this.headingTolerance = headingTolerance;
        return this;
    }

    public BSplineFollowerConstants setDistanceTolerance(double distanceTolerance) {
        this.distanceTolerance = distanceTolerance;
        return this;
    }

    public BSplineFollowerConstants setTTolerance(double tTolerance) {
        this.tTolerance = tTolerance;
        return this;
    }
}