package followers.constants;

import controllers.PDSController;
import controllers.PDSController.PDSCoefficients;
import drivetrains.BaseDrivetrain;
import followers.MovementFollower;
import geometry.Dist;
import localizers.BaseLocalizer;


/**
 * B-Spline path follower constants class.
 * Configured via the BSplineTuner OpMode for live dashboard tuning.
 * @author Sohum Arora - 22985 Paraducks
 */
public class BSplineFollowerConstants extends FollowerConstants {

    // PDS Coefficients instead of flat primitives!
    public PDSCoefficients translationCoeffs = new PDSController.PDSCoefficients(0.1, 0.0, 0.0, 0.0);
    public PDSCoefficients headingCoeffs = new PDSController.PDSCoefficients(0.4, 0.0, 0.0, 0.0);
    public PDSCoefficients velocityCoeffs = new PDSCoefficients();
    public PDSCoefficients driveCoeffs = new PDSCoefficients();
    public double kV = 0.01;
    public double kA = 0.01;
    public Dist velocityLimit = null;

    // Tolerances
    public double headingTolerance = Math.toRadians(1.0);
    public double distanceTolerance = 0.5;
    public double tTolerance = 0.95;
    public double maxLateralAccel = 0;

    public BSplineFollowerConstants() {}

    @Override
    public MovementFollower build(BaseDrivetrain<?> drivetrain, BaseLocalizer<?> localizer) {
        return new MovementFollower(this, drivetrain, localizer);
    }

    // --- Builder Setters for Constants.java ---

    public BSplineFollowerConstants setTranslationCoeffs(PDSCoefficients translationCoeffs) {
        this.translationCoeffs = translationCoeffs;
        return this;
    }

    public BSplineFollowerConstants setHeadingCoeffs(PDSCoefficients headingCoeffs) {
        this.headingCoeffs = headingCoeffs;
        return this;
    }

    public BSplineFollowerConstants setkV(double kV) {
        this.kV = kV;
        return this;
    }

    public BSplineFollowerConstants setkA(double kA) {
        this.kA = kA;
        return this;
    }

    public BSplineFollowerConstants setVelocityLimit(Dist velocityLimit) {
        this.velocityLimit = velocityLimit;
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
    public double getMaxLateralAccel() {
        return maxLateralAccel;
    }
}