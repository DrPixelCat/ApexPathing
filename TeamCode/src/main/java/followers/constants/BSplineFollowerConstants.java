package followers.constants;

import controllers.PDSController;
import drivetrains.Drivetrain;
import followers.BSplineFollower;
import localizers.Localizer;

/**
 * B-Spline path follower constants class.
 * Run {@link org.firstinspires.ftc.teamcode.tuning.manual.BSplineTuner} to determine the values of these constants and refer
 * to the ApexPathing documentation for how to properly tune these constants
 * @author Sohum Arora - 22985 Paraducks
 */
public class BSplineFollowerConstants extends FollowerConstants {
    // Tunable constants
    public PDSController.PDSCoefficients translationCoeffs = new PDSController.PDSCoefficients(0.1, 0.0, 0.0, 0.0);
    public PDSController.PDSCoefficients headingCoeffs = new PDSController.PDSCoefficients(0.4, 0.0, 0.0, 0.0);
    public double velocityFF = 0.01;

    // Tolerances
    public double headingTolerance = Math.toRadians(1.0);
    public double distanceTolerance = 0.5;
    public double tTolerance = 0.95;

    /**
     * Constructor for the BSplineFollowerConstants class
     */
    public BSplineFollowerConstants() {
        // Initialization if needed
    }

    @Override
    public BSplineFollower build(Drivetrain drivetrain, Localizer localizer) {
        return new BSplineFollower(this, drivetrain, localizer);
    }

    // region Setters
    /**
     * Sets the PDS coefficients for translation error.
     * @param translationCoeffs the new translational PDS coefficients
     * @return this instance for chaining
     */
    public BSplineFollowerConstants setTranslationCoeffs(PDSController.PDSCoefficients translationCoeffs) {
        this.translationCoeffs = translationCoeffs;
        return this;
    }

    /**
     * Sets the PDS coefficients for heading error.
     * @param headingCoeffs the new heading PDS coefficients
     * @return this instance for chaining
     */
    public BSplineFollowerConstants setHeadingCoeffs(PDSController.PDSCoefficients headingCoeffs) {
        this.headingCoeffs = headingCoeffs;
        return this;
    }

    /**
     * Sets the velocity feedforward gain.
     * @param velocityFF the new velocity feedforward gain
     * @return this instance for chaining
     */
    public BSplineFollowerConstants setVelocityFF(double velocityFF) {
        this.velocityFF = velocityFF;
        return this;
    }

    /**
     * Sets the heading error tolerance for the robot to consider its heading on target.
     * @param headingTolerance the tolerance in radians
     * @return this instance for chaining
     */
    public BSplineFollowerConstants setHeadingTolerance(double headingTolerance) {
        this.headingTolerance = headingTolerance;
        return this;
    }

    /**
     * Sets the distance error tolerance for the robot to consider its position on target.
     * @param distanceTolerance the tolerance in inches
     * @return this instance for chaining
     */
    public BSplineFollowerConstants setDistanceTolerance(double distanceTolerance) {
        this.distanceTolerance = distanceTolerance;
        return this;
    }

    /**
     * Sets the t-parameter tolerance for ending the spline path.
     * @param tTolerance the t tolerance value (typically close to 1.0)
     * @return this instance for chaining
     */
    public BSplineFollowerConstants setTTolerance(double tTolerance) {
        this.tTolerance = tTolerance;
        return this;
    }
}