package drivetrains;

import com.qualcomm.robotcore.hardware.HardwareMap;

import paths.geometry.Angle;
import paths.geometry.Dist;

/**
 * Abstract class implemented by all drivetrain configuration classes
 *
 * <p>When creating a drivetrain configuration, you must extend this class and implement the build()
 * method to return an instance of the corresponding drivetrain class using your configuration
 * class. Your constants should have a public scope and be initialized with default values.
 *
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public abstract class BaseDrivetrainConstants<T extends BaseDrivetrainConstants<T>> {
    /** Child classes should handle setting these up as needed. */
    public Motor flMotorConfig, frMotorConfig, blMotorConfig, brMotorConfig;

    public double maxPower = 1.0;
    public Angle headingTolerance = Angle.fromDeg(1.0);
    public Dist distanceTolerance = Dist.fromIn(0.5);
    public boolean robotCentric = true; // Robot/field centric

    /** Builds and returns an instance of the corresponding drivetrain class using this config. */
    public abstract BaseDrivetrain<T> build(HardwareMap hardwareMap);

    /** Set the maximum motor output limit for the drivetrain. The default is 1.0. */
    @SuppressWarnings("unchecked")
    public T setMaxPower(double maxPower) {
        if (!Double.isFinite(maxPower)) {
            throw new IllegalArgumentException("Maximum power must be finite");
        }
        this.maxPower = Math.max(0.0, Math.min(1.0, maxPower));
        return (T) this;
    }

    /**
     * Set whether the drivetrain should use robot-centric controls or field-centric controls in
     * TeleOp. The default is true (robot-centric).
     */
    @SuppressWarnings("unchecked")
    public T setRobotCentric(boolean robotCentric) {
        this.robotCentric = robotCentric;
        return (T) this;
    }

    /**
     * Set how close the drivetrain must be to the target heading to be considered "on target". The
     * default is 1 degree.
     */
    @SuppressWarnings("unchecked")
    public T setHeadingTolerance(Angle headingTolerance) {
        if (headingTolerance == null || headingTolerance.isNotFinite() || headingTolerance.getRad() < 0) {
            throw new IllegalArgumentException("Heading tolerance must be finite and nonnegative");
        }
        this.headingTolerance = headingTolerance;
        return (T) this;
    }

    /**
     * Set how close the drivetrain must be to the target position to be considered "on target". The
     * default is 0.5 inches.
     */
    @SuppressWarnings("unchecked")
    public T setDistanceTolerance(Dist distanceTolerance) {
        if (distanceTolerance == null || !Double.isFinite(distanceTolerance.getIn()) || distanceTolerance.getIn() < 0) {
            throw new IllegalArgumentException("Distance tolerance must be finite and nonnegative");
        }
        this.distanceTolerance = distanceTolerance;
        return (T) this;
    }

    /** Checks public configuration fields before hardware is bound. */
    public void validate() {
        if (!Double.isFinite(maxPower) || maxPower < 0 || maxPower > 1) {
            throw new IllegalArgumentException("Maximum power must be in [0, 1]");
        }
        setHeadingTolerance(headingTolerance);
        setDistanceTolerance(distanceTolerance);
    }
}
