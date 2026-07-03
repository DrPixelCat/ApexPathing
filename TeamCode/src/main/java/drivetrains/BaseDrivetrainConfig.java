package drivetrains;

import com.qualcomm.robotcore.hardware.HardwareMap;

import geometry.Angle;
import geometry.Dist;
import util.MotorFactory;

/**
 * Abstract class implemented by all drivetrain configuration classes
 *
 * <p>
 * When creating a drivetrain configuration, you must extend this class and implement the build()
 * method to return an instance of the corresponding drivetrain class using your configuration
 * class.
 * Your constants should have a public scope and be initialized with default values.
 * </p>
 *
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public abstract class BaseDrivetrainConfig<T extends BaseDrivetrainConfig<T>> {
    // Motors (only 2 motors are guaranteed, others are optional and may be null)
    // Child classes should handle setting these up as needed
    public MotorFactory flMotorConfig = new MotorFactory();
    public MotorFactory frMotorConfig = new MotorFactory();
    public MotorFactory blMotorConfig = null;
    public MotorFactory brMotorConfig = null;
    // Servos (for locking mecanum and butterfly)
    public Runnable toggleDriveServos = null;

    // Movement limits (zero = not enabled)
    public Dist linearVelocityLimit = Dist.zero();
    public Angle angularVelocityLimit = Angle.zero();
    public Dist linearAccelerationLimit = Dist.zero();
    public Angle angularAccelerationLimit = Angle.zero();
    public double maxPower = 1.0;

    // Other constants
    public boolean robotCentric = true; // Robot or field centric control

    /**
     * Builds and returns an instance of the corresponding drivetrain class using this
     * configuration.
     */
    public abstract BaseDrivetrain<?> build(HardwareMap hardwareMap);

    /**
     * Set the linear (X and Y) velocity limit for the drivetrain. The default is zero (disabled).
     */
    @SuppressWarnings("unchecked")
    public T setLinearVelocityLimit(Dist linearVelocityLimit) {
        this.linearVelocityLimit = linearVelocityLimit;
        return (T) this;
    }

    /**
     * Set the angular velocity limit for the drivetrain. The default is zero (disabled).
     */
    @SuppressWarnings("unchecked")
    public T setAngularVelocityLimit(Angle angularVelocityLimit) {
        this.angularVelocityLimit = angularVelocityLimit;
        return (T) this;
    }

    /**
     * Set the linear (X and Y) acceleration limit for the drivetrain. The default is zero
     * (disabled).
     */
    @SuppressWarnings("unchecked")
    public T setLinearAccelerationLimit(Dist linearAccelerationLimit) {
        this.linearAccelerationLimit = linearAccelerationLimit;
        return (T) this;
    }

    /**
     * Set the angular acceleration limit for the drivetrain. The default is zero (disabled).
     */
    @SuppressWarnings("unchecked")
    public T setAngularAccelerationLimit(Angle angularAccelerationLimit) {
        this.angularAccelerationLimit = angularAccelerationLimit;
        return (T) this;
    }

    /**
     * Set the maximum motor output limit for the drivetrain. The default is 1.0.
     */
    @SuppressWarnings("unchecked")
    public T setMaxPower(double maxPower) {
        this.maxPower = Math.max(Math.min(0.0, maxPower), 1.0); return (T) this;
    }

    /**
     * Set whether the drivetrain should use robot-centric controls or field-centric controls in
     * TeleOp.
     * The default is true (robot-centric).
     */
    @SuppressWarnings("unchecked")
    public T setRobotCentric(boolean robotCentric) {
        this.robotCentric = robotCentric;
        return (T) this;
    }

    /**
     * Get the motor configuration for the front left motor.
     */
    public MotorFactory getFlMotorConfig() {return flMotorConfig;}

    /**
     * Get the motor configuration for the front right motor.
     */
    public MotorFactory getFrMotorConfig() {return frMotorConfig;}

    /**
     * Get the motor configuration for the back left motor. May be null if not set.
     */
    public MotorFactory getBlMotorConfig() {return blMotorConfig;}

    /**
     * Get the motor configuration for the back right motor. May be null if not set.
     */
    public MotorFactory getBrMotorConfig() {return brMotorConfig;}
}
