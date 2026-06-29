package drivetrains;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Objects;

/**
 * Base class for all drivetrain controllers.
 *
 * <p>
 * This class handles motor initialization and provides common methods for driving and setting motor
 * powers. Specific drivetrain types (like Tank, Mecanum, etc.) should extend this class and
 * implement the moveWithVectors method to define how the drive, strafe, and turn vectors are
 * translated into motor powers.
 * </p>
 *
 * @param <T> the type of drivetrain configuration this drivetrain uses, which must extend {@link BaseDrivetrainConfig}
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public abstract class BaseDrivetrain<T extends BaseDrivetrainConfig<T>> {
    protected T config;

    // Note: front motors are guaranteed to be non-null, but rear motors may be null if not needed
    protected DcMotorEx flMotor, frMotor, blMotor, brMotor;

    // Power change deadzone to prevent unnecessary motor updates
    private static final double POWER_TOLERANCE = 0.005;
    private double lastFlPower = 0;
    private double lastFrPower = 0;
    private double lastBlPower = 0;
    private double lastBrPower = 0;

    /**
     * Your drivetrain class constructor should call this super constructor to initialize motors and
     * store the configuration.
     * @param config your drivetrain configuration object that is a child of {@link BaseDrivetrainConfig}
     * @param hardwareMap the hardware map to use for initializing motors
     */
    public BaseDrivetrain(T config, HardwareMap hardwareMap) {
        if (Objects.equals(config.flMotorConfig.getName(), "defaultMotorName")) {
            throw new IllegalArgumentException("Front left motor configuration is not set in the drivetrain config.");
        }
        if (Objects.equals(config.frMotorConfig.getName(), "defaultMotorName")) {
            throw new IllegalArgumentException("Front right motor configuration is not set in the drivetrain config.");
        }
        flMotor = config.flMotorConfig.build(hardwareMap);
        frMotor = config.frMotorConfig.build(hardwareMap);

        if (config.blMotorConfig != null) {
            if (Objects.equals(config.blMotorConfig.getName(), "defaultMotorName")) {
                throw new IllegalArgumentException("Back left motor configuration is not set in the drivetrain config.");
            }
            blMotor = config.blMotorConfig.build(hardwareMap);
        }
        if (config.brMotorConfig != null) {
            if (Objects.equals(config.brMotorConfig.getName(), "defaultMotorName")) {
                throw new IllegalArgumentException("Back right motor configuration is not set in the drivetrain config.");
            }
            brMotor = config.brMotorConfig.build(hardwareMap);
        }

        this.config = config;
    }

    /**
     * Moves the robot using the provided drive, strafe, and turn vectors.
     * The values are normalized and applied to the motors according to the mecanum drive formulas.
     * @param x the forward/backward movement vector (positive for forward, negative for backward)
     * @param y the left/right movement vector (positive for left, negative for right)
     * @param turn the rotation vector (positive for counterclockwise, negative for clockwise)
     */
    public abstract void moveWithVectors(double x, double y, double turn);

    /**
     * Drives the robot with provided joystick inputs and the robot's current heading. This method
     * is meant for field-centric control. If you are using robot-centric control, the robotHeading
     * parameter will be ignored, you can use the other drive method that doesn't require the
     * robot's heading.
     * @param x the forward/backward joystick input (positive for forward, negative for backward)
     * @param y the left/right joystick input (positive for left, negative for right)
     * @param turn the rotation joystick input (positive for counterclockwise, negative for clockwise)
     * @param robotHeading the current heading of the robot in radians, not used for robot centric control
     */
    public void drive(double x, double y, double turn, double robotHeading) {
        double adjX, adjY;
        if (!config.robotCentric) { // Field centric
            double cos = Math.cos(-robotHeading);
            double sin = Math.sin(-robotHeading);
            adjX = x * cos - y * sin;
            adjY = x * sin + y * cos;
        } else { adjX = x; adjY = y; }
        moveWithVectors(adjX, adjY, turn);
    }

    /**
     * Drives the robot with provided joystick inputs. This method is meant for robotic-centric
     * control. If you are using field-centric control, you have to use the other drive method that
     * requires the robot's current heading to be passed in as a parameter.
     * @param x the forward/backward joystick input (positive for forward, negative for backward)
     * @param y the left/right joystick input (positive for left, negative for right)
     * @param turn the rotation joystick input (positive for counterclockwise, negative for clockwise)
     */
    public void drive(double x, double y, double turn) { drive(x, y, turn, 0); }

    /**
     * Sets the power for each drivetrain motor, applying limits from the configurations. If your
     * drivetrain class does not use all 4 motors, just pass 0 for the motors you don't use.
     */
    public void setPowers(double flPower, double frPower, double blPower, double brPower) {
        // Motor power limiting
        double max = Math.max(0, Math.abs(flPower));
        max = Math.max(max, Math.abs(frPower));
        if (blMotor != null) max = Math.max(max, Math.abs(blPower));
        if (brMotor != null) max = Math.max(max, Math.abs(brPower));
        if (max > config.maxPower) {
            flPower = (flPower / max) * config.maxPower;
            frPower = (frPower / max) * config.maxPower;
            if (blMotor != null) blPower = (blPower / max) * config.maxPower;
            if (brMotor != null) brPower = (brPower / max) * config.maxPower;
        }

        // TODO: Add velocity and acceleration limiting

        // Write to motors only if the change exceeds the tolerance
        if (Math.abs(flPower - lastFlPower) > POWER_TOLERANCE) {
            flMotor.setPower(flPower); lastFlPower = flPower;
        }
        if (Math.abs(frPower - lastFrPower) > POWER_TOLERANCE) {
            frMotor.setPower(frPower); lastFrPower = frPower;
        }
        if (blMotor != null && Math.abs(blPower - lastBlPower) > POWER_TOLERANCE) {
            blMotor.setPower(blPower); lastBlPower = blPower;
        }
        if (brMotor != null && Math.abs(brPower - lastBrPower) > POWER_TOLERANCE) {
            brMotor.setPower(brPower); lastBrPower = brPower;
        }
    }

    /**
     * Stop all drivetrain actuators
     */
    public void stop() { setPowers(0, 0, 0, 0); };
}
