package drivetrains;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Base class for all drivetrain controllers.
 *
 * <p>This class handles motor initialization and provides common methods for driving and setting
 * motor powers. Specific drivetrain types (like Tank, Mecanum, etc.) should extend this class and
 * implement the moveWithVectors method to define how the drive, strafe, and turn vectors are
 * translated into motor powers.
 *
 * @param <T> the type of drivetrain configuration this drivetrain uses, which must extend
 *        {@link BaseDrivetrainConstants}
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public abstract class BaseDrivetrain<T extends BaseDrivetrainConstants<T>> {
    protected T constants;

    public enum DrivetrainType {
        COAXIAL_SWERVE,
        DUAL_ACTUATED,
        OMNI,
        MECANUM,
        TANK
    }

    private final DrivetrainType drivetrainType;
    private final boolean isHolonomic;

    /**
     * Legacy aliases for the first four slots. Fixed layouts may omit their rear motors.
     */
    protected DcMotorEx flMotor, frMotor, blMotor, brMotor;

    private static final double POWER_TOLERANCE = 0.001;
    private final DcMotorEx[] motors;
    private final double[] lastMotorPowers;
    private final double[] namedMotorPowers = new double[4];

    /**
     * Your drivetrain class constructor should call this super constructor to initialize motors and
     * store the configuration.
     *
     * @param constants your drivetrain configuration object that is a child of
     *                  {@link BaseDrivetrainConstants}
     * @param hardwareMap the hardware map to use for initializing motors
     */
    public BaseDrivetrain(T constants, HardwareMap hardwareMap, DrivetrainType drivetrainType) {
        this(constants, hardwareMap, drivetrainType, new Motor[]{constants.flMotorConfig,
                constants.frMotorConfig, constants.blMotorConfig, constants.brMotorConfig});
    }

    /** Initializes an indexed motor layout; fixed-drive subclasses retain their named slots. */
    protected BaseDrivetrain(T constants, HardwareMap hardwareMap, DrivetrainType drivetrainType,
                             Motor[] motorConfigs) {
        constants.validate();
        motors = new DcMotorEx[motorConfigs.length];
        lastMotorPowers = new double[motorConfigs.length];
        for (int i = 0; i < motorConfigs.length; i++) {
            if (motorConfigs[i] != null) { motors[i] = motorConfigs[i].build(hardwareMap); }
        }
        flMotor = motorAt(0); frMotor = motorAt(1);
        blMotor = motorAt(2); brMotor = motorAt(3);

        this.constants = constants;
        this.drivetrainType = drivetrainType;
        this.isHolonomic = drivetrainType != DrivetrainType.TANK;
    }

    /** Hardware-independent model factory. Overrides must support settings snapshots without actuating hardware. */
    public drivetrains.models.DriveModel createDriveModel(follower.FollowerConstants settings,
                                                          paths.movements.Path.PathType mode) {
        drivetrains.models.DriveModel model = drivetrains.models.DriveModels.defaultForDrivetrain(this,settings,mode);
        return model instanceof drivetrains.models.HolonomicDriveModel
                ? ((drivetrains.models.HolonomicDriveModel) model).withDirectionalKinematics(this::createDirectionalKinematics)
                : model;
    }

    /** Override for measured directional limits. Bind the returned provider to the supplied settings only. */
    protected drivetrains.models.DirectionalKinematics createDirectionalKinematics(follower.FollowerConstants settings) {
        return drivetrainType == DrivetrainType.MECANUM || drivetrainType == DrivetrainType.DUAL_ACTUATED
                ? drivetrains.models.DirectionalKinematics.mecanum(settings)
                : drivetrains.models.DirectionalKinematics.isotropic(settings);
    }

    /**
     * Moves the robot using the provided drive, strafe, and turn vectors, mixed by the subclass.
     *
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
     *
     * @param x forward/backward joystick input (positive for forward, negative for backward)
     * @param y left/right joystick input (positive for left, negative for right)
     * @param turn rotation joystick input (positive for counterclockwise, negative for clockwise)
     * @param robotHeadingRad current heading of the robot in radians, only used for field centric
     */
    public void drive(double x, double y, double turn, double robotHeadingRad) {
        double adjX, adjY;
        if (constants.robotCentric) {
            adjX = x;
            adjY = y;
        } else { // Field centric
            double cos = Math.cos(-robotHeadingRad);
            double sin = Math.sin(-robotHeadingRad);
            adjX = x * cos - y * sin;
            adjY = x * sin + y * cos;
        }
        moveWithVectors(adjX, adjY, turn);
    }

    /**
     * Drives the robot with provided joystick inputs. This method is meant for robotic-centric
     * control. If you are using field-centric control, you have to use the other drive method that
     * requires the robot's current heading to be passed in as a parameter.
     *
     * @param x forward/backward joystick input (positive for forward, negative for backward)
     * @param y left/right joystick input (positive for left, negative for right)
     * @param turn rotation joystick input (positive for counterclockwise, negative for clockwise)
     */
    public void drive(double x, double y, double turn) { drive(x, y, turn, 0); }

    /** @return the {@link DrivetrainType} of this drivetrain */
    public DrivetrainType getDrivetrainType() { return drivetrainType; }

    /** @return Whether the drivetrain is currently in a holonomic state or not */
    public boolean isHolonomic() { return isHolonomic; }

    /**
     * Sets the power for each drivetrain motor, applying limits from the configurations. If your
     * drivetrain class does not use all 4 motors, just pass 0 for the motors you don't use.
     */
    public void setPowers(double flPower, double frPower, double blPower, double brPower) {
        namedMotorPowers[0] = flPower; namedMotorPowers[1] = frPower;
        namedMotorPowers[2] = blPower; namedMotorPowers[3] = brPower;
        setMotorPowers(namedMotorPowers);
    }

    /** Applies one power per configured slot, scaling all motors together to the power limit. */
    protected final void setMotorPowers(double[] powers) {
        if (powers == null || powers.length != motors.length) {
            stop();
            throw new IllegalArgumentException("Provide one power per motor slot");
        }
        double peak = 0;
        for (int i = 0; i < powers.length; i++) {
            if (!Double.isFinite(powers[i])) {
                stop();
                throw new IllegalArgumentException("Motor powers must be finite");
            }
            if (motors[i] != null) { peak = Math.max(peak, Math.abs(powers[i])); }
        }
        if (!Double.isFinite(constants.maxPower) || constants.maxPower < 0 || constants.maxPower > 1) {
            stop();
            throw new IllegalStateException("Maximum power must be in [0, 1]");
        }
        for (int i = 0; i < powers.length; i++) {
            double power = peak > constants.maxPower ? powers[i] / peak * constants.maxPower : powers[i];
            if (motors[i] != null && (Math.abs(power - lastMotorPowers[i]) > POWER_TOLERANCE
                    || power == 0 && lastMotorPowers[i] != 0
                    || Math.abs(lastMotorPowers[i]) > constants.maxPower)) {
                motors[i].setPower(power);
                lastMotorPowers[i] = power;
            }
        }
    }

    /** Stop all drivetrain actuators */
    public void stop() {
        for (int i = 0; i < motors.length; i++) {
            if (motors[i] != null) { motors[i].setPower(0); }
            lastMotorPowers[i] = 0;
        }
    }

    private DcMotorEx motorAt(int index) { return index < motors.length ? motors[index] : null; }
    private double powerAt(int index) { return index < motors.length ? lastMotorPowers[index] : 0; }

    public double getLastFlPower() { return powerAt(0); }
    public double getLastFrPower() { return powerAt(1); }
    public double getLastBlPower() { return powerAt(2); }
    public double getLastBrPower() { return powerAt(3); }

    /** Number of indexed slots; fixed four-slot drives may have unused rear slots. */
    public int getMotorCount() { return motors.length; }
    public double getLastMotorPower(int index) { return lastMotorPowers[index]; }
    public double[] getLastMotorPowers() { return lastMotorPowers.clone(); }

    public double getMaxMotorPower() {
        double peak = 0;
        for (double power : lastMotorPowers) { peak = Math.max(peak, Math.abs(power)); }
        return peak;
    }

    public T getConstants() { return constants; }
}
