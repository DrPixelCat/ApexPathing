package drivetrains;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Objects;

import util.MotorFactory;

/**
 * Mecanum drivetrain controller
 *
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public class Mecanum extends BaseDrivetrain<Mecanum.Constants> {
    public Mecanum(Constants constants, HardwareMap hardwareMap) {
        super(constants, hardwareMap, DrivetrainType.MECANUM);

        if (Objects.equals(constants.blMotorConfig, null) ||
                Objects.equals(constants.brMotorConfig, null)) {
            throw new IllegalArgumentException(
                    "Back left and right motor configurations must be provided for a mecanum drivetrain"
            );
        }
    }

    @Override
    public void moveWithVectors(double x, double y, double turn) {
        // Mecanum kinematics explanation: https://www.youtube.com/watch?v=gnSW2QpkGXQ
        setPowers(x - y - turn, x + y + turn,
                x + y - turn, x - y + turn
        );
    }

    /** Configuration class for Mecanum drivetrain. */
    public static class Constants extends BaseDrivetrainConstants<Constants> {
        @Override
        public Mecanum build(HardwareMap hardwareMap) { return new Mecanum(this, hardwareMap); }

        /** Sets the front left motor configuration. */
        public Constants setFrontLeftMotor(MotorFactory motorFactory) {
            this.flMotorConfig = motorFactory;
            return this;
        }

        /** Sets the front right motor configuration. */
        public Constants setFrontRightMotor(MotorFactory motorFactory) {
            this.frMotorConfig = motorFactory;
            return this;
        }

        /** Sets the back left motor configuration. */
        public Constants setBackLeftMotor(MotorFactory motorFactory) {
            this.blMotorConfig = motorFactory;
            return this;
        }

        /** Sets the back right motor configuration. */
        public Constants setBackRightMotor(MotorFactory motorFactory) {
            this.brMotorConfig = motorFactory;
            return this;
        }
    }
}