package drivetrains;

import com.qualcomm.robotcore.hardware.HardwareMap;

import util.MotorFactory;

/**
 * Tank drivetrain controller (supports 2 and 4 motor configurations)
 *
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public class Tank extends BaseDrivetrain<Tank.Config> {
    public Tank(Config config, HardwareMap hardwareMap) {super(config, hardwareMap);}

    @Override
    public void moveWithVectors(double x, double y, double turn) {
        // Tank kinematics explanation (FRC, more in depth): https://www.youtube.com/watch?v=Ym34WI2rSdc
        // For FTC, not in depth, but shows simple code: https://www.youtube.com/watch?v=pREkiGl9yi0
        // 2 motor tank uses the front motors only, 4 motor tank uses all motors with the same power
        setPowers(x - turn, x + turn, x - turn, x + turn);
    }

    @Override
    public boolean isHolonomic() {
        return false;
    }

    /**
     * Configuration class for Tank drivetrain.
     */
    public static class Config extends BaseDrivetrainConfig<Config> {
        @Override
        public Tank build(HardwareMap hardwareMap) {return new Tank(this, hardwareMap);}

        /**
         * Sets the front left motor configuration.
         */
        public Config setFrontLeftMotor(MotorFactory motorFactory) {
            this.flMotorConfig = motorFactory;
            return this;
        }

        /**
         * Sets the front right motor configuration.
         */
        public Config setFrontRightMotor(MotorFactory motorFactory) {
            this.frMotorConfig = motorFactory;
            return this;
        }

        /**
         * Sets the back left motor configuration. Do not use this for 2 wheel tank
         */
        public Config setBackLeftMotor(MotorFactory motorFactory) {
            this.blMotorConfig = motorFactory;
            return this;
        }

        /**
         * Sets the back right motor configuration. Do not use this for 2 wheel tank
         */
        public Config setBackRightMotor(MotorFactory motorFactory) {
            this.brMotorConfig = motorFactory;
            return this;
        }
    }
}