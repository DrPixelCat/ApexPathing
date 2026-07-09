package drivetrains;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Objects;

import geometry.Angle;
import geometry.Vector;
import util.MotorFactory;

/**
 * Mecanum drivetrain controller
 *
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public class Mecanum extends BaseDrivetrain<Mecanum.Config> {
    public Mecanum(Config config, HardwareMap hardwareMap) {
        super(config, hardwareMap);

        if (Objects.equals(config.blMotorConfig, null) || Objects.equals(config.brMotorConfig,
                null)) {
            throw new IllegalArgumentException(
                    "Back left and right motor configurations must be provided for a mecanum " +
                            "drivetrain"
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

    @Override
    public boolean isHolonomic() {
        return true;
    }

    /**
     * Configuration class for Mecanum drivetrain.
     */
    public static class Config extends BaseDrivetrainConfig<Config> {
        @Override
        public Mecanum build(HardwareMap hardwareMap) {
            return new Mecanum(this, hardwareMap);
        }

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
         * Sets the back left motor configuration.
         */
        public Config setBackLeftMotor(MotorFactory motorFactory) {
            this.blMotorConfig = motorFactory;
            return this;
        }

        /**
         * Sets the back right motor configuration.
         */
        public Config setBackRightMotor(MotorFactory motorFactory) {
            this.brMotorConfig = motorFactory;
            return this;
        }
    }

    public static class MecanumDirectionalLut {

        public static class DirectionalKinematics {
            public final double maxVel;
            public final double maxAccel;
            public final double velMultiplier;
            public final double accelMultiplier;

            public DirectionalKinematics(double maxVel, double maxAccel, double velMultiplier,
                                         double accelMultiplier) {
                this.maxVel = maxVel;
                this.maxAccel = maxAccel;
                this.velMultiplier = velMultiplier;
                this.accelMultiplier = accelMultiplier;
            }
        }

        private final DirectionalKinematics[] lut = new DirectionalKinematics[360];

        /**
         * Precomputes the entire 360-degree kinematic capability of the mecanum drive.
         *
         * @param maxFwdVel   Absolute maximum forward velocity (in/s)
         * @param maxFwdAccel Absolute maximum forward acceleration (in/s^2)
         * @param maxSfeVel   Absolute maximum strafing velocity (in/s)
         * @param maxSfeAccel Absolute maximum strafing acceleration (in/s^2)
         */
        public MecanumDirectionalLut(double maxFwdVel, double maxFwdAccel, double maxSfeVel,
                                     double maxSfeAccel) {

            for (int i = 0; i < 360; i++) {
                double theta = Math.toRadians(i);

                // Local X matches forward drive power in moveWithVectors; local Y is strafe.
                double absForward = Math.abs(Math.cos(theta));
                double absStrafe = Math.abs(Math.sin(theta));

                // Absolute physical velocity caps
                double maxVel = 1.0 / ((absForward / maxFwdVel) + (absStrafe / maxSfeVel));
                double maxAccel = 1.0 / ((absForward / maxFwdAccel) + (absStrafe / maxSfeAccel));

                // Calculate the boost factors relative to pure forward motion
                double velMultiplier = maxFwdVel / maxVel;
                double accelMultiplier = maxFwdAccel / maxAccel;

                lut[i] = new DirectionalKinematics(maxVel, maxAccel, velMultiplier,
                        accelMultiplier);
            }
        }

        public DirectionalKinematics getKinematics(Vector globalDriveVector, Angle currentHeading) {
            if (globalDriveVector.getMagSq().getIn() < 1e-9) {
                return lut[0]; // Default to forward limits
            }

            Vector localVector = globalDriveVector.rotate(currentHeading.times(-1.0));
            double degrees = Math.toDegrees(localVector.getTheta().getRad());
            degrees = ((degrees % 360.0) + 360.0) % 360.0;

            int lowIndex = (int) Math.floor(degrees);
            int highIndex = (lowIndex + 1) % 360;
            double t = degrees - lowIndex;

            if (t < 1e-9) {
                return lut[lowIndex];
            }

            DirectionalKinematics low = lut[lowIndex];
            DirectionalKinematics high = lut[highIndex];

            return new DirectionalKinematics(
                    interpolate(low.maxVel, high.maxVel, t),
                    interpolate(low.maxAccel, high.maxAccel, t),
                    interpolate(low.velMultiplier, high.velMultiplier, t),
                    interpolate(low.accelMultiplier, high.accelMultiplier, t)
            );
        }

        private double interpolate(double low, double high, double t) {
            return low + ((high - low) * t);
        }
    }
}
