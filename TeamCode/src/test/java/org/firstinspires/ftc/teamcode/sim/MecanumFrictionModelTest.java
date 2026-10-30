package org.firstinspires.ftc.teamcode.sim;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.codeblooded.ftcodesim.hardware.devices.SimMotor;
import org.junit.Test;
import static org.junit.Assert.*;

/** Check the alpha release's actual motor response, without restoring snapshot physics. */
public class MecanumFrictionModelTest {
    @Test public void brakingAccelerationJumpsAtZeroCommand() {
        ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
        new org.firstinspires.ftc.teamcode.apexpathing.Constants()
                .drivetrainConstants().build(hardware.hardwareMap);
        SimMotor motor = (SimMotor) hardware.hardwareMap.get(DcMotorEx.class,
                ApexSimulation.FRONT_LEFT_MOTOR);
        double wheelRadius = 1.889765;
        for (double direction : new double[] {-1, 1}) {
            for (double speed : new double[] {20, 40, 60}) {
                double same = accelerationInTravelDirection(motor, speed, direction,
                        direction * .001, wheelRadius);
                double zero = accelerationInTravelDirection(motor, speed, direction,
                        0, wheelRadius);
                double opposite = accelerationInTravelDirection(motor, speed, direction,
                        -direction * .001, wheelRadius);
                System.out.printf("MOTOR TRANSITION: direction=%+.0f speed=%.0f "
                                + "same=%+.2f zero=%+.2f opposite=%+.2f in/s^2%n",
                        direction, speed, same, zero, opposite);
                // Neutral/opposing braking depends on speed; it is not a fixed 100 in/s^2 jump.
                double gain = motor.config.modelCoefficients[0]
                        * motor.config.voltageSensor.getVoltage() * wheelRadius;
                double speedLoss = motor.config.modelCoefficients[1];
                double neutralLoss = motor.config.zeroPowerBrakeCoefficients[3];
                double regenerativeLoss = motor.config.modelCoefficients[2];
                assertEquals("Powered response uses duty-dependent back EMF",
                        .001 * (gain - speedLoss * speed), same, 1e-8);
                assertEquals("Neutral BRAKE response scales with speed", -neutralLoss * speed, zero, 1e-8);
                assertEquals("Opposing duty adds regenerative braking",
                        -.001 * gain - regenerativeLoss * speed, opposite, 1e-8);
                assertTrue("Neutral BRAKE is stronger than tiny aligned duty", zero < same);
                assertTrue("Opposing duty is stronger than neutral BRAKE", opposite < zero);
            }
        }
    }

    private static double accelerationInTravelDirection(SimMotor motor, double speed,
                                                         double direction, double power,
                                                         double wheelRadius) {
        motor.setRollVelocity(direction * speed / wheelRadius);
        motor.setPower(power);
        motor.update(.00001);
        return direction * motor.getAcceleration() * wheelRadius;
    }

    @Test public void releaseMotorMovesAtLowDutyAndBrakesAtZeroDuty() {
        ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
        new org.firstinspires.ftc.teamcode.apexpathing.Constants()
                .drivetrainConstants().build(hardware.hardwareMap);
        SimMotor motor = (SimMotor) hardware.hardwareMap.get(DcMotorEx.class,
                ApexSimulation.FRONT_LEFT_MOTOR);
        for (double sign : new double[] {-1, 1}) {
            motor.setRollVelocity(0);
            motor.setPower(sign*.001);
            motor.update(.001);
            assertTrue(sign*motor.getVelocity() > 0);
            motor.setRollVelocity(sign*10);
            motor.setPower(0);
            motor.update(.001);
            assertTrue("Zero-duty braking opposes motion", sign*motor.getAcceleration() < 0);
            motor.setRollVelocity(sign*10);
            motor.setPower(sign*.001);
            motor.update(.001);
            assertTrue("Small same-direction duty differs from zero-duty braking",
                    sign*motor.getAcceleration() > 0);
        }
    }

}
