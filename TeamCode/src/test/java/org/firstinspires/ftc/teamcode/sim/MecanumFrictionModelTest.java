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
                assertTrue("Zero BRAKE jumps below same-direction duty", zero < same - 100);
                assertEquals("Tiny opposing duty stays near zero BRAKE", zero, opposite, 5);
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
