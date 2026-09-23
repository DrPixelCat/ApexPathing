package org.firstinspires.ftc.teamcode.sim;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.codeblooded.ftcodesim.hardware.devices.SimMotor;
import org.junit.Test;
import static org.junit.Assert.*;

/** Check the alpha release's actual motor response, without restoring snapshot physics. */
public class MecanumFrictionModelTest {
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
