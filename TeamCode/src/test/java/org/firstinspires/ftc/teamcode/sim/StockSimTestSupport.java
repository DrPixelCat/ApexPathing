package org.firstinspires.ftc.teamcode.sim;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.codeblooded.ftcodesim.ascope.boundaries.MotionVector;
import org.codeblooded.ftcodesim.hardware.devices.SimMotor;

/** Test state setup only. Physics is always the upstream mechanism, with no model overrides. */
final class StockSimTestSupport {
    private StockSimTestSupport() { }
    static SimMotor[] motors(ApexSimulation.Hardware hardware) {
        SimMotor[] motors = new SimMotor[hardware.drivetrain.motorNames.length];
        for (int i = 0; i < motors.length; i++) motors[i] = (SimMotor) hardware.hardwareMap.get(
                DcMotorEx.class, hardware.drivetrain.motorNames[i]);
        return motors;
    }
    static void setSpeed(ApexSimulation.Hardware hardware, double speed) {
        hardware.drivetrain.velocity = new MotionVector(speed, 0, 0);
        hardware.drivetrain.updateWheelRollVelocities();
    }
    static void step(ApexSimulation.Hardware hardware, double seconds) {
        ApexSimulation.stepPhysics(hardware, seconds);
    }
}
