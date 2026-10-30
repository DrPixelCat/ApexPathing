package org.firstinspires.ftc.teamcode.sim;

import drivetrains.BaseDrivetrain;
import org.firstinspires.ftc.teamcode.apexpathing.Constants;
import org.junit.Test;
import org.codeblooded.ftcodesim.hardware.devices.SimMotor;
import org.codeblooded.ftcodesim.hardware.drivetrain.SimMecanumConfig;
import static org.junit.Assert.assertTrue;

/** Exercise the complete release mechanism, including chassis friction. */
public class ReleaseDrivetrainPhysicsTest {
    @Test public void correctionStaticPowerOvercomesChassisBreakaway() {
        for (int axis = 0; axis < 3; axis++) {
            for (double sign : new double[]{-1, 1}) for (boolean moving : new boolean[]{false, true}) {
                ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
                BaseDrivetrain<?> drive = new Constants().drivetrainConstants().build(hardware.hardwareMap);
                SimMecanumConfig config = (SimMecanumConfig) hardware.drivetrain.config;
                SimMotor motor = StockSimTestSupport.motors(hardware)[0];
                double gain = motor.config.modelCoefficients[0] * motor.config.voltageSensor.getVoltage() * config.wheelRadius;
                double radius = (config.wheelbase + config.trackWidth) / 2;
                // At zero translation the stationary gate uses atan2(0,0), hence forward
                // drag. Sustained strafe motion must instead overcome strafe drag.
                double initialThreshold = axis == 2 ? config.turnNaturalDeceleration * radius / gain
                        : config.forwardNaturalDeceleration / gain;
                double sustainedThreshold = axis == 0 ? config.forwardNaturalDeceleration / gain
                        : axis == 1 ? config.strafeNaturalDeceleration / gain
                        : config.turnNaturalDeceleration * radius / gain;
                double power = sign * (moving ? 1.1 * sustainedThreshold : .9 * initialThreshold);
                double startX = hardware.drivetrain.position.x;
                double startY = hardware.drivetrain.position.y;
                drive.moveWithVectors(axis == 0 ? power : 0, axis == 1 ? power : 0, axis == 2 ? power : 0);
                try {
                    for (int i = 0; i < 100; i++) ApexSimulation.stepPhysics(hardware, .005);
                    double displacement = axis == 0 ? hardware.drivetrain.position.x - startX
                            : axis == 1 ? hardware.drivetrain.position.y - startY
                            : hardware.drivetrain.position.theta;
                    assertTrue("Breakaway axis=" + axis + " power=" + power,
                            moving ? sign * displacement > .001 : Math.abs(displacement) < 1e-8);
                } finally { drive.moveWithVectors(0, 0, 0); }
            }
        }
    }

    @Test public void fullPhysicsRemainsFiniteAndUsesApexAxes() {
        for (int axis = 0; axis < 3; axis++) {
            ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
            BaseDrivetrain<?> drive = new Constants().drivetrainConstants().build(hardware.hardwareMap);
            double startX = hardware.drivetrain.position.x;
            double startY = hardware.drivetrain.position.y;
            drive.moveWithVectors(axis == 0 ? .3 : 0, axis == 1 ? .3 : 0, axis == 2 ? .3 : 0);
            try {
                for (int i = 0; i < 100; i++) ApexSimulation.stepPhysics(hardware, .005);
                double displacement = axis == 0 ? hardware.drivetrain.position.x - startX
                        : axis == 1 ? hardware.drivetrain.position.y - startY
                        : hardware.drivetrain.position.theta;
                assertTrue("Positive Apex axis " + axis + " must produce positive motion", displacement > 0);
            } finally { drive.moveWithVectors(0, 0, 0); }
        }
    }
}
