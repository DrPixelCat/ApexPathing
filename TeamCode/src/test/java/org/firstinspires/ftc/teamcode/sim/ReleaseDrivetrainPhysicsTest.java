package org.firstinspires.ftc.teamcode.sim;

import drivetrains.BaseDrivetrain;
import org.firstinspires.ftc.teamcode.apexpathing.Constants;
import org.junit.Test;
import static org.junit.Assert.assertTrue;

/** Exercise the complete release mechanism, including chassis friction. */
public class ReleaseDrivetrainPhysicsTest {
    @Test public void correctionStaticPowerOvercomesChassisBreakaway() {
        for (int axis = 0; axis < 3; axis++) {
            for (double power : new double[] {.05, .08}) {
                ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
                BaseDrivetrain<?> drive = new Constants().drivetrainConstants().build(hardware.hardwareMap);
                double startX = hardware.drivetrain.position.x;
                double startY = hardware.drivetrain.position.y;
                drive.moveWithVectors(axis == 0 ? power : 0, axis == 1 ? power : 0, axis == 2 ? power : 0);
                try {
                    for (int i = 0; i < 100; i++) ApexSimulation.stepPhysics(hardware, .005);
                    double displacement = axis == 0 ? hardware.drivetrain.position.x - startX
                            : axis == 1 ? hardware.drivetrain.position.y - startY
                            : hardware.drivetrain.position.theta;
                    assertTrue("Breakaway axis=" + axis + " power=" + power,
                            power == .05 ? Math.abs(displacement) < 1e-8 : displacement > .001);
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
