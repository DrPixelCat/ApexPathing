package org.firstinspires.ftc.teamcode.sim;

import java.lang.reflect.Method;
import org.codeblooded.ftcodesim.ascope.boundaries.MotionVector;
import org.junit.Test;
import setup.follower.sysid.FeedforwardTuner;
import static org.junit.Assert.*;

/** Checks ramp travel and braking on the unmodified field from one initial placement. */
public class QuasistaticFieldSequenceTest {
    @Test public void forwardAndReverseRampsFitTheDiagonalWithoutRepositioning() throws Exception {
        Method command = FeedforwardTuner.class.getDeclaredMethod("trialPower",
                FeedforwardTuner.Axis.class, int.class, double.class);
        command.setAccessible(true);
        MeasuredMecanumExperimentTest.Rig rig = new MeasuredMecanumExperimentTest.Rig();
        rig.hardware.drivetrain.setPosition(new MotionVector(ApexSimulation.FIELD_CENTER_INCHES - 12,
                ApexSimulation.FIELD_CENTER_INCHES - 12, Math.PI / 4));
        double minimum = Double.POSITIVE_INFINITY, maximum = Double.NEGATIVE_INFINITY;
        try {
            for (int run = 0; run < 2; run++) {
                double startX = rig.hardware.drivetrain.position.x;
                double startY = rig.hardware.drivetrain.position.y;
                for (int tick = 0; tick <= 450; tick++) {
                    double power = (double) command.invoke(null, FeedforwardTuner.Axis.DRIVE, run, tick * .02);
                    rig.command(new double[]{power, power, power, power});
                    for (int sub = 0; sub < 4; sub++) { ApexSimulation.stepPhysics(rig.hardware, .005); }
                    double x = rig.hardware.drivetrain.position.x - ApexSimulation.FIELD_CENTER_INCHES;
                    double y = rig.hardware.drivetrain.position.y - ApexSimulation.FIELD_CENTER_INCHES;
                    minimum = Math.min(minimum, Math.min(x, y));
                    maximum = Math.max(maximum, Math.max(x, y));
                    assertTrue("Powered ramp remains clear of walls", Math.abs(x) < 48 && Math.abs(y) < 48);
                    double distance = Math.hypot(rig.hardware.drivetrain.position.x - startX,
                            rig.hardware.drivetrain.position.y - startY);
                    if (distance >= 48 || tick == 450) { break; }
                }
                rig.command(new double[4]);
                // Braking stays in the same physical run; no pose or velocity reset.
                for (int tick = 0; tick < 100; tick++) {
                    for (int sub = 0; sub < 4; sub++) { ApexSimulation.stepPhysics(rig.hardware, .005); }
                    double x = rig.hardware.drivetrain.position.x - ApexSimulation.FIELD_CENTER_INCHES;
                    double y = rig.hardware.drivetrain.position.y - ApexSimulation.FIELD_CENTER_INCHES;
                    minimum = Math.min(minimum, Math.min(x, y));
                    maximum = Math.max(maximum, Math.max(x, y));
                    assertTrue("Room for the robot and braking", Math.abs(x) < 48 && Math.abs(y) < 48);
                }
            }
            System.out.printf(java.util.Locale.US, "RAMP FIELD center coordinates min=%.3f max=%.3f%n", minimum, maximum);
        } finally {
            rig.command(new double[4]);
        }
    }
}
