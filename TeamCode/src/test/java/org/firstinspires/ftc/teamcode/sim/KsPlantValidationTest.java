package org.firstinspires.ftc.teamcode.sim;

import org.junit.Test;
import org.codeblooded.ftcodesim.ascope.boundaries.MotionVector;
import java.lang.reflect.*;
import static org.junit.Assert.*;

/** Exercise the production regression with complete stock-chassis observations, without wall-clock waits. */
public class KsPlantValidationTest {
    @Test public void driveAndHeadingKsAgreeWithLowSpeedPlantAuthority() throws Exception {
        Class<?> regressionType = Class.forName("setup.follower.sysid.SysIdRegression");
        Constructor<?> constructor = regressionType.getDeclaredConstructor(); constructor.setAccessible(true);
        Method add = regressionType.getDeclaredMethod("add", int.class, double.class, double.class, double.class, double.class);
        Method fitMethod = regressionType.getDeclaredMethod("fit", double.class);
        add.setAccessible(true); fitMethod.setAccessible(true);
        StringBuilder discrepancies = new StringBuilder();
        for (boolean heading : new boolean[]{false, true}) {
            Object regression = constructor.newInstance();
            MeasuredMecanumExperimentTest.Rig rig = new MeasuredMecanumExperimentTest.Rig();
            double gain = rig.motors[0].config.modelCoefficients[0]
                    * rig.motors[0].config.voltageSensor.getVoltage() * rig.hardware.drivetrain.config.wheelRadius;
            org.codeblooded.ftcodesim.hardware.drivetrain.SimMecanumConfig config =
                    (org.codeblooded.ftcodesim.hardware.drivetrain.SimMecanumConfig) rig.hardware.drivetrain.config;
            double expectedKs = heading ? config.turnNaturalDeceleration * (config.wheelbase + config.trackWidth) / (2 * gain)
                    : config.forwardNaturalDeceleration / gain;
            for (int run = 0; run < 4; run++) {
                double sign = run % 2 == 0 ? 1 : -1;
                rig.hardware.drivetrain.velocity = new MotionVector(0, 0, 0);
                rig.hardware.drivetrain.updateWheelRollVelocities();
                double sumVelocity = 0, sumPower = 0, startVelocity = 0;
                int windowSamples = 0;
                int ticks = run < 2 ? 450 : 100;
                for (int tick = 0; tick < ticks; tick++) {
                    double time = tick * .02;
                    double power = sign * (run < 2 ? Math.min(.9, .1 * time) : .5);
                    double before = heading ? rig.hardware.drivetrain.velocity.theta : rig.hardware.drivetrain.velocity.x;
                    rig.command(MeasuredMecanumExperimentTest.wheels(heading ? 0 : power, 0, heading ? power : 0, 1));
                    // Keep acquisition clear of field walls; preserve all velocities and motor physics.
                    rig.hardware.drivetrain.setPosition(new MotionVector(ApexSimulation.FIELD_CENTER_INCHES,
                            ApexSimulation.FIELD_CENTER_INCHES, 0));
                    for (int sub = 0; sub < 4; sub++) ApexSimulation.stepPhysics(rig.hardware, .005);
                    double after = heading ? rig.hardware.drivetrain.velocity.theta : rig.hardware.drivetrain.velocity.x;
                    double threshold = heading ? .05 : .5;
                    if (Math.abs(before) < threshold || Math.abs(after) < threshold) {
                        sumVelocity = sumPower = 0; windowSamples = 0; continue;
                    }
                    if (windowSamples == 0) startVelocity = before;
                    sumVelocity += (before + after) / 2;
                    sumPower += power;
                    if (++windowSamples == 5) {
                        add.invoke(regression, run, time + .02, sumVelocity / 5, (after - startVelocity) / .1, sumPower / 5);
                        sumVelocity = sumPower = 0; windowSamples = 0;
                    }
                }
            }
            Object fit = fitMethod.invoke(regression, heading ? .2 : 2);
            Field ksField = fit.getClass().getDeclaredField("kS"), validField = fit.getClass().getDeclaredField("valid");
            ksField.setAccessible(true); validField.setAccessible(true);
            double ks = ksField.getDouble(fit);
            System.out.printf(java.util.Locale.US, "KS VALIDATION axis=%s fitted=%.9f lowSpeedPlant=%.9f%n",
                    heading ? "HEADING" : "DRIVE", ks, expectedKs);
            assertTrue("Production regression accepts the four stock-plant runs", validField.getBoolean(fit));
            // The affine intercept approximates a speed-dependent plant; it is not an exact inverse.
            if (Math.abs(expectedKs - ks) > (heading ? .01 : .03)) {
                discrepancies.append(heading ? "HEADING" : "DRIVE").append(" fitted=").append(ks)
                        .append(" expected low-speed=").append(expectedKs).append(';');
            }
        }
        assertEquals("Joint fitted kS must agree with low-speed chassis authority", "", discrepancies.toString());
    }

    @Test public void breakawayReturnsLastObservedStationaryPowerOnTheStockPlant() throws Exception {
        Class<?> tuner = Class.forName("setup.follower.sysid.FeedforwardTuner");
        Method crossing = tuner.getDeclaredMethod("breakawayCrossingPower", int.class, double.class,
                double.class, double.class, double.class, double.class);
        crossing.setAccessible(true);
        for (boolean heading : new boolean[]{false, true}) for (int run = 0; run < 2; run++) {
            MeasuredMecanumExperimentTest.Rig rig = new MeasuredMecanumExperimentTest.Rig();
            double sign = run == 0 ? 1 : -1, lastStationary = 0;
            double threshold = heading ? .05 : .5;
            boolean detected = false;
            for (int step = 1; step <= 400; step++) {
                double power = sign * step * .001;
                double before = heading ? rig.hardware.drivetrain.velocity.theta : rig.hardware.drivetrain.velocity.x;
                rig.command(MeasuredMecanumExperimentTest.wheels(heading ? 0 : power, 0, heading ? power : 0, 1));
                for (int sub = 0; sub < 100; sub++) ApexSimulation.stepPhysics(rig.hardware, .005);
                double after = heading ? rig.hardware.drivetrain.velocity.theta : rig.hardware.drivetrain.velocity.x;
                double saved = (double) crossing.invoke(null, run, before, after, power, lastStationary, threshold);
                if (Double.isFinite(saved)) {
                    assertEquals("Save the observed stationary command, not the moving command", Math.abs(lastStationary), saved, 0);
                    assertTrue("Saved command precedes moving command", saved < Math.abs(power));
                    System.out.printf(java.util.Locale.US, "BREAKAWAY axis=%s direction=%+.0f stationary=%.6f moving=%.6f before=%.6f after=%.6f threshold=%.3f%n",
                            heading ? "HEADING" : "DRIVE", sign, saved, Math.abs(power), before, after, threshold);
                    detected = true; break;
                }
                if (Math.abs(after) < threshold) lastStationary = power;
            }
            assertTrue("Detect breakaway on each axis in both directions", detected);
        }
    }
}
