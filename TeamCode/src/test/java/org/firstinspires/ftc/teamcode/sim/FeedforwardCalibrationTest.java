package org.firstinspires.ftc.teamcode.sim;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.codeblooded.ftcodesim.hardware.devices.SimMotor;
import org.junit.Test;
import static org.junit.Assert.*;
import java.util.Locale;

/** Identify unloaded motor authority from observations, then validate held-out states and a ramp. */
public class FeedforwardCalibrationTest {
    private static double acceleration(SimMotor motor, double radius, double speed, double power) {
        motor.setRollVelocity(speed / radius);
        motor.setPower(power);
        motor.update(.001);
        return motor.getAcceleration() * radius;
    }

    private static double powerFor(SimMotor motor, double radius, double speed, double accel) {
        double lo = 0, hi = 1;
        assertTrue("Identification acceleration must be attainable", acceleration(motor, radius, speed, hi) >= accel);
        for (int i = 0; i < 40; i++) {
            double mid = (lo + hi) / 2;
            if (acceleration(motor, radius, speed, mid) > accel) hi = mid; else lo = mid;
        }
        return (lo + hi) / 2;
    }

    @Test public void characterizeThenValidateOnIndependentVelocities() {
        ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
        double radius = hardware.drivetrain.config.wheelRadius;
        SimMotor motor = (SimMotor) hardware.hardwareMap.get(DcMotorEx.class, ApexSimulation.FRONT_LEFT_MOTOR);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // a = (gain0 + gainSlope * speed) * duty. Use two moving accelerations at
        // each speed: neutral/coasting is a separate regime, not a holding-power sample.
        double n = 0, sx = 0, sy = 0, sxx = 0, sxy = 0;
        for (double speed = 6; speed <= 42; speed += 6) {
            double lowPower = powerFor(motor, radius, speed, 15);
            double highPower = powerFor(motor, radius, speed, 45);
            double gain = 30 / (highPower - lowPower);
            n++; sx += speed; sy += gain; sxx += speed * speed; sxy += speed * gain;
        }
        double gainSlope = (n * sxy - sx * sy) / (n * sxx - sx * sx);
        double gain0 = (sy - gainSlope * sx) / n;
        assertTrue("Powered authority must decrease with speed", gainSlope < 0);

        double squared = 0;
        int count = 0;
        for (double sign : new double[]{-1, 1}) for (double speed = 9; speed <= 39; speed += 6) {
            assertEquals("An unloaded FLOAT motor coasts; this is not chassis holding power",
                    0, acceleration(motor, radius, sign * speed, 0), 1e-9);
            for (double target : new double[]{12, 24, 36}) {
                double power = sign * target / (gain0 + gainSlope * speed);
                double actual = acceleration(motor, radius, sign * speed, power);
                squared += Math.pow(actual - sign * target, 2); count++;
            }
        }
        double probeRmse = Math.sqrt(squared / count);
        assertTrue("Measured authority must predict independent speed/acceleration probes", probeRmse < 1e-6);

        squared = 0; count = 0;
        for (double sign : new double[]{-1, 1}) {
            motor.setRollVelocity(sign * 6 / radius);
            for (int i = 0; i < 50; i++) {
                double target = 6 + 30 * i * .02;
                motor.setPower(sign * 30 / (gain0 + gainSlope * target));
                for (int k = 0; k < 20; k++) motor.update(.001);
                // Compare both target and observation at the end of the held-command interval.
                double error = sign * motor.getVelocity() * radius - (target + 30 * .02);
                squared += error * error; count++;
            }
        }
        double rampRmse = Math.sqrt(squared / count);
        System.out.printf(Locale.US, "MOTOR AUTHORITY gain0=%.9f gainSlope=%.9f heldOutRMSE=%.9f rampRMSE=%.9f%n",
                gain0, gainSlope, probeRmse, rampRmse);
        assertTrue("Speed-dependent authority must track an independent ramp without feedback", rampRmse < .1);
    }
}
