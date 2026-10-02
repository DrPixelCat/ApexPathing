package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.codeblooded.ftcodesim.hardware.devices.SimMotor;
import org.junit.Test;
import java.util.Locale;

/** Isolates the obsolete September 4 fit from path geometry, localization, and feedback. */
public class FeedforwardModelDiagnosticTest {
    @Test public void savedFitDoesNotMatchTheUpdatedMotorPlant() {
        ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
        SimMotor motor = (SimMotor) hardware.hardwareMap.get(DcMotorEx.class,
                ApexSimulation.FRONT_LEFT_MOTOR);
        double radius = 1.889765;
        double ks = 0.22225682732946256;
        double kv = 0.008331427055494441;
        double ka = 0.004965308609195665;
        double squared = 0;
        int count = 0;
        for (double velocity : new double[]{10, 20, 30, 40}) {
            for (double acceleration : new double[]{0, 30}) {
                double power = ks + kv * velocity + ka * acceleration;
                motor.setRollVelocity(velocity / radius);
                motor.setPower(power);
                motor.update(0.001);
                double actual = motor.getAcceleration() * radius;
                System.out.printf(Locale.US,
                        "MODEL CHECK v=%.1f targetA=%.1f power=%.4f actualA=%.3f error=%.3f%n",
                        velocity, acceleration, power, actual, actual - acceleration);
                squared += (actual-acceleration)*(actual-acceleration); count++;
            }
        }
        assertTrue("Obsolete fit must be rejected by independent acceleration probes", Math.sqrt(squared/count)>5);
        // Directly invert the actual loaded motor model at the same state as a control experiment.
        double velocity = 30 / radius;
        double drive = motor.config.modelCoefficients[0] * motor.config.voltageSensor.getVoltage();
        double backEmf = motor.config.modelCoefficients[1];
        // This motor-only probe excludes the chassis rolling-friction term.
        double holdingPower = backEmf * velocity / drive;
        motor.setRollVelocity(velocity);
        motor.setPower(holdingPower);
        motor.update(0.001);
        System.out.printf(Locale.US, "MODEL CONTROL holding power at 30 in/s = %.4f%n", holdingPower);
        assertEquals(0, motor.getAcceleration(), 1e-8);
    }
}
