package org.firstinspires.ftc.teamcode.sim;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.codeblooded.ftcodesim.hardware.devices.SimMotor;
import org.codeblooded.ftcodesim.hardware.drivetrain.SimMecanumConfig;
import org.junit.Test;

import static org.junit.Assert.*;

/** Checks the loaded upstream plant against its equations, independently of follower calibration. */
public class MecanumPlantPhysicsTest {
    @Test public void motorRegimesMatchTheRegisteredDutyDependentEquation() {
        ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
        SimMotor motor = StockSimTestSupport.motors(hardware)[0];
        double radius = hardware.drivetrain.config.wheelRadius;
        double gain = motor.config.modelCoefficients[0] * motor.config.voltageSensor.getVoltage() * radius;
        double speedLoss = motor.config.modelCoefficients[1];
        assertEquals(183, gain, 1e-9);
        assertEquals(150.0 / 65, speedLoss, 1e-12);
        for (DcMotor.ZeroPowerBehavior behavior : DcMotor.ZeroPowerBehavior.values()) {
            motor.setZeroPowerBehavior(behavior);
            for (double speed : new double[]{-60, -20, 0, 20, 60}) {
                for (double requested : new double[]{-2, -.6, -.001, -1e-6, 0, 1e-6, .001, .6, 2}) {
                    motor.setRollVelocity(speed / radius);
                    motor.setPower(requested);
                    double power = motor.getPower();
                    double[] coefficients = behavior == DcMotor.ZeroPowerBehavior.BRAKE && power == 0
                            ? motor.config.zeroPowerBrakeCoefficients : motor.config.modelCoefficients;
                    double expected = coefficients[0] * motor.config.voltageSensor.getVoltage() * radius * power;
                    if (Math.signum(speed) == Math.signum(power)) expected -= coefficients[1] * speed * Math.abs(power);
                    if (power != 0 && Math.signum(speed) != Math.signum(power)) expected -= coefficients[2] * speed;
                    if (power == 0) expected -= coefficients[3] * speed;
                    expected -= coefficients[4] * radius * Math.signum(speed);
                    motor.update(.0001);
                    assertEquals("speed=" + speed + " duty=" + power + " behavior=" + behavior,
                            expected, motor.getAcceleration() * radius, 1e-9);
                }
            }
        }
    }

    @Test public void chassisDragAndAllThreePowerRegimesMatchAxisProbes() {
        MeasuredMecanumExperimentTest.Rig rig = new MeasuredMecanumExperimentTest.Rig();
        SimMecanumConfig config = (SimMecanumConfig) rig.hardware.drivetrain.config;
        SimMotor motor = rig.motors[0];
        double gain = motor.config.modelCoefficients[0] * motor.config.voltageSensor.getVoltage() * config.wheelRadius;
        double speedLoss = motor.config.modelCoefficients[1];
        double radius = (config.wheelbase + config.trackWidth) / 2;
        for (int axis = 0; axis < 3; axis++) for (double sign : new double[]{-1, 1}) {
            double speed = axis == 2 ? .8 : 17;
            double lever = axis == 2 ? radius : 1;
            double drag = axis == 0 ? config.forwardNaturalDeceleration
                    : axis == 1 ? config.strafeNaturalDeceleration : config.turnNaturalDeceleration;
            for (double alignedPower : new double[]{.3, 0, -.1}) {
                double power = sign * alignedPower;
                double wheelSpeed = lever * speed;
                double wheelAcceleration = alignedPower > 0 ? (gain - speedLoss * wheelSpeed) * alignedPower
                        : gain * alignedPower - (alignedPower == 0
                            ? motor.config.zeroPowerBrakeCoefficients[3] : motor.config.modelCoefficients[2]) * wheelSpeed;
                double expected = sign * (wheelAcceleration / lever - drag);
                double[] actual = rig.probe(axis == 0 ? sign * speed : 0, axis == 1 ? sign * speed : 0,
                        axis == 2 ? sign * speed : 0,
                        MeasuredMecanumExperimentTest.wheels(axis == 0 ? power : 0,
                                axis == 1 ? power : 0, axis == 2 ? power : 0, 1));
                assertEquals("axis=" + axis + " sign=" + sign + " duty=" + power, expected, actual[axis], 1e-7);
            }
        }
    }

    @Test public void measuredCoupledCalibrationAgreesWithLoadedPlantAndSavedCalibration() {
        MeasuredMecanumExperimentTest.Model measured = MeasuredMecanumExperimentTest.calibrate();
        ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
        SimMecanumConfig config = (SimMecanumConfig) hardware.drivetrain.config;
        SimMotor motor = StockSimTestSupport.motors(hardware)[0];
        feedforward.MecanumWheelModel saved = SimulatorTuning.snapshot().motionCharacterization.mecanumWheelModel;
        double gain = motor.config.modelCoefficients[0] * motor.config.voltageSensor.getVoltage() * config.wheelRadius;
        assertEquals(gain, measured.gain, 1e-7);
        assertEquals(motor.config.modelCoefficients[1], measured.speedLoss, 1e-7);
        assertEquals(motor.config.modelCoefficients[2], measured.regen, 1e-7);
        assertEquals(motor.config.zeroPowerBrakeCoefficients[3], measured.neutral, 1e-7);
        assertEquals(config.forwardNaturalDeceleration, measured.forwardDrag, 1e-7);
        assertEquals(config.strafeNaturalDeceleration, measured.strafeDrag, 1e-7);
        assertEquals(config.turnNaturalDeceleration, measured.angularDrag, 1e-7);
        assertEquals((config.wheelbase + config.trackWidth) / 2, measured.radius, 1e-7);
        assertEquals(measured.gain, saved.gain, 1e-7);
        assertEquals(measured.speedLoss, saved.speedLoss, 1e-7);
        assertEquals(measured.regen, saved.regenerativeBraking, 1e-7);
        assertEquals(measured.neutral, saved.neutralBraking, 1e-7);
        assertEquals(measured.forwardDrag, saved.forwardDrag, 1e-7);
        assertEquals(measured.strafeDrag, saved.strafeDrag, 1e-7);
        assertEquals(measured.angularDrag, saved.angularDrag, 1e-7);
        assertEquals(measured.radius, saved.effectiveRadius, 1e-7);
        System.out.printf(java.util.Locale.US,
                "PLANT gain=%.9f speedLoss=%.9f regen=%.9f neutral=%.9f forwardDrag=%.9f strafeDrag=%.9f angularDrag=%.9f radius=%.9f runtime=%s%n",
                measured.gain, measured.speedLoss, measured.regen, measured.neutral, measured.forwardDrag,
                measured.strafeDrag, measured.angularDrag, measured.radius,
                SimMotor.class.getProtectionDomain().getCodeSource().getLocation());
    }
}
