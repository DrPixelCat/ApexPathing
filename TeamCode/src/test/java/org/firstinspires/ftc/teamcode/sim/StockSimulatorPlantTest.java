package org.firstinspires.ftc.teamcode.sim;

import org.codeblooded.ftcodesim.hardware.devices.SimMotor;
import org.codeblooded.ftcodesim.hardware.drivetrain.SimMecanumConfig;
import org.codeblooded.ftcodesim.hardware.drivetrain.SimulatedMecanum;
import org.codeblooded.ftcodesim.hardware.drivetrain.SimulatedTank;
import org.junit.Test;
import static org.junit.Assert.*;

/** Guard against substituting a different motor or chassis plant for FTCodeSim. */
public class StockSimulatorPlantTest {
    @Test public void upstreamExampleConfigurationAndConcreteClassesArePreserved() {
        ApexSimulation.Hardware h = ApexSimulation.createHardware();
        assertEquals(SimulatedMecanum.class,h.drivetrain.getClass());
        SimMecanumConfig c = (SimMecanumConfig)h.drivetrain.config;
        assertEquals(9.37008,c.wheelbase,0); assertEquals(9.13386,c.trackWidth,0);
        assertEquals(1.889765,c.wheelRadius,0); assertEquals(65,c.maxVelocity,0);
        assertEquals(150,c.maxAcceleration,0); assertEquals(33,c.naturalDeceleration,0);
        assertEquals(49,c.forwardNaturalDeceleration,0); assertEquals(85,c.strafeNaturalDeceleration,0);
        assertEquals(1,c.turnNaturalDeceleration,0); assertEquals(55,c.staticFriction,0);
        assertEquals(1e-3,c.staticVelocityRegion,0); assertEquals(13,c.nominalVoltage,0);
        assertEquals(.0014846306,c.quadraticBraking,0); assertEquals(.09533276,c.linearBraking,0);
        assertEquals(SimulatedTank.class,ApexSimulation.createTankHardware().drivetrain.getClass());
    }
    @Test public void registeredMotorModelAndBrakeCoefficientsMatchFreshUpstreamRegistration() {
        ApexSimulation.Hardware h = ApexSimulation.createHardware();
        SimulatedMecanum upstream = new SimulatedMecanum((SimMecanumConfig)h.drivetrain.config);
        StableSimHardwareMap referenceMap = new StableSimHardwareMap(); referenceMap.register(upstream);
        assertEquals(upstream.regenerativeBraking,h.drivetrain.regenerativeBraking,0);
        SimMotor[] actual = StockSimTestSupport.motors(h);
        for (int i=0;i<actual.length;i++) {
            SimMotor reference = (SimMotor)referenceMap.get(com.qualcomm.robotcore.hardware.DcMotorEx.class,
                    upstream.motorNames[i]);
            assertArrayEquals(reference.config.modelCoefficients,actual[i].config.modelCoefficients,0);
            assertArrayEquals(reference.config.zeroPowerBrakeCoefficients,actual[i].config.zeroPowerBrakeCoefficients,0);
            for (double velocity : new double[]{-12,0,12}) for (double power : new double[]{-.3,0,.3}) {
                reference.setRollVelocity(velocity); actual[i].setRollVelocity(velocity);
                reference.setPower(power); actual[i].setPower(power);
                reference.update(.001); actual[i].update(.001);
                assertEquals(reference.getAcceleration(),actual[i].getAcceleration(),0);
            }
        }
    }
}
