package org.firstinspires.ftc.teamcode.sim;

import org.codeblooded.fit.MotorModel;
import org.codeblooded.ftcodesim.hardware.devices.SimMotor;
import org.junit.Test;
import static org.junit.Assert.*;

public class SimulatorSysIdMotorOverrideTest {
    @Test public void scopeChangesOnlyPoweredBackEmfAndRestoresExactTerms() {
        ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
        SimMotor[] motors = StockSimTestSupport.motors(hardware);
        MotorModel.Term[][] original = new MotorModel.Term[motors.length][];
        for (int i=0; i<motors.length; i++) original[i] = motors[i].config.motorModel.terms.clone();
        Runnable restore = SimulatorSysIdMotorOverride.open(hardware.hardwareMap);
        try {
            for (int i=0; i<motors.length; i++) {
                MotorModel.Term[] terms = motors[i].config.motorModel.terms;
                assertEquals(-12, terms[1].calculate(12, .3, 13), 0);
                assertEquals(12, terms[1].calculate(-12, -.3, 13), 0);
                assertEquals(0, terms[1].calculate(12, -.3, 13), 0);
                assertEquals(0, terms[1].calculate(12, 0, 13), 0);
                for (int j=0; j<terms.length; j++) if(j!=1) assertSame(original[i][j], terms[j]);
            }
        } finally { restore.run(); }
        restore.run(); // Cleanup is safe to repeat.
        for (int i=0; i<motors.length; i++) assertArrayEquals(original[i], motors[i].config.motorModel.terms);
    }
}
