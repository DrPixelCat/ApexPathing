package drivetrains;

import org.junit.Test;
import support.FakeHardware;
import support.FakeHardware.MotorState;

import static org.junit.Assert.*;

public class MotorPowerRegressionTest {
    private final FakeHardware map=new FakeHardware();
    private final MotorState fl=map.motor("fl"),fr=map.motor("fr"),bl=map.motor("bl"),br=map.motor("br");
    private final Mecanum.Constants constants=new Mecanum.Constants()
            .setFrontLeftMotor(new Motor("fl")).setFrontRightMotor(new Motor("fr"))
            .setBackLeftMotor(new Motor("bl")).setBackRightMotor(new Motor("br"));

    @Test public void fractionalPowerLimitIsPreservedAndAppliedToAllMotors() {
        constants.setMaxPower(.3); assertEquals(.3,constants.maxPower,0);
        Mecanum drive=constants.build(map); drive.setPowers(1,-.5,.25,-1);
        assertEquals(.3,fl.power,0); assertEquals(-.15,fr.power,0);
        assertEquals(.075,bl.power,0); assertEquals(-.3,br.power,0);
        assertEquals(0,constants.setMaxPower(-1).maxPower,0);
        assertEquals(1,constants.setMaxPower(2).maxPower,0);
    }

    @Test public void stopSendsZeroEvenWhenTheLastPowerWasBelowTheWriteTolerance() {
        Mecanum drive=constants.build(map); drive.setPowers(.01,-.01,.01,-.01);
        drive.setPowers(.0005,-.0005,.0005,-.0005); drive.stop();
        assertEquals(0,fl.power,0); assertEquals(0,fr.power,0);
        assertEquals(0,bl.power,0); assertEquals(0,br.power,0);
        assertEquals(0,drive.getLastFlPower(),0);
    }

    @Test public void decreasingTheLimitByLessThanTheToleranceStillLimitsOutput() {
        Mecanum drive=constants.build(map); constants.maxPower=.5; drive.setPowers(1,1,1,1);
        constants.maxPower=.4995; drive.setPowers(1,1,1,1);
        assertEquals(.4995,fl.power,0); assertEquals(.4995,br.power,0);
    }

    @Test public void rejectsNonfinitePowerLimits() {
        assertThrows(IllegalArgumentException.class,()->constants.setMaxPower(Double.NaN));
        assertThrows(IllegalArgumentException.class,()->constants.setMaxPower(Double.POSITIVE_INFINITY));
    }
}
