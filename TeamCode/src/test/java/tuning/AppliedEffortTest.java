package tuning;

import drivetrains.BaseDrivetrain;
import drivetrains.Kiwi;
import drivetrains.Mecanum;
import drivetrains.Motor;
import drivetrains.Tank;
import org.junit.Test;
import support.FakeHardware;
import static org.junit.Assert.*;

public class AppliedEffortTest {
    @Test public void recoversAppliedEffortAfterLimitsForTwoAndFourMotorDrives() {
        FakeHardware map = hardware();
        BaseDrivetrain<?>[] drives = {
                new Tank.Constants().setFrontLeftMotor(new Motor("fl")).setFrontRightMotor(new Motor("fr"))
                        .setMaxPower(.4).build(map),
                new Mecanum.Constants().setFrontLeftMotor(new Motor("fl")).setFrontRightMotor(new Motor("fr"))
                        .setBackLeftMotor(new Motor("bl")).setBackRightMotor(new Motor("br"))
                        .setMaxPower(.4).build(map)};
        for (BaseDrivetrain<?> d : drives) {
            d.moveWithVectors(-.7, 0, 0);
            assertEquals(-.4, TuningMeasurements.appliedEffort(d, false, -.7), 1e-9);
            d.moveWithVectors(0, 0, .7);
            assertEquals(.4, TuningMeasurements.appliedEffort(d, true, .7), 1e-9);
            d.stop();
            assertEquals(0, TuningMeasurements.appliedEffort(d, true, 0), 0);
        }
    }

    @Test public void kiwiUsesItsOwnMixerRatherThanTankMotorSigns() {
        Kiwi d = new Kiwi.Constants().setFrontLeftMotor(new Motor("fl")).setFrontRightMotor(new Motor("fr"))
                .setBackMotor(new Motor("bl")).build(hardware());
        d.moveWithVectors(.3, 0, 0);
        assertEquals(.3, TuningMeasurements.appliedEffort(d, false, .3), 1e-9);
        d.moveWithVectors(0, 0, -.3);
        assertEquals(-.3, TuningMeasurements.appliedEffort(d, true, -.3), 1e-9);
    }

    private FakeHardware hardware() {
        FakeHardware map = new FakeHardware();
        map.motor("fl"); map.motor("fr"); map.motor("bl"); map.motor("br");
        return map;
    }
}
