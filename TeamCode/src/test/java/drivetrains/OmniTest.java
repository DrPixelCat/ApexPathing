package drivetrains;

import static org.junit.Assert.*;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import geometry.Angle;
import java.lang.reflect.Proxy;
import org.junit.Test;
import tuning.follower.TuningMeasurements;

public class OmniTest {
    private static final class Fixture {
        final Omni.Constants constants = new Omni.Constants();
        final double[] output;
        int builds;
        Fixture(int count, double offsetDegrees) {
            output = new double[count];
            constants.setFirstWheelAngle(Angle.fromDeg(offsetDegrees));
            for (int i = 0; i < count; i++) {
                final int index = i;
                constants.addWheel(new Motor("wheel" + i) {
                    @Override public DcMotorEx build(HardwareMap hardwareMap) {
                        builds++;
                        return (DcMotorEx) Proxy.newProxyInstance(DcMotorEx.class.getClassLoader(),
                                new Class<?>[]{DcMotorEx.class}, (proxy, method, args) -> {
                                    if (method.getName().equals("setPower")) { output[index] = (double) args[0]; }
                                    if (method.getName().equals("getPower")) { return output[index]; }
                                    return null;
                                });
                    }
                });
            }
        }
        Omni build() { return constants.build(null); }
    }

    @Test public void everyUniformLayoutSpansTranslationAndRotation() {
        for (int count : new int[]{3, 4, 5, 6, 8, 12, 37, 128}) {
            for (double offset : new double[]{0, 17, 90, -50, 720}) {
                Fixture fixture = new Fixture(count, offset);
                Omni drive = fixture.build();
                assertEquals(count, fixture.builds);
                assertEquals(count, drive.getMotorCount());
                drive.drive(.23, .37, .19);
                double forward = 0, strafe = 0, rotation = 0;
                for (int i = 0; i < count; i++) {
                    double angle = Math.toRadians(offset) + 2 * Math.PI * i / count;
                    double power = fixture.output[i];
                    forward += -Math.sin(angle) * power;
                    strafe += Math.cos(angle) * power;
                    rotation += power;
                }
                // Recover the requested twist independently from the wheel outputs.
                assertEquals(.23, 2 * forward / count, .001);
                assertEquals(.37, 2 * strafe / count, .001);
                assertEquals(.19, rotation / count, .001);
            }
        }
    }

    @Test public void fourWheelLayoutAndMountingOffsetHavePredictableSigns() {
        Fixture aligned = new Fixture(4, 0);
        aligned.build().drive(.2, .3, .1);
        assertArrayEquals(new double[]{.4, -.1, -.2, .3}, aligned.output, 1e-12);
        Fixture rotated = new Fixture(4, 90);
        rotated.build().drive(.2, .3, .1);
        assertArrayEquals(new double[]{-.1, -.2, .3, .4}, rotated.output, 1e-12);
    }

    @Test public void rotationAndFieldCentricCommandsUseAllWheels() {
        Fixture fixture = new Fixture(8, 0);
        fixture.constants.setRobotCentric(false);
        Omni drive = fixture.build();
        drive.drive(0, 0, .3);
        for (double power : fixture.output) { assertEquals(.3, power, 1e-12); }
        drive.drive(.2, 0, 0, Math.PI / 2);
        for (int i = 0; i < 8; i++) {
            assertEquals(-.2 * Math.cos(i * Math.PI / 4), fixture.output[i], 1e-12);
        }
    }

    @Test public void limitingStoppingAndTuningIncludeMotorsBeyondTheFourth() {
        Fixture fixture = new Fixture(6, 0);
        fixture.constants.setMaxPower(.6);
        Omni drive = fixture.build();
        drive.setWheelPowers(.1, -.2, .3, .4, 1.2, -2.4);
        double[] expected = {.025, -.05, .075, .1, .3, -.6};
        assertArrayEquals(expected, fixture.output, 1e-12);
        assertArrayEquals(expected, drive.getLastMotorPowers(), 1e-12);
        assertEquals(.6, TuningMeasurements.maxMotorPower(drive), 1e-12);
        double[] copy = drive.getLastMotorPowers();
        copy[5] = 0;
        assertEquals(-.6, drive.getLastMotorPower(5), 1e-12);
        drive.stop();
        assertArrayEquals(new double[6], fixture.output, 0);
        assertArrayEquals(new double[6], drive.getLastMotorPowers(), 0);
    }

    @Test public void axisTuningUsesBodyCommandUnitsIncludingAfterOutputLimiting() {
        Fixture fixture = new Fixture(3, 0);
        Omni drive = fixture.build();
        drive.drive(.6, 0, 0);
        assertEquals(.6 * Math.sqrt(3) / 2, TuningMeasurements.maxMotorPower(drive), 1e-12);
        assertEquals(.6, TuningMeasurements.axisCommandPower(drive,
                feedforward.MotionCharacterization.Axis.FORWARD), 1e-12);
        drive.drive(0, .6, 0);
        assertEquals(.6, TuningMeasurements.axisCommandPower(drive,
                feedforward.MotionCharacterization.Axis.STRAFE), 1e-12);
        drive.drive(0, 0, .6);
        assertEquals(.6, TuningMeasurements.axisCommandPower(drive,
                feedforward.MotionCharacterization.Axis.ANGULAR), 1e-12);
        fixture.constants.setMaxPower(.3);
        drive.drive(.8, 0, 0);
        assertEquals(.3, TuningMeasurements.maxMotorPower(drive), 1e-12);
        assertEquals(.6 / Math.sqrt(3), TuningMeasurements.axisCommandPower(drive,
                feedforward.MotionCharacterization.Axis.FORWARD), 1e-12);
    }

    @Test public void invalidCommandsStopEveryWheelBeforeFailing() {
        Fixture fixture = new Fixture(7, 0);
        Omni drive = fixture.build();
        drive.drive(0, 0, .4);
        assertThrows(IllegalArgumentException.class, () -> drive.drive(Double.NaN, 0, 0));
        assertArrayEquals(new double[7], fixture.output, 0);
        drive.drive(0, 0, .4);
        assertThrows(IllegalArgumentException.class, () -> drive.setWheelPowers(1, 2, 3));
        assertArrayEquals(new double[7], fixture.output, 0);
        drive.drive(0, 0, .4);
        fixture.constants.maxPower = Double.POSITIVE_INFINITY;
        assertThrows(IllegalStateException.class, () -> drive.drive(0, 0, .4));
        assertArrayEquals(new double[7], fixture.output, 0);
    }

    @Test public void invalidWheelLayoutsAreRejectedBeforeBindingHardware() {
        for (int count = 0; count < 3; count++) {
            Fixture fixture = new Fixture(count, 0);
            assertThrows(IllegalArgumentException.class, fixture::build);
            assertEquals(0, fixture.builds);
        }
        Fixture duplicate = new Fixture(3, 0);
        duplicate.constants.addWheel(new Motor("wheel0"));
        assertThrows(IllegalArgumentException.class, duplicate::build);
        assertEquals(0, duplicate.builds);
        Fixture fixedSlots = new Fixture(3, 0);
        fixedSlots.constants.flMotorConfig = new Motor("wrongSlot");
        assertThrows(IllegalArgumentException.class, fixedSlots::build);
        assertThrows(NullPointerException.class, () -> new Omni.Constants().addWheel(null));
        assertThrows(IllegalArgumentException.class,
                () -> new Omni.Constants().setFirstWheelAngle(Angle.fromRad(Double.NaN)));
        assertThrows(IllegalArgumentException.class,
                () -> new Omni.Constants().setFirstWheelAngle(null));
    }

    @Test public void existingDrivesKeepTheirConstructedGeometryWhenConfigurationIsExtended() {
        Fixture fixture = new Fixture(3, 0);
        Omni drive = fixture.build();
        fixture.constants.addWheel(new Motor("newWheel")).setFirstWheelAngle(Angle.fromDeg(90));
        drive.drive(0, .6, 0);
        assertEquals(3, drive.getMotorCount());
        assertArrayEquals(new double[]{.6, -.3, -.3}, fixture.output, 1e-12);
    }
}
