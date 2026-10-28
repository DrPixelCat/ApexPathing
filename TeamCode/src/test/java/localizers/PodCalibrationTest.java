package localizers;

import geometry.DistUnit;
import geometry.Vector;
import org.junit.Test;
import support.FakeHardware;
import support.FakeHardware.MotorState;
import static org.junit.Assert.*;

public class PodCalibrationTest {
    @Test public void threeWheelOffsetsUseAnExternalAngleDespiteWrongConfiguredSeparation() {
        FakeHardware map = new FakeHardware();
        MotorState left = map.motor("left"), right = map.motor("right"), strafe = map.motor("strafe");
        ThreeWheel l = new ThreeWheel(new ThreeWheel.Constants().setForwardLeftPodName("left")
                .setForwardRightPodName("right").setStrafePodName("strafe").setTicksPerInch(1000)
                .setOffsets(Vector.of(8, 0, DistUnit.IN)), map);
        PodCalibrationReading[] start = l.getPodCalibrationReadings();
        // One physically measured radian, true parallel separation=10 in and strafe offset=-2 in.
        left.ticks = 5000; right.ticks = -5000; strafe.ticks = -2000;
        l.update();
        PodCalibrationReading[] end = l.getPodCalibrationReadings();
        assertEquals(10, end[0].measuredOffset(start[0].travelIn, 1), 0);
        assertEquals(-2, end[1].measuredOffset(start[1].travelIn, 1), 0);
        assertEquals(8, end[0].configuredOffsetIn, 0);
        assertEquals(1.25, l.getPose().getHeading().getRad(), 1e-9);
    }

    @Test public void clockwiseTwoWheelRotationPreservesSignedOffsetConvention() {
        FakeHardware map = new FakeHardware();
        MotorState forward = map.motor("forward"), strafe = map.motor("strafe"); map.imu("imu");
        TwoWheel l = new TwoWheel(new TwoWheel.Constants().setForwardPodName("forward")
                .setStrafePodName("strafe").setIMUName("imu").setHubOrientation(null)
                .setTicksPerInch(100), map);
        PodCalibrationReading[] start = l.getPodCalibrationReadings();
        forward.ticks = -200; strafe.ticks = 300; map.yaw = -1;
        l.update();
        PodCalibrationReading[] end = l.getPodCalibrationReadings();
        assertEquals(2, end[0].measuredOffset(start[0].travelIn, -1), 0);
        assertEquals(-3, end[1].measuredOffset(start[1].travelIn, -1), 0);
    }
}
