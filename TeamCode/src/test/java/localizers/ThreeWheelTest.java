package localizers;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.codeblooded.ftcodesim.hardware.SimHardwareMap;
import org.junit.Test;

import java.lang.reflect.Proxy;
import java.util.concurrent.atomic.AtomicInteger;

import geometry.DistUnit;
import geometry.Pose;
import geometry.Vector;

import static org.junit.Assert.assertEquals;

public class ThreeWheelTest {
    @Test public void threeWheelTranslationAndSpinKeepTheirGeometry() {
        SimHardwareMap map = new SimHardwareMap();
        AtomicInteger left = registerEncoder(map, "left");
        AtomicInteger right = registerEncoder(map, "right");
        AtomicInteger strafe = registerEncoder(map, "strafe");
        ThreeWheel localizer = new ThreeWheel(new ThreeWheel.Constants()
                .setForwardLeftPodName("left").setForwardRightPodName("right")
                .setStrafePodName("strafe").setTicksPerInch(100)
                .setOffsets(Vector.of(10, 2, DistUnit.IN)), map);
        left.set(1000);
        right.set(1000);
        strafe.set(300);
        localizer.update();
        assertRawPose(localizer, 10, 3, 0);

        // Pure rotation: the perpendicular pod moves only because of its offset.
        left.addAndGet(500);
        right.addAndGet(-500);
        strafe.addAndGet(200);
        localizer.update();
        assertRawPose(localizer, 10, 3, 1);

        localizer.setPose(Pose.zero());
        localizer.update();
        assertRawPose(localizer, 0, 0, 0);
    }

    private static void assertRawPose(ThreeWheel localizer, double x, double y, double yaw) {
        Pose pose = localizer.getRawPose();
        assertEquals(x, pose.getX().getIn(), 1e-9);
        assertEquals(y, pose.getY().getIn(), 1e-9);
        assertEquals(yaw, pose.getHeading().getRad(), 1e-9);
    }

    private static AtomicInteger registerEncoder(SimHardwareMap map, String name) {
        AtomicInteger ticks = new AtomicInteger();
        DcMotorEx motor = (DcMotorEx) Proxy.newProxyInstance(DcMotorEx.class.getClassLoader(),
                new Class<?>[] {DcMotorEx.class}, (proxy, method, args) -> {
                    if (method.getName().equals("getCurrentPosition")) { return ticks.get(); }
                    if (method.getName().equals("getDeviceName")) { return name; }
                    if (method.getName().equals("hashCode")) { return System.identityHashCode(proxy); }
                    if (method.getName().equals("equals")) { return proxy == args[0]; }
                    return null;
                });
        map.register(name, motor);
        return ticks;
    }
}
