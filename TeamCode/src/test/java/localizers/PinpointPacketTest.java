package localizers;

import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import geometry.DistUnit;
import geometry.Vector;
import java.lang.reflect.Proxy;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.HashMap;
import java.util.Map;
import org.junit.Test;
import static org.junit.Assert.*;

public class PinpointPacketTest {
    private static final class Device {
        byte[] packet;
        final Map<Integer, byte[]> writes = new HashMap<>();
        final Pinpoint.Driver driver = new Pinpoint.Driver((I2cDeviceSynchSimple) Proxy.newProxyInstance(
                getClass().getClassLoader(), new Class<?>[]{I2cDeviceSynchSimple.class},
                (proxy, method, args) -> {
                    if (method.getName().equals("read")) { return packet; }
                    if (method.getName().equals("write") && args.length >= 2 && args[1] instanceof byte[]) {
                        writes.put((Integer) args[0], ((byte[]) args[1]).clone());
                    }
                    if (method.getReturnType() == boolean.class) { return false; }
                    if (method.getReturnType() == int.class) { return 0; }
                    return null;
                }), false);
        void sample(int status, float position, float velocity) {
            packet = ByteBuffer.allocate(40).order(ByteOrder.LITTLE_ENDIAN)
                    .putInt(status).putInt(20).putInt(12).putInt(34)
                    .putFloat(position).putFloat(0).putFloat(0)
                    .putFloat(velocity).putFloat(0).putFloat(0).array();
            driver.update();
        }
    }
    @Test public void writesOffsetsInMillimeters() {
        Device device = new Device();
        device.driver.setOffsets(Vector.of(100, -50, DistUnit.MM));
        assertEquals(100, ByteBuffer.wrap(device.writes.get(15)).order(ByteOrder.LITTLE_ENDIAN).getFloat(), 0);
        assertEquals(-50, ByteBuffer.wrap(device.writes.get(16)).order(ByteOrder.LITTLE_ENDIAN).getFloat(), 0);
    }
    @Test public void invalidPacketsDoNotRefreshAcceptedState() {
        Device device = new Device();
        device.sample(1, 100, 2000);
        assertTrue(device.driver.isLastSampleValid());
        assertEquals(2000, device.driver.getVelocity().getX().getMm(), 1e-9);
        long time = device.driver.getSampleTimeNanos();
        device.sample(2, 999, 0);
        assertFalse(device.driver.isLastSampleValid());
        assertEquals(time, device.driver.getSampleTimeNanos());
        assertEquals(100, device.driver.getPosition().getX().getMm(), 1e-9);
        device.sample(1, Float.NaN, 0);
        assertFalse(device.driver.isLastSampleValid());
        assertEquals(time, device.driver.getSampleTimeNanos());
    }
}
