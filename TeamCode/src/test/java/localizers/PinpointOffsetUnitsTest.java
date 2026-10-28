package localizers;

import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import geometry.DistUnit;
import geometry.Vector;
import java.lang.reflect.Proxy;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.HashMap;
import java.util.Map;
import org.junit.Test;
import static org.junit.Assert.*;

public class PinpointOffsetUnitsTest {
    @Test public void keepsOrdinaryHighSpeedMeasurementsAndRawPodTravelForCalibration() {
        byte[] packet = ByteBuffer.allocate(40).order(ByteOrder.LITTLE_ENDIAN)
                .putInt(1).putInt(20).putInt(1000).putInt(-2000)
                .putFloat(0).putFloat(0).putFloat(0)
                .putFloat(2000).putFloat(-1500).putFloat(.5f).array();
        I2cDeviceSynchSimple device = (I2cDeviceSynchSimple) Proxy.newProxyInstance(getClass().getClassLoader(),
                new Class<?>[]{I2cDeviceSynchSimple.class}, (proxy, method, args) -> {
                    if (method.getName().equals("read")) { return packet; }
                    if (method.getReturnType() == boolean.class) { return false; }
                    if (method.getReturnType() == int.class) { return 0; }
                    return null;
                });
        Pinpoint.Driver driver = new Pinpoint.Driver(device, false);
        HardwareMap map = new HardwareMap(null, null) {
            @Override public <T> T get(Class<? extends T> type, String name) { return type.cast(driver); }
        };
        Pinpoint l = new Pinpoint(new Pinpoint.Constants().setName("pinpoint")
                .setOffsets(4, -2, DistUnit.IN)
                .setEncoderDirections(Pinpoint.EncoderDirection.REVERSED, Pinpoint.EncoderDirection.FORWARD), map);
        l.update();
        assertEquals(2000, l.getRawVel().getX().getMm(), 1e-9);
        assertEquals(-1500, l.getRawVel().getY().getMm(), 1e-9);
        PodCalibrationReading[] readings = l.getPodCalibrationReadings();
        assertEquals(-1000 / 19.89436789 / 25.4, readings[0].travelIn, 1e-9);
        assertEquals(-2000 / 19.89436789 / 25.4, readings[1].travelIn, 1e-9);
        assertEquals(4, readings[0].configuredOffsetIn, 1e-9);
        assertEquals(-2, readings[1].configuredOffsetIn, 1e-9);
        assertTrue(readings[0].measuredOffset(0, 1) > 0);
    }

    @Test public void sendsPhysicalOffsetInMillimetersToHardware() {
        Map<Integer, byte[]> writes = new HashMap<>();
        I2cDeviceSynchSimple device = (I2cDeviceSynchSimple) Proxy.newProxyInstance(getClass().getClassLoader(),
                new Class<?>[]{I2cDeviceSynchSimple.class}, (proxy, method, args) -> {
                    if (method.getName().equals("write") && args[1] instanceof byte[]) {
                        writes.put((Integer) args[0], ((byte[]) args[1]).clone());
                    }
                    if (method.getReturnType() == boolean.class) { return false; }
                    if (method.getReturnType() == int.class) { return 0; }
                    return null;
                });
        Pinpoint.Driver driver = new Pinpoint.Driver(device, false);
        driver.setOffsets(Vector.of(4, -2, DistUnit.IN));
        assertEquals(101.6, ByteBuffer.wrap(writes.get(15)).order(ByteOrder.LITTLE_ENDIAN).getFloat(), 1e-5);
        assertEquals(-50.8, ByteBuffer.wrap(writes.get(16)).order(ByteOrder.LITTLE_ENDIAN).getFloat(), 1e-5);
    }
}
