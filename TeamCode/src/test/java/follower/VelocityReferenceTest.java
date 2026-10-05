package follower;

import org.junit.Test;
import static org.junit.Assert.*;

public class VelocityReferenceTest {
    @Test public void perfectHeldAccelerationHasNoManufacturedErrorWithDelayedSensors() {
        VelocityReference reference=new VelocityReference();
        assertEquals(0,reference.update(1_000_000_000L,1_000_000_000L,0,10),0);
        assertEquals(.1,reference.update(1_020_000_000L,1_010_000_000L,.1,10),1e-12);
        assertEquals(.2,reference.update(1_040_000_000L,1_020_000_000L,.2,10),1e-12);
        assertEquals(.2,reference.update(1_050_000_000L,1_020_000_000L,.2,10),1e-12);
        assertEquals(.6,reference.update(1_060_000_000L,1_060_000_000L,.6,-10),1e-12);
        assertEquals(.4,reference.update(1_080_000_000L,1_080_000_000L,.4,-10),1e-12);
    }
    @Test public void disturbanceIsComparedAtCurrentTimeAndClippingDoesNotWindUp() {
        VelocityReference reference=new VelocityReference();long time=1_000_000_000L;
        reference.update(time,time,0,10);
        assertEquals(.2,reference.update(time+20_000_000L,time+20_000_000L,0,10),1e-12);
        for(int i=1;i<50;i++) {
            reference.condition(0);
            time+=20_000_000L;
            assertEquals(.2,reference.update(time+20_000_000L,time+20_000_000L,0,10),1e-12);
        }
        reference.reset();
        assertEquals(5,reference.update(5_000_000_000L,5_000_000_000L,5,0),0);
    }
    @Test public void staleOrInvalidMeasurementsResetAndBandwidthRespectsElapsedTime() {
        VelocityReference reference=new VelocityReference();
        reference.update(1_000_000_000L,1_000_000_000L,2,10);
        reference.update(1_200_000_000L,1_100_000_000L,3,10);
        assertEquals(1/.3,reference.powerGain(10,1),1e-12);
        assertEquals(2,reference.update(2_000_000_000L,1_200_000_000L,2,10),0);
        assertFalse(reference.isUsable());assertEquals(0,reference.powerGain(1,100),0);
        assertEquals(7,reference.update(3_000_000_000L,3_000_000_000L,7,0),0);
        assertEquals(7,reference.update(3_020_000_000L,3_030_000_000L,7,0),0);
        assertFalse(reference.isUsable());
        assertEquals(0,reference.update(4_000_000_000L,4_000_000_000L,Double.NaN,0),0);
        assertFalse(reference.isUsable());
    }
    @Test public void brakingReferenceStopsWithoutReversing() {
        VelocityReference reference=new VelocityReference();
        reference.update(1_000_000_000L,1_000_000_000L,1,-10);
        assertEquals(0,reference.update(1_200_000_000L,1_200_000_000L,0,-10),0);
        assertEquals(0,reference.update(1_220_000_000L,1_220_000_000L,0,0),0);
    }
}
