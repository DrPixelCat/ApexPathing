package trajectory.profile;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class FFLutTimeTest {
    @Test public void squaredSpeedLookupPreservesEnergyAtLaunchAndStop() {
        FFLut lut = new FFLut(new MotionParameters[]{new MotionParameters(0,4,0,0,0),
                new MotionParameters(4,-4,0,0,2),new MotionParameters(0,0,0,0,4)});
        assertEquals(4, lut.getSquaredTangentialVelocity(.5), 1e-12);
        assertEquals(12, lut.getSquaredTangentialVelocity(2.5), 1e-12);
        assertEquals(0, lut.getSquaredTangentialVelocity(5), 0);
    }
    @Test public void independentReferenceObeysKinematicsDuringAccelerationAndBraking() {
        FFLut lut=new FFLut(new MotionParameters[]{
                new MotionParameters(0,4,0,0,0).setTimeSeconds(0),
                new MotionParameters(4,-4,0,0,2).setTimeSeconds(1),
                new MotionParameters(0,0,0,0,4).setTimeSeconds(2)});
        MotionParameters launch=lut.getKinematicParamsByTime(.25),brake=lut.getKinematicParamsByTime(1.25);
        assertEquals(.125,launch.getProgression(),1e-12); assertEquals(1,launch.getTangentialVel(),1e-12);
        assertEquals(4,launch.getTangentialAccel(),1e-12);
        assertEquals(2.875,brake.getProgression(),1e-12);assertEquals(3,brake.getTangentialVel(),1e-12);
        assertEquals(-4,brake.getTangentialAccel(),1e-12);
        double h=1e-5;
        assertEquals(brake.getTangentialVel(),(lut.getKinematicParamsByTime(1.25+h).getProgression()
                -lut.getKinematicParamsByTime(1.25-h).getProgression())/(2*h),1e-8);
        assertEquals(0,lut.getKinematicParamsByTime(3).getTangentialVel(),0);
    }
    @Test
    public void interpolatesPositionAndKinematicsByElapsedTime() {
        MotionParameters start = new MotionParameters(0.0, 0.0, 0.0, 4.0, 0.0)
                .setTimeSeconds(0.0);
        MotionParameters end = new MotionParameters(0.0, 0.0, 2.0, 0.0, 1.0)
                .setTimeSeconds(0.5);
        start.setMotorPower(0.2);
        end.setMotorPower(0.8);
        FFLut lut = new FFLut(new MotionParameters[]{start, end});

        MotionParameters midpoint = lut.getFFParamsByTime(0.25);
        assertEquals(0.5, midpoint.getDistAlongCurve(), 1e-9);
        assertEquals(1.0, midpoint.getAngularVel(), 1e-9);
        assertEquals(2.0, midpoint.getAngularAccel(), 1e-9);
        assertEquals(0.25, midpoint.getTimeSeconds(), 1e-9);
        assertEquals(0.5, midpoint.getMotorPower(), 1e-9);
        assertEquals(0.5, lut.getFFParams(0.5).getMotorPower(), 1e-9);
        assertEquals(0.5, lut.getDurationSeconds(), 1e-9);
    }

    @Test
    public void timeQueriesClampToTrajectoryEndpoints() {
        MotionParameters start = new MotionParameters().setTimeSeconds(0.0);
        MotionParameters end = new MotionParameters(0.0, 0.0, 0.0, 0.0, 2.0)
                .setTimeSeconds(1.0);
        FFLut lut = new FFLut(new MotionParameters[]{start, end});

        assertEquals(0.0, lut.getFFParamsByTime(-1.0).getDistAlongCurve(), 1e-9);
        assertEquals(2.0, lut.getFFParamsByTime(5.0).getDistAlongCurve(), 1e-9);
    }
}
