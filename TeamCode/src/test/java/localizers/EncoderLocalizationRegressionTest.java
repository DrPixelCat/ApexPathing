package localizers;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import geometry.Angle;
import geometry.DistUnit;
import geometry.Pose;
import geometry.Vector;
import org.junit.Test;
import support.FakeHardware;
import support.FakeHardware.MotorState;

import static org.junit.Assert.*;

public class EncoderLocalizationRegressionTest {
    private final FakeHardware map = new FakeHardware();
    // The fake IMU accepts its parameters without invoking Android OpenGL orientation code.
    private final RevHubOrientationOnRobot orientation = null;

    @Test public void mecanumWheelsReportForwardAndLeftMotionOncePerUpdate() {
        MotorState fl = map.motor("fl"), fr = map.motor("fr"), bl = map.motor("bl"), br = map.motor("br");
        map.imu("imu");
        DriveEncoders.Constants c = new DriveEncoders.Constants();
        c.frontLeftName="fl"; c.frontRightName="fr"; c.backLeftName="bl"; c.backRightName="br";
        c.imuName="imu"; c.hubOrientation=orientation; c.ticksPerInch=100;
        DriveEncoders localizer = new DriveEncoders(c, map);
        fl.ticks=fr.ticks=bl.ticks=br.ticks=1000;
        localizer.update();
        assertPose(localizer, 10, 0, 0);
        fl.ticks-=300; fr.ticks+=300; bl.ticks+=300; br.ticks-=300;
        localizer.update();
        assertPose(localizer, 10, 3, 0);
        localizer.update();
        assertPose(localizer, 10, 3, 0);
    }

    @Test public void twoFrontEncodersReportFullForwardDistanceWithoutInventingStrafe() {
        MotorState fl=map.motor("fl"), fr=map.motor("fr"); map.imu("imu");
        DriveEncoders.Constants c=new DriveEncoders.Constants();
        c.frontLeftName="fl"; c.frontRightName="fr"; c.imuName="imu";
        c.hubOrientation=orientation; c.ticksPerInch=100;
        DriveEncoders localizer=new DriveEncoders(c,map);
        fl.ticks=fr.ticks=1000; localizer.update();
        assertPose(localizer,10,0,0);
    }

    @Test public void twoPodsTrackTranslationAndCompensateClockwiseRotation() {
        MotorState forward=map.motor("forward"), strafe=map.motor("strafe"); map.imu("imu");
        TwoWheel localizer = new TwoWheel(new TwoWheel.Constants()
                .setForwardPodName("forward").setStrafePodName("strafe").setIMUName("imu")
                .setHubOrientation(orientation).setTicksPerInch(100)
                .setOffsets(Vector.of(2,3,DistUnit.IN)), map);
        forward.ticks=1000; strafe.ticks=300; localizer.update();
        assertPose(localizer,10,3,0);
        map.yaw=-.5; forward.ticks-=100; strafe.ticks-=150;
        localizer.update();
        assertPose(localizer,10,3,-.5);
    }

    @Test public void threePodsTrackTranslationAndPureRotation() {
        MotorState left=map.motor("left"), right=map.motor("right"), strafe=map.motor("strafe");
        ThreeWheel localizer = new ThreeWheel(new ThreeWheel.Constants()
                .setForwardLeftPodName("left").setForwardRightPodName("right")
                .setStrafePodName("strafe").setTicksPerInch(100)
                .setOffsets(Vector.of(10,2,DistUnit.IN)), map);
        left.ticks=right.ticks=1000; strafe.ticks=300; localizer.update();
        assertPose(localizer,10,3,0);
        left.ticks+=500; right.ticks-=500; strafe.ticks+=200; localizer.update();
        assertPose(localizer,10,3,1);
        localizer.setPose(Pose.zero()); localizer.update();
        assertPose(localizer,0,0,0);
        assertEquals(0,left.resets+right.resets+strafe.resets);
    }

    @Test public void simultaneousTranslationAndRotationFollowTheMeasuredArc() {
        MotorState forward=map.motor("forward"); map.motor("strafe"); map.imu("imu");
        TwoWheel localizer=new TwoWheel(new TwoWheel.Constants()
                .setForwardPodName("forward").setStrafePodName("strafe").setIMUName("imu")
                .setHubOrientation(orientation).setTicksPerInch(100),map);
        forward.ticks=1500; map.yaw=Math.PI/2; localizer.update();
        assertPose(localizer,30/Math.PI,30/Math.PI,Math.PI/2);
    }

    @Test public void podResetKeepsTheDriveMotorRunningAndRebasesDistance() {
        MotorState state=map.motor("pod"); state.ticks=1000;
        OdometryPod pod=new OdometryPod(map,"pod",100);
        pod.update(); assertEquals(0,pod.getDeltaInches(),0);
        state.ticks=1200; pod.update(); assertEquals(2,pod.getInches(),0);
        pod.reset(); assertEquals(0,pod.getDeltaInches(),0); assertEquals(0,pod.getInches(),0);
        state.ticks=1300; pod.update(); assertEquals(1,pod.getDeltaInches(),0);
        assertEquals(0,state.resets);
    }

    @Test public void poseDerivedAngularVelocityCrossesTheHeadingWrapWithoutSpiking() throws Exception {
        class Localizer extends BaseLocalizer<TwoWheel.Constants> {
            Localizer() { super(new TwoWheel.Constants()); }
            @Override public void update() { calculate(UpdateType.BOTH); }
            @Override public void setPose(Pose p) { resetKinematicEstimate(p); }
            void sample(double heading) { pose=new Pose(Vector.zero(),Angle.fromRad(heading)); update(); }
        }
        Localizer localizer=new Localizer(); localizer.sample(Math.PI-.01);
        java.lang.reflect.Field time=BaseLocalizer.class.getDeclaredField("prevTimeNs"); time.setAccessible(true);
        time.setLong(localizer,System.nanoTime()-20_000_000L);
        localizer.sample(-Math.PI+.01);
        double omega=localizer.getRawVel().getHeading().getRad();
        assertTrue("Small positive rotation must not become a full clockwise revolution: "+omega,omega>0 && omega<2);
    }

    private static void assertPose(BaseLocalizer<?> localizer,double x,double y,double yaw) {
        assertEquals(x,localizer.getPose().getX().getIn(),1e-9);
        assertEquals(y,localizer.getPose().getY().getIn(),1e-9);
        assertEquals(0,Angle.wrap(localizer.getPose().getHeading().getRad()-yaw),1e-9);
    }
}
