package localizers;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import geometry.Angle;
import geometry.Dist;
import geometry.Pose;
import geometry.Vector;
import util.DistUnit;

/**
 * This is the localizer for 2 dead wheel odometry featuring 1 parallel and 1 perpendicular wheel
 *
 * @author Topher F. - 23571 alumni
 */
public class TwoWheel extends BaseLocalizer<TwoWheel.Constants> {
    private final OdometryPod strafePod;
    private final OdometryPod forwardPod;
    private final IMU imu;

    /**
     * This variable is when we call setPose(), the yaw can be properly offset back, it's just
     * the yaw the bot
     * was at prior to setPose() being called
     */
    private double correction = 0.0;

    @Override
    public void update() {
        strafePod.update();
        forwardPod.update();

        double oldYaw = pose.getHeading(util.AngleUnit.RAD);
        double currentYaw =
                Angle.normalize(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - correction);
        double deltaYaw = Angle.normalize(currentYaw - oldYaw);
        double avgYaw = oldYaw + deltaYaw / 2.0;
        double deltaX = forwardPod.getDeltaInches() - config.DPar.get(DistUnit.IN) * deltaYaw;
        double deltaY = strafePod.getDeltaInches() - config.DPerp.get(DistUnit.IN) * deltaYaw;
        pose = new Pose(
                pose.getVec().plus(
                        new Vector(
                                Dist.fromIn(deltaX * Math.cos(avgYaw) - (deltaY * Math.sin(avgYaw))),
                                Dist.fromIn(deltaX * Math.sin(avgYaw) + deltaY * Math.cos(avgYaw))
                        )),

                Angle.fromRad(currentYaw)
        );
        calculate(UpdateType.BOTH);
    }


    @Override
    public void setPose(Pose newPose) {
        correction =
                imu.getRobotYawPitchRollAngles().getYaw(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS) - newPose.getHeading(util.AngleUnit.RAD);
        pose = newPose;
        strafePod.resetEncoder();
        forwardPod.resetEncoder();
    }

    public static class Constants extends BaseLocalizerConstants<Constants> {
        public String forwardPodName = "forwardPodName";
        public String strafePodName = "strafePodName";
        public String imuName = "imu";

        /**
         * DPar is the distance from the center of the bot to the parallel dead wheel
         */
        public Dist DPar = Dist.fromIn(1.0);

        /**
         * DPerp is the distance from the center of the bot to the perpendicular dead wheel
         */
        public Dist DPerp = Dist.fromIn(1.0);

        // Default to facing logo facing forward and USB facing up
        public RevHubOrientationOnRobot hubOrientationOnRobot =
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.FORWARD
                        , RevHubOrientationOnRobot.UsbFacingDirection.UP);

        // Default 1 tick on the encoder to 1.0 inches (very wrong, will let user know if
        // something is up)
        public double inchConversion = 1.0;

        @Override
        public BaseLocalizer<?> build(HardwareMap hardwareMap) {
            return new TwoWheel(this, hardwareMap);
        }
    }

    public void resetYaw() {
        imu.resetYaw();
    }

    public TwoWheel(Constants config, HardwareMap hardwareMap) {
        super(config);

        strafePod = new OdometryPod(hardwareMap, config.strafePodName, config.inchConversion);
        forwardPod = new OdometryPod(hardwareMap, config.forwardPodName, config.inchConversion);
        imu = hardwareMap.get(IMU.class, config.imuName);
        imu.initialize(new IMU.Parameters(config.hubOrientationOnRobot));
    }
}
