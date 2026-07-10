package localizers;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import geometry.Angle;
import geometry.Dist;
import geometry.Pose;
import geometry.Vector;

/**
 * This is the localizer for 4 drive encoders and an IMU
 *
 * @author Topher F. - 23571 alumni
 */
public class DriveEncoders extends BaseLocalizer<DriveEncoders.Config> {
    private final OdometryPod frontLeft, frontRight, backLeft, backRight;
    private final IMU imu;

    private double correction = 0.0;

    public DriveEncoders(DriveEncoders.Config config, HardwareMap hardwareMap) {
        super(config);

        frontLeft = new OdometryPod(hardwareMap, config.frontLeftName, config.inchConversion);
        frontRight = new OdometryPod(hardwareMap, config.frontRightName, config.inchConversion);
        backLeft = new OdometryPod(hardwareMap, config.backLeftName, config.inchConversion);
        backRight = new OdometryPod(hardwareMap, config.backRightName, config.inchConversion);

        imu = hardwareMap.get(IMU.class, config.imuName);
        imu.initialize(new IMU.Parameters(config.hubOrientationOnRobot));
    }

    @Override
    public void update() {
        double deltaY =
                (frontLeft.getDeltaInches() + frontRight.getDeltaInches() + backLeft.getDeltaInches() + backRight.getDeltaInches()) / 4.0;
        double deltaX =
                (-frontLeft.getDeltaInches() + frontRight.getDeltaInches() + backLeft.getDeltaInches() - backRight.getDeltaInches()) / 4.0;

        double oldYaw = pose.getHeading(util.AngleUnit.RAD);
        double currentYaw =
                Angle.normalize(imu.getRobotYawPitchRollAngles().getYaw() - correction);
        double deltaYaw = Angle.normalize(currentYaw - oldYaw);
        double avgYaw = oldYaw + deltaYaw / 2.0;

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
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - newPose.getHeading(util.AngleUnit.RAD);
        pose = newPose;
        backLeft.resetEncoder();
        frontLeft.resetEncoder();
        backRight.resetEncoder();
        frontRight.resetEncoder();
    }

    public void resetYaw() {
        imu.resetYaw();
    }

    public static class Config extends BaseLocalizerConfig<DriveEncoders.Config> {
        public String frontLeftName = "fLName";
        public String frontRightName = "fRName";
        public String backLeftName = "bLName";
        public String backRightName = "bRName";

        public String imuName = "imu";

        public double inchConversion = 1.0;

        public RevHubOrientationOnRobot hubOrientationOnRobot =
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.FORWARD
                        , RevHubOrientationOnRobot.UsbFacingDirection.UP);

        @Override
        public BaseLocalizer<?> build(HardwareMap hardwareMap) {
            return new DriveEncoders(this, hardwareMap);
        }
    }
}
