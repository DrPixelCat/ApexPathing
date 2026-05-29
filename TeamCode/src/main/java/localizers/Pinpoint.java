package localizers;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import localizers.constants.PinpointConstants;
import geometry.Vector;
import geometry.Angle;
import geometry.Pose;
import util.DistUnit;

/**
 * Localizer for the goBILDA Pinpoint Odometry Computer using the GoBildaPinpointDriver class.
 *
 * @author Sohum Arora 22985
 * @author Dylan B. 18597 RoboClovers - Delta
 */
public class Pinpoint extends Localizer {
    private final GoBildaPinpointDriver pinpoint;

    public Pinpoint(HardwareMap hardwareMap, PinpointConstants constants) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, constants.name);
        pinpoint.setOffsets(constants.xOffset, constants.yOffset, constants.distanceUnit); // Offsets
        if (constants.customEncoderResolution != 0) { // Encoder resolution
            pinpoint.setEncoderResolution(constants.customEncoderResolution, constants.distanceUnit);
        } else {
            pinpoint.setEncoderResolution(constants.encoderResolution);
        }
        pinpoint.setEncoderDirections(constants.xPodDirection, constants.yPodDirection); // Pod directions
        if (constants.yawScalar != 0) { // Yaw scalar
            pinpoint.setYawScalar(constants.yawScalar);
        }
        pinpoint.resetPosAndIMU(); // Reset IMU
    }

    @Override
    public void update() {
        pinpoint.update();
        currentPose = new Pose(
                Vector.of(
                        pinpoint.getPosX(DistanceUnit.INCH), pinpoint.getPosY(DistanceUnit.INCH),
                        DistUnit.IN
                ), Angle.fromRad(pinpoint.getHeading(AngleUnit.RADIANS))
        );
        currentVelocity = new Pose(
                Vector.of(
                        pinpoint.getVelX(DistanceUnit.INCH), pinpoint.getVelY(DistanceUnit.INCH),
                        DistUnit.IN
                ), Angle.fromRad(pinpoint.getHeading(AngleUnit.RADIANS))
        );
    }

    @Override
    public void setPose(Pose pose) {
        pinpoint.setPosition(new Pose2D(
                DistanceUnit.INCH, pose.getX().getIn(), pose.getY().getIn(),
                AngleUnit.RADIANS, pose.getHeading().getRad()
        ));
    }
}