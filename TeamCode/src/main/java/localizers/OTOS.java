package localizers;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import geometry.Vector;
import localizers.constants.OTOSConstants;
import geometry.Angle;
import geometry.Pose;
import util.DistUnit;

/**
 * Localizer for the SparkFun OTOS (Odometry Tracking and Orientation System) using the SparkFunOTOS
 * class.
 *
 * @author Dylan B. 18597 RoboClovers - Delta
 */
public class OTOS extends Localizer {
    private final SparkFunOTOS otos;

    public OTOS(HardwareMap hardwareMap, OTOSConstants constants) {
        this.otos = hardwareMap.get(SparkFunOTOS.class, constants.name);
        this.otos.setLinearUnit(DistanceUnit.INCH);
        this.otos.setAngularUnit(AngleUnit.RADIANS);
        this.otos.setOffset(toSparkfunPose2D(constants.offset));
        this.otos.setLinearScalar(constants.linearScalar);
        this.otos.setAngularScalar(constants.headingScalar);
        this.otos.calibrateImu();
        this.otos.resetTracking();
    }

    private SparkFunOTOS.Pose2D toSparkfunPose2D(Pose pose) {
        return new SparkFunOTOS.Pose2D(
                pose.getX().getIn(),
                pose.getY().getIn(),
                pose.getHeading().getRad()
        );
    }

    @Override
    public void update() {
        SparkFunOTOS.Pose2D pos = new SparkFunOTOS.Pose2D();
        SparkFunOTOS.Pose2D vel = new SparkFunOTOS.Pose2D();
        SparkFunOTOS.Pose2D acc = new SparkFunOTOS.Pose2D();
        otos.getPosVelAcc(pos, vel, acc);

        // NOTE: ADD ACCELERATION IF NEEDED
        currentPose = new Pose(Vector.of(pos.x, pos.y, DistUnit.IN), Angle.fromRad(pos.h));
        currentVelocity = new Pose(Vector.of(vel.x, vel.y, DistUnit.IN), Angle.fromRad(vel.h));
    }

    @Override
    public void setPose(Pose pose) { otos.setPosition(toSparkfunPose2D(pose)); }
}