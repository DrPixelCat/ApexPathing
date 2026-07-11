package localizers;

import com.qualcomm.robotcore.hardware.HardwareMap;

import geometry.Angle;
import geometry.Dist;
import geometry.Pose;
import geometry.Vector;
import util.AngleUnit;
import util.DistUnit;

public class ThreeWheel extends BaseLocalizer<ThreeWheel.Constants> {
    private final OdometryPod strafePod;
    private final OdometryPod forwardPodLeft;
    private final OdometryPod forwardPodRight;

    @Override
    public void update() {
        strafePod.update();
        forwardPodLeft.update();
        forwardPodRight.update();

        double oldYaw = pose.getHeading(AngleUnit.RAD);

        double deltaYaw =
                (forwardPodLeft.getDeltaInches() - forwardPodRight.getDeltaInches()) / config.DPar.get(DistUnit.IN);
        double deltaX = (forwardPodLeft.getDeltaInches() + forwardPodRight.getDeltaInches()) / 2.0;
        double deltaY = strafePod.getDeltaInches() - deltaYaw * config.DPerp.get(DistUnit.IN);

        double yaw = Angle.normalize(oldYaw + deltaYaw);
        double avgYaw = oldYaw + deltaYaw / 2.0;

        pose = new Pose(
                pose.getVec().plus(
                        new Vector(
                                Dist.fromIn(deltaX * Math.cos(avgYaw) - deltaY * Math.sin(avgYaw)),
                                Dist.fromIn(deltaX * Math.sin(avgYaw) + deltaY * Math.cos(avgYaw))
                        )),
                Angle.fromRad(yaw)
        );
        calculate(UpdateType.BOTH);
    }

    @Override
    public void setPose(Pose newPose) {
        pose = newPose;
        strafePod.resetEncoder();
        forwardPodRight.resetEncoder();
        forwardPodLeft.resetEncoder();
    }

    public static class Constants extends BaseLocalizerConstants<Constants> {
        public String forwardPodLeftName = "forwardPodLeftName";
        public String forwardPodRightName = "forwardPodRightName";
        public String strafePodName = "strafePodName";

        /**
         * DPar is the distance from the parallel dead wheels to each other (aka track width)
         */
        public Dist DPar = Dist.fromIn(1.0);

        /**
         * DPerp is the distance from the center of the bot to the perpendicular dead wheel
         */
        public Dist DPerp = Dist.fromIn(1.0);

        // Default to facing logo facing forward and USB facing up
        // Default 1 tick on the encoder to 1.0 inches (very wrong, will let user know if
        // something is up)
        public double inchConversion = 1.0;

        @Override
        public BaseLocalizer<?> build(HardwareMap hardwareMap) {
            return new ThreeWheel(this, hardwareMap);
        }
    }

    public ThreeWheel(Constants config, HardwareMap hardwareMap) {
        super(config);

        strafePod = new OdometryPod(hardwareMap, config.strafePodName, config.inchConversion);
        forwardPodLeft = new OdometryPod(hardwareMap, config.forwardPodLeftName,
                config.inchConversion);
        forwardPodRight = new OdometryPod(hardwareMap, config.forwardPodRightName,
                config.inchConversion);
    }
}
