package localizers;

import com.qualcomm.robotcore.hardware.HardwareMap;

import geometry.Angle;
import geometry.Pose;
import geometry.Vector;
import geometry.DistUnit;

/**
 * A localizer that uses 3 dead wheel odometry pods (2 parallel and 1 perpendicular)
 *
 * @author Topher F. - 23571 alum
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public class ThreeWheel extends BaseLocalizer<ThreeWheel.Constants> {
    private final OdometryPod forwardLeftPod, forwardRightPod, strafePod;
    private final double forwardOffsetIn, strafeOffsetIn;

    public ThreeWheel(Constants constants, HardwareMap hardwareMap) {
        super(constants);

        this.strafePod = new OdometryPod(
                hardwareMap, constants.strafePodName, constants.ticksPerInch, constants.strafePodReversed
        );
        this.forwardLeftPod = new OdometryPod(
                hardwareMap, constants.forwardLeftPodName, constants.ticksPerInch, constants.forwardLeftPodReversed
        );
        this.forwardRightPod = new OdometryPod(
                hardwareMap, constants.forwardRightPodName, constants.ticksPerInch, constants.forwardRightPodReversed
        );

        this.forwardOffsetIn = constants.offsets.getX(DistUnit.IN);
        this.strafeOffsetIn = constants.offsets.getY(DistUnit.IN);
    }

    @Override
    public void update() {
        forwardLeftPod.update();
        forwardRightPod.update();
        strafePod.update();
        double oldYaw = pose.getHeading(geometry.AngleUnit.RAD);
        double deltaYaw = (forwardLeftPod.getDeltaInches() - forwardRightPod.getDeltaInches()) /
                forwardOffsetIn;
        double yaw = Angle.normalize(oldYaw + deltaYaw);
        double forward = (forwardLeftPod.getDeltaInches() + forwardRightPod.getDeltaInches()) / 2.0;
        double strafe = strafePod.getDeltaInches() - deltaYaw * strafeOffsetIn;
        pose = integrateArc(pose, forward, strafe, deltaYaw, yaw);

        calculate(UpdateType.BOTH);
    }


    @Override
    public void setPose(Pose newPose) {
        pose = newPose;

        forwardLeftPod.reset();
        forwardRightPod.reset();
        strafePod.reset();
        resetKinematicEstimate(newPose);
    }

    @Override public PodCalibrationReading[] getPodCalibrationReadings() {
        return new PodCalibrationReading[]{
                new PodCalibrationReading("parallel_separation", forwardLeftPod.getInches() -
                        forwardRightPod.getInches(), forwardOffsetIn, 1),
                new PodCalibrationReading("strafe_offset", strafePod.getInches(), strafeOffsetIn, 1)};
    }

    public int[] getPodTicks() { return new int[]{forwardLeftPod.getTicks(), forwardRightPod.getTicks(), strafePod.getTicks()}; }

    public static class Constants implements BaseLocalizerConstants<Constants> {
        public String forwardLeftPodName;
        public String forwardRightPodName;
        public String strafePodName;
        public Vector offsets = Vector.zero();
        public double ticksPerInch = 1.0;
        public boolean forwardLeftPodReversed, forwardRightPodReversed, strafePodReversed;

        public Constants setEncoderDirections(boolean left, boolean right, boolean strafe) {
            forwardLeftPodReversed = left; forwardRightPodReversed = right; strafePodReversed = strafe; return this;
        }

        @Override
        public BaseLocalizer<?> build(HardwareMap hardwareMap) {
            if (this.forwardLeftPodName == null || this.forwardRightPodName == null ||
                    this.strafePodName == null) {
                throw new IllegalArgumentException(
                        "You must call setForwardLeftPodName, setForwardRightPodName, and " +
                                "setStrafePodName to set the names of the motor ports that hold " +
                                "the odometry pod encoders."
                );
            }

            return new ThreeWheel(this, hardwareMap);
        }

        /** Sets the name of the motor that the forward left pod encoder is attached to. */
        public ThreeWheel.Constants setForwardLeftPodName(String name) {
            this.forwardLeftPodName = name;
            return this;
        }

        /** Sets the name of the motor that the forward right pod encoder is attached to. */
        public ThreeWheel.Constants setForwardRightPodName(String name) {
            this.forwardRightPodName = name;
            return this;
        }

        /** Sets the name of the motor that the strafe pod encoder is attached to. */
        public ThreeWheel.Constants setStrafePodName(String name) {
            this.strafePodName = name;
            return this;
        }

        /**
         * Sets the offsets of the odometry pods where x is the forward pods offset from the center
         * of the robot and y is the distance between the strafe pods
         */
        public ThreeWheel.Constants setOffsets(Vector offsets) {
            this.offsets = offsets;
            return this;
        }

        /** Sets the number of encoder ticks per inch of travel. */
        public ThreeWheel.Constants setTicksPerInch(double ticksPerInch) {
            this.ticksPerInch = ticksPerInch;
            return this;
        }
    }
}
