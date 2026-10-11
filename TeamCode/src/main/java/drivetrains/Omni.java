package drivetrains;

import com.qualcomm.robotcore.hardware.HardwareMap;
import geometry.Angle;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Objects;
import java.util.Set;

/**
 * Three or more tangent-driving omni wheels uniformly spaced around a circle centered on the robot.
 * Wheel positions are measured counterclockwise from forward (+x), with left as +y.
 * Positive motor power must propel each wheel in its counterclockwise tangent direction.
 */
public class Omni extends BaseDrivetrain<Omni.Constants> {
    private final drivetrains.models.WheelProjectionModel projections;
    private final double[] wheelPowers;

    public Omni(Constants constants, HardwareMap hardwareMap) {
        super(constants, hardwareMap, DrivetrainType.OMNI, constants.wheels.toArray(new Motor[0]));
        int count = getMotorCount();
        double[] forwardProjection = new double[count], strafeProjection = new double[count];
        wheelPowers = new double[count];
        double first = Angle.normalize(constants.firstWheelAngle.getRad());
        for (int i = 0; i < count; i++) {
            double angle = first + i * 2 * Math.PI / count;
            forwardProjection[i] = -Math.sin(angle);
            strafeProjection[i] = Math.cos(angle);
        }
        projections = new drivetrains.models.WheelProjectionModel(forwardProjection,strafeProjection,1);
    }

    @Override public void moveWithVectors(double x, double y, double turn) {
        projections.mix(x,y,turn,wheelPowers);
        setMotorPowers(wheelPowers);
    }

    /** Peak wheel power required by translation alone, before output limiting. */
    public double translationDemand(double x, double y) {
        return projections.demand(x,y,0);
    }
    public drivetrains.models.WheelProjectionModel getWheelProjectionModel() { return projections; }
    public double[] getForwardProjections() { return projections.getForwardProjections(); }
    public double[] getStrafeProjections() { return projections.getStrafeProjections(); }
    @Override public drivetrains.models.DriveModel createDriveModel(follower.FollowerConstants settings,
                                                                    paths.movements.Path.PathType mode) {
        return mode == paths.movements.Path.PathType.HOLONOMIC
                ? new drivetrains.models.HolonomicDriveModel(settings, projections)
                    .withDirectionalKinematics(this::createDirectionalKinematics)
                : super.createDriveModel(settings, mode);
    }
    @Override protected drivetrains.models.DirectionalKinematics createDirectionalKinematics(follower.FollowerConstants settings) {
        return drivetrains.models.DirectionalKinematics.projected(settings,projections);
    }

    /** Direct wheel output in registration order, with the same coupled power limiting as driving. */
    public void setWheelPowers(double... powers) { setMotorPowers(powers); }

    /**
     * Register at least three distinct motors in counterclockwise order. Wheel count is derived
     * from that list. Motor directions can be reversed with Motor.reverse() to match the tangent.
     */
    public static class Constants extends BaseDrivetrainConstants<Constants> {
        private final List<Motor> wheels = new ArrayList<>();
        private Angle firstWheelAngle = Angle.zero();

        public Constants addWheel(Motor motor) {
            wheels.add(Objects.requireNonNull(motor, "Wheel motor"));
            return this;
        }

        /** Radial position of the first wheel, measured counterclockwise from forward; default 0. */
        public Constants setFirstWheelAngle(Angle angle) {
            if (angle == null || angle.isNotFinite()) {
                throw new IllegalArgumentException("First wheel angle must be finite");
            }
            firstWheelAngle = angle;
            return this;
        }

        public int getWheelCount() { return wheels.size(); }

        @Override public void validate() {
            super.validate();
            if (wheels.size() < 3) { throw new IllegalArgumentException("An omni drive requires at least 3 wheels"); }
            if (flMotorConfig != null || frMotorConfig != null || blMotorConfig != null || brMotorConfig != null) {
                throw new IllegalArgumentException("Configure omni motors with addWheel(), not fixed motor slots");
            }
            Set<String> names = new HashSet<>();
            for (Motor motor : wheels) {
                String name = motor.getName();
                if (name == null || name.trim().isEmpty() || !names.add(name)) {
                    throw new IllegalArgumentException("Omni wheel motors must have distinct, nonempty hardware names");
                }
            }
        }

        @Override public Omni build(HardwareMap hardwareMap) { return new Omni(this, hardwareMap); }
    }
}
