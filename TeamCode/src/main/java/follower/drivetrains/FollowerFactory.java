package follower.drivetrains;

import com.qualcomm.robotcore.hardware.HardwareMap;
import config.ApexConstants;
import drivetrains.BaseDrivetrain;
import drivetrains.CoaxialSwerve;
import drivetrains.DualActuated;
import drivetrains.Mecanum;
import drivetrains.Omni;
import follower.Follower;
import java.util.Objects;
import java.util.function.LongSupplier;

/** Construction bridge used by {@link Follower#create}; builds each drivetrain exactly once. */
public final class FollowerFactory {
    private FollowerFactory() { }

    public static Follower create(ApexConstants constants, HardwareMap hardwareMap,
                                  boolean tuningMode, LongSupplier clock) {
        Objects.requireNonNull(clock, "Monotonic clock");
        BaseDrivetrain<?> drivetrain = constants.drivetrainConstants().build(hardwareMap);
        if (drivetrain instanceof DualActuated) {
            return new DualActuatedFollower(constants, hardwareMap, tuningMode, clock, drivetrain);
        }
        if (drivetrain instanceof Mecanum) {
            return new MecanumFollower(constants, hardwareMap, tuningMode, clock, drivetrain);
        }
        if (drivetrain instanceof CoaxialSwerve) {
            return new SwerveFollower(constants, hardwareMap, tuningMode, clock, drivetrain);
        }
        if (drivetrain instanceof Omni) {
            return new OmniFollower(constants, hardwareMap, tuningMode, clock, drivetrain);
        }
        return drivetrain.isHolonomic()
                ? new HolonomicFollower(constants, hardwareMap, tuningMode, clock, drivetrain)
                : new TankFollower(constants, hardwareMap, tuningMode, clock, drivetrain);
    }
}
