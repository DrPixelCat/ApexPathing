package follower.drivetrains;

import follower.Follower;
import follower.internal.FollowerRuntime.PathController;

import com.qualcomm.robotcore.hardware.HardwareMap;
import config.ApexConstants;
import drivetrains.BaseDrivetrain;
import drivetrains.DualActuated;
import geometry.Pose;
import java.util.function.LongSupplier;
import paths.movements.Path;

/** Complete follower for a fixed differential drivetrain, using Ramsete path tracking. */
public final class TankFollower extends Follower {
    private final PathController pathFollower;

    public TankFollower(ApexConstants constants, HardwareMap hardwareMap) {
        this(constants, hardwareMap, false);
    }

    public TankFollower(ApexConstants constants, HardwareMap hardwareMap, boolean tuningMode) {
        this(constants, hardwareMap, tuningMode, System::nanoTime);
    }

    public TankFollower(ApexConstants constants, HardwareMap hardwareMap, boolean tuningMode,
                        LongSupplier monotonicClock) {
        this(constants, hardwareMap, tuningMode, monotonicClock,
                constants.drivetrainConstants().build(hardwareMap));
    }

    TankFollower(ApexConstants constants, HardwareMap hardwareMap, boolean tuningMode,
                 LongSupplier monotonicClock, BaseDrivetrain<?> drivetrain) {
        super(constants, hardwareMap, tuningMode, monotonicClock, requireCompatible(drivetrain));
        pathFollower = createTankPathFollower();
    }

    private static BaseDrivetrain<?> requireCompatible(BaseDrivetrain<?> drivetrain) {
        if (drivetrain instanceof DualActuated || drivetrain.isHolonomic()) {
            throw new IllegalArgumentException("TankFollower requires a fixed differential drivetrain");
        }
        return drivetrain;
    }

    @Override protected MovementOutcome updatePath(Path path) {
        return executePath(path, pathFollower);
    }

    @Override protected void holdPose(Pose target) { holdPose(target, false); }

    @Override protected void resetPathControllers() {
        // The existing tank algorithm has no additional reset behavior.
    }
}
