package follower.drivetrains;

import follower.Follower;
import follower.internal.FollowerRuntime.PathController;

import com.qualcomm.robotcore.hardware.HardwareMap;
import config.ApexConstants;
import drivetrains.BaseDrivetrain;
import drivetrains.DualActuated;
import feedforward.CenteredBrakingFeedforward;
import paths.geometry.Pose;
import java.util.function.LongSupplier;
import paths.movements.Path;

/**
 * Shared holonomic implementation for {@link MecanumFollower}, {@link SwerveFollower},
 * and {@link OmniFollower}.
 * Also serves custom fixed holonomic drives through {@link Follower#create}.
 */
public class HolonomicFollower extends Follower {
    private final PathController pathFollower;

    public HolonomicFollower(ApexConstants constants, HardwareMap hardwareMap) {
        this(constants, hardwareMap, false);
    }

    public HolonomicFollower(ApexConstants constants, HardwareMap hardwareMap, boolean tuningMode) {
        this(constants, hardwareMap, tuningMode, System::nanoTime);
    }

    public HolonomicFollower(ApexConstants constants, HardwareMap hardwareMap, boolean tuningMode,
                             LongSupplier monotonicClock) {
        this(constants, hardwareMap, tuningMode, monotonicClock,
                constants.drivetrainConstants().build(hardwareMap));
    }

    protected HolonomicFollower(ApexConstants constants, HardwareMap hardwareMap, boolean tuningMode,
                                LongSupplier monotonicClock, BaseDrivetrain<?> drivetrain) {
        super(constants, hardwareMap, tuningMode, monotonicClock, requireCompatible(drivetrain));
        pathFollower = createHolonomicPathFollower();
    }

    private static BaseDrivetrain<?> requireCompatible(BaseDrivetrain<?> drivetrain) {
        if (drivetrain instanceof DualActuated || !drivetrain.isHolonomic()) {
            throw new IllegalArgumentException("HolonomicFollower requires a fixed holonomic drivetrain");
        }
        return drivetrain;
    }

    @Override protected MovementOutcome updatePath(Path path) {
        return executePath(path, pathFollower);
    }

    @Override protected void holdPose(Pose target) { holdPose(target, true); }

    @Override protected void resetPathControllers() { pathFollower.reset(); }

    @Override public void setBrakingFeedforward(CenteredBrakingFeedforward model) {
        pathFollower.setBrakingFeedforward(model);
    }
}
