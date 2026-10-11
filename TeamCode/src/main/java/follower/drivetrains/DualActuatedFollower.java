package follower.drivetrains;

import follower.Follower;
import follower.FollowerConstants;
import follower.internal.FollowerRuntime.PathController;

import com.qualcomm.robotcore.hardware.HardwareMap;
import config.ApexConstants;
import drivetrains.BaseDrivetrain;
import drivetrains.DualActuated;
import feedforward.CenteredBrakingFeedforward;
import geometry.Pose;
import java.util.function.LongSupplier;
import paths.movements.FollowerMovement;
import paths.movements.Path;
import paths.movements.Turn;

/** Switches tracking algorithms on one dual-actuated drivetrain and shared movement lifecycle. */
public final class DualActuatedFollower extends Follower {
    private final DualActuated drivetrain;
    private final PathController holonomicPathFollower;
    private final PathController tankPathFollower;

    public DualActuatedFollower(ApexConstants constants, HardwareMap hardwareMap) {
        this(constants, hardwareMap, false);
    }

    public DualActuatedFollower(ApexConstants constants, HardwareMap hardwareMap, boolean tuningMode) {
        this(constants, hardwareMap, tuningMode, System::nanoTime);
    }

    public DualActuatedFollower(ApexConstants constants, HardwareMap hardwareMap, boolean tuningMode,
                                LongSupplier monotonicClock) {
        this(constants, hardwareMap, tuningMode, monotonicClock,
                constants.drivetrainConstants().build(hardwareMap));
    }

    DualActuatedFollower(ApexConstants constants, HardwareMap hardwareMap, boolean tuningMode,
                         LongSupplier monotonicClock, BaseDrivetrain<?> drivetrain) {
        super(constants, hardwareMap, tuningMode, monotonicClock, requireCompatible(drivetrain));
        this.drivetrain = (DualActuated) drivetrain;
        holonomicPathFollower = createHolonomicPathFollower();
        tankPathFollower = createTankPathFollower();
    }

    private static BaseDrivetrain<?> requireCompatible(BaseDrivetrain<?> drivetrain) {
        if (!(drivetrain instanceof DualActuated)) {
            throw new IllegalArgumentException("DualActuatedFollower requires a dual-actuated drivetrain");
        }
        return drivetrain;
    }

    @Override protected MovementOutcome updatePath(Path path) {
        return executePath(path, drivetrain.isHolonomic() ? holonomicPathFollower : tankPathFollower);
    }

    @Override protected void holdPose(Pose target) { holdPose(target, drivetrain.isHolonomic()); }

    @Override protected void resetPathControllers() { holonomicPathFollower.reset(); }

    @Override public void setBrakingFeedforward(CenteredBrakingFeedforward model) {
        holonomicPathFollower.setBrakingFeedforward(model);
    }

    @Override protected boolean isDriveTransitioning() { return drivetrain.isTransitioning(); }

    @Override protected void prepareMovement(FollowerMovement movement) {
        FollowerConstants.Profile intended = movement instanceof Path
                ? ((Path) movement).getPathType() == Path.PathType.TANK
                        ? FollowerConstants.Profile.TANK : FollowerConstants.Profile.HOLONOMIC
                : movement instanceof Turn && ((Turn) movement).getDriveProfile() != null
                        ? ((Turn) movement).getDriveProfile() : currentProfile();
        // Validate saved settings before actuating the hardware.
        getConstants().forProfile(intended);
        if (intended == FollowerConstants.Profile.TANK) {
            drivetrain.activateTractionState();
        } else {
            drivetrain.activateHolonomicState();
        }
        synchronizeDriveProfile();
    }

    @Override protected void synchronizeDriveProfile() {
        FollowerConstants.Profile profile = currentProfile();
        FollowerMovement movement = getCurrentMovement();
        if (movement instanceof Path
                && ((((Path) movement).getPathType() == Path.PathType.TANK)
                        != (profile == FollowerConstants.Profile.TANK))) {
            stop();
            throw new IllegalStateException("Drivetrain mode changed during an active path");
        }
        selectDriveProfile(profile);
    }

    private FollowerConstants.Profile currentProfile() {
        return drivetrain.isHolonomic()
                ? FollowerConstants.Profile.HOLONOMIC : FollowerConstants.Profile.TANK;
    }
}
