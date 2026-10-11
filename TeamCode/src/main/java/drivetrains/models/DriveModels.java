package drivetrains.models;

import drivetrains.BaseDrivetrain;
import follower.FollowerConstants;
import paths.movements.Path;
import drivetrains.models.HolonomicDriveModel.Metric;

/** Chooses physical models without reading or changing actuator mode. */
public final class DriveModels {
    private DriveModels() { }
    public static HolonomicDriveModel mecanum(FollowerConstants constants) { return new HolonomicDriveModel(constants,Metric.MECANUM); }
    public static HolonomicDriveModel swerve(FollowerConstants constants) { return new HolonomicDriveModel(constants,Metric.ISOTROPIC); }
    public static HolonomicDriveModel omni(FollowerConstants constants) { return new HolonomicDriveModel(constants,Metric.OMNI); }
    public static DriveModel tank(FollowerConstants constants) { return new TankDriveModel(constants); }
    public static DriveModel forPath(FollowerConstants constants, Path.PathType pathType) {
        return forType(constants.drivetrainType, constants, pathType);
    }
    public static DriveModel forDrivetrain(BaseDrivetrain<?> drivetrain, FollowerConstants constants, Path.PathType pathType) {
        DriveModel model = java.util.Objects.requireNonNull(drivetrain.createDriveModel(constants,pathType),"Drive model");
        if (model.isHolonomic() != (pathType == Path.PathType.HOLONOMIC))
            throw new IllegalArgumentException("Drivetrain model does not match requested path mode");
        return model;
    }
    /** Default implementation for BaseDrivetrain.createDriveModel; bypasses the override to avoid recursion. */
    public static DriveModel defaultForDrivetrain(BaseDrivetrain<?> drivetrain, FollowerConstants constants, Path.PathType pathType) {
        return forType(drivetrain.getDrivetrainType(),constants,pathType);
    }
    private static DriveModel forType(BaseDrivetrain.DrivetrainType type, FollowerConstants constants, Path.PathType pathType) {
        if (pathType == Path.PathType.TANK) return new TankDriveModel(constants);
        if (type == BaseDrivetrain.DrivetrainType.COAXIAL_SWERVE) return new HolonomicDriveModel(constants, Metric.ISOTROPIC);
        if (type == BaseDrivetrain.DrivetrainType.OMNI) return new HolonomicDriveModel(constants, Metric.OMNI);
        // Dual-actuated motors use body-axis calibration; the mecanum-only inverse wheel model
        // cannot be applied by their actuator path.
        return new HolonomicDriveModel(constants,Metric.MECANUM,null,null,type != BaseDrivetrain.DrivetrainType.DUAL_ACTUATED);
    }
}
