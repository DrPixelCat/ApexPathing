package follower.drivetrains;


import com.qualcomm.robotcore.hardware.HardwareMap;
import config.ApexConstants;
import drivetrains.BaseDrivetrain;
import drivetrains.CoaxialSwerve;
import java.util.function.LongSupplier;

/** Coaxial swerve follower with isotropic power allocation and module steering. */
public final class SwerveFollower extends HolonomicFollower {
    public SwerveFollower(ApexConstants constants, HardwareMap hardwareMap) {
        this(constants, hardwareMap, false);
    }

    public SwerveFollower(ApexConstants constants, HardwareMap hardwareMap, boolean tuningMode) {
        this(constants, hardwareMap, tuningMode, System::nanoTime);
    }

    public SwerveFollower(ApexConstants constants, HardwareMap hardwareMap, boolean tuningMode,
                          LongSupplier monotonicClock) {
        this(constants, hardwareMap, tuningMode, monotonicClock,
                constants.drivetrainConstants().build(hardwareMap));
    }

    SwerveFollower(ApexConstants constants, HardwareMap hardwareMap, boolean tuningMode,
                   LongSupplier monotonicClock, BaseDrivetrain<?> drivetrain) {
        super(constants, hardwareMap, tuningMode, monotonicClock, requireCompatible(drivetrain));
    }

    private static BaseDrivetrain<?> requireCompatible(BaseDrivetrain<?> drivetrain) {
        if (!(drivetrain instanceof CoaxialSwerve)) {
            throw new IllegalArgumentException("SwerveFollower requires a coaxial swerve drivetrain");
        }
        return drivetrain;
    }
}
