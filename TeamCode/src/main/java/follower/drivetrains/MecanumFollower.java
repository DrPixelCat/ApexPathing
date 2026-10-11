package follower.drivetrains;


import com.qualcomm.robotcore.hardware.HardwareMap;
import config.ApexConstants;
import drivetrains.BaseDrivetrain;
import drivetrains.Mecanum;
import java.util.function.LongSupplier;

/** Mecanum follower with directional power allocation and calibrated wheel-model support. */
public final class MecanumFollower extends HolonomicFollower {
    public MecanumFollower(ApexConstants constants, HardwareMap hardwareMap) {
        this(constants, hardwareMap, false);
    }

    public MecanumFollower(ApexConstants constants, HardwareMap hardwareMap, boolean tuningMode) {
        this(constants, hardwareMap, tuningMode, System::nanoTime);
    }

    public MecanumFollower(ApexConstants constants, HardwareMap hardwareMap, boolean tuningMode,
                           LongSupplier monotonicClock) {
        this(constants, hardwareMap, tuningMode, monotonicClock,
                constants.drivetrainConstants().build(hardwareMap));
    }

    MecanumFollower(ApexConstants constants, HardwareMap hardwareMap, boolean tuningMode,
                    LongSupplier monotonicClock, BaseDrivetrain<?> drivetrain) {
        super(constants, hardwareMap, tuningMode, monotonicClock, requireCompatible(drivetrain));
    }

    private static BaseDrivetrain<?> requireCompatible(BaseDrivetrain<?> drivetrain) {
        if (!(drivetrain instanceof Mecanum)) {
            throw new IllegalArgumentException("MecanumFollower requires a mecanum drivetrain");
        }
        return drivetrain;
    }
}
