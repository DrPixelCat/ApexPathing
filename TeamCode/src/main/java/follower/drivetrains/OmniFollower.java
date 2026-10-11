package follower.drivetrains;

import com.qualcomm.robotcore.hardware.HardwareMap;
import config.ApexConstants;
import drivetrains.BaseDrivetrain;
import drivetrains.Omni;
import java.util.function.LongSupplier;

/** Holonomic follower for a uniform circular omni drive with any wheel count of at least three. */
public final class OmniFollower extends HolonomicFollower {
    public OmniFollower(ApexConstants constants, HardwareMap hardwareMap) {
        this(constants, hardwareMap, false);
    }

    public OmniFollower(ApexConstants constants, HardwareMap hardwareMap, boolean tuningMode) {
        this(constants, hardwareMap, tuningMode, System::nanoTime);
    }

    public OmniFollower(ApexConstants constants, HardwareMap hardwareMap, boolean tuningMode,
                        LongSupplier monotonicClock) {
        this(constants, hardwareMap, tuningMode, monotonicClock,
                constants.drivetrainConstants().build(hardwareMap));
    }

    OmniFollower(ApexConstants constants, HardwareMap hardwareMap, boolean tuningMode,
                 LongSupplier monotonicClock, BaseDrivetrain<?> drivetrain) {
        super(constants, hardwareMap, tuningMode, monotonicClock, requireCompatible(drivetrain));
    }

    private static BaseDrivetrain<?> requireCompatible(BaseDrivetrain<?> drivetrain) {
        if (!(drivetrain instanceof Omni)) {
            throw new IllegalArgumentException("OmniFollower requires a uniform circular omni drivetrain");
        }
        return drivetrain;
    }
}
