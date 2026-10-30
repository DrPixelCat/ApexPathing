package setup.follower;

import drivetrains.BaseDrivetrain;
import drivetrains.Omni;
import feedforward.MotionCharacterization.Axis;

/** Motion and motor-output measurements shared by follower tuning workflows. */
public final class TuningMeasurements {
    private TuningMeasurements() { }

    public static double maxMotorPower(BaseDrivetrain<?> drivetrain) {
        return drivetrain.getMaxMotorPower();
    }

    /**
     * Magnitude of the delivered single-axis command during characterization. Omni wheel duty
     * is a projection of body-axis demand; undo that projection before fitting command gains.
     * Call only while commanding one robot-relative axis at a time.
     */
    public static double axisCommandPower(BaseDrivetrain<?> drivetrain, Axis axis) {
        double peak = maxMotorPower(drivetrain);
        if (drivetrain instanceof Omni && axis != Axis.ANGULAR) {
            Omni omni = (Omni) drivetrain;
            double projection = axis == Axis.FORWARD
                    ? omni.translationDemand(1, 0) : omni.translationDemand(0, 1);
            return peak / projection;
        }
        return peak;
    }

    /** Projects field coordinates onto a fixed robot-relative forward or strafe axis. */
    public static double projectVelocity(double x, double y, double heading, boolean strafe) {
        return strafe ? -x * Math.sin(heading) + y * Math.cos(heading)
                : x * Math.cos(heading) + y * Math.sin(heading);
    }
}
