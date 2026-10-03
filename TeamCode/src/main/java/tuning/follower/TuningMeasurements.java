package tuning.follower;

import drivetrains.BaseDrivetrain;

/** Motion and motor-output measurements shared by follower tuning workflows. */
public final class TuningMeasurements {
    private TuningMeasurements() { }

    public static double maxMotorPower(BaseDrivetrain<?> drivetrain) {
        return Math.max(Math.max(Math.abs(drivetrain.getLastFlPower()),
                        Math.abs(drivetrain.getLastFrPower())),
                Math.max(Math.abs(drivetrain.getLastBlPower()),
                        Math.abs(drivetrain.getLastBrPower())));
    }

    /** Projects field coordinates onto a fixed robot-relative forward or strafe axis. */
    public static double projectVelocity(double x, double y, double heading, boolean strafe) {
        return strafe ? -x * Math.sin(heading) + y * Math.cos(heading)
                : x * Math.cos(heading) + y * Math.sin(heading);
    }
}