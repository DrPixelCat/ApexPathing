package tuning;

import geometry.Pose;
import drivetrains.BaseDrivetrain;

/** Localizers expose field-frame translation; pure drive tests need the robot's forward component. */
public final class TuningMeasurements {
    private TuningMeasurements() { }

    public static double forward(Pose fieldMotion, Pose pose) {
        double heading = pose.getHeading().getRad();
        return Math.cos(heading) * fieldMotion.getX().getIn() + Math.sin(heading) * fieldMotion.getY().getIn();
    }

    /** Inverts the built-in mixers for a pure forward or turn command after power limiting. */
    public static double appliedEffort(BaseDrivetrain<?> d, boolean angular, double requested) {
        double left = d.getLastFlPower(), right = d.getLastFrPower();
        switch (d.getDrivetrainType()) {
            case KIWI:
                return angular ? -(left + right + d.getLastBlPower()) / 3 : (right - left) / Math.sqrt(3);
            case COAXIAL_SWERVE:
                // Pure drive and pure turn have equal wheel magnitudes; module optimization can
                // flip individual motor signs. Steering alignment is intentionally not inferred.
                double magnitude = Math.max(Math.max(Math.abs(left), Math.abs(right)),
                        Math.max(Math.abs(d.getLastBlPower()), Math.abs(d.getLastBrPower())));
                return requested == 0 ? 0 : Math.copySign(magnitude, requested);
            default:
                return angular ? (right - left) / 2 : (right + left) / 2;
        }
    }
}
