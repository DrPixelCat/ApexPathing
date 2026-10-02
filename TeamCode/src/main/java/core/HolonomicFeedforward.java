package core;

import geometry.DistUnit;
import geometry.Vector;

/** Signed body-axis reference powers shared by holonomic planning and execution. */
public final class HolonomicFeedforward {
    public static final double MAX_CENTRIPETAL_POWER = 0.65;

    private HolonomicFeedforward() { }

    public static double motionSign(double velocity, double acceleration) {
        return Math.abs(velocity) > 1e-6 ? Math.signum(velocity)
                : Math.abs(acceleration) > 1e-6 ? Math.signum(acceleration) : 0;
    }

    public static Vector translation(FollowerConstants constants, Vector bodyDirection,
                                     double velocity, double acceleration, double kV) {
        return translation(constants, bodyDirection, velocity, acceleration, kV, motionSign(velocity, acceleration));
    }
    public static Vector translation(FollowerConstants constants, Vector bodyDirection,
                                     double velocity, double acceleration, double kV, double staticSign) {
        double legacy = velocity * kV + acceleration * constants.getTranslationalKA(velocity, acceleration)
                + staticSign * constants.translationalFeedforwardKS;
        return constants.characterizedTranslation(bodyDirection, velocity, acceleration,
                bodyDirection.times(legacy));
    }

    public static Vector centripetal(Vector bodyDirection, double velocity,
                                     double curvature, double coefficient) {
        return Vector.of(-bodyDirection.getY().getIn(), bodyDirection.getX().getIn(), DistUnit.IN)
                .times(velocity * velocity * curvature * coefficient);
    }

    public static double heading(FollowerConstants constants, double omega, double alpha) {
        double legacy = omega * constants.angularKV + alpha * constants.angularKA
                + motionSign(omega, alpha) * constants.angularFeedforwardKS;
        return constants.characterizedPower(MotionCharacterization.Axis.ANGULAR, omega, alpha, legacy);
    }

    /** Exact largest absolute mecanum wheel command after signed vectors are combined. */
    public static double mecanumDemand(Vector bodyPower, double headingPower) {
        return Math.abs(bodyPower.getX().getIn()) + Math.abs(bodyPower.getY().getIn())
                + Math.abs(headingPower);
    }
}
