package feedforward;

import feedforward.MotionCharacterization.Axis;
import follower.FollowerConstants;
import paths.geometry.DistUnit;
import paths.geometry.Vector;

/** Pure reference-power calculation. Callers own the output; no sensors, feedback, or motor writes. */
public final class MotionPowerCalculator {
    public static final double MAX_CENTRIPETAL_POWER = .65;
    private final FollowerConstants constants;
    private final boolean holonomic;
    @FunctionalInterface
    public interface CommandDemand { double demand(double x,double y,double turn); }
    private final CommandDemand commandDemand, planningDemand;

    public MotionPowerCalculator(FollowerConstants constants, boolean holonomic,
                                 CommandDemand commandDemand, CommandDemand planningDemand) {
        this.constants = java.util.Objects.requireNonNull(constants);
        this.holonomic = holonomic;
        this.commandDemand = java.util.Objects.requireNonNull(commandDemand);
        this.planningDemand = java.util.Objects.requireNonNull(planningDemand);
    }

    public static double motionSign(double velocity, double acceleration) {
        return Math.abs(velocity) > 1e-6 ? Math.signum(velocity)
                : Math.abs(acceleration) > 1e-6 ? Math.signum(acceleration) : 0;
    }

    public static final class Output {
        public double forward, strafe, heading, normalForward, normalStrafe, demand;
        public Vector translation() { return Vector.of(forward, strafe, DistUnit.IN); }
        public Vector normal() { return Vector.of(normalForward, normalStrafe, DistUnit.IN); }
    }

    /** Runtime convenience overload with the reference motion's static sign. */
    public Output calculate(Vector body, double v, double a, double omega, double alpha, double normalAcceleration) {
        Output result = new Output();
        calculate(body.getX().getIn(), body.getY().getIn(), v, a, omega, alpha, normalAcceleration,
                false, motionSign(v, a), result);
        return result;
    }

    /**
     * Planning uses the optional kV reserve and tank's worse travel direction. Runtime supplies its static-sign policy.
     * Body direction must be normalized; normal acceleration is signed toward its left normal.
     * Linear quantities use inches/seconds; angular quantities use radians/seconds; output is normalized power.
     */
    public void calculate(double bx, double by, double v, double a, double omega, double alpha, double normalAcceleration,
                          boolean planning, double staticSign, Output result) {
        double kV = planning && Double.isFinite(constants.profileVelocityPowerCoefficient)
                && constants.profileVelocityPowerCoefficient > 0
                ? constants.profileVelocityPowerCoefficient : constants.translationalKV;
        double tangent = v * kV + a * constants.getTranslationalKA(v, a) + staticSign * constants.translationalFeedforwardKS;
        result.forward = translationPower(Axis.FORWARD, holonomic ? bx : 1, v, a, tangent);
        result.strafe = holonomic ? translationPower(Axis.STRAFE, by, v, a, tangent) : 0;
        double normal = holonomic ? normalAcceleration * constants.kCentripetal : 0;
        if (!planning) normal = Math.max(-MAX_CENTRIPETAL_POWER, Math.min(MAX_CENTRIPETAL_POWER, normal));
        result.normalForward = -by * normal;
        result.normalStrafe = bx * normal;
        result.heading = axisPower(Axis.ANGULAR, omega, alpha, omega * constants.angularKV + alpha * constants.angularKA
                + (holonomic ? motionSign(omega, alpha) * constants.angularFeedforwardKS : 0));
        CommandDemand demand = planning ? planningDemand : commandDemand;
        result.demand = demand.demand(result.forward + result.normalForward,result.strafe + result.normalStrafe,result.heading);
        // Scalar path speed carries no travel sign, so tank planning reserves both calibrated directions.
        if (planning && !holonomic) result.demand = Math.max(result.demand,
                demand.demand(axisPower(Axis.FORWARD,-v,-a,-tangent),0,result.heading));
    }

    private double translationPower(Axis axis, double component, double v, double a, double tangent) {
        double legacy = component * tangent;
        return holonomic && v * a < -1e-6 ? legacy : axisPower(axis, component * v, component * a, legacy);
    }

    private double axisPower(Axis axis, double velocity, double acceleration, double legacy) {
        return constants.characterizedPower(axis, velocity, acceleration, legacy);
    }
}
