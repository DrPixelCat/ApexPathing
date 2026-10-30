package follower.internal;

import feedforward.MecanumWheelModel;
import paths.geometry.Angle;
import paths.geometry.Vector;
import trajectory.TrajectorySample;

/** Stateful inverse-wheel preview and pulse compensation, isolated from path tracking feedback. */
final class MecanumWheelFeedforward {
    private final MecanumWheelModel.Demand demand = new MecanumWheelModel.Demand();
    private final double[] debt = {.5, .5, .5, .5};
    private final double[] predictionDebt = new double[4], powers = new double[4];
    private Vector velocity = Vector.zero(), acceleration = Vector.zero();
    private double omega;

    void reset() { java.util.Arrays.fill(debt, .5); }
    double[] powers() { return powers; }

    boolean prepare(MecanumWheelModel model, TrajectorySample target, TrajectorySample forceReference,
                    Angle heading, Vector measuredVelocity, double measuredOmega,
                    boolean independentReference, boolean feedbackEnabled) {
        double lead = model.commandPreviewSeconds;
        double referenceOmega = feedbackEnabled ? measuredOmega : forceReference.angularVelocity;
        Angle referenceHeading = Angle.fromRad(heading.getRad() + referenceOmega * lead);
        velocity = (independentReference ? forceReference.velocity
                : (feedbackEnabled ? measuredVelocity : target.velocity).plus(target.acceleration.times(lead)))
                .rotate(referenceHeading.times(-1));
        acceleration = forceReference.acceleration.rotate(referenceHeading.times(-1));
        omega = referenceOmega + (independentReference ? 0 : target.angularAcceleration * lead);
        model.solve(velocity.getX().getIn(), velocity.getY().getIn(), omega,
                acceleration.getX().getIn(), acceleration.getY().getIn(), forceReference.angularAcceleration, demand);
        if (!demand.insideDomain || !Double.isFinite(demand.peak)) {
            // Let the caller fall back to bounded body-axis allocation outside the measured domain.
            reset();
            return false;
        }
        System.arraycopy(debt, 0, predictionDebt, 0, 4);
        demand.pulse(feedbackEnabled ? predictionDebt : debt, powers);
        return true;
    }

    /** Apply allocated feedback as acceleration, then consume pulse debt once for the actual output. */
    double applyFeedback(MecanumWheelModel model, Vector correction, double angularCorrection,
                         double fade, double targetAlpha, double[] output) {
        double vx = velocity.getX().getIn(), vy = velocity.getY().getIn(), mag = Math.hypot(vx, vy);
        double drag = model.dragMagnitude(vx, vy);
        double dx = mag > 1e-9 ? drag * vx / mag : 0, dy = mag > 1e-9 ? drag * vy / mag : 0;
        double turnDrag = model.angularDrag * Math.signum(omega);
        double scale = model.solveBounded(vx, vy, omega,
                (acceleration.getX().getIn() + dx) * fade - dx + correction.getX().getIn(),
                (acceleration.getY().getIn() + dy) * fade - dy + correction.getY().getIn(),
                (targetAlpha + turnDrag) * fade - turnDrag + angularCorrection, 1, demand);
        demand.pulse(debt, output);
        return scale;
    }
}
