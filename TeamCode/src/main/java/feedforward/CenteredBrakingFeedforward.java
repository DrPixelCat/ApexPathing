package feedforward;

import paths.geometry.DistUnit;
import paths.geometry.Vector;
import paths.movements.Path;
import trajectory.TrajectorySample;
import trajectory.profile.MotionParameters;

/** Optional measured forward-axis braking. Each follower owns its pulse state.
 * Calibration is supplied explicitly; no simulator coefficients are built in. */
public final class CenteredBrakingFeedforward {
    private final double[] coefficients;
    private final double maximumSpeed;
    private double dutyDebt = .6;
    private double previousDirection;
    private Path previousPath;

    public CenteredBrakingFeedforward(double[] coefficients, double maximumSpeed) {
        if (coefficients == null || coefficients.length != 8 || !Double.isFinite(maximumSpeed) || maximumSpeed <= 0) {
            throw new IllegalArgumentException("Eight measured coefficients and a positive speed domain required");
        }
        this.coefficients = coefficients.clone(); this.maximumSpeed = maximumSpeed;
        for (double value : coefficients) {
            if (!Double.isFinite(value)) { throw new IllegalArgumentException("Finite braking coefficients required"); }
        }
        for (double speed : new double[]{0, maximumSpeed}) {
            if (coefficients[0] + coefficients[1] * speed <= 0
                    || coefficients[2] + coefficients[3] * speed <= 0) {
                throw new IllegalArgumentException("Positive measured braking authority required");
            }
        }
    }

    public void reset() { dutyDebt = .6; previousDirection = 0; previousPath = null; }

    /** Returns null outside forward/reverse braking or the calibrated speed domain. */
    public Result calculate(Path path, TrajectorySample target, Vector bodyDirection, Vector legacy) {
        double v = target.tangentialVelocity(), a = target.tangentialAcceleration();
        double direction = Math.signum(bodyDirection.getX().getIn() * v);
        if (previousPath != path || direction != previousDirection) { dutyDebt = .6; }
        previousPath = path; previousDirection = direction;
        if (Math.abs(bodyDirection.getX().getIn()) < .999 || Math.abs(v) > maximumSpeed + 1e-6
                || Math.abs(v) < 1e-6 || v * a >= -1e-6) {
            dutyDebt = .6;
            return null;
        }
        double speed = Math.abs(v), acceleration = a * Math.signum(v);
        double lead = .0025 * Math.min(1, speed / 12);
        if (path.getFeedforwardLut() != null) {
            MotionParameters preview = path.getFeedforwardLut().getKinematicParams(
                    Math.min(path.getParametricPath().getLengthIn(), target.progressIn + speed * lead));
            acceleration = preview.getTangentialAccel() * Math.signum(v);
        }
        double modelSpeed = Math.max(0, speed + acceleration * lead);
        double forwardGain = coefficients[0] + coefficients[1] * modelSpeed;
        double reverseGain = coefficients[2] + coefficients[3] * modelSpeed;
        double forwardOffset = coefficients[4] + coefficients[5] * modelSpeed;
        double reverseOffset = coefficients[6] + coefficients[7] * modelSpeed;
        double forwardPower = (acceleration - forwardOffset) / forwardGain;
        double reversePower = (acceleration - reverseOffset) / reverseGain;
        double pulse, mean;
        if (forwardPower >= .02) { pulse = mean = forwardPower; }
        else if (reversePower <= -.02) { pulse = mean = reversePower; }
        else {
            double forwardAccel = forwardOffset + forwardGain * .02;
            double reverseAccel = reverseOffset - reverseGain * .02;
            double duty = Math.max(0, Math.min(1, (forwardAccel - acceleration) / (forwardAccel - reverseAccel)));
            mean = (1 - duty) * .02 - duty * .02;
            dutyDebt += duty;
            if (dutyDebt >= 1) { pulse = -.02; dutyDebt -= 1; }
            else { pulse = .02; }
        }
        return new Result(Vector.of(direction * pulse, legacy.getY().getIn(), DistUnit.IN),
                Vector.of(direction * mean, legacy.getY().getIn(), DistUnit.IN));
    }

    public static final class Result {
        public final Vector pulseBodyPower, meanBodyPower;
        private Result(Vector pulseBodyPower, Vector meanBodyPower) {
            this.pulseBodyPower = pulseBodyPower; this.meanBodyPower = meanBodyPower;
        }
    }
}
