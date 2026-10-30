package localizers.util;

import util.DeltaTime;

import paths.geometry.Angle;
import paths.geometry.DistUnit;
import paths.geometry.Pose;
import paths.geometry.Vector;

/**
 * Sensor-specific pose model and adaptive tuning, backed by the reusable Kalman core.
 *
 * @author Joel H
 */
public final class PoseEstimator {
    private static final double INITIAL_DT = 0.02;

    public static final class Tuning {
        /** Position measurement variance and continuous white-jerk process spectral density. */
        public final AdaptiveKalmanFilter.KalmanTuning x, y, heading;
        public final double xVelocityVariance, yVelocityVariance, headingVelocityVariance;

        public Tuning(AdaptiveKalmanFilter.KalmanTuning x, AdaptiveKalmanFilter.KalmanTuning y,
                      AdaptiveKalmanFilter.KalmanTuning heading) {
            this(x, y, heading, 0.25, 0.25, 0.01);
        }

        public Tuning(AdaptiveKalmanFilter.KalmanTuning x, AdaptiveKalmanFilter.KalmanTuning y,
                      AdaptiveKalmanFilter.KalmanTuning heading,
                      double xVelocityVariance, double yVelocityVariance, double headingVelocityVariance) {
            validate(x); validate(y); validate(heading);
            validateVariance(xVelocityVariance); validateVariance(yVelocityVariance);
            validateVariance(headingVelocityVariance);
            this.x = x; this.y = y; this.heading = heading;
            this.xVelocityVariance = xVelocityVariance;
            this.yVelocityVariance = yVelocityVariance;
            this.headingVelocityVariance = headingVelocityVariance;
        }

        public static Tuning defaults() {
            return new Tuning(new AdaptiveKalmanFilter.KalmanTuning(0.0025, 625),
                    new AdaptiveKalmanFilter.KalmanTuning(0.0025, 625),
                    new AdaptiveKalmanFilter.KalmanTuning(0.0001, 25));
        }

        private static void validate(AdaptiveKalmanFilter.KalmanTuning value) {
            if (value == null) { throw new IllegalArgumentException("Tuning cannot be null"); }
            validateVariance(value.measurementVariance); validateVariance(value.processVariance);
        }

        private static void validateVariance(double value) {
            if (!Double.isFinite(value) || value < 1e-12 || value > 1e12) {
                throw new IllegalArgumentException("Variances must be finite and between 1e-12 and 1e12");
            }
        }
    }

    private final Axis[] axes = {new Axis(), new Axis(), new Axis()};
    private final DeltaTime timing = new DeltaTime(INITIAL_DT);
    private Pose raw = Pose.zero(), estimate = Pose.zero(), velocity = Pose.zero(), acceleration = Pose.zero();
    private double unwrappedHeading;

    public PoseEstimator() { setTuning(Tuning.defaults()); setAutoTuning(true); }

    public Pose getPose() { return estimate; }
    public Pose getRawPose() { return raw; }
    public Pose getVelocity() { return velocity; }
    public Pose getAcceleration() { return acceleration; }
    public boolean isInitialized() { return timing.isInitialized(); }
    public boolean isAutoTuning() { return axes[0].auto || axes[1].auto || axes[2].auto; }
    public boolean isAutoTuning(int axis) { return axes[axis].auto; }
    public void setAutoTuning(boolean enabled) { setAutoTuning(enabled, enabled, enabled); }
    public void setAutoTuning(boolean x, boolean y, boolean heading) {
        axes[0].auto = x; axes[1].auto = y; axes[2].auto = heading;
    }

    public Tuning getTuning() {
        return new Tuning(axes[0].tuning(), axes[1].tuning(), axes[2].tuning(),
                axes[0].rv, axes[1].rv, axes[2].rv);
    }

    public void setTuning(Tuning tuning) {
        if (tuning == null) { throw new IllegalArgumentException("Tuning cannot be null"); }
        axes[0].configure(tuning.x, tuning.xVelocityVariance);
        axes[1].configure(tuning.y, tuning.yVelocityVariance);
        axes[2].configure(tuning.heading, tuning.headingVelocityVariance);
    }

    /** Compatibility with velocity calibration; Q now belongs to the shared state. */
    public AdaptiveKalmanFilter.KalmanTuning getVelocityTuning(int axis) {
        Axis a = axes[axis];
        return new AdaptiveKalmanFilter.KalmanTuning(a.rv, a.q);
    }

    public void setVelocityTuning(int axis, AdaptiveKalmanFilter.KalmanTuning tuning) {
        if (tuning == null || !Double.isFinite(tuning.measurementVariance)
                || !Double.isFinite(tuning.processVariance)
                || tuning.measurementVariance <= 0 || tuning.processVariance <= 0) {
            throw new IllegalArgumentException("Velocity tuning must be finite and positive");
        }
        Axis a = axes[axis];
        // Legacy velocity-only auto-calibration allowed nearly zero values. Accept those files
        // while keeping the joint estimator within its supported numerical range.
        a.rv = a.baseRv = clamp(tuning.measurementVariance, 1e-12, 1e12);
        a.q = a.baseQ = clamp(tuning.processVariance, 1e-12, 1e12);
    }

    /** Position-only diagnostic, retained for API compatibility. */
    public double[][] getCovariance() {
        return new double[][] {{axes[0].filter.getCovariance(0, 0), 0, 0},
                {0, axes[1].filter.getCovariance(0, 0), 0}, {0, 0, axes[2].filter.getCovariance(0, 0)}};
    }

    /** Defensive covariance ordered [x,vx,ax,y,vy,ay,heading,angularVelocity,angularAcceleration]. */
    public double[][] getStateCovariance() {
        double[][] result = new double[9][9];
        for (int axis = 0; axis < 3; axis++) {
            double[][] covariance = axes[axis].filter.getCovariance();
            for (int i = 0; i < 3; i++) {
                System.arraycopy(covariance[i], 0, result[axis * 3 + i], axis * 3, 3);
            }
        }
        return result;
    }

    /** Clears motion history, retaining calibrated noise and automatic-tuning switches. */
    public void reset(Pose pose) {
        if (!finite(pose)) { throw new IllegalArgumentException("Reset pose must be finite"); }
        raw = estimate = pose;
        velocity = acceleration = Pose.zero();
        unwrappedHeading = pose.getHeading().getRad();
        timing.reset();
        for (Axis axis : axes) { axis.clear(); }
    }

    /** Clears rates without replacing native readings or the current filtered position. */
    public void resetDerivatives() {
        double heading = axes[2].filter.getState(0);
        axes[0].initialize(estimate.getX().getIn(), 0.0);
        axes[1].initialize(estimate.getY().getIn(), 0.0);
        axes[2].initialize(heading, 0.0);
        velocity = acceleration = Pose.zero();
    }

    public boolean updateOdometry(Pose measurement, long timestampNanos) {
        return updateOdometry(measurement, null, timestampNanos);
    }

    /**
     * Native velocity is optional. Never pass a finite difference of these same position samples:
     * it adds no observation. When a device supplies both, each likelihood receives half weight
     * (covariance intersection with equal weights), avoiding an independence assumption.
     */
    public boolean updateOdometry(Pose measurement, Pose nativeVelocity, long timestampNanos) {
        if (!finite(measurement) || (nativeVelocity != null && !finite(nativeVelocity))
                || timestampNanos < 0 || (timing.isInitialized() && !timing.isAfterPrevious(timestampNanos))) { return false; }
        boolean reacquire = !timing.isInitialized() || timing.isLongGap(timestampNanos);
        double dt = reacquire ? INITIAL_DT : timing.elapsed(timestampNanos);
        if (!timing.isInitialized()) {
            // Keep the reset's angle reference, including multiple turns.
            unwrappedHeading = raw.getHeading().getRad()
                    + Angle.wrap(measurement.getHeading().getRad() - raw.getHeading().getRad());
        } else {
            unwrappedHeading += Angle.wrap(measurement.getHeading().getRad() - raw.getHeading().getRad());
        }
        double[] positions = {measurement.getX().getIn(), measurement.getY().getIn(), unwrappedHeading};
        double[] speeds = nativeVelocity == null ? null : new double[] {
                nativeVelocity.getX().getIn(), nativeVelocity.getY().getIn(), nativeVelocity.getHeading().getRad()};
        for (int i = 0; i < 3; i++) {
            if (reacquire) { axes[i].initialize(positions[i], speeds == null ? null : speeds[i]); }
            else { axes[i].update(positions[i], speeds == null ? null : speeds[i], dt); }
        }
        estimate = statePose(0); velocity = statePose(1); acceleration = statePose(2);
        raw = measurement; timing.reset(timestampNanos);
        return true;
    }

    private Pose statePose(int state) {
        return new Pose(Vector.of(axes[0].filter.getState(state), axes[1].filter.getState(state), DistUnit.IN),
                Angle.fromRad(axes[2].filter.getState(state)));
    }

    public static boolean finite(Pose pose) {
        return pose != null && Double.isFinite(pose.getX().getIn()) && Double.isFinite(pose.getY().getIn())
                && Double.isFinite(pose.getHeading().getRad());
    }

    private static double clamp(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }

    private static final class Axis {
        private final KalmanFilter filter = new KalmanFilter(3);
        private double r, rv, q, baseR, baseRv, baseQ;
        private double previousPosition, previousVelocity;
        private boolean hadVelocity, auto;
        private double smoothedNis = 1;
        private int gateCount, gateDirection;

        void configure(AdaptiveKalmanFilter.KalmanTuning tuning, double velocityVariance) {
            r = baseR = tuning.measurementVariance;
            q = baseQ = tuning.processVariance;
            rv = baseRv = velocityVariance;
        }

        AdaptiveKalmanFilter.KalmanTuning tuning() { return new AdaptiveKalmanFilter.KalmanTuning(r, q); }

        void clear() {
            filter.clear();
            hadVelocity = false; smoothedNis = 1; gateCount = gateDirection = 0;
        }

        void initialize(double position, Double speed) {
            clear();
            previousPosition = position;
            previousVelocity = speed == null ? 0 : speed; hadVelocity = speed != null;
            filter.reset(new double[] {position, previousVelocity, 0}, new double[][] {
                    {4 * r, 0, 0},
                    {0, speed == null ? Math.max(rv, r / (INITIAL_DT * INITIAL_DT)) : 2 * rv, 0},
                    {0, 0, Math.max(q * INITIAL_DT, rv / (INITIAL_DT * INITIAL_DT))}});
        }

        void update(double position, Double speed, double dt) {
            double dt2 = dt * dt, dt3 = dt2 * dt, dt4 = dt3 * dt, dt5 = dt4 * dt;
            double predictedDisplacement = filter.getState(1) * dt + filter.getState(2) * dt2 / 2;
            double predictedVelocityChange = filter.getState(2) * dt;
            filter.predict(new double[][] {{1, dt, dt2 / 2}, {0, 1, dt}, {0, 0, 1}},
                    new double[][] {{q * dt5 / 20, q * dt4 / 8, q * dt3 / 6},
                            {q * dt4 / 8, q * dt3 / 3, q * dt2 / 2}, {q * dt3 / 6, q * dt2 / 2, q * dt}});
            double posInnovation = position - filter.getState(0);
            double velInnovation = speed == null ? 0 : speed - filter.getState(1);
            double normalized = posInnovation / Math.sqrt(filter.getCovariance(0, 0) + r);
            double velocityNormalized = speed == null ? 0 : velInnovation / Math.sqrt(filter.getCovariance(1, 1) + rv);
            double dominant = Math.abs(normalized) >= Math.abs(velocityNormalized) ? normalized : velocityNormalized;
            if (Math.abs(dominant) > 4.5) {
                int direction = dominant > 0 ? 1 : -1;
                gateCount = direction == gateDirection ? gateCount + 1 : 1;
                gateDirection = direction;
            } else { gateCount = gateDirection = 0; }
            if (auto) {
                double alpha = -Math.expm1(-dt / 2);
                double difference = position - previousPosition - predictedDisplacement;
                if (Math.abs(normalized) <= 4.5) { r = adaptVariance(r, difference, alpha, baseR); }
                if (speed != null && hadVelocity && Math.abs(velocityNormalized) <= 4.5) {
                    rv = adaptVariance(rv, speed - previousVelocity - predictedVelocityChange, alpha, baseRv);
                }
                double nis = speed == null ? normalized * normalized
                        : (normalized * normalized + velocityNormalized * velocityNormalized) / 2;
                smoothedNis += (-Math.expm1(-dt)) * (Math.min(25, nis) - smoothedNis);
                double drive = clamp(0.5 * (smoothedNis - 1) + (gateCount >= 3 ? 3 : 0), -0.4, 3);
                q = clamp(q * Math.exp((-Math.expm1(-dt)) * drive),
                        Math.max(1e-12, baseQ * 0.1), Math.min(1e12, baseQ * 10));
            }
            // Equal-weight covariance intersection of position-only and velocity-only posteriors
            // sharing this prediction is exactly these half-weight information updates. Compute
            // robust weights from the SAME prediction before either observation changes the state.
            double weight = speed == null ? 1 : 2;
            filter.correctComponent(position, 0, weight * r * robustScale(normalized));
            if (speed != null) { filter.correctComponent(speed, 1, weight * rv * robustScale(velocityNormalized)); }
            previousPosition = position;
            hadVelocity = speed != null;
            if (speed != null) { previousVelocity = speed; }
        }

        private double robustScale(double normalized) {
            double scale = Math.abs(normalized) / 4.5;
            if (gateCount >= 3) { scale /= Math.min(4, 1 + gateCount); }
            return Math.max(1, scale);
        }

        private static double adaptVariance(double current, double difference, double alpha, double baseline) {
            double candidate = clamp(0.5 * difference * difference, current * 0.025, current * 25);
            return clamp(current + alpha * (candidate - current),
                    Math.max(1e-12, baseline * 0.25), Math.min(1e12, baseline * 4));
        }
    }
}
