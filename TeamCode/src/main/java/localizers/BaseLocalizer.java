package localizers;

import paths.geometry.Angle;
import paths.geometry.Dist;
import paths.geometry.DistUnit;
import paths.geometry.Pose;
import paths.geometry.Vector;
import localizers.util.AdaptiveKalmanFilter;
import localizers.util.DataFilter;
import localizers.util.FilterState;
import localizers.util.LowPassFilter;
import localizers.util.PoseEstimator;

/**
 * Base class for all localizers.
 *
 * <p>
 * This class provides common properties and methods for localizers, such as storing the current
 * factory, velocity, and acceleration estimates. Specific localizer types (like odometry, IMU-based,
 * etc.) should extend this class and implement the update() method that updates these estimates
 * based on sensor data.
 * </p>
 *
 * @param <T> the type of localizer configuration this drivetrain uses, which must extend
 *        {@link BaseLocalizerConstants}
 * @author Dylan B. - 18597 RoboClovers - Delta
 * @author Joel H
 */
public abstract class BaseLocalizer<T extends BaseLocalizerConstants<T>> {
    private static final AdaptiveKalmanFilter.KalmanTuning DEFAULT_TRANSLATION_KALMAN =
            new AdaptiveKalmanFilter.KalmanTuning(0.25, 625.0);
    private static final AdaptiveKalmanFilter.KalmanTuning DEFAULT_HEADING_KALMAN =
            new AdaptiveKalmanFilter.KalmanTuning(0.01, 25.0);
    public enum VelocityFilterMode {
        ADAPTIVE_KALMAN,
        MOVING_AVERAGE
    }

    protected T config;
    private VelocityFilterMode velocityFilterMode;
    private final PoseEstimator poseEstimator = new PoseEstimator();
    private boolean poseFilteringEnabled = true;
    private Pose lastAcceptedPose = Pose.zero();

    // Legacy fallback filters, used only when pose smoothing is bypassed or moving average selected.
    private DataFilter xFilter;
    private DataFilter yFilter;
    private DataFilter headingFilter;

    protected enum UpdateType { VELOCITY, ACCELERATION, BOTH }

    protected Pose pose = Pose.zero();
    protected Pose velocity = Pose.zero();
    protected Pose rawVelocity = Pose.zero();
    protected Pose acceleration = Pose.zero();
    protected Pose rawAcceleration = Pose.zero();

    /** History shared by pose-derived and native-velocity localizers. */
    private Pose prevPose = Pose.zero();
    private Pose prevRawVelocity = Pose.zero();
    private long lastMeasurementTimeNanos = -1;
    private boolean lastMeasurementValid;

    private int lastSize = 0;
    private int FILTER_WINDOW_SIZE = 7; //TODO: Verify this number and make it a constant or delete it
    private boolean isTuning = false;

    /**
     * Your localizer class constructor should call this super constructor to store the
     * configuration.
     *
     * @param config your localizer configuration object that is a child of
     *               {@link BaseLocalizerConstants}
     */
    public BaseLocalizer(T config) {
        this(config, VelocityFilterMode.ADAPTIVE_KALMAN);
    }

    protected BaseLocalizer(T config, VelocityFilterMode velocityFilterMode) {
        this.config = config;
        if (config != null) { poseEstimator.setTuning(config.getPoseKalmanTuning()); }
        setVelocityFilterModeInternal(velocityFilterMode);
    }

    /** @return the current factory estimate of the robot from the localizer */
    public Pose getPose() {
        return poseFilteringEnabled && poseEstimator.isInitialized() ? poseEstimator.getPose() : pose;
    }

    /** Unsmoothed native pose, used for geometry calibration and comparisons. */
    public Pose getRawPose() { return pose; }
    public double[][] getPoseCovariance() { return poseEstimator.getCovariance(); }
    public double[][] getKinematicCovariance() { return poseEstimator.getStateCovariance(); }
    public PoseEstimator.Tuning getPoseKalmanTuning() { return poseEstimator.getTuning(); }
    public void setPoseKalmanTuning(PoseEstimator.Tuning tuning) { poseEstimator.setTuning(tuning); }
    public void setPoseAutoTuning(boolean enabled) { poseEstimator.setAutoTuning(enabled); }
    public void setPoseAutoTuning(boolean x, boolean y, boolean heading) { poseEstimator.setAutoTuning(x, y, heading); }
    public boolean isPoseAutoTuning() { return poseEstimator.isAutoTuning(); }
    public boolean isPoseAutoTuning(int axis) { return poseEstimator.isAutoTuning(axis); }
    public boolean isPoseFilteringEnabled() { return poseFilteringEnabled; }
    public void setPoseFilteringEnabled(boolean enabled) {
        if (poseFilteringEnabled != enabled) {
            poseFilteringEnabled = enabled;
            poseEstimator.reset(pose);
            resetMotionHistory(pose);
        }
    }

    /** @return the current velocity estimate of the robot from the localizer */
    public Pose getVel() {
        return usesJointEstimator() && poseEstimator.isInitialized() ? poseEstimator.getVelocity() : velocity;
    }

    /**
     * NOTE: ONLY USE THIS IF YOU KNOW WHAT YOU ARE DOING
     *  @return the current raw velocity estimate of the robot from the localizer
     */
    public Pose getRawVel() { return rawVelocity; }

    /** @return the current acceleration estimate of the robot from the localizer */
    public Pose getAccel() {
        return usesJointEstimator() && poseEstimator.isInitialized() ? poseEstimator.getAcceleration() : acceleration;
    }

    private boolean usesJointEstimator() {
        return poseFilteringEnabled && velocityFilterMode == VelocityFilterMode.ADAPTIVE_KALMAN;
    }

    /**
     * NOTE: ONLY USE THIS IF YOU KNOW WHAT YOU ARE DOING
     *  @return the current raw acceleration estimate of the robot from the localizer
     */
    public Pose getRawAccel() { return rawAcceleration; }

    /** Returns the timestamp of the last valid measurement accepted by this localizer. */
    public long getLastMeasurementTimeNanos() { return lastMeasurementTimeNanos; }

    /** Returns whether the most recent hardware update supplied a valid measurement. */
    public boolean isLastMeasurementValid() { return lastMeasurementValid; }

    /** @return the velocity filter's moving-average window size in samples */
    public int getFilterWindowSize() { return FILTER_WINDOW_SIZE; }

    public VelocityFilterMode getVelocityFilterMode() { return velocityFilterMode; }

    /**
     * Selects the canonical public velocity estimator. Changing modes clears only kinematic
     * history; the current pose is preserved.
     */
    public void setVelocityFilterMode(VelocityFilterMode mode) {
        if (mode == null || mode == velocityFilterMode) { return; }
        setVelocityFilterModeInternal(mode);
        poseEstimator.resetDerivatives();
        resetMotionHistory(pose);
    }

    private void setVelocityFilterModeInternal(VelocityFilterMode mode) {
        velocityFilterMode = mode == null ? VelocityFilterMode.ADAPTIVE_KALMAN : mode;
        if (velocityFilterMode == VelocityFilterMode.ADAPTIVE_KALMAN) {
            xFilter = new AdaptiveKalmanFilter();
            yFilter = new AdaptiveKalmanFilter();
            headingFilter = new AdaptiveKalmanFilter();
            ((AdaptiveKalmanFilter) xFilter).setTuning(DEFAULT_TRANSLATION_KALMAN);
            ((AdaptiveKalmanFilter) yFilter).setTuning(DEFAULT_TRANSLATION_KALMAN);
            ((AdaptiveKalmanFilter) headingFilter).setTuning(DEFAULT_HEADING_KALMAN);
        } else {
            xFilter = new LowPassFilter();
            yFilter = new LowPassFilter();
            headingFilter = new LowPassFilter();
            applyMovingAverageWindow();
        }
    }

    /**
     * Update the localizer's factory, velocity, and acceleration estimates. This method should be
     * called regularly in a loop. If your localizer doesn't give velocity and/or acceleration, you
     * can use the calculate() method to update one or both using math
     */
    public abstract void update();

    /**
     * Set the localizer's current factory estimate with the given {@link Pose}
     * Note: Don't worry about updating this classes factory field, it will be updated in the next
     * update() call.
     */
    public abstract void setPose(Pose newPose);

    /**
     * Integrates a robot-frame displacement as a constant-curvature arc.
     *
     * <p>The translational delta is the body-frame twist accumulated over the update. The SE(2)
     * exponential scales its midpoint-heading projection by the arc-to-chord ratio, which also
     * has a well-behaved straight-line limit.</p>
     */
    protected static Pose integrateArc(Pose start, double deltaX, double deltaY,
                                       double deltaHeading, double finalHeading) {
        double halfHeading = deltaHeading * 0.5;
        double chordScale = Math.abs(halfHeading) < 1e-6
                ? 1.0 - halfHeading * halfHeading / 6.0
                : Math.sin(halfHeading) / halfHeading;
        double projectionHeading = start.getHeading().getRad() + halfHeading;
        double cos = Math.cos(projectionHeading);
        double sin = Math.sin(projectionHeading);
        double fieldDeltaX = chordScale * (deltaX * cos - deltaY * sin);
        double fieldDeltaY = chordScale * (deltaX * sin + deltaY * cos);
        return new Pose(
                Vector.of(start.getX().getIn() + fieldDeltaX,
                        start.getY().getIn() + fieldDeltaY, DistUnit.IN),
                Angle.fromRad(finalHeading));
    }

    /**
     * Calculates the current velocity and/or acceleration for localizers that don't natively
     * support it
     **/
    protected void calculate(UpdateType updateType) {
        calculate(updateType, System.nanoTime());
    }

    /** Uses one acquisition timestamp for pose and derivative filters. */
    protected void calculate(UpdateType updateType, long currentTimeNs) {
        long previousTimeNs = lastMeasurementTimeNanos;
        if (!acceptPoseSample(currentTimeNs, null)) { return; }
        markMeasurementValid(currentTimeNs);

        if (previousTimeNs < 0) {
            prevPose = pose;
            return;
        }

        // Sensor derivatives use the actual acquisition interval, including long gaps.
        // A clamped controller interval can otherwise freeze raw velocity after a pause.
        double dt = (currentTimeNs - previousTimeNs) * 1e-9;

        Pose delta = new Pose(pose.getVec().minus(prevPose.getVec()),
                Angle.fromRad(Angle.wrap(pose.getHeading().getRad() - prevPose.getHeading().getRad())));
        applyVelocityMeasurement(delta.div(dt), updateType, dt, currentTimeNs);
        prevPose = pose;
    }

    /**
     * Updates the same pose/velocity/acceleration state with a native velocity measurement.
     * Localizers with hardware-provided velocity should use this instead of assigning
     * {@link #velocity} directly.
     */
    protected void calculate(Pose measuredVelocity) {
        calculate(measuredVelocity, System.nanoTime());
    }

    protected void calculate(Pose measuredVelocity, long currentTimeNs) {
        long previousTimeNs = lastMeasurementTimeNanos;
        if (!PoseEstimator.finite(measuredVelocity)) { markMeasurementInvalid(); return; }
        if (!acceptPoseSample(currentTimeNs, measuredVelocity)) { return; }
        markMeasurementValid(currentTimeNs);
        if (previousTimeNs < 0) {
            rawVelocity = measuredVelocity;
            rawAcceleration = Pose.zero();
            applyFilterState(UpdateType.BOTH, currentTimeNs);
            prevPose = pose;
            prevRawVelocity = measuredVelocity;
            return;
        }

        double dt = (currentTimeNs - previousTimeNs) * 1e-9;

        applyVelocityMeasurement(measuredVelocity, UpdateType.BOTH, dt, currentTimeNs);
        prevPose = pose;
    }

    private void applyVelocityMeasurement(Pose measuredVelocity, UpdateType updateType,
                                          double dt, long currentTimeNs) {
        rawVelocity = measuredVelocity;
        rawAcceleration = rawVelocity.minus(prevRawVelocity).div(dt);
        applyFilterState(updateType, currentTimeNs);
        prevRawVelocity = rawVelocity;
    }

    private boolean acceptPoseSample(long currentTimeNs, Pose measuredVelocity) {
        if (!PoseEstimator.finite(pose) || (lastMeasurementTimeNanos != -1
                && currentTimeNs <= lastMeasurementTimeNanos) || currentTimeNs < 0) {
            markMeasurementInvalid();
            return false;
        }
        // Explicit moving-average mode keeps its existing velocity estimator. The pose estimator
        // receives position only in that mode, so native velocity is never smoothed twice.
        Pose nativeVelocity = usesJointEstimator() ? measuredVelocity : null;
        if (poseFilteringEnabled && !poseEstimator.updateOdometry(pose, nativeVelocity, currentTimeNs)) {
            markMeasurementInvalid();
            return false;
        }
        lastAcceptedPose = pose;
        return true;
    }

    private void applyFilterState(UpdateType updateType, long currentTimeNs) {
        if (usesJointEstimator()) { return; }

        // Update sample size if using LowPass
        if (FILTER_WINDOW_SIZE != lastSize
                && velocityFilterMode == VelocityFilterMode.MOVING_AVERAGE) {
            applyMovingAverageWindow();
        }

        // Filters MUST be updated every loop to maintain mathematical state continuity,
        // regardless of the UpdateType requested.
        FilterState xState = xFilter.update(rawVelocity.getX().getIn(), currentTimeNs);
        FilterState yState = yFilter.update(rawVelocity.getY().getIn(), currentTimeNs);
        FilterState headingState = headingFilter.update(rawVelocity.getHeading().getRad(), currentTimeNs);

        if (updateType == UpdateType.BOTH || updateType == UpdateType.VELOCITY) {
            velocity = new Pose(
                    new Vector(Dist.fromIn(xState.value()), Dist.fromIn(yState.value())),
                    Angle.fromRad(headingState.value())
            );
        }

        if (updateType == UpdateType.BOTH || updateType == UpdateType.ACCELERATION) {
            acceleration = new Pose(
                    new Vector(Dist.fromIn(xState.rate()), Dist.fromIn(yState.rate())),
                    Angle.fromRad(headingState.rate())
            );
        }

    }

    /** Clears motion history after an odometry pose reset or simulator staging teleport. */
    protected void resetKinematicEstimate(Pose newPose) {
        poseEstimator.reset(newPose);
        pose = newPose;
        lastAcceptedPose = newPose;
        resetMotionHistory(newPose);
    }

    private void resetMotionHistory(Pose newPose) {
        velocity = Pose.zero();
        rawVelocity = Pose.zero();
        acceleration = Pose.zero();
        rawAcceleration = Pose.zero();
        prevPose = newPose;
        prevRawVelocity = Pose.zero();
        lastMeasurementTimeNanos = -1;
        lastMeasurementValid = false;
        xFilter.reset();
        yFilter.reset();
        headingFilter.reset();
    }

    /** Marks a hardware update invalid without pretending a cached value is a new sample. */
    protected void markMeasurementInvalid() {
        lastMeasurementValid = false;
        pose = lastAcceptedPose;
    }

    private void markMeasurementValid(long timestamp) {
        lastMeasurementTimeNanos = timestamp;
        lastMeasurementValid = true;
    }

    public void setIsTuning(boolean isTuning) {
        setKalmanAutoTuning(isTuning, isTuning, isTuning);
    }

    /** Enables one-time calibration independently for translation, strafe, and heading. */
    public void setKalmanAutoTuning(boolean tuneX, boolean tuneY, boolean tuneHeading) {
        if (velocityFilterMode != VelocityFilterMode.ADAPTIVE_KALMAN) {
            isTuning = false;
            return;
        }
        ((AdaptiveKalmanFilter) xFilter).setAutoTuning(tuneX);
        ((AdaptiveKalmanFilter) yFilter).setAutoTuning(tuneY);
        ((AdaptiveKalmanFilter) headingFilter).setAutoTuning(tuneHeading);
        poseEstimator.setAutoTuning(tuneX, tuneY, tuneHeading);
        isTuning = tuneX || tuneY || tuneHeading;
    }

    public boolean isTuningVelocityFilter() {
        return usesJointEstimator() ? poseEstimator.isAutoTuning() : isTuning;
    }

    // TUNING INJECTION/EXTRACTION

    public AdaptiveKalmanFilter.KalmanTuning getXKalmanTuning() {
        if (usesJointEstimator()) { return poseEstimator.getVelocityTuning(0); }
        return kalman(xFilter) != null ? kalman(xFilter).getTuning() : null;
    }

    public void setXKalmanTuning(AdaptiveKalmanFilter.KalmanTuning tuning) {
        if (kalman(xFilter) != null && tuning != null) {
            poseEstimator.setVelocityTuning(0, tuning);
            kalman(xFilter).setTuning(tuning);
        }
    }

    public AdaptiveKalmanFilter.KalmanTuning getYKalmanTuning() {
        if (usesJointEstimator()) { return poseEstimator.getVelocityTuning(1); }
        return kalman(yFilter) != null ? kalman(yFilter).getTuning() : null;
    }

    public void setYKalmanTuning(AdaptiveKalmanFilter.KalmanTuning tuning) {
        if (kalman(yFilter) != null && tuning != null) {
            poseEstimator.setVelocityTuning(1, tuning);
            kalman(yFilter).setTuning(tuning);
        }
    }

    public AdaptiveKalmanFilter.KalmanTuning getHeadingKalmanTuning() {
        if (usesJointEstimator()) { return poseEstimator.getVelocityTuning(2); }
        return kalman(headingFilter) != null ? kalman(headingFilter).getTuning() : null;
    }

    public void setHeadingKalmanTuning(AdaptiveKalmanFilter.KalmanTuning tuning) {
        if (kalman(headingFilter) != null && tuning != null) {
            poseEstimator.setVelocityTuning(2, tuning);
            kalman(headingFilter).setTuning(tuning);
        }
    }

    private AdaptiveKalmanFilter kalman(DataFilter filter) {
        return filter instanceof AdaptiveKalmanFilter ? (AdaptiveKalmanFilter) filter : null;
    }

    private void applyMovingAverageWindow() {
        ((LowPassFilter) xFilter).setSampleSize(FILTER_WINDOW_SIZE);
        ((LowPassFilter) yFilter).setSampleSize(FILTER_WINDOW_SIZE);
        ((LowPassFilter) headingFilter).setSampleSize(FILTER_WINDOW_SIZE);
        lastSize = FILTER_WINDOW_SIZE;
    }

    // TODO: Delete this temporary method once a value is settled on
    /**
     * FOR INTERNAL USE ONLY, DO NOT USE
     * @param windowSize the size of the filter window
     */
    public void setFilterWindow(int windowSize) {
        if (windowSize < 1) { throw new IllegalArgumentException("Filter window must be positive"); }
        this.FILTER_WINDOW_SIZE = windowSize;
        if (velocityFilterMode == VelocityFilterMode.MOVING_AVERAGE) { applyMovingAverageWindow(); }
    }
}
