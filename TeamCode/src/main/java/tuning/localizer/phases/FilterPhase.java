package tuning.localizer.phases;

import util.DeltaTime;

import drivetrains.TranslationAxis;

import tuning.localizer.LocalizationTunerContext;
import tuning.localizer.TuningPhase;

import geometry.Pose;
import localizers.BaseLocalizer;
import localizers.util.AdaptiveKalmanFilter;
import tuning.TuningCsvWriter;

/** Guides data collection for the unified Kalman estimator or legacy moving-average mode. */
public final class FilterPhase extends TuningPhase {
    private static final int STILL_SAMPLES = 75;
    private static final int STOP_SAMPLES = 75;
    private static final double REQUIRED_X_INCHES = 100;
    private static final double REQUIRED_Y_INCHES = 100;
    private static final double REQUIRED_HEADING_RADIANS = 10 * Math.PI;
    private static final double MAX_INTEGRATION_SECONDS = 0.10;

    private enum Stage { STILL, MOVE, STOP }

    private BaseLocalizer.VelocityFilterMode selected =
            BaseLocalizer.VelocityFilterMode.ADAPTIVE_KALMAN;
    private int window = 7;
    private FilterMetrics metrics;
    private TuningCsvWriter csv;
    private long started;
    private final DeltaTime timing = new DeltaTime();
    private double xMovement;
    private double yMovement;
    private double headingMovement;
    private Stage stage;
    private boolean requiresYMovement;
    private boolean recordingStarted;
    private boolean accepted;
    private BaseLocalizer.VelocityFilterMode originalMode;
    private int originalWindow;
    private localizers.util.PoseEstimator.Tuning originalTuning;
    private final boolean[] originalAutoTuning = new boolean[3];

    public FilterPhase(LocalizationTunerContext context) { super(context); }
    @Override protected String getName() { return "Position, velocity and acceleration filter"; }

    @Override protected void reset() {
        selected = BaseLocalizer.VelocityFilterMode.ADAPTIVE_KALMAN;
        window = context.getLocalizer().getFilterWindowSize();
        metrics = new FilterMetrics();
        stage = Stage.STILL;
        recordingStarted = false;
    }

    @Override protected void showPrepare() {
        if (opMode.gamepad1.dpadLeftWasPressed() || opMode.gamepad1.dpadRightWasPressed()) {
            selected = selected == BaseLocalizer.VelocityFilterMode.ADAPTIVE_KALMAN
                    ? BaseLocalizer.VelocityFilterMode.MOVING_AVERAGE
                    : BaseLocalizer.VelocityFilterMode.ADAPTIVE_KALMAN;
        }
        if (selected == BaseLocalizer.VelocityFilterMode.MOVING_AVERAGE) {
            if (opMode.gamepad1.dpadUpWasPressed()) { window = Math.min(50, window + 1); }
            if (opMode.gamepad1.dpadDownWasPressed()) { window = Math.max(1, window - 1); }
        }
        context.getTelemetry().addData("Selected", display(selected));
        if (selected == BaseLocalizer.VelocityFilterMode.MOVING_AVERAGE) {
            context.getTelemetry().addData("Window", window);
            context.getTelemetry().addLine("Dpad Up/Down: adjust window");
        }
        context.getTelemetry().addLine("Dpad Left/Right: choose estimator");
        context.getTelemetry().addLine("Keep still, then move in every available direction.");
        context.getTelemetry().addLine("A: begin");
    }

    @Override protected void beginRecording() {
        accepted = false;
        originalMode = context.getLocalizer().getVelocityFilterMode();
        originalWindow = context.getLocalizer().getFilterWindowSize();
        originalTuning = context.getLocalizer().getPoseKalmanTuning();
        for (int axis = 0; axis < 3; axis++) {
            originalAutoTuning[axis] = context.getLocalizer().isPoseAutoTuning(axis);
        }
        recordingStarted = true;
        requiresYMovement = context.getDrivetrain().isHolonomic()
                && context.getAdapter().supportsDistance(TranslationAxis.STRAFE);
        context.getLocalizer().setVelocityFilterMode(selected);
        if (selected == BaseLocalizer.VelocityFilterMode.MOVING_AVERAGE) {
            context.getLocalizer().setFilterWindow(window);
        } else {
            context.getLocalizer().setKalmanAutoTuning(true,
                    requiresYMovement, true);
        }
        metrics = new FilterMetrics();
        csv = TuningCsvWriter.open("localization_filter", "time_s", "stage", "valid",
                "raw_x", "raw_y", "raw_heading", "filtered_x", "filtered_y",
                "filtered_heading", "accel_x", "accel_y", "accel_heading");
        started = System.nanoTime();
        timing.reset(started);
        xMovement = 0.0;
        yMovement = 0.0;
        headingMovement = 0.0;
        stage = Stage.STILL;
    }

    @Override protected boolean record() {
        long now = System.nanoTime();
        long elapsed = now - started;
        double dt = Math.min(MAX_INTEGRATION_SECONDS,
                Math.max(0.0, timing.update(now)));

        boolean initialStill = stage == Stage.STILL;
        boolean driving = stage == Stage.MOVE;
        boolean stopping = stage == Stage.STOP;
        if (driving) { context.manualDrive(true); } else { context.stop(); }

        Pose raw = context.getLocalizer().getRawVel();
        Pose filtered = context.getLocalizer().getVel();
        Pose accel = context.getLocalizer().getAccel();
        boolean valid = context.getLocalizer().isLastMeasurementValid();
        metrics.sample(raw, filtered, initialStill, stopping, valid);
        if (driving && valid) { accumulateMovement(raw, dt); }
        writeCsv(DeltaTime.seconds(elapsed), stage.name(),
                raw, filtered, accel);

        if (stage == Stage.STILL) {
            context.getTelemetry().addLine("Keep the robot still...");
            if (metrics.getStationarySamples() >= STILL_SAMPLES) { stage = Stage.MOVE; }
        } else if (stage == Stage.MOVE) {
            showMovement();
            if (hasEnoughMovement()) {
                context.stop();
                stage = Stage.STOP;
            }
        } else {
            context.getTelemetry().addLine("Release the sticks and keep still...");
        }
        return stage == Stage.STOP && metrics.getStopSamples() >= STOP_SAMPLES;
    }

    @Override protected void finishRecording() {
        context.getLocalizer().setKalmanAutoTuning(false, false, false);
        context.getLocalizer().setPoseAutoTuning(false);
        if (csv != null) { csv.close(); }
    }

    @Override protected void cleanup() {
        if (!recordingStarted) { return; }
        context.getLocalizer().setKalmanAutoTuning(false, false, false);
        context.getLocalizer().setPoseAutoTuning(false);
        if (csv != null) { csv.close(); }
        if (!accepted) {
            context.getLocalizer().setVelocityFilterMode(originalMode);
            context.getLocalizer().setFilterWindow(originalWindow);
            context.getLocalizer().setPoseKalmanTuning(originalTuning);
        }
        context.getLocalizer().setPoseAutoTuning(originalAutoTuning[0], originalAutoTuning[1], originalAutoTuning[2]);
        recordingStarted = false;
    }

    @Override protected void showReview() {
        context.getTelemetry().addData("Estimator", display(selected));
        context.getTelemetry().addData("Raw still RMS (in/s)", context.formatNumber(metrics.rawStationaryRms()));
        context.getTelemetry().addData("Filtered still RMS (in/s)", context.formatNumber(metrics.filteredStationaryRms()));
        context.getTelemetry().addData("Stop residual RMS (in/s)", context.formatNumber(metrics.stopResidualRms()));
        context.getTelemetry().addData("Raw angular RMS (rad/s)", context.formatNumber(metrics.rawAngularRms()));
        context.getTelemetry().addData("Filtered angular RMS (rad/s)", context.formatNumber(metrics.filteredAngularRms()));
        context.getTelemetry().addData("Stop angular RMS (rad/s)", context.formatNumber(metrics.stopAngularRms()));
        if (metrics.getInvalidSamples() > 0) {
            context.getTelemetry().addData("Invalid samples", metrics.getInvalidSamples());
        }
        if (selected == BaseLocalizer.VelocityFilterMode.MOVING_AVERAGE) {
            context.getTelemetry().addData("Window", context.getLocalizer().getFilterWindowSize());
        } else { showKalman(); }
        if (csv != null && csv.getError() != null) {
            context.getTelemetry().addData("CSV error", csv.getError());
        }
    }

    @Override protected boolean accept() {
        accepted = context.acceptFilter();
        return accepted;
    }

    private void showKalman() {
        addTuning("X", context.getLocalizer().getXKalmanTuning());
        addTuning("Y", context.getLocalizer().getYKalmanTuning());
        addTuning("Heading", context.getLocalizer().getHeadingKalmanTuning());
    }

    private void addTuning(String axis, AdaptiveKalmanFilter.KalmanTuning tuning) {
        if (tuning == null) { return; }
        context.getTelemetry().addData(axis + " velocity R / jerk Q",
                context.formatNumber(tuning.measurementVariance) + " / "
                        + context.formatNumber(tuning.processVariance));
    }

    private void accumulateMovement(Pose velocity, double dt) {
        geometry.Vector body = velocity.getVec().rotate(context.getLocalizer().getPose().getHeading().times(-1));
        xMovement += Math.abs(body.getX().getIn()) * dt;
        yMovement += Math.abs(body.getY().getIn()) * dt;
        headingMovement += Math.abs(velocity.getHeading().getRad()) * dt;
    }

    private boolean hasEnoughMovement() {
        return xMovement >= REQUIRED_X_INCHES
                && (!requiresYMovement || yMovement >= REQUIRED_Y_INCHES)
                && headingMovement >= REQUIRED_HEADING_RADIANS;
    }

    private void showMovement() {
        context.getTelemetry().addData("Move X",
                progress(xMovement, REQUIRED_X_INCHES, "in"));
        if (requiresYMovement) {
            context.getTelemetry().addData("Move Y",
                    progress(yMovement, REQUIRED_Y_INCHES, "in"));
        }
        context.getTelemetry().addData("Turn",
                progress(Math.toDegrees(headingMovement),
                        Math.toDegrees(REQUIRED_HEADING_RADIANS), "deg"));
    }

    private String progress(double current, double required, String unit) {
        return context.formatNumber(Math.min(current, required)) + " / "
                + context.formatNumber(required) + " " + unit;
    }

    private void writeCsv(double time, String stage, Pose raw, Pose filtered, Pose accel) {
        if (csv == null) { return; }
        csv.writeRow(time, stage, context.getLocalizer().isLastMeasurementValid(),
                raw.getX().getIn(), raw.getY().getIn(), raw.getHeading().getRad(),
                filtered.getX().getIn(), filtered.getY().getIn(), filtered.getHeading().getRad(),
                accel.getX().getIn(), accel.getY().getIn(), accel.getHeading().getRad());
    }

    private static String display(BaseLocalizer.VelocityFilterMode mode) {
        return mode == BaseLocalizer.VelocityFilterMode.ADAPTIVE_KALMAN
                ? "Unified adaptive Kalman" : "Moving average";
    }
}
