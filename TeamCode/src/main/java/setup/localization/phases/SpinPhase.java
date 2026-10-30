package setup.localization.phases;

import setup.AdjustmentStep;

import util.DeltaTime;

import setup.localization.calibration.CalibrationCandidate;
import setup.localization.calibration.CalibrationSnapshot;
import setup.localization.calibration.SpinTrial;

import setup.localization.LocalizationTunerContext;
import setup.localization.TuningPhase;

import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;

import java.util.Collections;

import paths.geometry.Angle;
import localizers.util.LowPassFilter;

/** Calibrates rotational geometry from five counterclockwise rotations. */
public final class SpinPhase extends TuningPhase {
    static final double TURN_VELOCITY_RAD_PER_SECOND = Math.toRadians(45);
    static final double POWER_RAMP_PER_SECOND = 0.03;
    private static final double MAX_TURN_POWER = 0.50;
    private static final double MIN_MANUAL_ANGLE_DEGREES = 180.0;
    private static final double TARGET_ROTATIONS = 5.0;
    private static final double TARGET_ANGLE_RADIANS = TARGET_ROTATIONS * 2.0 * Math.PI;

    private enum Stage { SPIN, SETTLE, ENTER_ANGLE, COMPLETE }

    private final RelativeImuAccumulator imuAngle = new RelativeImuAccumulator();
    private IMU imu;
    private boolean useImu = true;
    private CalibrationSnapshot start;
    private double reportedAngle;
    private double previousHeading;
    private double manualAngleDegrees;
    private final AdjustmentStep manualIncrementDegrees =
            new AdjustmentStep(15, 1, 90, 5);
    private double turnPower;
    private double measuredAngularVelocity;
    private final LowPassFilter angularVelocityFilter = new LowPassFilter();
    private boolean turnVelocityReached;
    private final DeltaTime timing = new DeltaTime();
    private long spinStarted;
    private Stage stage;
    private long settleStarted;
    private CalibrationCandidate candidate;

    public SpinPhase(LocalizationTunerContext context) { super(context); }
    @Override protected String getName() { return "Rotation geometry"; }

    @Override protected void reset() {
        candidate = null;
        manualAngleDegrees = 360.0;
        manualIncrementDegrees.reset();
        imuAngle.clearAxis();
        try { imu = opMode == null ? null : opMode.hardwareMap.get(IMU.class, context.getImuName()); }
        catch (RuntimeException ignored) { imu = null; }
        useImu = imu != null;
    }

    @Override protected void updatePrepare() {
        if ((opMode.gamepad1.dpadLeftWasPressed() || opMode.gamepad1.dpadRightWasPressed())
                && imu != null) {
            useImu = !useImu;
        }
    }

    @Override protected void showPrepare() {
        context.getView().addData("Reference", useImu ? "IMU" : "Manual");
        if (imu != null) {
            context.getView().addLine("Dpad Left/Right: change reference");
        }
        context.getView().addLine("Robot spins left at 45 deg/s for 5 rotations.");
        if (!useImu) { context.getView().addLine("Measure the total rotation."); }
        context.getView().addLine("A: begin");
    }

    @Override protected void beginRecording() {
        context.stop();
        start = context.getAdapter().snapshot(context.getLocalizer());
        previousHeading = start.pose.getHeading().getRad();
        reportedAngle = 0.0;
        turnPower = 0.0;
        measuredAngularVelocity = 0.0;
        turnVelocityReached = false;
        if (useImu) { imuAngle.resetMeasurement(readQuaternion()); }
        spinStarted = System.nanoTime();
        timing.reset(spinStarted);
        angularVelocityFilter.reset();
        stage = Stage.SPIN;
    }

    @Override protected boolean record() {
        updateReportedAngle();
        if (useImu && stage != Stage.ENTER_ANGLE) { imuAngle.update(readQuaternion()); }
        switch (stage) {
            case SPIN:
                spin();
                break;
            case SETTLE:
                context.getView().addLine("Stopping...");
                if ((System.nanoTime() - settleStarted) * 1e-9 >= 0.4) {
                    if (useImu) { finishTrial(imuAngle.getAngleRad()); stage = Stage.COMPLETE; }
                    else {
                        manualAngleDegrees = Math.max(MIN_MANUAL_ANGLE_DEGREES,
                                Math.abs(Math.toDegrees(reportedAngle)));
                        stage = Stage.ENTER_ANGLE;
                    }
                }
                break;
            case ENTER_ANGLE:
                editManualAngle();
                showAngleEntry();
                if (confirmPressed()) {
                    finishTrial(Math.toRadians(manualAngleDegrees));
                    stage = Stage.COMPLETE;
                }
                break;
            case COMPLETE: break;
        }
        return stage == Stage.COMPLETE;
    }

    private void spin() {
        long elapsed = System.nanoTime() - spinStarted;
        if (elapsed > 120_000_000_000L) {
            throw new IllegalStateException("Spin calibration timed out");
        }
        if (!context.getLocalizer().isLastMeasurementValid()) {
            throw new IllegalStateException("Spin calibration lost localization");
        }
        double measuredAngle = useImu ? imuAngle.getAngleRad() : reportedAngle;
        context.getView().addData("Rotations",
                context.formatNumber(Math.min(TARGET_ROTATIONS,
                        Math.abs(measuredAngle) / (2.0 * Math.PI)))
                        + " / " + context.formatNumber(TARGET_ROTATIONS));
        if (Math.abs(measuredAngle) >= TARGET_ANGLE_RADIANS) {
            context.stop();
            stage = Stage.SETTLE;
            settleStarted = System.nanoTime();
            return;
        }

        double dt = Math.min(0.10, Math.max(1e-3,
                timing.update(System.nanoTime())));
        measuredAngularVelocity = angularVelocityFilter.update(measuredAngle).rate();
        if (!turnVelocityReached) {
            if (Math.abs(measuredAngularVelocity) >= TURN_VELOCITY_RAD_PER_SECOND) {
                turnVelocityReached = true;
            } else {
                turnPower = rampTurnPower(turnPower, dt);
            }
        }
        context.getDrivetrain().moveWithVectors(0.0, 0.0, turnPower);

        context.getView().addData("Turn velocity",
                context.formatNumber(Math.toDegrees(measuredAngularVelocity))
                        + " / 45 deg/s");
        context.getView().addData("Turn power", context.formatNumber(turnPower));
        context.getView().addData("Power", turnVelocityReached ? "latched" : "ramping");
        return;
    }

    private void showAngleEntry() {
        context.getView().addData("Measured rotation",
                context.formatNumber(manualAngleDegrees) + " deg");
        context.getView().addData("Increment",
                context.formatNumber(manualIncrementDegrees.get()) + " deg");
        context.getView().addLine("Up/Down: edit   Left/Right: increment");
        context.getView().addLine("A: confirm");
    }

    @Override protected void showReview() {
        for (String metric : candidate.getMetrics()) { context.getView().addLine(metric); }
        if (useImu) {
            context.getView().addData("IMU axis", String.format(java.util.Locale.US,
                    "%.2f, %.2f, %.2f", imuAngle.getSpinAxisX(), imuAngle.getSpinAxisY(),
                    imuAngle.getSpinAxisZ()));
        }
    }

    @Override protected boolean accept() { return context.acceptGeometry("ROTATION", candidate); }

    private void finishTrial(double referenceAngle) {
        CalibrationSnapshot end = context.getAdapter().snapshot(context.getLocalizer());
        SpinTrial trial = new SpinTrial(start, end, referenceAngle, reportedAngle);
        candidate = context.getAdapter().fitSpin(context.getConfig(),
                Collections.singletonList(trial));
    }

    private void editManualAngle() {
        if (opMode.gamepad1.dpadLeftWasPressed()) {
            manualIncrementDegrees.finer();
        }
        if (opMode.gamepad1.dpadRightWasPressed()) {
            manualIncrementDegrees.coarser();
        }
        if (opMode.gamepad1.dpadUpWasPressed()) {
            manualAngleDegrees += manualIncrementDegrees.delta(1);
        }
        if (opMode.gamepad1.dpadDownWasPressed()) {
            manualAngleDegrees = Math.max(MIN_MANUAL_ANGLE_DEGREES,
                    manualAngleDegrees + manualIncrementDegrees.delta(-1));
        }
    }

    private Quaternion readQuaternion() {
        try { return imu.getRobotOrientationAsQuaternion(); }
        catch (RuntimeException failure) {
            throw new IllegalStateException("Device 'imu' stopped returning orientation", failure);
        }
    }

    private void updateReportedAngle() {
        double heading = context.getLocalizer().getRawPose().getHeading().getRad();
        reportedAngle += Angle.wrap(heading - previousHeading);
        previousHeading = heading;
    }

    /** Raises power at a fixed rate during the one-time velocity-discovery ramp. */
    static double rampTurnPower(double power, double dt) {
        return Math.max(0.0, Math.min(MAX_TURN_POWER,
                power + POWER_RAMP_PER_SECOND * Math.max(0.0, dt)));
    }

}
