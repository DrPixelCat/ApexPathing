package org.firstinspires.ftc.teamcode.apexpathing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;
import java.util.List;

import drivetrains.BaseDrivetrain;
import follower.FollowerConstants;
import localizers.BaseLocalizer;
import localizers.LocalizerSet;
import paths.geometry.AngleUnit;
import paths.geometry.Pose;
import paths.geometry.Vector;
import setup.logging.TuningCsvWriter;

@TeleOp(name = "Velocity Test", group = "Apex Pathing")
public class VelocityTest extends OpMode {
    private static final double RAMP_RATE = .1;
    private static final double MAX_RAMP_POWER = .9;
    private static final double[] STEP_POWERS = {.35, .45, .55, .70, .85, 1.0};
    private static final double STEP_SECONDS = 2;
    private int selectedPower, direction = 1, captureDirection;
    private boolean stepCapture, releaseRequired, previousLeft, previousRight, previousX;
    private double capturePower;
    private static final String[] CSV_HEADER = {"time_s", "applied_forward_power",
            "filtered_forward_velocity_in_s", "battery_voltage_v", "acceleration_in_s2",
            "acceleration_window_time_s", "acceleration_window_velocity_in_s", "acceleration_window_power"};
    private BaseDrivetrain<?> drive;
    private BaseLocalizer<?> localizer;
    private final List<Object[]> samples = new ArrayList<>();
    private boolean recording;
    private long captureStartNanos;
    private int captureNumber, sampleCount;
    private String csvStatus = "No capture saved";

    @Override
    public void init() {
        Constants robot = new Constants();
        if (robot.robotProfile() == null) {
            throw new IllegalStateException("No saved robot setup. Run Robot Setup first.");
        }
        drive = robot.drivetrainConstants().build(hardwareMap);
        localizer = new LocalizerSet(robot, hardwareMap, drive.getDrivetrainType(),
                drive.isHolonomic() ? FollowerConstants.Profile.HOLONOMIC : FollowerConstants.Profile.TANK)
                .getLocalizer();
    }

    @Override
    public void loop() {
        localizer.update();
        boolean left = gamepad1.dpad_left, right = gamepad1.dpad_right, reverse = gamepad1.x;
        if (!recording) {
            if (left && !previousLeft) { selectedPower = Math.max(0, selectedPower - 1); }
            if (right && !previousRight) { selectedPower = Math.min(STEP_POWERS.length - 1, selectedPower + 1); }
            if (reverse && !previousX) { direction = -direction; }
        }
        previousLeft = left; previousRight = right; previousX = reverse;
        if (!gamepad1.a && !gamepad1.b) { releaseRequired = false; }
        double precedingPower = forwardPower();
        long now = System.nanoTime();
        if (!recording && !releaseRequired && (gamepad1.a || gamepad1.b)) {
            // Steps begin from rest so different powers cross comparable velocity ranges.
            if (gamepad1.b && (!localizer.isLastMeasurementValid()
                    || localizer.getVel().getVec().getMag().getIn() > .5
                    || Math.abs(localizer.getVel().getHeading(AngleUnit.RAD)) > .05)) {
                drive.stop();
                releaseRequired = true;
                csvStatus = "Wait until stationary, then press B again.";
            } else {
                samples.clear(); sampleCount = 0; captureNumber++;
                captureStartNanos = now;
                stepCapture = gamepad1.b;
                captureDirection = direction;
                capturePower = STEP_POWERS[selectedPower];
                recording = true;
            }
        }
        if (recording) {
            double seconds = (now - captureStartNanos) * 1e-9;
            boolean held = stepCapture ? gamepad1.b : gamepad1.a;
            if (!held || stepCapture && seconds >= STEP_SECONDS) {
                drive.stop();
                recordSample(seconds, precedingPower);
                saveCapture();
                releaseRequired = gamepad1.a || gamepad1.b;
            } else {
                double power = captureDirection * (stepCapture ? capturePower : Math.min(MAX_RAMP_POWER, RAMP_RATE * seconds));
                drive.moveWithVectors(power, 0, 0);
                recordSample(seconds, precedingPower);
            }
        } else if (releaseRequired) {
            drive.stop();
        } else {
            drive.drive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x,
                    localizer.getPose().getHeading().getRad());
        }

        Pose pose = localizer.getPose();
        Pose velocity = localizer.getVel();
        Vector robotVelocity = velocity.getVec().rotate(pose.getHeading().times(-1));
        telemetry.addData("Capture", recording ? (stepCapture ? "Step" : "Ramp") : "Idle");
        telemetry.addData("Step power", "%.2f", direction * STEP_POWERS[selectedPower]);
        telemetry.addData("Power / velocity (in/s)", "%.2f / %.2f", forwardPower(), robotVelocity.getX().getIn());
        if (!recording) {
            telemetry.addLine("Hold A: ramp | B: step (max 2s) | Release: stop/save");
            telemetry.addLine("Dpad: power | X: reverse | Sticks: drive");
        }
        telemetry.addData("CSV", csvStatus);
        telemetry.update();
    }

    private double forwardPower() {
        return (drive.getLastFlPower() + drive.getLastFrPower()
                + drive.getLastBlPower() + drive.getLastBrPower()) / 4;
    }

    /** Keep acquisition free of file I/O; measurements describe the preceding motor command. */
    private void recordSample(double seconds, double precedingPower) {
        if (!localizer.isLastMeasurementValid()) { return; }
        Vector filteredRobot = localizer.getVel().getVec()
                .rotate(localizer.getPose().getHeading().times(-1));
        double voltage = Double.POSITIVE_INFINITY;
        for (com.qualcomm.robotcore.hardware.VoltageSensor sensor : hardwareMap.voltageSensor) {
            double reading = sensor.getVoltage();
            if (Double.isFinite(reading) && reading > 0) { voltage = Math.min(voltage, reading); }
        }
        samples.add(new Object[]{seconds, precedingPower, filteredRobot.getX().getIn(),
                Double.isFinite(voltage) ? voltage : "", "", "", "", ""});
        sampleCount = samples.size();
    }

    /** Save one file per hold, after the motors have stopped. */
    private void saveCapture() {
        recording = false;
        TuningCsvWriter csv = TuningCsvWriter.openBuffered((stepCapture ? "velocity_step_" : "velocity_ramp_")
                + captureDirection + "_" + captureNumber, CSV_HEADER);
        try {
            for (int i = 0; i < samples.size(); i++) {
                addAcceleration(i);
                csv.writeRow(samples.get(i));
                // Bound the writer's string buffer even for a long capture.
                if ((i + 1) % 200 == 0) { csv.flush(); }
            }
        } finally {
            csv.close();
        }
        csvStatus = csv.getError() == null ? csv.getPath() : "CSV error: " + csv.getError();
        samples.clear();
    }

    /** Fit a centered 0.1-second velocity window; keep its matching time, speed and power. */
    private void addAcceleration(int index) {
        Object[] row = samples.get(index);
        double center = (double) row[0], time = 0, velocity = 0, power = 0;
        int first = index, last = index;
        while (first > 0 && center - (double) samples.get(first - 1)[0] <= .05) { first--; }
        while (last + 1 < samples.size() && (double) samples.get(last + 1)[0] - center <= .05) { last++; }
        if (last - first < 3 || center - (double) samples.get(first)[0] < .03
                || (double) samples.get(last)[0] - center < .03) { return; }
        for (int i = first; i <= last; i++) {
            Object[] sample = samples.get(i);
            if (stepCapture && Math.abs((double) sample[1] - (double) row[1]) > 1e-6) { return; }
            if (i > first && (double) sample[0] - (double) samples.get(i - 1)[0] > .05) { return; }
            time += (double) sample[0]; velocity += (double) sample[2]; power += (double) sample[1];
        }
        int count = last - first + 1;
        time /= count; velocity /= count; power /= count;
        double variance = 0, covariance = 0;
        for (int i = first; i <= last; i++) {
            Object[] sample = samples.get(i);
            double offset = (double) sample[0] - time;
            variance += offset * offset;
            covariance += offset * ((double) sample[2] - velocity);
        }
        row[4] = covariance / variance;
        row[5] = time; row[6] = velocity; row[7] = power;
    }

    @Override
    public void stop() {
        if (drive != null) { drive.stop(); }
        if (recording) { saveCapture(); }
    }
}
