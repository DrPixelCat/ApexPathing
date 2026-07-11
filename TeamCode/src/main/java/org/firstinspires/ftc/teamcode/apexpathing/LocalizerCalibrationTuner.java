package org.firstinspires.ftc.teamcode.apexpathing;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.json.JSONObject;

import java.io.File;
import java.io.FileWriter;

import geometry.Pose;
import localizers.BaseLocalizer;

/**
 * Sensor-independent, user-guided localizer calibration. It measures scale and axis sign without
 * creating a Follower, so it can run before follower constants exist. Results are recommendations:
 * apply them to the selected Pinpoint, OTOS, two-wheel, or three-wheel constants explicitly.
 */
@TeleOp(name = "Apex Localizer Calibration", group = "Apex Pathing Tuning")
public class LocalizerCalibrationTuner extends LinearOpMode {
    public static double knownLinearDistanceIn = 48.0;
    public static double knownTurns = 5.0;

    private enum Stage {
        CAPTURE_FORWARD_START, CAPTURE_FORWARD_END,
        CAPTURE_STRAFE_START, CAPTURE_STRAFE_END,
        CAPTURE_ROTATION_START, CAPTURE_ROTATION_END, COMPLETE
    }

    @Override
    public void runOpMode() {
        BaseLocalizer<?> localizer = new Constants().localizerConstants().build(hardwareMap);
        Stage stage = Stage.CAPTURE_FORWARD_START;
        Pose start = Pose.zero();
        double forwardScale = Double.NaN;
        double strafeScale = Double.NaN;
        double angularScale = Double.NaN;
        double forwardSign = 0.0;
        double strafeSign = 0.0;
        double angularSign = 0.0;
        double rotationDriftX = 0.0;
        double rotationDriftY = 0.0;
        double unwrappedHeading = 0.0;
        double previousHeading = 0.0;
        boolean lastA = false;
        boolean lastB = false;

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            localizer.update();
            Pose pose = localizer.getPose();
            boolean aPressed = gamepad1.a && !lastA;
            boolean bPressed = gamepad1.b && !lastB;

            if (stage == Stage.CAPTURE_ROTATION_END) {
                double currentHeading = pose.getHeading().getRad();
                unwrappedHeading += AngleUnit.normalizeRadians(currentHeading - previousHeading);
                previousHeading = currentHeading;
            }

            switch (stage) {
                case CAPTURE_FORWARD_START:
                    telemetry.addLine("Place the robot at the marked start, facing +X. Press A.");
                    if (aPressed) { start = pose; stage = Stage.CAPTURE_FORWARD_END; }
                    break;
                case CAPTURE_FORWARD_END:
                    telemetry.addLine("Push the robot straight forward exactly " + knownLinearDistanceIn + " in. Press A.");
                    if (aPressed) {
                        double measured = pose.getX().minus(start.getX()).getIn();
                        forwardScale = scale(knownLinearDistanceIn, measured);
                        forwardSign = Math.signum(measured);
                        stage = Stage.CAPTURE_STRAFE_START;
                    }
                    break;
                case CAPTURE_STRAFE_START:
                    telemetry.addLine("Place at a new mark. Press A for strafe test, or B to skip (tank).");
                    if (aPressed) { start = pose; stage = Stage.CAPTURE_STRAFE_END; }
                    if (bPressed) { stage = Stage.CAPTURE_ROTATION_START; }
                    break;
                case CAPTURE_STRAFE_END:
                    telemetry.addLine("Push left (+Y) exactly " + knownLinearDistanceIn + " in. Press A.");
                    if (aPressed) {
                        double measured = pose.getY().minus(start.getY()).getIn();
                        strafeScale = scale(knownLinearDistanceIn, measured);
                        strafeSign = Math.signum(measured);
                        stage = Stage.CAPTURE_ROTATION_START;
                    }
                    break;
                case CAPTURE_ROTATION_START:
                    telemetry.addLine("Mark the robot position. Press A, then rotate CCW exactly " + knownTurns + " turns.");
                    if (aPressed) {
                        start = pose;
                        unwrappedHeading = 0.0;
                        previousHeading = pose.getHeading().getRad();
                        stage = Stage.CAPTURE_ROTATION_END;
                    }
                    break;
                case CAPTURE_ROTATION_END:
                    telemetry.addLine("Rotate CCW exactly " + knownTurns + " turns, then press A.");
                    telemetry.addData("Tracked turns", unwrappedHeading / (2.0 * Math.PI));
                    if (aPressed) {
                        angularScale = scale(knownTurns * 2.0 * Math.PI, unwrappedHeading);
                        angularSign = Math.signum(unwrappedHeading);
                        rotationDriftX = pose.getX().minus(start.getX()).getIn();
                        rotationDriftY = pose.getY().minus(start.getY()).getIn();
                        save(forwardScale, strafeScale, angularScale, forwardSign, strafeSign,
                                angularSign, rotationDriftX, rotationDriftY);
                        stage = Stage.COMPLETE;
                    }
                    break;
                case COMPLETE:
                    telemetry.addLine("Calibration recommendations saved to FIRST/ApexPathing/localizer-calibration.json");
                    telemetry.addData("Forward multiplier", forwardScale);
                    telemetry.addData("Strafe multiplier", strafeScale);
                    telemetry.addData("Angular multiplier", angularScale);
                    telemetry.addData("Rotation drift X", rotationDriftX);
                    telemetry.addData("Rotation drift Y", rotationDriftY);
                    break;
            }

            telemetry.addData("Pose", pose);
            telemetry.update();
            lastA = gamepad1.a;
            lastB = gamepad1.b;
        }
    }

    private static double scale(double known, double measured) {
        return Math.abs(measured) < 1e-6 ? Double.NaN : known / Math.abs(measured);
    }

    private void save(double forwardScale, double strafeScale, double angularScale,
                      double forwardSign, double strafeSign, double angularSign,
                      double rotationDriftX, double rotationDriftY) {
        try {
            JSONObject json = new JSONObject();
            putFinite(json, "forwardScaleMultiplier", forwardScale);
            putFinite(json, "strafeScaleMultiplier", strafeScale);
            putFinite(json, "angularScaleMultiplier", angularScale);
            json.put("forwardAxisSign", forwardSign);
            json.put("strafeAxisSign", strafeSign);
            json.put("angularAxisSign", angularSign);
            json.put("rotationDriftXIn", rotationDriftX);
            json.put("rotationDriftYIn", rotationDriftY);
            File folder = new File(Environment.getExternalStorageDirectory(), "FIRST/ApexPathing");
            if (!folder.exists() && !folder.mkdirs()) throw new IllegalStateException("Cannot create output folder");
            FileWriter writer = new FileWriter(new File(folder, "localizer-calibration.json"));
            writer.write(json.toString(4));
            writer.close();
        } catch (Exception e) {
            telemetry.addLine("Could not save calibration: " + e.getMessage());
        }
    }

    private static void putFinite(JSONObject json, String key, double value) throws Exception {
        json.put(key, Double.isFinite(value) ? value : JSONObject.NULL);
    }
}
