package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import core.ApexConfig;
import core.Follower;
import core.FollowerConstants;
import controllers.PDSController.PDSCoefficients;
import drivetrains.BaseDrivetrainConfig;
import localizers.BaseLocalizerConfig;
import geometry.Angle;
import geometry.Dist;
import org.firstinspires.ftc.teamcode.Constants;

/**
 * Single unified automatic tuner capable of completely tuning a robot for Apex in minutes in just a single OpMode!
 * All you have to do is follow the telemetry instructions and press a couple buttons here and there
 * Once you have run this tuner, your robot is fully tuned and ready to go Path its way to the Peaks™️😁
 */
@TeleOp(name = "Follower Tuner", group = "Apex Pathing Tuning")
public class FollowerTuner extends LinearOpMode {
    enum TuningState {
        HEADING,
        TRANSLATION,
        VELOCITY_FF,
        LATERAL_ACCEL,
        COMPLETE
    }

    private TuningState currentState = TuningState.HEADING;
    private int phase = 1;

    private boolean headingRun = false, translationRun = false, velocityRun = false,
    accelRun = false;
    private double headingP = 0.0, headingD = 0.0, headingS = 0.0;
    private double translationP = 0.0, translationD = 0.0, translationS = 0.0;
    private double velocityFF = 0.0;
    private double maxLateralAccel = 0.0;
    private double headingToleranceDeg = 1.0;
    private double distanceToleranceIn = 0.5;
    private double tTolerance = 0.95;
    private Follower follower;
    private final Constants baseConstants = new Constants();
    private final FollowerConstants followerConstants = new FollowerConstants();

    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastA = false;
    private int activeCoeffIndex = 0;
    private final String[] coeffNames = {"kP", "kD", "kS"};

    @Override
    public void runOpMode() throws InterruptedException {
        phase = 1;

        FollowerConstants defaults = baseConstants.followerConfig().loadFromJson();

        headingP = defaults.headingCoeffs.kP;
        headingD = defaults.headingCoeffs.kD;
        headingS = defaults.headingCoeffs.kS;

        translationP = defaults.driveCoeffs.kP;
        translationD = defaults.driveCoeffs.kD;
        translationS = defaults.driveCoeffs.kS;

        velocityFF = defaults.lateralKV;
        headingToleranceDeg = defaults.headingTolerance.getDeg();
        distanceToleranceIn = defaults.distanceTolerance.getIn();
        tTolerance = defaults.tTolerance;
        maxLateralAccel = defaults.maxLateralAccel;

        while (opModeInInit()) {
            telemetry.addLine("Robot Initialized");
            telemetry.addLine("Tuning order:\n 1) Heading PDS \n 2) Translation PDS \n 3) Velocity FF \n 4) Max Lateral Accel");
            telemetry.addLine("Run the OpMode to proceed with the Heading Tuner");
            telemetry.addLine("Press 'A' (cross) to directly run the Translation Tuner if you have already run the Heading Tuner");
            telemetry.addLine("Press 'B' (circle) to directly run the Velocity FF Tuner if you have already run the Heading Tuner and Translation Tuner");
            telemetry.addLine("Once all of these 3 tuners are complete, ");
            telemetry.addLine("IMPORTANT: Do NOT run the tuners out of order");

            if (gamepad1.a) {
                phase = 2;
                headingRun = true;
            } else if (gamepad1.b) {
                phase = 3;
                headingRun = true;
                translationRun = true;
            }

            switch (phase) {
                case 1:
                    currentState = TuningState.HEADING;
                    break;
                case 2:
                    currentState = TuningState.TRANSLATION;
                    break;
                case 3:
                    currentState = TuningState.VELOCITY_FF;
                    break;
            }

            telemetry.addData("Selected Phase", phase);
            telemetry.addData("Configured Initial State", currentState.name());
            telemetry.update();
        }

        updateFollowerConfig();
        follower = new Follower(customConfig, hardwareMap);

        waitForStart();

        while (opModeIsActive() && currentState != TuningState.COMPLETE && !isStopRequested()) {
            switch (phase) {
                case 1:
                    currentState = TuningState.HEADING;
                    break;
                case 2:
                    currentState = TuningState.TRANSLATION;
                    break;
                case 3:
                    currentState = TuningState.VELOCITY_FF; break;
                case 4:
                    currentState = TuningState.LATERAL_ACCEL;
                    break;
            }

            runTuner(currentState);

            if (phase < 4) {
                phase++;
            } else {
                accelRun = true;
            }

            if (headingRun && translationRun && velocityRun && accelRun) {
                currentState = TuningState.COMPLETE;
            }
        }

        saveConstantsToJson();

        while (opModeIsActive()) {
            telemetry.addData("Status", "All Tuning Cycles Complete! Configuration Saved to JSON.");
            telemetry.update();
            follower.teleOpDrive(0, 0, 0);
        }
    }

    private void runTuner(TuningState state) {
        switch (state) {
            case HEADING: headingRun = true; break;
            case TRANSLATION: translationRun = true; break;
            case VELOCITY_FF: velocityRun = true; break;
            case LATERAL_ACCEL: accelRun = true; break;
        }

        activeCoeffIndex = 0; // Reset active param to P when switching states
        verifyValues();

        follower.teleOpDrive(0, 0, 0);
        sleep(1000);
    }

    private void verifyValues() {
        while (opModeIsActive()) {
            telemetry.addData("Current Tuning State", currentState.name());
            telemetry.addLine("Press 'A' (cross) to ACCEPT values & advance to next tuning phase.");
            telemetry.addLine();

            if (currentState == TuningState.HEADING || currentState == TuningState.TRANSLATION) {
                telemetry.addLine("Use D-Pad UP/DOWN to select P/D/S.");
                if (gamepad1.dpad_up && !lastDpadUp) activeCoeffIndex = Math.max(0, activeCoeffIndex - 1);
                if (gamepad1.dpad_down && !lastDpadDown) activeCoeffIndex = Math.min(2, activeCoeffIndex + 1);
                telemetry.addData(">>> Editing", coeffNames[activeCoeffIndex] + " <<<");
            }

            lastDpadUp = gamepad1.dpad_up;
            lastDpadDown = gamepad1.dpad_down;

            telemetry.addLine("Use D-Pad LEFT/RIGHT to increment/decrement active target.");
            telemetry.addLine("Hold RIGHT BUMPER to increase adjustment step from 0.005 to 0.05.");

            double adjustStep = gamepad1.right_bumper ? 0.05 : 0.005;

            if (gamepad1.dpad_right && !lastDpadRight) modifyActiveParameter(adjustStep);
            lastDpadRight = gamepad1.dpad_right;

            if (gamepad1.dpad_left && !lastDpadLeft) modifyActiveParameter(-adjustStep);
            lastDpadLeft = gamepad1.dpad_left;

            updateFollowerConfig();

            if (currentState == TuningState.LATERAL_ACCEL) {
                telemetry.addLine();
                telemetry.addLine("Hold 'X' to perform a high-speed curve to measure peak lateral acceleration.");
                if (gamepad1.x) {
                    follower.teleOpDrive(0, 1.0, 1.0);
                    double empiricalAccel = Math.abs(follower.getAcceleration().getY().getIn());
                    if (empiricalAccel > maxLateralAccel) {
                        maxLateralAccel = empiricalAccel;
                    }
                } else {
                    follower.teleOpDrive(-gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x);
                }
            } else {
                follower.teleOpDrive(-gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x);
            }

            renderTunerTelemetry();

            if (gamepad1.a && !lastA) {
                lastA = true;
                break;
            }
            lastA = gamepad1.a;
        }
    }

    private void modifyActiveParameter(double amount) {
        switch (currentState) {
            case HEADING:
                if (activeCoeffIndex == 0) headingP = Math.max(0, headingP + amount);
                else if (activeCoeffIndex == 1) headingD = Math.max(0, headingD + amount);
                else headingS = Math.max(0, headingS + amount);
                break;
            case TRANSLATION:
                if (activeCoeffIndex == 0) translationP = Math.max(0, translationP + amount);
                else if (activeCoeffIndex == 1) translationD = Math.max(0, translationD + amount);
                else translationS = Math.max(0, translationS + amount);
                break;
            case VELOCITY_FF:
                velocityFF = Math.max(0, velocityFF + amount);
                break;
            case LATERAL_ACCEL:
                maxLateralAccel = Math.max(0, maxLateralAccel + (amount * 20.0));
                break;
        }
    }

    private void renderTunerTelemetry() {
        if (currentState == TuningState.HEADING) {
            telemetry.addData("Heading P", headingP);
            telemetry.addData("Heading D", headingD);
            telemetry.addData("Heading S", headingS);
        } else if (currentState == TuningState.TRANSLATION) {
            telemetry.addData("Translation P", translationP);
            telemetry.addData("Translation D", translationD);
            telemetry.addData("Translation S", translationS);
        } else {
            telemetry.addData("Live Velocity FF (kV)", velocityFF);
            telemetry.addData("Calculated Max Lateral Accel", maxLateralAccel);
        }
        telemetry.addData("Robot Pose Components", follower.getPose().toString());
        telemetry.addData("Live Lateral Acceleration", Math.abs(follower.getAcceleration().getY().getIn()));
        telemetry.update();
    }

    private void updateFollowerConfig() {
        followerConstants.headingCoeffs = new PDSCoefficients(headingP, headingD, headingS, 0);
        followerConstants.driveCoeffs = new PDSCoefficients(translationP, translationD, translationS, 0);
        followerConstants.lateralCoeffs = new PDSCoefficients(translationP, translationD, translationS, 0);
        followerConstants.lateralKV = velocityFF;
        followerConstants.headingTolerance = Angle.fromDeg(headingToleranceDeg);
        followerConstants.distanceTolerance = Dist.fromIn(distanceToleranceIn);
        followerConstants.tTolerance = tTolerance;
        followerConstants.maxLateralAccel = maxLateralAccel;
    }

    private final ApexConfig customConfig = new ApexConfig() {
        @Override
        public BaseDrivetrainConfig<?> drivetrainConfig() {
            return baseConstants.drivetrainConfig();
        }

        @Override
        public BaseLocalizerConfig<?> localizerConfig() {
            return baseConstants.localizerConfig();
        }

        @Override
        public FollowerConstants followerConfig() {
            return followerConstants;
        }
    };

    private void saveConstantsToJson() {
        String jsonPayload = "{\n" +
                "  \"headingP\": " + headingP + ",\n" +
                "  \"headingD\": " + headingD + ",\n" +
                "  \"headingS\": " + headingS + ",\n" +
                "  \"translationP\": " + translationP + ",\n" +
                "  \"translationD\": " + translationD + ",\n" +
                "  \"translationS\": " + translationS + ",\n" +
                "  \"velocityFF\": " + velocityFF + ",\n" +
                "  \"headingToleranceDeg\": " + headingToleranceDeg + ",\n" +
                "  \"distanceToleranceIn\": " + distanceToleranceIn + ",\n" +
                "  \"tTolerance\": " + tTolerance + ",\n" +
                "  \"maxLateralAccel\": " + maxLateralAccel + "\n" +
                "}";

        try {
            File outputFolder = new File("/sdcard/FIRST");
            if (!outputFolder.exists()) {
                outputFolder.mkdirs();
            }
            File constantsFile = new File(outputFolder, "FollowerConstants.json");
            FileWriter fileWriter = new FileWriter(constantsFile);
            fileWriter.write(jsonPayload);
            fileWriter.close();
            telemetry.addLine("SUCCESS: Calibration file exported to disk folder.");
        } catch (IOException e) {
            telemetry.addLine("FATAL ERROR: Local flash systems rejected file write command.");
        }
        telemetry.update();
    }
}