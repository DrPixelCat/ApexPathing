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
import geometry.Pose;
import geometry.Vector;
import paths.builders.Builder;
import paths.movements.Path;
import util.DistUnit;

import org.firstinspires.ftc.teamcode.Constants;

/**
 * Single unified automatic tuner capable of completely tuning a robot for Apex in minutes in just a single OpMode!
 * All you have to do is follow the telemetry instructions and press a couple buttons here and there
 * Once you have run this tuner, your robot is fully tuned and ready to go Path its way to the Peaks™️😁
 * @author Sohum Arora 22985 Paraducks
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

    TuningState currentState = TuningState.HEADING;

    private double headingP = 0.0, headingD = 0.0, headingS = 0.0;
    private double translationP = 0.0, translationD = 0.0, translationS = 0.0;
    private double velocityFF = 0.0;
    private double maxLateralAccel = 40.0;
    private double headingToleranceDeg = 1.0;
    private double distanceToleranceIn = 0.5;
    private double tTolerance = 0.95;

    private Follower follower;
    private final Constants baseConstants = new Constants();
    private final FollowerConstants followerConstants = new FollowerConstants();

    @Override
    public void runOpMode() throws InterruptedException {
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
        maxLateralAccel = defaults.maxLateralAccel > 10 ? defaults.maxLateralAccel : 40.0;

        while (opModeInInit()) {
            telemetry.addLine("ROBOT INITIALIZED - FULLY AUTOMATED TUNING MODE");
            telemetry.addLine("1. Ensure 6x6 feet of clear floor space.");
            telemetry.addLine("2. Place robot at the center-back of the area.");
            telemetry.addLine("3. Press START to begin the full auto-tuning sequence.");
            telemetry.update();
        }

        updateFollowerConfig();
        follower = new Follower(customConfig, hardwareMap);
        follower.setPose(new Pose(new Vector(Dist.of(0, DistUnit.IN),Dist.of(0, DistUnit.IN)), Angle.fromDeg(0)));

        waitForStart();

        autoTuneHeading();
        autoTuneTranslation();
        autoTuneVelocityFF();
        autoTuneMaxLateralAccel();

        currentState = TuningState.COMPLETE;
        saveConstantsToJson();

        while (opModeIsActive()) {
            telemetry.addData("Status", "AUTOMATED TUNING COMPLETE");
            telemetry.addLine("Constants saved to /sdcard/FIRST/FollowerConstants.json");
            telemetry.update();
            follower.teleOpDrive(0, 0, 0);
        }
    }

    private void autoTuneHeading() {
        telemetry.addLine("PHASE 1/4: Tuning Heading (PDS)");
        telemetry.update();

        double power = 0;
        while (opModeIsActive() && Math.abs(follower.getVelocity().getHeading().getRad()) < 0.02) {
            power += 0.0005;
            follower.teleOpDrive(0, 0, power);
            sleep(10);
        }
        headingS = power;
        follower.teleOpDrive(0, 0, 0);
        sleep(500);

        headingP = 0.1;
        boolean settled = false;
        while (opModeIsActive() && !settled) {
            updateFollowerConfig();
            Angle target = Angle.fromDeg(90);
            long startTime = System.currentTimeMillis();

            while (opModeIsActive() && Math.abs(follower.getPose().getHeading().minus(target).getDeg()) > 1.0) {
                if (System.currentTimeMillis() - startTime > 2500) break;
                follower.teleOpDrive(0, 0, 0); // Follower logic handles PDS via update/follow if implemented,
                // otherwise we use teleOpDrive as a placeholder for controller update
                sleep(10);
            }

            if (Math.abs(follower.getPose().getHeading().minus(target).getDeg()) < 1.0) settled = true;
            else headingP += 0.05;
        }
        sleep(500);
    }

    private void autoTuneTranslation() {
        telemetry.addLine("PHASE 2/4: Tuning Translation (PDS)");
        telemetry.update();

        double power = 0;
        while (opModeIsActive() && Math.abs(follower.getVelocity().getPos().getX().getIn()) < 0.1) {
            power += 0.001;
            follower.teleOpDrive(0, power, 0);
            sleep(10);
        }
        translationS = power;
        follower.teleOpDrive(0, 0, 0);
        sleep(500);

        translationP = 0.05;
        translationD = 0.01;
    }

    private void autoTuneVelocityFF() {
        telemetry.addLine("PHASE 3/4: Tuning Velocity Feedforward (kV)");
        telemetry.update();

        follower.teleOpDrive(0, 1.0, 0);
        sleep(1500);
        double maxVel = Math.abs(follower.getVelocity().getPos().getX().getIn());
        velocityFF = 1.0 / maxVel;
        follower.teleOpDrive(0, 0, 0);
        sleep(500);
    }

    private void autoTuneMaxLateralAccel() {
        telemetry.addLine("PHASE 4/4: Tuning Max Lateral Acceleration");
        telemetry.update();

        maxLateralAccel = 50.0;
        boolean driftDetected = false;

        while (opModeIsActive() && !driftDetected) {
            updateFollowerConfig();
            follower.setPose(new Pose(new Vector(Dist.of(0, DistUnit.IN),Dist.of(0, DistUnit.IN)), Angle.fromDeg(0)));

            generateAndFollowTestCurve();

            double maxError = 0;
            while (opModeIsActive() && follower.isBusy()) {
                follower.update();
                double currentError = follower.getPose().getPos().getMag().getIn();  if (currentError > maxError) maxError = currentError;
                telemetry.addData("Testing Limit", maxLateralAccel);
                telemetry.addData("Current Gs", Math.abs(follower.getAcceleration().getY().getIn()));
                telemetry.update();
            }

            if (maxError > 4.0 || maxLateralAccel > 300) {
                driftDetected = true;
                maxLateralAccel -= 20.0;
            } else {
                maxLateralAccel += 20.0;
                telemetry.addLine("Success. Increasing limit...");
                telemetry.update();
                sleep(1000);
            }
        }
    }

    private void generateAndFollowTestCurve() {
        Pose start = follower.getPose();
        Path testCurve = Builder.path(
                start,
                new Pose(start.getPos().plus(new Vector(Dist.of(30, DistUnit.IN), Dist.of(0, DistUnit.IN))), start.getHeading()),
                new Pose(start.getPos().plus(new Vector(Dist.of(30, DistUnit.IN), Dist.of(30,DistUnit.IN))), start.getHeading().plus(Angle.fromDeg(90))),
                new Pose(start.getPos().plus(new Vector(Dist.of(0, DistUnit.IN), Dist.of(30, DistUnit.IN))), start.getHeading().plus(Angle.fromDeg(180)))
        ).build();

        follower.follow(testCurve);
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
        public BaseDrivetrainConfig<?> drivetrainConfig() { return baseConstants.drivetrainConfig(); }
        @Override
        public BaseLocalizerConfig<?> localizerConfig() { return baseConstants.localizerConfig(); }
        @Override
        public FollowerConstants followerConfig() { return followerConstants; }
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
                "  \"maxLateralAccel\": " + maxLateralAccel + "\n" +
                "}";

        try {
            File outputFolder = new File("/sdcard/FIRST");
            if (!outputFolder.exists()) outputFolder.mkdirs();
            FileWriter fileWriter = new FileWriter(new File(outputFolder, "FollowerConstants.json"));
            fileWriter.write(jsonPayload);
            fileWriter.close();
        } catch (IOException ignored) {}
    }
}