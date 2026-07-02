package org.firstinspires.ftc.teamcode.apexpathing.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.apexpathing.FollowerTuner;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import controllers.PDSController.PDSCoefficients;
import core.Follower;
import core.FollowerConstants;
import geometry.Angle;
import geometry.Dist;
import geometry.Pose;
import geometry.Vector;
import util.DistUnit;

public class TunerContext {
    private final LinearOpMode opMode;
    private final FollowerConstants followerConstants;
    private Follower follower;

    public final ElapsedTime timer = new ElapsedTime();

    public double headingP;
    public double headingD;
    public double headingS;
    public double translationP;
    public double translationD;
    public double translationS;
    public double velocityFF;
    public double maxLateralAccel = 40.0;
    public double headingToleranceDeg;
    public double distanceToleranceIn;
    public double tTolerance;

    public TunerContext(LinearOpMode opMode, FollowerConstants followerConstants) {
        this.opMode = opMode;
        this.followerConstants = followerConstants;
    }

    public void loadFrom(FollowerConstants defaults) {
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
    }

    public void setFollower(Follower follower) {
        this.follower = follower;
    }

    public Follower follower() {
        return follower;
    }

    public Telemetry telemetry() {
        return opMode.telemetry;
    }

    public boolean isActive() {
        return opMode.opModeIsActive() && !opMode.isStopRequested();
    }

    public void sleep(long milliseconds) throws InterruptedException {
        opMode.sleep(milliseconds);
    }

    public void resetPose() {
        follower.setPose(new Pose(
                new Vector(Dist.of(0, DistUnit.IN), Dist.of(0, DistUnit.IN)),
                Angle.fromDeg(0)
        ));
    }

    public void driveWithGamepad() {
        follower.teleOpDrive(-opMode.gamepad1.left_stick_x, opMode.gamepad1.left_stick_y, -opMode.gamepad1.right_stick_x);
    }

    public void stopDrive() {
        follower.teleOpDrive(0, 0, 0);
    }

    public double manualGuess() {
        return FollowerTuner.kSGuess;
    }

    public void setManualGuess(double value) {
        FollowerTuner.kSGuess = value;
    }

    public void updateFollowerConfig() {
        followerConstants.headingCoeffs = new PDSCoefficients(headingP, headingD, headingS, 0);
        followerConstants.driveCoeffs = new PDSCoefficients(translationP, translationD, translationS, 0);
        followerConstants.lateralCoeffs = new PDSCoefficients(translationP, translationD, translationS, 0);
        followerConstants.lateralKV = velocityFF;
        followerConstants.headingTolerance = Angle.fromDeg(headingToleranceDeg);
        followerConstants.distanceTolerance = Dist.fromIn(distanceToleranceIn);
        followerConstants.tTolerance = tTolerance;
        followerConstants.maxLateralAccel = maxLateralAccel;
    }

    public void saveConstantsToJson() {
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
        } catch (IOException ignored) {
            telemetry().addLine("WARNING: Values were not saved successfully");
            telemetry().addData("Heading P", headingP);
            telemetry().addData("Heading D", headingD);
            telemetry().addData("Heading S", headingS);
            telemetry().addData("Translation P", translationP);
            telemetry().addData("Translation D", translationD);
            telemetry().addData("Translation S", translationS);
            telemetry().addData("Velocity FF", velocityFF);
            telemetry().addData("Max Lateral Accel", maxLateralAccel);
            telemetry().update();
        }
    }
}
