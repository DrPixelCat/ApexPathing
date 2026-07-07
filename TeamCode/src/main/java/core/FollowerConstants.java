package core;

import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.Locale;

import controllers.PDSController.PDSCoefficients;
import geometry.Angle;
import geometry.Dist;

/**
 * Apex Pathing FollowerConstants class
 * Internally assigns the coefficient values determined through tuning directly,
 * thereby eliminating the need to manually tune and set the values in the Constants file!
 *
 * @author Sohum Arora 22985 Paraducks
 */
public class FollowerConstants {

    public enum DrivetrainType {
        COAXIAL_SWERVE,
        DUAL_ACTUATED,
        KIWI,
        MECANUM,
        TANK
    }

    public DrivetrainType drivetrainType = DrivetrainType.MECANUM;

    public PDSCoefficients headingCoeffs = new PDSCoefficients();
    public PDSCoefficients translationalCoeffs = new PDSCoefficients();
    public double velocityFeedbackGain = 0.0;
    public double translationalKV = 0.0, translationalKA = 0.0;
    public double angularKV = 0.0, angularKA = 0.0;
    public double Kcentripetal = 0.0;
    public Dist forwardVelocityLimit = Dist.fromIn(0);
    public Dist forwardAccelerationLimit = Dist.fromIn(0);
    public Dist strafeVelocityLimit = Dist.fromIn(0);
    public Dist strafeAccelerationLimit = Dist.fromIn(0);
    public Angle angularVelocityLimit = Angle.fromDeg(0);
    public Angle angularAccelerationLimit = Angle.fromDeg(0);
    public Angle headingTolerance = Angle.fromDeg(1.0);
    public Dist distanceTolerance = Dist.fromIn(0.5);

    public FollowerConstants() {
        loadValues();
    }

    private void loadValues() {
        File file = new File("/sdcard/FIRST/FollowerConstants.json");
        if (file.exists()) {
            try {
                BufferedReader reader = new BufferedReader(new FileReader(file));
                StringBuilder sb = new StringBuilder();
                String line;
                while ((line = reader.readLine()) != null) sb.append(line);
                reader.close();

                JSONObject json = new JSONObject(sb.toString());

                String dtString = json.optString("drivetrainType", null);
                if (dtString == null || dtString.equals("null") || dtString.trim().isEmpty()) {
                    throw new IllegalArgumentException("Missing drivetrain type!");
                }
                try {
                    this.drivetrainType = DrivetrainType.valueOf(dtString.toUpperCase(Locale.ROOT));
                } catch (IllegalArgumentException e) {
                    throw new IllegalArgumentException("Invalid drivetrain type: " + dtString);
                }

                headingCoeffs = new PDSCoefficients(
                        json.optDouble("headingP", 0),
                        json.optDouble("headingD", 0),
                        json.optDouble("headingS", 0), 0);

                double tP = json.optDouble("translationP", 0);
                double tD = json.optDouble("translationD", 0);
                double tS = json.optDouble("translationS", 0);
                translationalCoeffs = new PDSCoefficients(tP, tD, tS, 0);

                translationalKV = json.optDouble("translationKV", translationalKV);
                translationalKA = json.optDouble("translationKA", translationalKA);
                angularKV = json.optDouble("angularKV", angularKV);
                angularKA = json.optDouble("angularKA", angularKA);
                Kcentripetal = json.optDouble("KC", Kcentripetal);
                headingTolerance = Angle.fromDeg(json.optDouble("headingToleranceDeg", 1.0));
                distanceTolerance = Dist.fromIn(json.optDouble("distanceToleranceIn", 0.5));

                forwardVelocityLimit = Dist.fromIn(json.optDouble(
                        "forwardVelocityLimitInPerSec", 0));
                forwardAccelerationLimit = Dist.fromIn(json.optDouble(
                        "forwardVelocityLimitInPerSec2", 0));
                strafeVelocityLimit = Dist.fromIn(json.optDouble(
                        "strafeVelocityLimitInPerSec", 0));
                strafeAccelerationLimit = Dist.fromIn(json.optDouble(
                        "strafeAccelerationLimitInPerSec2", 0));
                angularVelocityLimit = Angle.fromDeg(json.optDouble(
                        "angularVelocityLimitRadPerSec", 0));
                angularAccelerationLimit = Angle.fromDeg(json.optDouble(
                        "angularAccelerationLimitRadPerSec2", 0));
            } catch (Exception ignored) {
                // defaults to 0 values everywhere
            }
        }
    }

    public FollowerConstants inject(
            DrivetrainType drivetrainType,
            PDSCoefficients headingCoeffs,
            PDSCoefficients translationalCoeffs,
            double velocityFeedbackGain,
            double translationalKV,
            double translationalKA,
            double angularKV,
            double angularKA,
            double Kcentripetal,
            Dist forwardVelocityLimit,
            Dist forwardAccelerationLimit,
            Dist strafeVelocityLimit,
            Dist strafeAccelerationLimit,
            Angle angularVelocityLimit,
            Angle angularAccelerationLimit,
            Angle headingTolerance,
            Dist distanceTolerance
    ) {
        this.drivetrainType = drivetrainType;
        this.headingCoeffs = headingCoeffs;
        this.translationalCoeffs = translationalCoeffs;
        this.velocityFeedbackGain = velocityFeedbackGain;
        this.translationalKV = translationalKV;
        this.translationalKA = translationalKA;
        this.angularKV = angularKV;
        this.angularKA = angularKA;
        this.Kcentripetal = Kcentripetal;
        this.forwardVelocityLimit = forwardVelocityLimit;
        this.forwardAccelerationLimit = forwardAccelerationLimit;
        this.strafeVelocityLimit = strafeVelocityLimit;
        this.strafeAccelerationLimit = strafeAccelerationLimit;
        this.angularVelocityLimit = angularVelocityLimit;
        this.angularAccelerationLimit = angularAccelerationLimit;
        this.headingTolerance = headingTolerance;
        this.distanceTolerance = distanceTolerance;
        return this;
    }

    public FollowerConstants getConstants() {
        return this;
    }
}