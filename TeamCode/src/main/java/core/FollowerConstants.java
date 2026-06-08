package core;

import controllers.PDSController.PDSCoefficients;
import geometry.Angle;
import geometry.Dist;
import org.json.JSONObject;
import java.io.File;
import java.io.BufferedReader;
import java.io.FileReader;

/**
 * Apex Pathing FollowerConstants class
 * Internally assigns the coefficient values determined through tuning directly,
 * thereby eliminating the need to manually tune and set the values in the Constants file!
 * @author Sohum Arora 22985 Paraducks
 */
public class FollowerConstants {
    public PDSCoefficients headingCoeffs;
    public PDSCoefficients lateralCoeffs;
    public PDSCoefficients driveCoeffs;
    public PDSCoefficients velocityCoeffs;

    public double headingKV = 0.0, headingKA = 0.0;
    public double lateralKV = 0.0, lateralKA = 0.0;
    public Dist velocityLimit = null;

    public Angle headingTolerance;
    public Dist distanceTolerance;
    public double tTolerance;
    public double maxLateralAccel;

    public FollowerConstants() { //Values 0 to begin with
        this.headingCoeffs = new PDSCoefficients();
        this.driveCoeffs = new PDSCoefficients();
        this.lateralCoeffs = new PDSCoefficients();
        this.velocityCoeffs = new PDSCoefficients();
        this.headingTolerance = Angle.fromDeg(1.0);
        this.distanceTolerance = Dist.fromIn(0.5);
        this.tTolerance = 0.95;
        this.maxLateralAccel = 40.0;
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

                this.headingCoeffs = new PDSCoefficients(
                        json.optDouble("headingP", 0),
                        json.optDouble("headingD", 0),
                        json.optDouble("headingS", 0), 0);

                double tP = json.optDouble("translationP", 0);
                double tD = json.optDouble("translationD", 0);
                double tS = json.optDouble("translationS", 0);
                this.driveCoeffs = new PDSCoefficients(tP, tD, tS, 0);
                this.lateralCoeffs = new PDSCoefficients(tP, tD, tS, 0);

                this.lateralKV = json.optDouble("velocityFF", this.lateralKV);
                this.maxLateralAccel = json.optDouble("maxLateralAccel", this.maxLateralAccel);
                this.headingTolerance = Angle.fromDeg(json.optDouble("headingToleranceDeg", 1.0));
                this.distanceTolerance = Dist.fromIn(json.optDouble("distanceToleranceIn", 0.5));
                this.tTolerance = json.optDouble("tTolerance", 0.95);

            } catch (Exception ignored) {
                //defaults to 0 values everywhere
            }
        }
    }
    public FollowerConstants loadFromJson() {
        return this;
    }
}