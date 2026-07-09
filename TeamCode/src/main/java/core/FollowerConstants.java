package core;

import android.os.Environment;

import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;

import controllers.PDSController.PDSCoefficients;
import geometry.Angle;
import geometry.Dist;

/**
 * Class to hold constants for the Follower class. These constants are loaded from a JSON file
 * created by the tuners. If the file does not exist or cannot be read, default values will be used.
 *
 * @author Sohum Arora 22985 Paraducks
 * @author Dylan B. 18597 RoboClovers - Delta
 * @author DrPixelCat
 */
public class FollowerConstants {
    /*
     * Note to developers:
     * If you want to add new constants, create the variable here and add it to the loadValues() and
     * toJson() methods. This will ensure that the new constants are loaded from the JSON file and
     * saved back to it.
     */
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
    public Angle angularVelocityLimit = Angle.fromRad(0);
    public Angle angularAccelerationLimit = Angle.fromRad(0);

    public FollowerConstants() {
        loadValues();
    }

    private double loadDouble(JSONObject json, String key) {
        try {
            return json.getDouble(key);
        } catch (Exception e) {
            return 0.0;
        }
    }

    private void loadValues() {
        File file = new File(
                Environment.getExternalStorageDirectory().getPath() +
                        "/FIRST/ApexPathing/constants.json"
        );
        if (file.exists()) {
            JSONObject json;
            try {
                BufferedReader reader = new BufferedReader(new FileReader(file));
                StringBuilder sb = new StringBuilder();
                String line;
                while ((line = reader.readLine()) != null) sb.append(line);
                reader.close();
                json = new JSONObject(sb.toString());
            } catch (Exception e) {
                return;
            }

            headingCoeffs.setkP(loadDouble(json, "headingP"));
            headingCoeffs.setkD(loadDouble(json, "headingD"));
            headingCoeffs.setkS((loadDouble(json, "headingS")));

            translationalCoeffs.setkP(loadDouble(json, "translationalP"));
            translationalCoeffs.setkD(loadDouble(json, "translationalD"));
            translationalCoeffs.setkS(loadDouble(json, "translationalS"));

            translationalKV = loadDouble(json, "translationKV");
            translationalKA = loadDouble(json, "translationKA");
            angularKV = loadDouble(json, "angularKV");
            angularKA = loadDouble(json, "angularKA");
            Kcentripetal = loadDouble(json, "Kcentripetal");

            forwardVelocityLimit = Dist.fromIn(loadDouble(
                    json, "forwardVelocityLimitInPerSec"));
            forwardAccelerationLimit = Dist.fromIn(loadDouble(
                    json, "forwardVelocityLimitInPerSec2"));
            strafeVelocityLimit = Dist.fromIn(loadDouble(
                    json, "strafeVelocityLimitInPerSec"));
            strafeAccelerationLimit = Dist.fromIn(loadDouble(
                    json, "strafeAccelerationLimitInPerSec2"));
            angularVelocityLimit = Angle.fromRad(loadDouble(
                    json, "angularVelocityLimitRadPerSec"));
            angularAccelerationLimit = Angle.fromRad(loadDouble(
                    json, "angularAccelerationLimitRadPerSec2"));
        }
    }

    public JSONObject toJson() {
        JSONObject json = new JSONObject();
        try {
            json.put("headingP", headingCoeffs.kP);
            json.put("headingD", headingCoeffs.kD);
            json.put("headingS", headingCoeffs.kS);

            json.put("translationalP", translationalCoeffs.kP);
            json.put("translationalD", translationalCoeffs.kD);
            json.put("translationalS", translationalCoeffs.kS);

            json.put("translationKV", translationalKV);
            json.put("translationKA", translationalKA);
            json.put("angularKV", angularKV);
            json.put("angularKA", angularKA);
            json.put("Kcentripetal", Kcentripetal);

            json.put("forwardVelocityLimitInPerSec", forwardVelocityLimit.getIn());
            json.put("forwardVelocityLimitInPerSec2", forwardAccelerationLimit.getIn());
            json.put("strafeVelocityLimitInPerSec", strafeVelocityLimit.getIn());
            json.put("strafeAccelerationLimitInPerSec2", strafeAccelerationLimit.getIn());
            json.put("angularVelocityLimitRadPerSec", angularVelocityLimit.getRad());
            json.put("angularAccelerationLimitRadPerSec2", angularAccelerationLimit.getRad());
        } catch (Exception e) {
            // Handle exception if needed
        }
        return json;
    }
}