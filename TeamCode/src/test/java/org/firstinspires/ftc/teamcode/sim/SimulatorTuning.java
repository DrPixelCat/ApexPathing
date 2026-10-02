package org.firstinspires.ftc.teamcode.sim;

import core.FollowerConstants;
import org.json.JSONObject;
import java.io.*;
import java.nio.charset.StandardCharsets;

/** Versioned simulator calibration; never substitutes for tuning a physical robot. */
final class SimulatorTuning {
    private SimulatorTuning() { }

    static FollowerConstants snapshot() {
        InputStream stream = SimulatorTuning.class.getResourceAsStream("/simulator/mecanum-alpha1.json");
        if (stream == null) throw new IllegalStateException("Missing simulator calibration");
        try (BufferedReader reader = new BufferedReader(new InputStreamReader(stream, StandardCharsets.UTF_8))) {
            StringBuilder json = new StringBuilder();
            String line;
            while ((line = reader.readLine()) != null) json.append(line);
            return FollowerConstants.fromJson(new JSONObject(json.toString()));
        } catch (IOException | org.json.JSONException e) {
            throw new IllegalStateException("Cannot read simulator calibration", e);
        }
    }

    static void apply(FollowerConstants target) {
        FollowerConstants c = snapshot();
        target.motionCharacterization = c.motionCharacterization;
        target.angularCoeffs = c.angularCoeffs;
        target.translationalCoeffs = c.translationalCoeffs;
        target.angularKV = c.angularKV;
        target.angularKA = c.angularKA;
        target.angularFeedforwardKS = c.angularFeedforwardKS;
        target.translationalKV = c.translationalKV;
        target.translationalKA = c.translationalKA;
        target.translationalFeedforwardKS = c.translationalFeedforwardKS;
        target.velocityFeedbackGain = c.velocityFeedbackGain;
        target.brakingVelocityFeedbackGain = c.brakingVelocityFeedbackGain;
        target.brakingOverspeedToleranceIn = c.brakingOverspeedToleranceIn;
        target.brakingPulsePower = c.brakingPulsePower;
        target.tankBrakingPulsePower = c.tankBrakingPulsePower;
        target.brakingPulseCooldownCycles = c.brakingPulseCooldownCycles;
        target.brakingNeutralPowerMargin = c.brakingNeutralPowerMargin;
        target.angularVelocityFeedbackGain = c.angularVelocityFeedbackGain;
        target.kCentripetal = c.kCentripetal;
        target.profileCornerSpeedScale = c.profileCornerSpeedScale;
        target.profileVelocityPowerCoefficient = c.profileVelocityPowerCoefficient;
        target.brakingCornerToleranceIn = c.brakingCornerToleranceIn;
        target.tankBrakingCornerToleranceIn = c.tankBrakingCornerToleranceIn;
        target.forwardVelLimitIn = c.forwardVelLimitIn;
        target.forwardAccelLimitIn = c.forwardAccelLimitIn;
        target.strafeVelLimitIn = c.strafeVelLimitIn;
        target.strafeAccelLimitIn = c.strafeAccelLimitIn;
        target.angularVelLimitRad = c.angularVelLimitRad;
        target.angularAccelLimitRad = c.angularAccelLimitRad;
    }
}
