package follower;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;

import org.json.JSONObject;
import org.junit.Test;

public class FollowerConstantsVelocityFeedbackTest {
    @Test public void obsoletePursuitAndProjectionSettingsAreIgnoredAndProportionalGainPersists() throws Exception {
        FollowerConstants constants = FollowerConstants.fromJson(new JSONObject().put("drivetrainType", "MECANUM")
                .put("velocityFeedbackGain", .04).put("velocityPursuitTimeSeconds", .35)
                .put("velocityErrorProjectionTimeSeconds", .05));
        assertEquals(.04, constants.velocityFeedbackGain, 0);
        assertFalse(constants.toJson().toString().contains("velocityPursuitTimeSeconds"));
        assertFalse(constants.toJson().toString().contains("velocityErrorProjectionTimeSeconds"));
        assertEquals(.04, FollowerConstants.fromJson(constants.toJson()).velocityFeedbackGain, 0);
    }
    @Test
    public void derivesFeedbackFromIdentifiedVelocityAndAccelerationTerms() throws Exception {
        FollowerConstants constants = FollowerConstants.fromJson(
                new JSONObject().put("drivetrainType", "TANK"));
        constants.translationalKV = 0.01256026949979914;
        constants.translationalKA = 0.006544195488076752;
        constants.angularKV = 0.08770967781140003;
        constants.angularKA = 0.04546577686017766;

        constants.deriveVelocityFeedbackGains();

        assertEquals(0.05288168538096838, constants.velocityFeedbackGain, 1e-12);
        assertEquals(FollowerConstants.MAX_ANGULAR_VELOCITY_FEEDBACK_GAIN,
                constants.angularVelocityFeedbackGain, 0.0);
    }

    @Test
    public void clampsDerivedFeedbackToSafeNonnegativeRange() {
        assertEquals(0.0,
                FollowerConstants.velocityFeedbackGain(0.1, 0.005, 0.25), 0.0);
        assertEquals(0.10,
                FollowerConstants.velocityFeedbackGain(0.01, 0.1, 0.10), 0.0);
        assertEquals(0.0,
                FollowerConstants.velocityFeedbackGain(Double.NaN, 0.1, 0.10), 0.0);
    }
}
