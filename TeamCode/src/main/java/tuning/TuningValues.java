package tuning;

import controllers.PDSController.PDSCoefficients;
import core.FollowerConstants;

/**
 * @author Sohum Arora - 22985 Paraducks
 */
public class TuningValues {
    PDSCoefficients heading;
    PDSCoefficients translation;
    double forwardVelocity;
    double forwardAcceleration;
    double strafeVelocity;
    double strafeAcceleration;
    double angularVelocity;
    double angularAcceleration;
    double translationKV;
    double translationKA;
    double angularKV;
    double angularKA;
    double centripetal;
    double translationFeedback;
    double angularFeedback;

    public TuningValues(FollowerConstants constants) {
        heading = copy(constants.headingCoeffs);
        translation = copy(constants.translationalCoeffs);
        forwardVelocity = constants.forwardVelLimitIn;
        forwardAcceleration = constants.forwardAccelLimitIn;
        strafeVelocity = constants.strafeVelLimitIn;
        strafeAcceleration = constants.strafeAccelLimitIn;
        angularVelocity = constants.angularVelLimitRad;
        angularAcceleration = constants.angularAccelLimitRad;
        translationKV = constants.translationalKV;
        translationKA = constants.translationalKA;
        angularKV = constants.angularKV;
        angularKA = constants.angularKA;
        centripetal = constants.Kcentripetal;
        translationFeedback = constants.velocityFeedbackGain;
        angularFeedback = constants.angularVelocityFeedbackGain;
    }

    PDSCoefficients copy(PDSCoefficients coefficients) {
        return new PDSCoefficients(coefficients.kP, coefficients.kD, coefficients.kS);
    }

    boolean allPositive(double... values) {
        for (double value : values) {
            if (!Double.isFinite(value) || value <= 0.0) {
                return false;
            }
        }
        return true;
    }

    void saveHeading(TunerContext context) {
        context.constants.headingCoeffs.setkP(heading.kP);
        context.constants.headingCoeffs.setkD(heading.kD);
        context.constants.headingCoeffs.setkS(heading.kS);
        context.getFollower().setHeadingTuning(heading);
    }

    void saveMovement(TunerContext context) {
        context.constants.translationalCoeffs.setkP(translation.kP);
        context.constants.translationalCoeffs.setkD(translation.kD);
        context.constants.translationalCoeffs.setkS(translation.kS);
        context.constants.forwardVelLimitIn = forwardVelocity;
        context.constants.forwardAccelLimitIn = forwardAcceleration;
        context.constants.strafeVelLimitIn = strafeVelocity;
        context.constants.strafeAccelLimitIn = strafeAcceleration;
        context.constants.angularVelLimitRad = angularVelocity;
        context.constants.angularAccelLimitRad = angularAcceleration;
        context.constants.translationalKV = translationKV;
        context.constants.translationalKA = translationKA;
        context.constants.angularKV = angularKV;
        context.constants.angularKA = angularKA;
        context.getFollower().setMovementTuning(translation, translationKV, translationKA, angularKV,
                angularKA, forwardVelocity, strafeVelocity);
    }

    void saveCentripetal(TunerContext context) {
        context.constants.Kcentripetal = centripetal;
        context.getFollower().setCentripetalTuning(centripetal);
    }

    void saveFeedback(TunerContext context) {
        context.constants.velocityFeedbackGain = translationFeedback;
        context.constants.angularVelocityFeedbackGain = angularFeedback;
        context.getFollower().setVelocityFeedbackTuning(translationFeedback, angularFeedback);
    }
}
