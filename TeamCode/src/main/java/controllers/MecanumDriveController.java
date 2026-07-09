package controllers;

import geometry.Angle;
import geometry.Dist;
import geometry.Vector;

/**
 * A class designed inspired by Wolfpack Machina (18438) to account for mecanum drive inefficiencies
 * and move a robot point-to-point
 *
 * @author DrPixelCat24
 */
public class MecanumDriveController {
    private final double strafePenaltyRatio;
    public final PDSController pds;
    public final Dist tolerance;

    /**
     * @param maxForwardVelocity The maximum forward velocity of the robot
     * @param maxStrafeVelocity  The maximum strafe velocity of the robot
     * @param coefficients The coefficients for the PDkS controller
     * @param tolerance The distance at which power is no longer applied
     */
    public MecanumDriveController(Dist maxForwardVelocity, Dist maxStrafeVelocity,
                                  PDSController.PDSCoefficients coefficients, Dist tolerance) {
        this.tolerance = tolerance;
        this.strafePenaltyRatio = maxForwardVelocity.getIn() / maxStrafeVelocity.getIn();
        this.pds = new PDSController(coefficients);
    }

    public Vector calculatePointToPoint(Vector targetPos, Vector currentPos, Angle currentHeading) {
        Vector fieldError = targetPos.minus(currentPos);
        if (fieldError.getMag().getIn() < 0.01) return Vector.zero();

        double basePower = pds.calculateFromError(fieldError.getMag().getIn());
        Vector rawFieldVector = Vector.fromPolar(Dist.fromIn(basePower), fieldError.getTheta());

        return applyMecanumCorrections(rawFieldVector, currentHeading);
    }

    public Vector applyMecanumCorrections(Vector rawFieldCentricPower, Angle currentHeading) {
        Vector localVector = rawFieldCentricPower.rotate(currentHeading.times(-1.0));

        Vector correctedLocalVector = new Vector(
                Dist.fromIn(localVector.getX().getIn() * strafePenaltyRatio),
                localVector.getY()
        );

        return correctedLocalVector.rotate(currentHeading);
    }
}