package trajectory.generators;

import follower.FollowerConstants;
import trajectory.profile.DynamicMotionProfile;
import trajectory.profile.FFLut;
import trajectory.profile.MotionParameters;
import paths.movements.Turn;

/**
 * Generates a one-dimensional angular profile for point turns.
 *
 * <p>A turn does not move along a path, so the profile only fills {@code omega} and
 * {@code alpha}-related limits. The same LUT type is used so the follower can query turns and
 * paths through the same feedforward interface.
 *
 * @author Joel H
 */
public class TurnProfileGenerator {

    private final double angularKV;
    private final double angularKA;
    private final double headingKS;
    private FollowerConstants constants;

    /** Maximum angular velocity allowed for the turn, in radians per second. */
    private double omega_max;
    /** Maximum angular acceleration allowed for the turn, in radians per second squared. */
    private double alpha_max;

    /** Creates a generator from one explicit follower-configuration source. */
    public TurnProfileGenerator(double omega_max, double alpha_max, FollowerConstants constants) {
        this(omega_max, alpha_max, constants.angularKV, constants.angularKA,
                constants.angularFeedforwardKS);
        this.constants = constants;
    }

    /**
     * Creates a turn profile generator with drivetrain power-model coefficients.
     *
     * @param angularKV normalized power per radian/second
     * @param angularKA normalized power per radian/second-squared
     * @param headingKS normalized static-friction power
     */
    public TurnProfileGenerator(double omega_max, double alpha_max,
                                double angularKV, double angularKA, double headingKS) {
        validateConstraints(omega_max, alpha_max);
        validatePowerCoefficients(angularKV, angularKA, headingKS);
        this.omega_max = omega_max;
        this.alpha_max = alpha_max;
        this.angularKV = angularKV;
        this.angularKA = angularKA;
        this.headingKS = headingKS;
    }

    /** Updates the angular limits without allocating a new generator.*/
    public void setConstraints(double omega_max, double alpha_max) {
        validateConstraints(omega_max, alpha_max);
        this.omega_max = omega_max;
        this.alpha_max = alpha_max;
    }

    /**
     * Builds a power-limited angular profile for the requested turn.
     *
     * <p>The pass structure mirrors the path generator: start with max velocity, sweep backward to
     * make sure the profile can stop, then sweep forward to make sure it can accelerate from rest.
     *
     * @param turn turn movement to profile
     * @return feedforward LUT containing angular velocity targets
     */
    public FFLut generate(Turn turn) {
        double signedTurn = turn.getStartPose().getHeading()
                .getShortestAngleTo(turn.getEndPose().getHeading()).getRad();
        double turnLengthRads = Math.abs(signedTurn);

        if (turnLengthRads < 1e-9) {
            MotionParameters stationary = new MotionParameters();
            stationary.setDistAlongCurve(0.0);
            return new FFLut(new MotionParameters[]{stationary});
        }

        // Define structural bounds and target density (~2 degrees per step index)
        double targetRadPerStep = Math.toRadians(2.0);
        int minSteps = 15;
        int maxSteps = 200;

        // Calculate adaptive step size based on total sweep length
        int steps = (int) Math.ceil(turnLengthRads / targetRadPerStep) + 1;
        steps = Math.max(minSteps, Math.min(maxSteps, steps));

        // The profile's independent variable is angular arc length s, not time or the signed
        // heading itself. Its LUT keys span s = 0 at the authored start through turnLengthRads.
        MotionParameters[] lut = new MotionParameters[steps];
        DynamicMotionProfile.PowerModel powerModel = new DynamicMotionProfile.PowerModel(
                omega_max, alpha_max, angularKV, angularKA, headingKS);
        if (constants != null && constants.motionCharacterization != null) {
            double direction = Math.signum(signedTurn);
            powerModel = new DynamicMotionProfile.PowerModel(
                    constants.calibratedVelocityLimit(feedforward.MotionCharacterization.Axis.ANGULAR, omega_max),
                    alpha_max, angularKV, angularKA, headingKS, constants.profilePowerLimit(),
                    (v, a) -> Math.abs(constants.characterizedPower(feedforward.MotionCharacterization.Axis.ANGULAR,
                            direction * v, direction * a, direction * (angularKV * v + angularKA * a + headingKS))));
        }
        DynamicMotionProfile profile = DynamicMotionProfile.forPowerLimitedDistance(
                turnLengthRads, steps, powerModel);
        DynamicMotionProfile.State[] scalarStates = profile.getStates();

        // Convert the scalar profile into signed angular states. Acceleration belongs to the
        // segment beginning at each sample so the first row can command the turn from rest.
        double direction = Math.signum(signedTurn);
        for (int i = 0; i < steps; i++) {
            DynamicMotionProfile.State state = scalarStates[i];
            lut[i] = new MotionParameters();
            lut[i].setDistAlongCurve(state.position);
            lut[i].setTimeSeconds(state.timeSeconds);
            lut[i].setTangentialVel(0.0);
            lut[i].setAngularVel(direction * state.velocity);
            lut[i].setAngularAccel(direction * state.acceleration);
        }

        return new FFLut(lut);
    }

    private static void validateConstraints(double omegaMax, double alphaMax) {
        if (!Double.isFinite(omegaMax) || omegaMax <= 0.0) {
            throw new IllegalArgumentException("Maximum angular velocity must be positive.");
        }
        if (!Double.isFinite(alphaMax) || alphaMax <= 0.0) {
            throw new IllegalArgumentException("Maximum angular acceleration must be positive.");
        }
    }

    private static void validatePowerCoefficients(double angularKV, double angularKA,
                                                  double headingKS) {
        if (!Double.isFinite(angularKV) || angularKV < 0.0 ||
                !Double.isFinite(angularKA) || angularKA < 0.0 ||
                !Double.isFinite(headingKS) || headingKS < 0.0 || headingKS >= 1.0) {
            throw new IllegalArgumentException(
                    "Angular kV/kA must be nonnegative and heading kS must be in [0, 1)."
            );
        }
    }

}
