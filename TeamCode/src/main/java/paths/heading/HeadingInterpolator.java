package paths.heading;

import geometry.Angle;
import geometry.Vector;
import java.util.function.Function;

/**
 * Calculates and manages the target heading of the robot along a path segment.
 * <p>
 * This class is stateful and relies on constructor overloading to guarantee
 * that the correct parameters (such as start/end angles or custom offsets)
 * are provided for the chosen {@link InterpolationStyle}.
 * <p>
 * Author: DrPixelCat
 */
public class HeadingInterpolator {

    private final InterpolationStyle style;

    // State parameters
    private Angle startHeading;
    private Angle endHeading;
    private Angle customOffset;
    private Function<Double, Angle> customFunction;

    // region Constructors

    /**
     * Constructor for path-dependent styles that require no extra parameters.
     * <p>
     * Supported styles: {@link InterpolationStyle#TANGENT_FORWARD}
     *
     * @param style The interpolation style to use.
     * @throws IllegalArgumentException if the provided style requires additional parameters.
     */
    public HeadingInterpolator(InterpolationStyle style) {
        if (style != InterpolationStyle.TANGENT_FORWARD) {
            throw new IllegalArgumentException(style.name() + " requires additional parameters.");
        }
        this.style = style;
    }

    /**
     * Constructor for styles that require exactly one target Angle (or offset).
     * <p>
     * Supported styles:
     * {@link InterpolationStyle#CONSTANT_START_HEADING},
     * {@link InterpolationStyle#CONSTANT_END_HEADING},
     * {@link InterpolationStyle#TANGENT_CUSTOM}
     *
     * @param style The interpolation style to use.
     * @param targetOrOffset The target heading or the angular offset, depending on the style.
     * @throws IllegalArgumentException if the provided style requires a different number of parameters.
     */
    public HeadingInterpolator(InterpolationStyle style, Angle targetOrOffset) {
        this.style = style;
        switch (style) {
            case CONSTANT_START_HEADING:
                this.startHeading = targetOrOffset.copy();
                break;
            case CONSTANT_END_HEADING:
                this.endHeading = targetOrOffset.copy();
                break;
            case TANGENT_CUSTOM:
                this.customOffset = targetOrOffset.copy();
                break;
            default:
                throw new IllegalArgumentException("Invalid 1-angle constructor for style: " + style.name());
        }
    }

    /**
     * Constructor for styles that require both start and end boundaries to calculate total travel.
     * <p>
     * Supported styles:
     * {@link InterpolationStyle#SMOOTH_START_TO_END},
     * {@link InterpolationStyle#TANGENT_OPTIMAL}
     *
     * @param style The interpolation style to use.
     * @param startHeading The robot's heading at the start of the segment.
     * @param endHeading The robot's required heading at the end of the segment.
     * @throws IllegalArgumentException if the provided style requires a different number of parameters.
     */
    public HeadingInterpolator(InterpolationStyle style, Angle startHeading, Angle endHeading) {
        if (style != InterpolationStyle.SMOOTH_START_TO_END && style != InterpolationStyle.TANGENT_OPTIMAL) {
            throw new IllegalArgumentException("Invalid 2-angle constructor for style: " + style.name());
        }
        this.style = style;
        this.startHeading = startHeading.copy();
        this.endHeading = endHeading.copy();
    }

    /**
     * Constructor for a custom user-defined heading profile.
     * <p>
     * Supported styles: {@link InterpolationStyle#CUSTOM_DIST_FUNCTION}
     *
     * @param customFunction A lambda that takes distance percentage 's' (0.0 to 1.0) and returns an Angle.
     */
    public HeadingInterpolator(Function<Double, Angle> customFunction) {
        this.style = InterpolationStyle.CUSTOM_DIST_FUNCTION;
        this.customFunction = customFunction;
    }

    // endregion

    // region Core Update Logic

    /**
     * Calculates the target heading for the robot based on the current interpolation style.
     *
     * @param s The distance percentage along the segment [0.0, 1.0]
     * @param pathTangent The 2D forward tangent vector of the path at 's'
     * @return The target Angle the robot should face
     */
    public Angle getHeading(double s, Vector pathTangent) {
        switch (style) {
            case CONSTANT_START_HEADING:
                return startHeading.copy();

            case CONSTANT_END_HEADING:
                return endHeading.copy();

            case TANGENT_FORWARD:
                return pathTangent.getTheta();

            case TANGENT_CUSTOM:
                return pathTangent.getTheta().plus(customOffset);

            case TANGENT_OPTIMAL:
                return calculateOptimalTangent(pathTangent);

            case SMOOTH_START_TO_END:
                return calculateShortestPathLerp(s);

            case CUSTOM_DIST_FUNCTION:
                return customFunction.apply(s);

            default:
                throw new IllegalStateException("Unhandled heading interpolation style: " + style.name());
        }
    }

    // endregion

    // region Math Helpers

    /**
     * Aligns with the tangent, but chooses the direction (forward or backward)
     * that minimizes the TOTAL angular travel (entry turn + exit turn).
     *
     * @param tangent The forward tangent vector of the path.
     * @return The optimal Angle to track (either the forward or backward tangent).
     */
    private Angle calculateOptimalTangent(Vector tangent) {
        Angle forwardTangent = tangent.getTheta();
        Angle backwardTangent = forwardTangent.plus(Angle.fromRad(Math.PI));

        // Total rotation cost if drive forward
        double entryCostFwd = Math.abs(getShortestAngularDifference(startHeading, forwardTangent));
        double exitCostFwd  = Math.abs(getShortestAngularDifference(forwardTangent, endHeading));
        double totalCostFwd = entryCostFwd + exitCostFwd;

        // Total rotation cost if drive backward
        double entryCostBwd = Math.abs(getShortestAngularDifference(startHeading, backwardTangent));
        double exitCostBwd  = Math.abs(getShortestAngularDifference(backwardTangent, endHeading));
        double totalCostBwd = entryCostBwd + exitCostBwd;

        // Pick the orientation with the smallest total rotational requirement
        return totalCostBwd < totalCostFwd ? backwardTangent : forwardTangent;
    }

    /**
     * Interpolates between startHeading and endHeading via the shortest rotational path,
     * applying a cubic ease-in-out profile so rotational acceleration does not instantly spike.
     *
     * @param s The distance percentage along the segment [0.0, 1.0].
     * @return The profiled, interpolated Angle.
     */
    private Angle calculateShortestPathLerp(double s) {
        s = Math.max(0.0, Math.min(1.0, s));

        // Apply a Cubic Smoothstep (Ease-In-Out) profile to 's'
        // Equation: f(s) = 3s^2 - 2s^3
        double profiledS = (3.0 * s * s) - (2.0 * s * s * s);

        double diffRad = getShortestAngularDifference(startHeading, endHeading);
        double targetRad = startHeading.getRad() + (diffRad * profiledS);

        return Angle.fromRad(targetRad);
    }

    /**
     * Calculates the shortest signed angular difference between two angles in radians.
     * Result is always in the range [-PI, PI].
     *
     * @param from The starting Angle.
     * @param to The target Angle.
     * @return The shortest angular difference in radians.
     */
    private double getShortestAngularDifference(Angle from, Angle to) {
        double diff = to.getRad() - from.getRad();

        // Wrap the difference into the [-PI, PI] range
        diff = (diff + Math.PI) % (2 * Math.PI) - Math.PI;
        if (diff < -Math.PI) {
            diff += 2 * Math.PI;
        }
        return diff;
    }

    // endregion
}