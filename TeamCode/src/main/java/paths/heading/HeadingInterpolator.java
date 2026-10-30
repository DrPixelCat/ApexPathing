package paths.heading;

import paths.geometry.Angle;
import paths.geometry.Vector;

/**
 * Defines the contract for calculating target heading profiles across different drivetrain
 * kinematics.
 */
@SuppressWarnings("ClassNamePrefixedWithPackageName")
public interface HeadingInterpolator {
    /** Opt in only for tangent/backward-tangent heading with matching curvature derivatives. */
    default boolean supportsTank() { return false; }
    /** Target heading. All s arguments in this interface mean remaining distance in inches. */
    Angle getHeadingTarg(double s, Vector pathTangent, Vector finalTangent);

    /** First derivative with respect to forward distance traveled, in rad/in (not remaining distance). */
    double getHeadingFirstDerivative(double s, double kappa, Vector finalTangent);

    /** Second derivative with respect to forward distance traveled, in rad/in^2. */
    double getHeadingSecondDerivative(double s, double dKappa, Vector finalTangent);

    /** Sets the total path length, primarily used for terminal blending math. */
    void setPathLength(double lengthInches);

    /** Detached configuration for stored trajectories. Immutable implementations may return
     * themselves; custom mutable interpolators must override this method. */
    default HeadingInterpolator copyForTrajectory() { return this; }

    /** Nonnegative revisions permit online LUT-node caching. Mutable custom implementations
     * can retain the default to request fresh evaluations every update. */
    default long getConfigurationRevision() { return -1; }
}
