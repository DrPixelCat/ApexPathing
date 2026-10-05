package paths.builders;

import java.util.ArrayList;
import java.util.List;

import geometry.Angle;
import geometry.ArcPose;
import geometry.Dist;
import geometry.Pose;
import geometry.Vector;
import paths.constraint.PathConstraint;
import paths.heading.InterpolationStyle;
import paths.movements.Path;

/**
 * Provides shared functionality and structure for building paths for all drivetrain types.
 *
 * @author DrPixelCat - 7842 alum
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public abstract class PathBuilder<T extends PathBuilder<T>> {
    protected Path path;
    private boolean built;

    protected void beginBuild() {
        if (built) { throw new IllegalStateException("Path builders can only be built once"); }
        built = true;
    }

    @SuppressWarnings("unchecked")
    public T setPlanningConstants(follower.FollowerConstants constants) {
        path.setPlanningConstants(constants);
        return (T) this;
    }
    protected InterpolationStyle style = InterpolationStyle.TANGENT_FORWARD;

    final Pose[] rawPoses;
    final List<Runnable> buildTasks = new ArrayList<Runnable>();

    /**
     * Creates a new PathBuilder using the provided poses.
     *
     * @param type The type of the path to be built.
     * @param poses A sequence of Pose objects defining the path. Must contain at least two poses.
     *              Endpoints cannot be ArcPoses.
     */
    public PathBuilder(Path.PathType type, Pose... poses) {
        this.rawPoses = java.util.Objects.requireNonNull(poses, "Poses").clone();
        this.path = new Path(type);
        if (poses.length < 2) {
            throw new IllegalArgumentException("A B-Spline must be created with > 1 points!");
        }
        for (Pose pose : rawPoses) {
            if (pose == null || !pose.getVec().isFinite() || pose.getHeading().isNotFinite()) {
                throw new IllegalArgumentException("Path poses must be finite");
            }
            if (pose instanceof ArcPose && (((ArcPose) pose).getRadius() == null
                    || !Double.isFinite(((ArcPose) pose).getRadius().getIn())
                    || ((ArcPose) pose).getRadius().getIn() <= 0)) {
                throw new IllegalArgumentException("Arc radius must be positive and finite");
            }
        }
        if (poses[0] instanceof ArcPose || poses[poses.length - 1] instanceof ArcPose) {
            throw new IllegalArgumentException("Endpoints can't be arcs!");
        }
    }

    /**
     * Sets the normalized motor-power ceiling for generated profiles, in (0, 1]. Omission uses
     * the drivetrain default (1.0, or 0.88 of calibrated available power when characterized).
     * A value of 1.0 uses all calibrated available power. Velocity and acceleration constraints
     * still apply in both online and stored modes.
     */
    @SuppressWarnings("unchecked")
    public T setProfilePowerLimit(double powerLimit) {
        path.setProfilePowerLimit(powerLimit);
        return (T) this;
    }

    /** Overrides the automatically sized, fixed online planning horizon. */
    @SuppressWarnings("unchecked") public T setLookaheadDistance(Dist distance) {
        path.setLookaheadDistanceIn(distance.getIn()); return (T) this;
    }
    @SuppressWarnings("unchecked") public T setMemoryGridSpacing(Dist spacing) {
        path.setMemoryGridSpacingIn(spacing.getIn()); return (T) this;
    }
    @SuppressWarnings("unchecked") public T setMemoryGridPadding(Dist padding) {
        path.setMemoryGridPaddingIn(padding.getIn()); return (T) this;
    }

    /**
     * Adds a kinematic constraint to the path at a specific distance percentage.
     *
     * <p>Both build modes enforce translational and angular velocity and acceleration limits.
     *
     * @param constraint The {@link PathConstraint} to be added to the path
     * @return The current PathBuilder instance for method chaining.
     */
    @SuppressWarnings("unchecked")
    public T addConstraint(PathConstraint constraint) {
        if (!Double.isFinite(constraint.getS()) || constraint.getS() >= 1 || constraint.getS() < 0) {
            throw new IllegalArgumentException("Constraint progress must be in [0, 1)");
        }
        path.addConstraint(constraint);
        return (T) this;
    }

    /**
     * Overrides the default (SMOOTH_START_TO_END for holonomic, TANGENT_FORWARD for tank)
     * interpolation with a different {@link InterpolationStyle}. If a non-tangent interpolation
     * style is used for a tank drivetrain, an {@link IllegalArgumentException} will be thrown.
     *
     * @param style The interpolation style to apply.
     * @return The current PathBuilder instance for method chaining.
     */
    public abstract T interpolateWith(InterpolationStyle style);

    /**
     * Overrides the interpolation style, providing a custom angular offset. Used primarily for
     * {@link InterpolationStyle#TANGENT_CUSTOM}. This method is only available for holonomic
     * drivetrains.
     *
     * @param style The interpolation style to apply.
     * @param angleOffset The fixed angle to offset the calculation by.
     * @return The current HolonomicPathBuilder instance for method chaining.
     */
    public HolonomicPathBuilder interpolateWith(InterpolationStyle style, Angle angleOffset) {
        throw new UnsupportedOperationException(
                "This method is only available for holonomic drivetrains!"
        ); // The HolonomicPathBuilder overrides this method.
    }

    /**
     * Overrides the interpolation style, providing a fixed field point to face. Used primarily for
     * {@link InterpolationStyle#FACING_POINT}. This method is only available for holonomic
     * drivetrains.
     *
     * @param style The interpolation style to apply.
     * @param pointToFace The field coordinate the robot should face.
     * @return The current HolonomicPathBuilder instance for method chaining.
     */
    public HolonomicPathBuilder interpolateWith(InterpolationStyle style, Vector pointToFace) {
        throw new UnsupportedOperationException(
                "This method is only available for holonomic drivetrains!"
        ); // The HolonomicPathBuilder overrides this method.
    }

    /**
     * Overrides the interpolation style, providing a fixed field point and angular offset. Used
     * primarily for {@link InterpolationStyle#FACING_POINT}. This method is only available for
     * holonomic drivetrains.
     *
     * @param style The interpolation style to apply.
     * @param pointToFace The field coordinate the robot should face.
     * @param angleOffset The fixed angle to offset the facing direction by.
     * @return The current HolonomicPathBuilder instance for method chaining.
     */
    public HolonomicPathBuilder interpolateWith(InterpolationStyle style, Vector pointToFace,
                                                Angle angleOffset) {
        throw new UnsupportedOperationException(
                "This method is only available for holonomic drivetrains!"
        ); // The HolonomicPathBuilder overrides this method.
    }

    /**
     * Adds a heading node for NODE_BASED interpolation. Automatically sets the interpolation style
     * to NODE_BASED. This method is only available for holonomic drivetrains.
     *
     * @param pct The distance percentage [0.0, 1.0].
     * @param target The target Angle at this point.
     * @return The current HolonomicPathBuilder instance for method chaining.
     */
    public HolonomicPathBuilder addHeadingNode(double pct, Angle target) {
        throw new UnsupportedOperationException(
                "This method is only available for holonomic drivetrains!"
        ); // The HolonomicPathBuilder overrides this method.
    }

    /**
     * Sets how far from the end of the path the robot should start rotating to face its final
     * target direction. This method is only available for holonomic drivetrains.
     *
     * @param distanceFromEnd The distance away from the end of the path.
     * @return The current HolonomicPathBuilder instance for method chaining.
     */
    public HolonomicPathBuilder setDistanceToStartFinalTurn(Dist distanceFromEnd) {
        throw new UnsupportedOperationException(
                "This method is only available for holonomic drivetrains!"
        ); // The HolonomicPathBuilder overrides this method.
    }

    /**
     * Attaches an executable callback based on the physical distance percentage.
     *
     * @param s The physical distance percentage [0.0, 1.0].
     * @param action The method to execute when the robot reaches the specified distance.
     * @return The current PathBuilder instance for method chaining.
     */
    @SuppressWarnings("unchecked")
    public T addDistanceCallback(double s, Runnable action) {
        this.buildTasks.add(() -> path.addDistanceCallback(s, action));
        return (T) this; // Safe cast because T is always a subclass of PathBuilder
    }

    /**
     * Attaches an executable callback based on the robot reaching a target heading.
     *
     * @param angle The Angle at which the callback should trigger.
     * @param action The method to execute when the robot reaches the specified heading.
     * @return The current PathBuilder instance for method chaining.
     */
    public abstract T addAngularCallback(Angle angle, Runnable action);

    /**
     * Builds the path geometry without generating a physical motion profile. The follower will
     * plan a bounded trajectory online from measured speed.
     *
     * @return The constructed Path.
     */
    public abstract Path quickBuild();

    /**
     * Builds the path geometry and solves a complete kinematically constrained feedforward
     * motion profile and a bilinearly interpolated vector field in RAM.
     *
     * @return The constructed Path with its immutable stored field.
     */
    public abstract Path buildToMemory();
}
