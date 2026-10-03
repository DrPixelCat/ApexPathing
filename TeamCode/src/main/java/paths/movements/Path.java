package paths.movements;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import follower.FollowerConstants;
import drivetrains.BaseDrivetrain;
import trajectory.profile.FFLut;
import trajectory.generators.BaseProfileGenerator;
import trajectory.generators.MecanumProfileGenerator;
import trajectory.generators.SwerveProfileGenerator;
import trajectory.generators.TankProfileGenerator;
import geometry.Angle;
import geometry.AngleUnit;
import geometry.Dist;
import geometry.ParametricSegment;
import geometry.PathPoint;
import geometry.PathSegment;
import geometry.Pose;
import geometry.Vector;
import paths.Callback;
import paths.constraint.AngularConstraint;
import paths.constraint.PathConstraint;
import paths.constraint.PathConstraint.Type;
import paths.constraint.TranslationalConstraint;
import paths.heading.HeadingInterpolator;
import paths.heading.ReversedHeadingInterpolator;

/**
 * Represents a complete, navigable geometric route for the robot to follow.
 *
 * <p>A {@code Path} encapsulates a continuous parametric curve (e.g., a B-Spline), its associated
 * heading interpolation strategy, and any scheduled mechanical callbacks triggered along the route.
 *
 * @author DrPixelCat - 7842 alum
 * @author Sohum Arora - 22985 Paraducks
 */
public class Path extends FollowerMovement {
    public enum PathType { HOLONOMIC, TANK }
    public enum BuildMode { ONLINE, MEMORY }

    private final List<String> buildWarnings = new ArrayList<String>();
    private final ArrayList<PathConstraint> constraints = new ArrayList<PathConstraint>();
    private final PathType pathType;

    private PathSegment parametricPath;
    private HeadingInterpolator interpolator;
    private FFLut FFLut;
    private boolean isAccelBoosted = false;
    private BuildMode buildMode = BuildMode.ONLINE;
    private trajectory.TrajectoryField2D trajectoryField;
    private Double lookaheadDistanceIn;
    private double memoryGridSpacingIn = 1.0, memoryGridPaddingIn = 12.0;

    public Double getLookaheadDistanceIn() { return lookaheadDistanceIn; }
    public void setLookaheadDistanceIn(double distance) {
        requirePositive("Lookahead distance", distance); lookaheadDistanceIn = distance;
    }
    public double getMemoryGridSpacingIn() { return memoryGridSpacingIn; }
    public void setMemoryGridSpacingIn(double spacing) {
        requirePositive("Grid spacing", spacing); memoryGridSpacingIn = spacing;
    }
    public double getMemoryGridPaddingIn() { return memoryGridPaddingIn; }
    public void setMemoryGridPaddingIn(double padding) {
        if (!Double.isFinite(padding) || padding < 0) { throw new IllegalArgumentException("Grid padding must be finite and nonnegative."); }
        memoryGridPaddingIn = padding;
    }
    private static void requirePositive(String name, double value) {
        if (!Double.isFinite(value) || value <= 0) { throw new IllegalArgumentException(name + " must be finite and positive."); }
    }
    public trajectory.TrajectoryField2D getTrajectoryField() { return trajectoryField; }
    public FollowerConstants getPlanningConstants() {
        return FollowerConstants.getInstance().forProfile(pathType == PathType.TANK
                ? FollowerConstants.Profile.TANK : FollowerConstants.Profile.HOLONOMIC);
    }
    public BaseProfileGenerator createGenerator(FollowerConstants constants) {
        return pathType == PathType.TANK ? new TankProfileGenerator(constants, this)
                : constants.drivetrainType == BaseDrivetrain.DrivetrainType.COAXIAL_SWERVE
                ? new SwerveProfileGenerator(constants, this) : new MecanumProfileGenerator(constants, this);
    }
    private boolean reverseConstraintProgress;
    private Double profilePowerLimit;

    /** Optional normalized profile-power ceiling; null uses the active drivetrain's default. */
    public Double getProfilePowerLimit() { return profilePowerLimit; }

    /**
     * Sets the profile-power ceiling in (0, 1]. The generator also respects calibrated available
     * motor power. Stored trajectories retain the ceiling used when they were built.
     */
    public void setProfilePowerLimit(double powerLimit) {
        if (!Double.isFinite(powerLimit) || powerLimit <= 0.0 || powerLimit > 1.0) {
            throw new IllegalArgumentException("Profile power limit must be finite and in (0, 1].");
        }
        profilePowerLimit = powerLimit;
    }

    /**
     * Creates a path object for the robot to follow
     *
     * @param pathType {@link PathType}: HOLONOMIC, or TANK
     */
    public Path(PathType pathType) { this.pathType = pathType; }

    /** Attaches a distance callback to this traversal and returns the path for chaining. */
    public Path addDistanceCallback(double progress, Runnable action) {
        addCallback(new Callback(progress, action));
        return this;
    }

    /** Attaches a heading callback to this traversal and returns the path for chaining. */
    public Path addAngularCallback(Angle heading, Runnable action) {
        addCallback(new Callback(heading, action));
        return this;
    }

    /** @param constraint The kinematic constraint to apply */
    private long constraintRevision;
    private boolean liveCustomConstraints;
    public void addConstraint(PathConstraint constraint) {
        constraints.add(constraint); constraintRevision++;
        if (!constraint.addChangeListener(() -> constraintRevision++)) { liveCustomConstraints = true; }
    }

    /** A negative revision requests live sampling for custom mutable constraints. */
    public long getConstraintRevision() { return liveCustomConstraints ? -1 : constraintRevision; }

    /** @return the path's kinematic constraints */
    public PathConstraint[] getConstraints() { return constraints.toArray(new PathConstraint[0]); }

    /**
     * Legacy inspection of the authored velocity schedule. Acts as a step
     * function: the velocity limit changes when the robot crosses a constraint's 't' threshold.
     *
     * @param t The current path percentage [0.0, 1.0].
     * @param defaultLimit The hardware maximum velocity from FollowerConstants.
     * @return The active velocity limit in inches per second.
     */
    @Deprecated public double getQuickVelocityLimit(double t, double defaultLimit) {
        double constraintProgress = constraintProgress(t);
        double currentLimit = defaultLimit;
        double highestS = -1.0;

        for (PathConstraint baseConstraint : constraints) {
            if (baseConstraint instanceof TranslationalConstraint) {
                TranslationalConstraint constraint = (TranslationalConstraint) baseConstraint;
                if (constraint.getType() == Type.VELOCITY) {
                    if (constraintProgress >= constraint.getS() &&
                            constraint.getS() > highestS) {
                        currentLimit = constraint.getValueIn();
                        highestS = constraint.getS();
                    }
                }
            }
        }
        return currentLimit;
    }

    /** @param endPose The final target pose of this path. */
    public void setEndPose(Pose endPose) { this.endPose = endPose; }

    /** @return The generated LUT points from the ParametricPath */
    public PathPoint[] getGeneratedPoints() { return parametricPath.getPointLUT().clone(); }

    /**
     * Injects the calculated geometric curve (e.g., a B-Spline) that defines
     * the physical route the robot will drive.
     *
     * @param path The compiled path segment.
     */
    public void setParametricPath(PathSegment path) { this.parametricPath = path; }

    /**
     * Retrieves the geometric curve defining the physical route.
     *
     * @return The compiled path segment.
     */
    public PathSegment getParametricPath() { return parametricPath; }

    /** @param interpolator The heading interpolator to use */
    public void setInterpolator(HeadingInterpolator interpolator) {
        this.interpolator = interpolator;
    }

    /** @return The heading interpolator. */
    public HeadingInterpolator getInterpolator() { return interpolator; }

    /** @return The type of path: HOLONOMIC, or TANK */
    public PathType getPathType() { return pathType; }

    /** @return The feedforward look-up table */
    public FFLut getFeedforwardLut() { return FFLut; }

    /** @param FFLut The path's motion profile as a {@link FFLut} */
    public void setFeedforwardLut(FFLut FFLut) { this.FFLut = FFLut; }

    /** @return true if built with buildToMemory(), false if built with quickBuild() */
    @Deprecated public boolean isProfiled() { return buildMode == BuildMode.MEMORY; }

    public BuildMode getBuildMode() { return buildMode; }

    public void setBuildMode(BuildMode buildMode) { this.buildMode = buildMode; }

    /** Maps traversal progress into the coordinate system used by the constraint schedule. */
    public double constraintProgress(double traversalProgress) {
        return reverseConstraintProgress ? 1.0 - traversalProgress : traversalProgress;
    }

    /** Determines if this path should be followed with boosted acceleration. */
    public void useBoostedAccel() { isAccelBoosted = true; }

    /** @return Whether the path should be followed with boosted acceleration or not */
    public boolean isAccelBoosted() { return isAccelBoosted; }

    /**
     * Logs a non-fatal warning generated during the path building process (e.g., missing headings,
     * ignored waypoints. Duplicate warnings are ignored to prevent telemetry spam on the driver
     * station.
     *
     * @param warning The warning string.
     */
    public void addWarning(String warning) {
        if (!buildWarnings.contains(warning)) {
            buildWarnings.add(warning);
        }
    }

    /** @return A read-only list of warning strings. */
    public List<String> getWarnings() { return Collections.unmodifiableList(buildWarnings); }

    /**
     * Returns a fresh path over the same field geometry in the opposite direction. Callbacks are
     * deliberately not copied because they are side effects belonging to a specific traversal.
     */
    @Override
    public Path reversed() {
        final PathSegment sourceSegment = parametricPath;
        PathSegment reversedSegment = new PathSegment(new ParametricSegment() {
            @Override public double[] getBoundsIn() { return sourceSegment.getBoundsIn(); }
            @Override
            public Vector getPosition(double t) {
                return sourceSegment.getPosition(1.0 - t);
            }

            @Override
            public Vector getFirstDerivative(double t) {
                return sourceSegment.getFirstDerivative(1.0 - t).times(-1.0);
            }

            @Override
            public Vector getSecondDerivative(double t) {
                return sourceSegment.getSecondDerivative(1.0 - t);
            }
        });

        Path result = new Path(pathType);
        result.setParametricPath(reversedSegment);
        result.setInterpolator(new ReversedHeadingInterpolator(
                interpolator, sourceSegment.getLengthIn(),
                sourceSegment.getFirstDerivative(1.0)));

        Vector endTangent = reversedSegment.getFirstDerivative(1.0);
        Angle endHeading = result.getInterpolator().getHeadingTarg(
                0.0, endTangent, endTangent);
        result.setEndPose(new Pose(reversedSegment.getPosition(1.0), endHeading));

        for (PathConstraint constraint : constraints) {
            result.addConstraint(copyConstraint(constraint));
        }
        result.reverseConstraintProgress = !reverseConstraintProgress;
        result.isAccelBoosted = isAccelBoosted;
        result.buildMode = buildMode;
        result.profilePowerLimit = profilePowerLimit;
        result.lookaheadDistanceIn = lookaheadDistanceIn;
        result.memoryGridSpacingIn = memoryGridSpacingIn;
        result.memoryGridPaddingIn = memoryGridPaddingIn;
        for (String warning : buildWarnings) { result.addWarning(warning); }
        result.precomputeTrajectory();
        return result;
    }

    /** Builder initialization: computes stored data once and never replaces it. */
    public void precomputeTrajectory() {
        if (buildMode == BuildMode.ONLINE || trajectoryField != null) { return; }
        FollowerConstants constants = getPlanningConstants();
        FFLut nominal = new FFLut(createGenerator(constants).generateRestToRest());
        trajectory.TrajectoryField2D field = new trajectory.TrajectoryField2D(this, nominal,
                memoryGridSpacingIn, memoryGridPaddingIn);
        FFLut = nominal; trajectoryField = field;
    }

    private static PathConstraint copyConstraint(PathConstraint constraint) {
        if (constraint instanceof TranslationalConstraint) {
            TranslationalConstraint translational = (TranslationalConstraint) constraint;
            return new TranslationalConstraint(translational.getS(), translational.getType(),
                    Dist.fromIn(translational.getValueIn()));
        }
        if (constraint instanceof AngularConstraint) {
            AngularConstraint angular = (AngularConstraint) constraint;
            return new AngularConstraint(angular.getS(), angular.getType(),
                    Angle.of(angular.getValueRad(), AngleUnit.RAD));
        }
        throw new IllegalArgumentException("Unsupported path constraint type: " +
                constraint.getClass().getName());
    }
}
