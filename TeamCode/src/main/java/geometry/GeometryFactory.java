package geometry;

import follower.Follower;
import paths.builders.HolonomicPathBuilder;
import paths.builders.TankPathBuilder;
import paths.builders.TurnBuilder;
import paths.movements.Path;
import paths.movements.Turn;

/**
 * A factory class for creating {@link Pose}, {@link ArcPose}, {@link Path}, {@link Turn},
 * {@link Vector}, {@link Dist}, and {@link Angle} objects with specified units.
 *
 * @author Dylan B. - 18597 RoboClovers - Delta
 * @author DrPixelCat - 7842 alum
 */
@SuppressWarnings("ClassNamePrefixedWithPackageName")
public class GeometryFactory {
    public enum PoseMirror { NONE, X, Y }

    private final Follower follower;
    private PoseMirror mirror = PoseMirror.NONE;
    private DistUnit distUnit = DistUnit.IN;
    private AngleUnit angleUnit = AngleUnit.DEG;

    // region Constructors

    /** Creates a factory with default units and mirroring; builders select drive mode. */
    public GeometryFactory() { this.follower = null; }

    /** Creates builders using a snapshot of this follower's planning settings. */
    public GeometryFactory(Follower follower) { this.follower = java.util.Objects.requireNonNull(follower, "Follower"); }

    /** Applies the configured mirroring to a {@link Pose}. */
    private Pose applyMirror(Pose pose) {
        if (mirror == PoseMirror.NONE) { return pose; }
        return mirror == PoseMirror.X ? pose.mirrorX() : pose.mirrorY();
    }

    // endregion
    // region Setters

    /** Sets the factory mirroring configuration. Defaults to {@link PoseMirror#NONE}. */
    public GeometryFactory setPoseMirror(PoseMirror mirror) {
        this.mirror = java.util.Objects.requireNonNull(mirror, "Pose mirror");
        return this;
    }

    /** Sets the distance unit used for inputs. Defaults to {@link DistUnit#IN} (inches). */
    public GeometryFactory setDistUnit(DistUnit distUnit) {
        this.distUnit = java.util.Objects.requireNonNull(distUnit, "Distance unit");
        return this;
    }

    /** Sets the angle unit used for inputs. Defaults to {@link AngleUnit#DEG} (degrees). */
    public GeometryFactory setAngleUnit(AngleUnit angleUnit) {
        this.angleUnit = java.util.Objects.requireNonNull(angleUnit, "Angle unit");
        return this;
    }


    // endregion
    // region Getters

    /** @return the factory mirroring configuration as a {@link PoseMirror}. */
    public PoseMirror getPoseMirror() { return mirror; }

    /** @return the {@link DistUnit} used for inputs. */
    public DistUnit getDistUnit() { return distUnit; }

    /** @return the {@link AngleUnit} used for inputs. */
    public AngleUnit getAngleUnit() { return angleUnit; }


    // endregion
    // region Poses and arc poses

    /**
     * Creates a Pose from the given (x, y, heading) values in the configured units and mirroring.
     */
    public Pose pose(double x, double y, double heading) {
        Pose pose = new Pose(Vector.of(x, y, distUnit), Angle.of(heading, angleUnit));
        return applyMirror(pose);
    }

    /**
     * Creates a Pose from the given (x, y) values in the configured units and mirroring with a
     * heading of 0.
     */
    public Pose pose(double x, double y) { return pose(x, y, 0); }

    /** Creates a factory with a heading of 0 at the origin. */
    public Pose pose() { return pose(0, 0, 0); }

    /**
     * Creates an ArcPose from the given (x, y) and radius values in the configured units and
     * mirroring.
     */
    public ArcPose arcPose(double x, double y, double radius) {
        ArcPose pose = new ArcPose(Vector.of(x, y, distUnit), Dist.of(radius, distUnit));
        return mirror == PoseMirror.X ? pose.mirrorX()
                : mirror == PoseMirror.Y ? pose.mirrorY() : pose;
    }

    // endregion
    // region Paths and turns

    /** Builds a tank path; following it selects traction mode on a dual-actuated drive. */
    public TankPathBuilder tankPath(Pose... poses) {
        TankPathBuilder builder = new TankPathBuilder(poses);
        if (follower != null) { builder.setPlanningConstants(follower.getConstants()); }
        return builder;
    }

    /** Builds a holonomic path; following it selects holonomic mode on a dual-actuated drive. */
    public HolonomicPathBuilder holonomicPath(Pose... poses) {
        HolonomicPathBuilder builder = new HolonomicPathBuilder(poses);
        if (follower != null) { builder.setPlanningConstants(follower.getConstants()); }
        return builder;
    }

    /** Creates a {@link TurnBuilder} from the given start pose. */
    public TurnBuilder turn(Pose startPose) {
        return follower == null ? new TurnBuilder(startPose)
                : new TurnBuilder(startPose, follower.getConstants());
    }

    // endregion
    // region Vectors

    /** Creates a Vector from the given (x, y) values in the configured units. */
    public Vector vector(double x, double y) { return Vector.of(x, y, distUnit); }

    /** Creates a Vector at the origin. */
    public Vector vector() { return vector(0, 0); }

    // endregion
    // region Distances and angles

    /** Creates a Dist from the given value in the configured units. */
    public Dist dist(double value) { return Dist.of(value, distUnit); }

    /** Creates a Dist of 0. */
    public Dist dist() { return dist(0); }

    /** Creates an Angle from the given value in the configured units. */
    public Angle angle(double value) { return Angle.of(value, angleUnit); }

    /** Creates an Angle of 0. */
    public Angle angle() { return angle(0); }

    // endregion
}
