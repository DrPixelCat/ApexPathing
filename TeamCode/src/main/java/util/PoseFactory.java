package util;

import geometry.Angle;
import geometry.ArcPose;
import geometry.Dist;
import geometry.Pose;
import geometry.Vector;

/**
 * A factory class for creating {@link Pose} objects from various types of input including many
 * quality of life methods for units and mirroring.
 *
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public class PoseFactory {
    /** Axis to mirror the Pose across, if any. */
    public enum Mirror { NONE, X, Y }

    private Mirror mirror;
    private DistUnit distUnit;
    private AngleUnit angleUnit;

    // region Constructors
    /** Creates a PoseFactory with the specified units and mirroring configuration. */
    public PoseFactory(DistUnit distUnit, AngleUnit angleUnit, Mirror mirror) {
        this.distUnit = distUnit; this.angleUnit = angleUnit; this.mirror = mirror;
    }

    /** Creates a PoseFactory with the specified units. */
    public PoseFactory(DistUnit distUnit, AngleUnit angleUnit) {
        this(distUnit, angleUnit, Mirror.NONE);
    }
    // endregion

    // region Internal utility methods
    /** Applies the configured mirroring to a Pose. */
    private Pose applyMirror(Pose pose) {
        if (mirror == Mirror.NONE) return pose;
        return mirror == Mirror.X ? pose.mirrorX() : pose.mirrorY();
    }
    // endregion

    // region Setters
    /** Sets the mirroring configuration for this PoseFactory. */
    public void setMirror(Mirror mirror) { this.mirror = mirror; }

    /** Sets the distance unit for this PoseFactory. */
    public void setDistUnit(DistUnit distUnit) { this.distUnit = distUnit; }

    /** Sets the angle unit for this PoseFactory. */
    public void setAngleUnit(AngleUnit angleUnit) { this.angleUnit = angleUnit; }
    // endregion

    // region Getters
    /** @return the mirroring configuration for this PoseFactory. */
    public Mirror getMirror() { return mirror; }

    /** @return the distance unit for this PoseFactory. */
    public DistUnit getDistUnit() { return distUnit; }

    /** @return the angle unit for this PoseFactory. */
    public AngleUnit getAngleUnit() { return angleUnit; }
    // endregion

    // region Pose creation methods
    /** Creates a Pose from the given (x, y, heading) values in the configured units and mirroring. */
    public Pose of(double x, double y, double heading) {
        Pose pose = new Pose(Vector.of(x, y, distUnit), Angle.of(heading, angleUnit));
        return applyMirror(pose);
    }

    /** Creates a Pose from the given (x, y) values in the configured units with a heading of 0 */
    public Pose of(double x, double y) { return of(x, y, 0); }

    /** Creates an ArcPose from the given (x, y) and radius values in the configured units and mirroring. */
    public ArcPose arcPoseOf(double x, double y, double radius) {
        ArcPose pose = new ArcPose(Vector.of(x, y, distUnit), Dist.of(radius, distUnit));
        return (ArcPose) applyMirror(pose); // Will always return an ArcPose, so we can just cast it
    }
    // endregion
}