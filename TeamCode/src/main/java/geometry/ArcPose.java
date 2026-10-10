package geometry;

/**
 * Represents a specialized waypoint in a path that should be smoothed into an arc.
 *
 * <p>When passed into a path builder, an {@code ArcPose} acts as a marker indicating that the sharp
 * corner at this location should be dynamically split and rounded. The {@code radius} defines how
 * far from the corner the smoothing begins and ends.
 *
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public class ArcPose extends Pose {
    private final Dist radius;

    /** Creates an ArcPose from a position Vector and a radius. */
    public ArcPose(Vector position, Dist radius) {
        super(position, Angle.fromRad(0));
        this.radius = radius;
    }

    /** @return the radius of this ArcPose. */
    public Dist getRadius() { return radius; }

    @Override public boolean equals(Object other) {
        return super.equals(other) && radius.equals(((ArcPose) other).radius);
    }
    @Override public int hashCode() { return 31 * super.hashCode() + radius.hashCode(); }

    @Override public ArcPose mirrorX() { return new ArcPose(getVec().mirrorX(), radius); }
    @Override public ArcPose mirrorY() { return new ArcPose(getVec().mirrorY(), radius); }
    @Override public ArcPose copy() { return new ArcPose(getVec(), radius); }
}
