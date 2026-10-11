package drivetrains.models;

/** Physical limits along a robot-relative direction, in inches/second and inches/second squared. */
public final class DirectionalLimits {
    public final double maxVelocity, maxAcceleration;
    public DirectionalLimits(double maxVelocity, double maxAcceleration) {
        if (!Double.isFinite(maxVelocity) || maxVelocity <= 0
                || !Double.isFinite(maxAcceleration) || maxAcceleration <= 0)
            throw new IllegalArgumentException("Directional limits must be finite and positive");
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
    }
}
