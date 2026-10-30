package localizers.util;

import paths.geometry.Angle;
import paths.geometry.DistUnit;
import paths.geometry.Pose;
import paths.geometry.Vector;
import java.util.Random;

/** Test-only seeded sensor noise. Simulator ground truth and production localizers remain untouched. */
public final class LocalizationNoise {
    private final Random random;
    private final double positionStddev, headingStddev, velocityStddev, angularVelocityStddev, correlation;

    public LocalizationNoise(long seed, double positionStddev, double headingStddev,
            double velocityStddev, double angularVelocityStddev, double correlation) {
        random = new Random(seed);
        this.positionStddev = positionStddev; this.headingStddev = headingStddev;
        this.velocityStddev = velocityStddev; this.angularVelocityStddev = angularVelocityStddev;
        this.correlation = correlation;
        if (positionStddev < 0 || headingStddev < 0 || velocityStddev < 0 || angularVelocityStddev < 0
                || !Double.isFinite(correlation) || Math.abs(correlation) > 1) {
            throw new IllegalArgumentException("Invalid test noise");
        }
    }

    public Sample sample(Pose position, Pose velocity) {
        double[] p = {position.getX().getIn(), position.getY().getIn(), position.getHeading().getRad()};
        double[] v = {velocity.getX().getIn(), velocity.getY().getIn(), velocity.getHeading().getRad()};
        for (int axis = 0; axis < 3; axis++) {
            double common = random.nextGaussian();
            double independent = random.nextGaussian();
            p[axis] += common * (axis == 2 ? headingStddev : positionStddev);
            v[axis] += (correlation * common + Math.sqrt(1 - correlation * correlation) * independent)
                    * (axis == 2 ? angularVelocityStddev : velocityStddev);
        }
        return new Sample(pose(p), pose(v));
    }

    private static Pose pose(double[] values) {
        return new Pose(Vector.of(values[0], values[1], DistUnit.IN), Angle.fromRad(values[2]));
    }

    public static final class Sample {
        public final Pose position, velocity;
        private Sample(Pose position, Pose velocity) { this.position = position; this.velocity = velocity; }
    }
}
