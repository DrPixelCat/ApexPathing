package trajectory;

import trajectory.profile.FFLut;
import trajectory.profile.MotionParameters;
import paths.geometry.PathSegment;
import paths.geometry.Vector;
import paths.movements.Path;

/**
 * Immutable rectangular, bilinearly interpolated field. No per-node Vector objects are retained.
 *
 * @author Joel H
 */
public final class TrajectoryField2D {
    private static final int CHANNELS = 12;
    private final PathSegment segment;
    private final paths.heading.HeadingInterpolator interpolator;
    private final FFLut nominal;
    private final double spacing, minX, minY;
    private final int columns, rows;
    private final double[] data;

    public TrajectoryField2D(Path path, FFLut nominal, double spacing, double padding) {
        if (!Double.isFinite(spacing) || spacing <= 0 || !Double.isFinite(padding) || padding < 0) {
            throw new IllegalArgumentException("Positive spacing and nonnegative padding required.");
        }
        this.segment = path.getParametricPath(); this.interpolator = path.getInterpolator().copyForTrajectory();
        this.nominal = nominal; this.spacing = spacing;
        double[] bounds = segment.getBoundsIn();
        double x0 = bounds[0], y0 = bounds[1], x1 = bounds[2], y1 = bounds[3];
        minX = x0 - padding; minY = y0 - padding;
        double length = Math.ceil((x1 - x0 + 2 * padding) / spacing) + 1;
        double width = Math.ceil((y1 - y0 + 2 * padding) / spacing) + 1;
        if (!Double.isFinite(length * width) || length * width * CHANNELS > Integer.MAX_VALUE - 8) {
            throw new IllegalArgumentException("Grid is too large; increase spacing.");
        }
        columns = Math.max(2, (int) length);
        rows = Math.max(2, (int) width);
        long storageBytes = (long) columns * rows * CHANNELS * Double.BYTES;
        if (storageBytes > 32L * 1024 * 1024) {
            throw new IllegalArgumentException("Trajectory grid exceeds 32 MiB; increase spacing or use online planning");
        }
        data = new double[Math.multiplyExact(Math.multiplyExact(columns, rows), CHANNELS)];
        for (int row = 0; row < rows; row++) {
            for (int col = 0; col < columns; col++) {
                Vector position = TrajectorySample.vector(minX + col * spacing, minY + row * spacing);
                double t = segment.getBestT(position);
                double s = segment.getLengthIn() - segment.getDistanceToEndIn(segment.getPosition(t), t);
                MotionParameters motion = s <= nominal.getFirstIntervalEnd() || s >= nominal.getLastIntervalStart()
                        ? nominal.getKinematicParams(s) : nominal.getFFParams(s);
                TrajectorySample value = TrajectorySample.onPath(path, s, position, motion, false);
                int offset = (row * columns + col) * CHANNELS;
                data[offset] = value.velocity.getX().getIn(); data[offset + 1] = value.velocity.getY().getIn();
                data[offset + 2] = value.acceleration.getX().getIn(); data[offset + 3] = value.acceleration.getY().getIn();
                data[offset + 4] = value.toPath.getX().getIn(); data[offset + 5] = value.toPath.getY().getIn();
                data[offset + 6] = value.tangent.getX().getIn(); data[offset + 7] = value.tangent.getY().getIn();
                data[offset + 8] = value.progressIn; data[offset + 9] = value.angularVelocity; data[offset + 10] = value.angularAcceleration;
                data[offset + 11] = value.expectedMotorPower;
            }
        }
    }

    public double getMinXIn() { return minX; }
    public double getMinYIn() { return minY; }
    public double getMaxXIn() { return minX + (columns - 1) * spacing; }
    public double getMaxYIn() { return minY + (rows - 1) * spacing; }
    public double getSpacingIn() { return spacing; }
    public int getNodeCount() { return columns * rows; }
    public long getStorageBytes() { return (long) data.length * Double.BYTES; }

    public TrajectorySample sample(Vector position) {
        if (!position.isFinite()) { throw new IllegalArgumentException("Finite field position required."); }
        double x = Math.max(minX, Math.min(getMaxXIn(), position.getX().getIn()));
        double y = Math.max(minY, Math.min(getMaxYIn(), position.getY().getIn()));
        double gridX = (x - minX) / spacing, gridY = (y - minY) / spacing;
        int col = Math.min(columns - 2, (int) Math.floor(gridX));
        int row = Math.min(rows - 2, (int) Math.floor(gridY));
        double tx = gridX - col, ty = gridY - row;
        int a = (row * columns + col) * CHANNELS, b = a + CHANNELS;
        int c = a + columns * CHANNELS, d = c + CHANNELS;
        double[] output = new double[CHANNELS];
        for (int channel = 0; channel < CHANNELS; channel++) {
            output[channel] = (1 - ty) * ((1 - tx) * data[a + channel] + tx * data[b + channel])
                    + ty * ((1 - tx) * data[c + channel] + tx * data[d + channel]);
        }
        double s = Math.max(0, Math.min(segment.getLengthIn(), output[8]));
        double t = segment.getTAtDistance(s), remaining = segment.getLengthIn() - s;
        Vector tangent = TrajectorySample.vector(output[6], output[7]).normalize();
        if (tangent.getMag().getIn() < 1e-9) { tangent = segment.getFirstDerivative(t).normalize(); }
        Vector velocity = TrajectorySample.vector(output[0], output[1]);
        Vector acceleration = TrajectorySample.vector(output[2], output[3]);
        double omega = output[9], alpha = output[10];
        double expectedPower = output[11];
        // Bilinear blending of endpoint zero rows must not erase launch or terminal braking.
        MotionParameters boundary = nominal.getKinematicParams(s);
        // Interpolation across a sharp reduction must respect the stored stopping envelope.
        if (s <= nominal.getFirstIntervalEnd() || s >= nominal.getLastIntervalStart()
                || velocity.getMag().getIn() > boundary.getTangentialVel() + 1e-6) {
            velocity = tangent.times(boundary.getTangentialVel());
            acceleration = tangent.times(boundary.getTangentialAccel()).plus(PathSegment.calculateLeftNormal(tangent)
                    .times(boundary.getTangentialVel() * boundary.getTangentialVel() * segment.getSignedCurvature(t)));
            omega = boundary.getAngularVel(); alpha = boundary.getAngularAccel();
            expectedPower = boundary.getMotorPower();
        }
        Vector representedPoint = TrajectorySample.vector(x + output[4], y + output[5]);
        // Tangent heading styles must use the interpolated tangent too. A tiny mismatch
        // otherwise activates a second calibrated body-axis feedforward near straight sections.
        return new TrajectorySample(velocity, acceleration, representedPoint.minus(position), tangent, representedPoint,
                s, remaining, t, interpolator.getHeadingTarg(remaining, tangent, segment.getFirstDerivative(1)),
                omega, alpha, false, x != position.getX().getIn() || y != position.getY().getIn())
                .withExpectedMotorPower(expectedPower);
    }
}
