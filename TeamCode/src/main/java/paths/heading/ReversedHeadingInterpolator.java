package paths.heading;

import paths.geometry.Angle;
import paths.geometry.Vector;

/** Replays another heading interpolator over the same path in reverse traversal order. */
public final class ReversedHeadingInterpolator implements HeadingInterpolator {
    private final HeadingInterpolator source;
    private final Vector sourceFinalTangent;
    private double pathLength;
    private long configurationRevision;
    @Override public boolean supportsTank() { return source.supportsTank(); }

    public ReversedHeadingInterpolator(HeadingInterpolator source, double pathLength,
                                       Vector sourceFinalTangent) {
        this.source = source;
        this.pathLength = pathLength;
        this.sourceFinalTangent = sourceFinalTangent;
    }

    @Override
    public Angle getHeadingTarg(double s, Vector pathTangent, Vector finalTangent) {
        return source.getHeadingTarg(pathLength - s, pathTangent.times(-1.0),
                sourceFinalTangent);
    }

    @Override
    public double getHeadingFirstDerivative(double s, double kappa, Vector finalTangent) {
        return -source.getHeadingFirstDerivative(pathLength - s, -kappa,
                sourceFinalTangent);
    }

    @Override
    public double getHeadingSecondDerivative(double s, double dKappa, Vector finalTangent) {
        return source.getHeadingSecondDerivative(pathLength - s, dKappa,
                sourceFinalTangent);
    }

    @Override
    public void setPathLength(double lengthInches) { this.pathLength = lengthInches; configurationRevision++; }

    @Override public long getConfigurationRevision() {
        long revision = source.getConfigurationRevision();
        return revision < 0 ? -1 : revision + configurationRevision;
    }

    @Override public HeadingInterpolator copyForTrajectory() {
        return new ReversedHeadingInterpolator(source.copyForTrajectory(), pathLength, sourceFinalTangent);
    }
}
