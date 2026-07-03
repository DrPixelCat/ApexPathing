package paths.builders;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.function.Function;

import core.FollowerConstants;
import feedforward.FeedforwardLut;
import feedforward.holonomic.mecanum.MecanumProfileGenerator;
import feedforward.holonomic.swerve.SwerveProfileGenerator;
import geometry.Angle;
import geometry.ArcPose;
import geometry.BSpline;
import geometry.CubicSpline1D;
import geometry.Dist;
import geometry.PathSegment;
import geometry.Pose;
import geometry.Vector;
import paths.callbacks.Callback;
import paths.constraint.PathConstraint;
import paths.heading.HeadingNode;
import paths.heading.HolonomicInterpolationStyle;
import paths.heading.HolonomicInterpolator;
import paths.movements.Path;

/**
 * A builder class designed to construct a {@link Path} fluently for holonomic drivetrains.
 * This class captures path configurations (waypoints, interpolators, callbacks)
 * in any order and defers geometric and kinematic compilation until a build method is called.
 *
 * @author DrPixelCat
 * @author Dylan B. - 18597 RoboClovers - Delta
 * @author
 */
public class HolonomicPathBuilder {
    public Path path;
    private final Pose[] rawPoses;
    private final Pose startPose;
    private final Pose expectedEndPose;
    private Dist blendWindow = null;

    private HolonomicInterpolationStyle currentStyle =
            HolonomicInterpolationStyle.SMOOTH_START_TO_END;
    private Angle customOffset = null;
    private final Function<Double, Angle> customFunction = null;

    private final List<Runnable> buildTasks = new ArrayList<>();
    private final List<HeadingNode> headingNodes = new ArrayList<>();

    /**
     * Creates a new HolonomicPathBuilder using the provided poses.
     *
     * @param poses A sequence of Pose objects defining the path. Must contain at least two poses
     *              . Endpoints cannot be ArcPoses.
     */
    public HolonomicPathBuilder(Pose... poses) {
        this.path = new Path(Path.PathType.HOLONOMIC);
        if (poses.length < 2) {
            throw new IllegalArgumentException("A B-Spline must be created with > 1 points!");
        }
        if (poses[0] instanceof ArcPose || poses[poses.length - 1] instanceof ArcPose) {
            throw new IllegalArgumentException("Endpoints can't be arcs!");
        }
        this.rawPoses = poses;
        this.startPose = poses[0];
        this.expectedEndPose = poses[poses.length - 1];
    }

    /**
     * Overrides the default (SMOOTH_START_TO_END) interpolation with a different
     * {@link HolonomicInterpolationStyle}.
     *
     * @param style The interpolation style to apply.
     * @return The current HolonomicPathBuilder instance for method chaining.
     */
    public HolonomicPathBuilder interpolateWith(HolonomicInterpolationStyle style) {
        this.currentStyle = style;
        return this;
    }

    /**
     * Overrides the interpolation style, providing a custom angular offset.
     * Used primarily for {@link HolonomicInterpolationStyle#TANGENT_CUSTOM}.
     *
     * @param style       The interpolation style to apply.
     * @param angleOffset The fixed angle to offset the calculation by.
     * @return The current HolonomicPathBuilder instance for method chaining.
     */
    public HolonomicPathBuilder interpolateWith(HolonomicInterpolationStyle style,
                                                Angle angleOffset) {
        this.currentStyle = style;
        this.customOffset = angleOffset;
        return this;
    }

    /**
     * Adds a heading node for NODE_BASED interpolation.
     * Automatically sets the interpolation style to NODE_BASED.
     *
     * @param pct    The distance percentage [0.0, 1.0].
     * @param target The target Angle at this point.
     * @return The current HolonomicPathBuilder instance for method chaining.
     */
    public HolonomicPathBuilder addHeadingNode(double pct, Angle target) {
        this.currentStyle = HolonomicInterpolationStyle.NODE_BASED;
        this.headingNodes.add(new HeadingNode(pct, target));
        return this;
    }

    /**
     * Sets how far from the end of the path the robot should start rotating to face its final
     * target direction.
     *
     * @param distanceFromEnd The distance away from the end of the path.
     * @return The current HolonomicPathBuilder instance for method chaining.
     */
    public HolonomicPathBuilder setDistanceToStartFinalTurn(Dist distanceFromEnd) {
        this.blendWindow = distanceFromEnd;
        return this;
    }

    /**
     * Adds a kinematic constraint to the path at a specific distance percentage from 0 to 1.
     * <p>
     * NOTE: Only velocity can be limited on a quickBuild
     * </p>
     *
     * @param constraint The {@link PathConstraint} to apply
     * @return The current HolonomicPathBuilder instance for method chaining.
     */
    public HolonomicPathBuilder addConstraint(PathConstraint constraint) {
        if (constraint.getS() >= 1.0 || constraint.getS() < 0.0) {
            constraint.setS(Math.min(Math.max(constraint.getS(), 0.0), 0.9));
            path.addWarning("s must be within [0, 1] bounds! Normalized to " + constraint.getS() + " for safety.");
        }
        path.addConstraint(constraint);
        return this;
    }

    /**
     * Attaches an executable callback based on the physical distance percentage.
     *
     * @param s      The physical distance percentage [0.0, 1.0].
     * @param action The code to execute.
     * @return The current HolonomicPathBuilder instance for method chaining.
     */
    public HolonomicPathBuilder addDistanceCallback(double s, Runnable action) {
        buildTasks.add(() -> path.addCallback(new Callback(s, action)));
        return this;
    }

    /**
     * Attaches an executable callback based on the robot reaching a target heading.
     *
     * @param angle  The Angle at which the callback should trigger.
     * @param action The code to execute.
     * @return The current HolonomicPathBuilder instance for method chaining.
     */
    public HolonomicPathBuilder addAngularCallback(Angle angle, Runnable action) {
        buildTasks.add(() -> {
            if (currentStyle == HolonomicInterpolationStyle.SMOOTH_START_TO_END) {
                Angle startRad = rawPoses[0].getHeading();
                Angle endRad = expectedEndPose.getHeading();

                if (Double.isFinite(startRad.getRad()) && Double.isFinite(endRad.getRad())) {
                    double totalDiff = startRad.getShortestAngleTo(endRad).getRad();
                    double targetDiff = startRad.getShortestAngleTo(angle).getRad();

                    if (Math.abs(totalDiff) < 1e-6) {
                        if (Math.abs(targetDiff) > 1e-6) {
                            throw new IllegalArgumentException("Angular callback out of bounds: " +
                                    "The path's target heading is constant.");
                        }
                    } else if ((totalDiff * targetDiff < 0) || (Math.abs(targetDiff) > Math.abs(totalDiff))) {
                        throw new IllegalArgumentException("Angular callback is outside the sweep" +
                                " range of the start and end headings.");
                    }
                }
            }
            path.addCallback(new Callback(angle, action));
        });
        return this;
    }

    /**
     * Internal method to compile the B-Spline geometry, process arc poses, and initialize the
     * interpolator.
     */
    private void compileGeometry() {
        ArrayList<Pose> processedPoses = new ArrayList<>(rawPoses.length * 2);
        processedPoses.add(rawPoses[0]);

        boolean intermediateWarningSent = false;

        for (int i = 1; i < rawPoses.length - 1; i++) {
            Pose currentPose = rawPoses[i];

            if (!intermediateWarningSent && Double.isFinite(currentPose.getHeading().getRad())) {
                path.addWarning("APEX WARNING: Intermediate B-Spline headings are currently " +
                        "ignored! Only the final pose heading controls the end heading.");
                intermediateWarningSent = true;
            }

            if (currentPose instanceof ArcPose) {
                ArcPose arcPose = (ArcPose) currentPose;
                double radius = arcPose.getRadius().getIn();

                if (radius < 2.0) {
                    throw new IllegalArgumentException("ArcPose radius must be at least 2.0 " +
                            "inches.");
                }

                Pose prevPose = rawPoses[i - 1];
                Pose nextPose = rawPoses[i + 1];

                Vector vecToLast = prevPose.getVec().minus(arcPose.getVec());
                Vector vecToNext = nextPose.getVec().minus(arcPose.getVec());

                double distToLast = vecToLast.getMag().getIn();
                double distToNext = vecToNext.getMag().getIn();

                if (radius > distToLast || radius > distToNext) {
                    throw new IllegalArgumentException("ArcPose radius exceeds distance to " +
                            "adjacent control points.");
                }

                Vector p1Vec = arcPose.getVec().plus(vecToLast.times(radius / distToLast));
                Vector p2Vec = arcPose.getVec().plus(vecToNext.times(radius / distToNext));

                processedPoses.add(new Pose(p1Vec, arcPose.getHeading()));
                processedPoses.add(currentPose);
                processedPoses.add(new Pose(p2Vec, arcPose.getHeading()));
            } else {
                processedPoses.add(currentPose);
            }
        }

        processedPoses.add(rawPoses[rawPoses.length - 1]);

        Vector[] vectors = new Vector[processedPoses.size()];
        for (int i = 0; i < processedPoses.size(); i++) {
            vectors[i] = processedPoses.get(i).getVec();
        }

        PathSegment curve = new PathSegment(new BSpline(vectors));
        path.setParametricPath(curve);

        Angle startH = startPose.getHeading();
        Angle endH = expectedEndPose.getHeading();

        CubicSpline1D spline = null;
        if (currentStyle == HolonomicInterpolationStyle.NODE_BASED) {

            // Automatically inject path boundary headings if the user didn't explicitly define them
            boolean hasStart = false;
            boolean hasEnd = false;
            for (HeadingNode node : headingNodes) {
                if (Math.abs(node.pct - 0.0) < 1e-6) hasStart = true;
                if (Math.abs(node.pct - 1.0) < 1e-6) hasEnd = true;
            }

            if (!hasStart && Double.isFinite(startH.getRad())) {
                headingNodes.add(new HeadingNode(0.0, startH));
            }
            if (!hasEnd && Double.isFinite(endH.getRad())) {
                headingNodes.add(new HeadingNode(1.0, endH));
            }

            Collections.sort(headingNodes);

            if (headingNodes.size() < 2) {
                throw new IllegalStateException("NODE_BASED interpolation requires at least a " +
                        "start and end heading.");
            }

            double[] x = new double[headingNodes.size()];
            double[] y = new double[headingNodes.size()];

            x[0] = headingNodes.get(0).pct;
            y[0] = headingNodes.get(0).target.getRad();

            // Unwrap shortest delta to ensure smooth continuous math across 2-PI bounds
            for (int i = 1; i < headingNodes.size(); i++) {
                x[i] = headingNodes.get(i).pct;
                double shortestDelta =
                        Angle.fromRad(y[i - 1]).getShortestAngleTo(headingNodes.get(i).target).getRad();
                y[i] = y[i - 1] + shortestDelta;
            }
            spline = new CubicSpline1D(x, y);
        }

        boolean missingParams =
                (currentStyle == HolonomicInterpolationStyle.CONSTANT_START_HEADING && !Double.isFinite(startH.getRad())) ||
                        (currentStyle == HolonomicInterpolationStyle.CONSTANT_END_HEADING && !Double.isFinite(endH.getRad())) ||
                        (currentStyle == HolonomicInterpolationStyle.TANGENT_CUSTOM && (customOffset == null || !Double.isFinite(customOffset.getRad()))) ||
                        (currentStyle == HolonomicInterpolationStyle.SMOOTH_START_TO_END && (!Double.isFinite(startH.getRad()) || !Double.isFinite(endH.getRad())));

        if (missingParams) {
            path.addWarning("APEX WARNING: " + currentStyle.name() + " is missing required " +
                    "parameters! Falling back to TANGENT_FORWARD.");
            currentStyle = HolonomicInterpolationStyle.TANGENT_FORWARD;
        }

        if (currentStyle == HolonomicInterpolationStyle.TANGENT_OPTIMAL) {
            if (!Double.isFinite(startH.getRad())) {
                currentStyle = HolonomicInterpolationStyle.TANGENT_FORWARD;
            } else {
                Vector startTangent = curve.getFirstDerivative(0.0);
                double fwdError =
                        Math.abs(startH.getShortestAngleTo(startTangent.getTheta()).getRad());
                double bwdError =
                        Math.abs(startH.getShortestAngleTo(
                                startTangent.getTheta().plus(Angle.fromRad(Math.PI))).getRad());
                if (bwdError < fwdError) {
                    currentStyle = HolonomicInterpolationStyle.TANGENT_CUSTOM;
                    customOffset = Angle.fromRad(Math.PI);
                } else {
                    currentStyle = HolonomicInterpolationStyle.TANGENT_FORWARD;
                }
            }
        }

        HolonomicInterpolator interpolator = new HolonomicInterpolator(currentStyle, startH, endH
                , customOffset, spline);
        interpolator.setPathLength(curve.getLengthIn());
        if (blendWindow != null) {
            interpolator.setBlendWindow(blendWindow.getIn());
        }
        path.setInterpolator(interpolator);
        path.setEndPose(expectedEndPose);

        for (Runnable task : buildTasks) {
            task.run();
        }
    }

    /**
     * Builds the path geometry without generating a physical motion profile.
     * The follower will automatically use dynamic velocity-bounded feedback.
     *
     * @return The constructed Path.
     */
    public Path quickBuild() {
        compileGeometry();
        path.setFeedforwardLut(null);
        return path;
    }

    /**
     * Builds the path geometry and solves a complete kinematically constrained feedforward
     * motion profile.
     *
     * @return The constructed Path with an attached FeedforwardLut.
     */
    public Path profiledBuild() {
        compileGeometry();
        FollowerConstants config = new FollowerConstants();

        if (config.drivetrainType == FollowerConstants.DrivetrainType.COAXIAL_SWERVE) {
            SwerveProfileGenerator generator = new SwerveProfileGenerator(config, path);
            path.setFeedforwardLut(new FeedforwardLut(generator.generate()));
        } else {
            MecanumProfileGenerator generator = new MecanumProfileGenerator(config, path);
            path.setFeedforwardLut(new FeedforwardLut(generator.generate()));
        }

        return path;
    }
}
