package paths.movements;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import paths.callbacks.Callback;
import paths.heading.HeadingInterpolator;
import geometry.PathSegment;
import geometry.Pose;

/**
 * Represents a complete, navigable geometric route for the robot to follow.
 * <p>
 * A {@code Path} encapsulates a continuous parametric curve (e.g., a B-Spline),
 * its associated heading interpolation strategy, and any scheduled mechanical
 * callbacks triggered along the route.
 * <p>
 * @author DrPixelCat
 * @author Sohum Arora 22985 Paraducks
 */
public class Path implements FollowerMovement {
    private final List<String> buildWarnings = new ArrayList<>();
    private final ArrayList<Callback> callbacks = new ArrayList<>();

    private PathSegment parametricPath;
    private HeadingInterpolator interpolator;
    private Pose endPose;

    /**
     * Attaches an executable mechanical/software action to this path.
     * * @param callback The callback object defining the trigger point and executable action.
     */
    public void addCallback(Callback callback) { callbacks.add(callback); }

    /**
     * Retrieves all scheduled actions attached to this path.
     * * @return An array of callbacks scheduled along the route.
     */
    public Callback[] getCallbacks() { return callbacks.toArray(new Callback[0]); }

    /**
     * Sets the final target pose (coordinates and heading) of this path.
     * * @param endPose The geometric terminus of the route.
     */
    public void setEndPose(Pose endPose) { this.endPose = endPose; }

    /**
     * Retrieves the final target pose of this path.
     * * @return The geometric terminus of the route.
     */
    public Pose getEndPose() { return endPose; }

    /**
     * Injects the calculated geometric curve (e.g., a B-Spline) that defines
     * the physical route the robot will drive.
     * * @param parametricPath The compiled path segment.
     */
    public void setParametricPath(PathSegment parametricPath) {
        this.parametricPath = parametricPath;
    }

    /**
     * Retrieves the geometric curve defining the physical route.
     * * @return The compiled path segment.
     */
    public PathSegment getParametricPath() { return parametricPath; }

    /**
     * Injects the strategy used to calculate the robot's target heading
     * at any point along the curve.
     * * @param interpolator The heading generation strategy.
     */
    public void setInterpolator(HeadingInterpolator interpolator) {
        this.interpolator = interpolator;
    }

    /**
     * Retrieves the heading strategy for this path.
     * * @return The heading interpolator.
     */
    public HeadingInterpolator getInterpolator() { return interpolator; }

    /**
     * Logs a non-fatal warning generated during the path building process
     * (e.g., missing headings, ignored waypoints).
     * <p>
     * Duplicate warnings are ignored to prevent telemetry spam on the driver station.
     *
     * @param warning The localized warning string.
     */
    public void addWarning(String warning) {
        if (!buildWarnings.contains(warning)) {
            buildWarnings.add(warning);
        }
    }

    /**
     * Retrieves an unmodifiable view of all warnings generated during path construction.
     * These should be pushed to the driver station telemetry during initialization.
     *
     * @return A read-only list of warning strings.
     */
    public List<String> getWarnings() {
        // Return an unmodifiable view to prevent external modification of the internal list
        return Collections.unmodifiableList(buildWarnings);
    }
}