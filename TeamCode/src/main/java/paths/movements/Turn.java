package paths.movements;

import java.util.ArrayList;
import java.util.List;

import paths.callbacks.Callback;
import geometry.Angle;
import geometry.Pose;

/**
 * Represents a stationary point-turn movement.
 * <p>
 * The robot will remain at its starting (x, y) coordinates and rotate 
 * to the specified target heading.
 */
public class Turn implements FollowerMovement {
    private final Pose startPose;
    private final Pose endPose;
    private final List<Callback> callbacks = new ArrayList<>();

    /**
     * Constructs a Turn movement.
     * @param startPose The robot's state at the beginning of the turn.
     * @param targetHeading The final angle the robot should face.
     */
    public Turn(Pose startPose, Angle targetHeading) {
        this.startPose = startPose;
        // The end pose shares the same X/Y, but updates the heading
        this.endPose = new Pose(startPose.getPos(), targetHeading);
    }

    public void addCallback(Callback callback) { callbacks.add(callback); }

    public Callback[] getCallbacks() { return callbacks.toArray(new Callback[0]); }

    public Pose getStartPose() { return startPose; }

    @Override
    public Pose getEndPose() { return endPose; }
}