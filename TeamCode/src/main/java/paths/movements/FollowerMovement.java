package paths.movements;

import geometry.Pose;

/**
 * Abstract base class extended by {@link Path} and {@link Turn}
 * Used in order to track the robot's progression along a particular Path or a Turn, enabling the
 * implementation of robust FSMs in autonomous code
 *
 * @author Sohum Arora 22985 Paraducks
 */
public abstract class FollowerMovement {
    private boolean started = false;
    private boolean ended = false;

    /**
     * Gets the expected final factory of the robot after this movement is completed.
     * This is critical for linking sequential builders together!
     *
     * @return The final Pose.
     */
    public abstract Pose getEndPose();

    /**
     * Below are methods to track a robot's progress along a given FollowerMovement, enabling the
     * implementation of a robust FSM
     */
    public boolean hasStarted() {
        return started;
    }

    public boolean hasEnded() {
        return ended;
    }

    public void setStarted(boolean started) {
        this.started = started;
    }

    public void setEnded(boolean ended) {
        this.ended = ended;
    }
    public Path toPath() {return (Path) this;}
    public Turn toTurn() {return (Turn) this;}
}