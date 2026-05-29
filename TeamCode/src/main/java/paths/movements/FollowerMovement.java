package paths.movements;

import geometry.Pose;

/**
 * A marker interface representing any executable navigational action 
 * (e.g., driving a spline path, executing a point turn, or holding a pose).
 * <p>
 * Your Follower class will maintain a queue or list of these movements 
 * and execute them sequentially.
 */
public interface FollowerMovement {
    
    /**
     * Gets the expected final pose of the robot after this movement is completed.
     * This is critical for linking sequential builders together!
     * @return The final Pose.
     */
    Pose getEndPose();
}