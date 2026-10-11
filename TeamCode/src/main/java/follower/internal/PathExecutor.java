package follower.internal;

import follower.Follower;

import geometry.PathSegment;
import geometry.Pose;
import localizers.BaseLocalizer;
import paths.movements.Path;

/** Internal tracking algorithm operating on the lifecycle state owned by one follower. */
interface PathExecutor {
    Follower.MovementOutcome update(Path path, PathSegment segment, Pose current,
                                   BaseLocalizer<?> localizer, MovementCallbackRunner callbacks,
                                   FollowerPathState state, boolean headingEnabled, boolean driveEnabled);
}
