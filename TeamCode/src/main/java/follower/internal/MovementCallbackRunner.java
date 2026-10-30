package follower.internal;


import paths.geometry.Angle;
import paths.Callback;
import paths.movements.FollowerMovement;
import paths.movements.Path;
import paths.movements.Turn;

/** Evaluates movement callbacks and tracks angular sweeps between follower ticks. */
final class MovementCallbackRunner {
    private Angle lastHeading;
    private final java.util.Set<Callback> fired = java.util.Collections.newSetFromMap(new java.util.IdentityHashMap<>());

    void process(FollowerMovement movement, double progress, Angle currentHeading) {
        Callback[] callbacks = callbacksFor(movement);
        if (callbacks != null) {
            for (Callback callback : callbacks) {
                if (!fired.contains(callback) && shouldTrigger(callback, progress, currentHeading)) {
                    fired.add(callback);
                    callback.getAction().run();
                    callback.setTriggered(true);
                }
            }
        }
        lastHeading = currentHeading;
    }

    void reset() {
        lastHeading = null;
        fired.clear();
    }

    private static Callback[] callbacksFor(FollowerMovement movement) {
        if (movement instanceof Path || movement instanceof Turn) {
            return movement.getCallbacks();
        }
        return null;
    }

    private boolean shouldTrigger(Callback callback, double progress, Angle currentHeading) {
        if (callback.getType() == Callback.CallbackType.DISTANCE) {
            return progress >= 0.0 && progress >= callback.getS();
        }
        if (callback.getType() != Callback.CallbackType.ANGLE) {
            return false;
        }

        double error = Math.abs(currentHeading.getShortestAngleTo(callback.getTheta()).getRad());
        if (error < Math.toRadians(1.0)) {
            return true;
        }
        if (lastHeading == null) {
            return false;
        }

        double tickSweep = lastHeading.getShortestAngleTo(currentHeading).getRad();
        double targetSweep = lastHeading.getShortestAngleTo(callback.getTheta()).getRad();
        return Math.signum(tickSweep) == Math.signum(targetSweep)
                && Math.abs(targetSweep) <= Math.abs(tickSweep);
    }
}
