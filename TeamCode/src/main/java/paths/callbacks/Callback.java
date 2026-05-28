package paths.callbacks;

public interface Callback {
    /**
     * Gets the action to be executed when the callback condition is met.
     */
    Runnable getAction();

    /**
     * Checks if the callback has already been triggered.
     */
    boolean isTriggered();

    /**
     * Sets the triggered state of the callback.
     */
    void setTriggered(boolean triggered);
}