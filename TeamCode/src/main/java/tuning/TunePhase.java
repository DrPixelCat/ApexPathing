package tuning;

/**
 * Base class for an Apex Pathing tuning phase.
 *
 * @author Sohum Arora - 22985 Paraducks
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public abstract class TunePhase {
    private State state = State.INIT;
    protected final TuneContext context;
    private boolean manualMode;

    protected TunePhase(TuneContext context) { this.context = context; }

    enum State { INIT, RUNNING, COMPLETE }

    /**
     * Updates the tuning phase. This method should be called repeatedly until it returns true.
     */
    public final boolean update(boolean aWasPressed, boolean bPressed) {
        switch (state) {
            case INIT:
                boolean initialized = showStart(aWasPressed, bPressed);
                if (initialized) {
                    state = State.RUNNING;
                }
                break;
            case RUNNING:
                boolean complete;
                if (manualMode) {
                    complete = runManual(aWasPressed, bPressed);
                } else {
                    complete = runAuto(aWasPressed, bPressed);
                }

                if (complete) {
                    state = State.COMPLETE;
                }
                break;
            case COMPLETE:
                showEnd();
                if (bPressed) {
                    return true;
                }
                break;
        }

        return false;
    }

    /**
     * @return true when initialization is complete and the tuning phase should begin running.
     */
    private boolean showStart(boolean aWasPressed, boolean bPressed) {
        context.getTelemetry().addLine(name() + " phase initialized");

        if (hasManual() && hasAuto()) {
            if (aWasPressed) {
                manualMode = !manualMode;
            }
            context.getTelemetry().addData("Selected Mode", manualMode ? "MANUAL" : "AUTOMATIC");
            context.getTelemetry().addLine("A - Toggle automatic/manual");
        }

        context.getTelemetry().addLine("B - Start tuning");
        context.getTelemetry().update();

        if (bPressed) {
            start();
            return true;
        }

        return false;
    }

    private void showEnd() {
        context.getTelemetry().addLine(name() + " phase complete with results:");
        showResults();
        context.getTelemetry().addLine("B - Exit");
        context.getTelemetry().update();
    }

    /**
     * @return the name of this phase as a string for display purposes.
     */
    protected abstract String name();

    /**
     * @return true if manual tuning is possible for this phase, false otherwise.
     */
    protected abstract boolean hasManual();

    /**
     * @return true if automatic tuning is possible for this phase, false otherwise.
     */
    protected abstract boolean hasAuto();

    /** Initializes the tuning phase (for automatic or manual) */
    protected abstract void start();

    /**
     * This method should perform manual tuning updates. It will be called repeatedly until it
     * returns true.
     * @return true if the manual tuning is complete, false otherwise
     */
    protected abstract boolean runManual(boolean aWasPressed, boolean bWasPressed);

    protected final boolean isManual() { return manualMode; }

    /**
     * This method should perform automatic tuning updates. It will be called repeatedly until it
     * returns true.
     *
     * @return true if the automatic tuning is complete, false otherwise
     */
    protected abstract boolean runAuto(boolean aWasPressed, boolean bWasPressed);

    /**
     * This method should use the telemetry (context.getTelemetry()) to report the results of
     * the tuning phase. It will be called repeatedly until the user exits the phase.
     */
    protected abstract void showResults();
}
