package tuning;

import com.qualcomm.robotcore.util.ElapsedTime;

import controllers.PDSController;
import controllers.PDSController.PDSCoefficients;
import geometry.Pose;

/**
 * Base class for an Apex Pathing tuning phase.
 *
 * @author Sohum Arora - 22985 Paraducks
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public abstract class TuningPhase {
    private State state = State.INIT;
    protected final TunerContext context;
    private boolean manualMode;

    protected TuningPhase(TunerContext context) { this.context = context; }

    enum State { INIT, RUNNING, COMPLETE }

    /**
     * Updates the tuning phase. This method should be called repeatedly until it returns true.
     */
    public final boolean update(boolean aWasPressed, boolean bPressed) {
        switch (state) {
            case INIT:
                boolean initialized = initLoop(aWasPressed, bPressed);
                if (initialized) {
                    state = State.RUNNING;
                }
                break;
            case RUNNING:
                boolean complete;
                if (manualMode) {
                    complete = manualUpdate();
                } else {
                    complete = automaticUpdate();
                }

                if (complete) {
                    state = State.COMPLETE;
                }
                break;
            case COMPLETE:
                boolean exit = endLoop(bPressed);
                if (exit) {
                    return true;
                }
                break;
        }

        return false;
    }

    /**
     * @return true when initialization is complete and the tuning phase should begin running.
     */
    private boolean initLoop(boolean aWasPressed, boolean bPressed) {
        context.getTelemetry().addLine(getPhaseName() + " phase initialized");

        if (manualTuneIsPossible() && autoTuneIsPossible()) {
            context.getTelemetry().addData("Selected Mode:", manualMode ? "Manual" : "Automatic");
            context.getTelemetry().addLine("A - Toggle mode");

            if (aWasPressed) {
                manualMode = !manualMode;
            }
        }

        context.getTelemetry().addLine("B - Start tuning");
        context.getTelemetry().update();

        if (bPressed) {
            init();
            return true;
        }

        return false;
    }

    /**
     * @return true when the user has exited the phase.
     */
    private boolean endLoop(boolean bWasPressed) {
        context.getTelemetry().addLine(getPhaseName() + " phase complete with these results:");
        reportResults();
        context.getTelemetry().addLine("B - Exit phase");
        context.getTelemetry().update();

        return bWasPressed;
    }

    /**
     * @return the name of this phase as a string for display purposes.
     */
    protected abstract String getPhaseName();

    /**
     * @return true if manual tuning is possible for this phase, false otherwise.
     */
    protected abstract boolean manualTuneIsPossible();

    /**
     * @return true if automatic tuning is possible for this phase, false otherwise.
     */
    protected abstract boolean autoTuneIsPossible();

    /** Initializes the tuning phase (for automatic or manual) */
    protected abstract void init();

    /**
     * This method should perform manual tuning updates. It will be called repeatedly until it
     * returns true.
     * @return true if the manual tuning is complete, false otherwise
     */
    protected abstract boolean manualUpdate();

    /**
     * This method should perform automatic tuning updates. It will be called repeatedly until it
     * returns true.
     *
     * @return true if the automatic tuning is complete, false otherwise
     */
    protected abstract boolean automaticUpdate();

    /**
     * This method should use the telemetry (context.getTelemetry()) to report the results of
     * the tuning phase. It will be called repeatedly until the user exits the phase.
     */
    protected abstract void reportResults();

    protected static class BinarySearch {
        private double maxGuess;
        private double minGuess;
        private final double convergenceThreshold;
        private double guess;

        /**
         * Creates a new BinarySearch instance with the given parameters.
         * @param minGuess Minimum guess value for the search (it will not go below this value)
         * @param maxGuess Maximum guess value for the search (it will not go above this value)
         * @param convergenceThreshold The threshold for convergence. If the difference between
         *                             the last guess and the current guess is less than or equal
         *                             to this value, the search is considered converged.
         */
        public BinarySearch(double minGuess, double maxGuess, double convergenceThreshold) {
            this.maxGuess = maxGuess;
            this.minGuess = minGuess;
            this.convergenceThreshold = convergenceThreshold;
            this.guess = (maxGuess + minGuess) / 2;
        }

        /**
         * @return false if the guess has converged and the search should stop, true otherwise
         */
        public boolean updateGuess(boolean increase) {
            double lastGuess = guess;
            if (increase) {
                minGuess = guess;
            } else {
                maxGuess = guess;
            }
            guess = (maxGuess + minGuess) / 2;

            return !(Math.abs(lastGuess - guess) <= convergenceThreshold);
        }

        /**
         * @return the current guess value
         */
        public double getGuess() {
            return guess;
        }
    }

    protected static class PDSRoutine {
        final double HAS_MOVED_THRESHOLD = 0.025;
        final double HAS_MOVED_THRESHOLD_HEADING = 0.01;
        final double TIME_PER_GUESS_MS = 500;
        final double PD_TUNER_DURATION = 2000;

        private Axis axis;
        private ElapsedTime timer = new ElapsedTime();
        private PDSController controller;
        private BinarySearch kSSearch;

        private boolean tuningKS = true;
        private double startTime = 0;
        private double maxAccel = 0; // inches/radians per second squared
        private double velAtTimestamp = 0; // inches/radians per second
        private double timestamp = 0;

        public enum Axis { DRIVE, STRAFE, HEADING }

        public PDSRoutine(TunerContext context, Axis axis) {
            this.kSSearch = new BinarySearch(0, 0.4, 0.01);
            this.axis = axis;
            context.getFollower().disableAllControllers();
            context.getFollower().setPose(Pose.zero());

            this.controller = new PDSController(new PDSCoefficients());
            if (this.axis == Axis.HEADING) { this.controller.setAngularController(); }
        }

        public void start() {
            timer.reset();
            controller.getCoefficients().setkS(kSSearch.getGuess());
        }

        private void moveInDirection(TunerContext context, double power) {
            switch (axis) {
                case DRIVE:
                    context.getFollower().getDrivetrain().moveWithVectors(power, 0, 0);
                    break;
                case STRAFE:
                    context.getFollower().getDrivetrain().moveWithVectors(0, power, 0);
                    break;
                case HEADING:
                    context.getFollower().getDrivetrain().moveWithVectors(0, 0, power);
                    break;
            }
        }

        public double getPoseAxis(Pose pose) {
            double value = 0;
            switch (axis) {
                case DRIVE:
                    value = pose.getX().getIn();
                    break;
                case STRAFE:
                    value = pose.getY().getIn();
                    break;
                case HEADING:
                    value = pose.getHeading().getRad();
                    break;
            }
            return value;
        }

        public boolean update(TunerContext context) {
            if (tuningKS) {
                moveInDirection(context, controller.calculate(getPoseAxis(context.getFollower().getPose())));
                if (timer.milliseconds() >= TIME_PER_GUESS_MS) {
                    Pose pose = context.getFollower().getPose();
                    tuningKS = !kSSearch.updateGuess(
                            Math.abs(pose.getX().getIn()) > HAS_MOVED_THRESHOLD ||
                                    Math.abs(pose.getY().getIn()) > HAS_MOVED_THRESHOLD ||
                                    Math.abs(pose.getHeading().getRad()) > HAS_MOVED_THRESHOLD_HEADING
                    );
                    if (tuningKS) {
                        controller.getCoefficients().setkS(kSSearch.getGuess());
                    } else { // Moving to PD
                        startTime = System.nanoTime();
                    }
                    timer.reset();
                    context.getFollower().setPose(Pose.zero());
                    context.getFollower().stop();
                }
            } else { // Tuning PD
                moveInDirection(context, 1.0);
                double accel = getPoseAxis(context.getFollower().getAcceleration());
                if (accel > maxAccel) {
                    maxAccel = accel;
                    timestamp = (System.nanoTime() - startTime) / 1.0e9;
                    velAtTimestamp = getPoseAxis(context.getFollower().getVelocity());
                }
                if (timer.milliseconds() >= PD_TUNER_DURATION) {
                    // Derive kP and kD using Ziegler-Nichols formulas
                    double L = timestamp - (velAtTimestamp / maxAccel); // Delay time
                    controller.getCoefficients().setkP(1.2 / (L * maxAccel));
                    controller.getCoefficients().setkD(0.6 / maxAccel);
                    return true;
                }
            }
            return false;
        }

        public PDSCoefficients getCoefficients() {
            return controller.getCoefficients();
        }
    }
}