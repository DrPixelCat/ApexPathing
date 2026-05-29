package controllers;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import geometry.Angle;
import geometry.Dist;

/**
 * Base class for all controllers. Handles common logic like tolerance checking, deadzone, and time anomaly detection.
 * Subclasses implement the specific control algorithm in computeOutput().
 *
 * @author Joel - 7842 Browncoats Alumni
 * @author Dylan B. 18597 RoboClovers - Delta
 */
public abstract class Controller {
    private boolean angularController = false;
    private double tolerance; // Radians for angular controllers, inches for linear controllers
    private double maxOutput = 1.0;
    private double deadzone = 0; // No output when abs(error) < deadzone

    protected double target = 0.0;
    protected double lastError = 0.0;
    protected boolean timeAnomalyDetected = false;
    private boolean firstRun = true;
    private boolean isAtTarget = false;
    private long lastTimestamp;

    public Controller() { this.lastTimestamp = System.nanoTime(); }

    public void setAngularController() { this.angularController = true; }
    public boolean isAngularController() { return angularController; }

    public void setTolerance(Angle tolerance) {
        if (!angularController) {
            throw new IllegalStateException("Cannot set angular tolerance on a linear controller");
        }
        this.tolerance = tolerance.getRad();
    }
    public void setTolerance(Dist tolerance) {
        if (angularController) {
            throw new IllegalStateException("Cannot set linear tolerance on an angular controller");
        }
        this.tolerance = tolerance.getIn();
    }
    public double getTolerance() { return tolerance; }

    public void setMaxOutput(double maxOutput) { this.maxOutput = maxOutput; }
    public double getMaxOutput() { return maxOutput; }

    public void setDeadzone(double deadzone) { this.deadzone = deadzone; }
    public double getDeadzone() { return deadzone; }

    public void setTarget(double target) { this.target = target; }
    public double getTarget() { return target; }

    public boolean isAtTarget() { return isAtTarget; }

    /**
     * Resets the controller state. Call this right before starting a new movement
     * to prevent derivative kick and reset the timer.
     */
    public void reset() {
        this.isAtTarget = false;
        this.firstRun = true;
        this.lastTimestamp = System.nanoTime();
    }

    /**
     * Calculates the output based on the current state and the internally set target.
     * @param current The current physical reading (e.g., position, angle)
     * @return The control output
     */
    public synchronized double calculate(double current) {
        return calculateFromError(target - current);
    }

    /**
     * Calculates the output directly from a pre-calculated error.
     * Bypasses the internal target-current math.
     * @param error The calculated error (Target - Current)
     * @return The control output
     */
    public synchronized double calculateFromError(double error) {
        long currentNano = System.nanoTime();

        // Convert nanoseconds to seconds for standard unit gains
        double deltaTime = (currentNano - lastTimestamp) / 1_000_000_000.0;

        // Detect if loop is too fast (div by zero risk) or too slow (integral/derivative spike)
        timeAnomalyDetected = deltaTime < 1E-6 || deltaTime > 0.15;

        double actualError = angularController ? AngleUnit.normalizeRadians(error) : error;
        isAtTarget = Math.abs(actualError) < tolerance;

        // Initialize lastError on first run to prevent derivative kick from 0
        if (firstRun) { deltaTime = 0.0; firstRun = false; }

        // Subclass-specific calculation
        double rawOutput = computeOutput(actualError, lastError, deltaTime);
        if (Math.abs(rawOutput) < deadzone) {
            return 0;
        }
        rawOutput = Math.max(-this.maxOutput, Math.min(this.maxOutput, rawOutput));

        lastTimestamp = currentNano;
        lastError = actualError;

        return rawOutput;
    }

    public double getError() { return lastError; }

    /**
     * @param error Difference between goal and current position.
     * @param lastError Error from the previous loop.
     * @param deltaTime Time elapsed since last loop in seconds.
     */
    protected abstract double computeOutput(double error, double lastError, double deltaTime);
}