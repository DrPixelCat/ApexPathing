package controllers;

import java.util.EnumSet;
import java.util.Objects;
import java.util.function.LongSupplier;

import geometry.Angle;
import geometry.Scalar;

/**
 * A composable feedback controller whose terms are selected with a fluent API.
 *
 * <p>Each call that starts on this controller creates an independent calculation. For example:
 *
 * <pre>{@code
 * Scalar response = controller.p().d().s().calculate(error);
 * Scalar responseWithFeedforward = controller.p().vel(velocity).accel(acceleration)
 *         .calculate(error);
 * }</pre>
 *
 * <p>The terminal {@link Calculation#calculate(double)} call samples and updates the controller's
 * derivative state once, regardless of which terms are selected.
 */
public class FeedbackController {
    public static final double LINEAR_STATIC_DEADBAND = 0.25;
    public static final double ANGULAR_STATIC_DEADBAND = Math.toRadians(0.75);

    /** Translational error where static-friction compensation turns off, in inches. */
    private double linearStaticDeadband = LINEAR_STATIC_DEADBAND;
    /** Angular error where static-friction compensation turns off, in radians. */
    private double angularStaticDeadband = ANGULAR_STATIC_DEADBAND;

    private enum Component {
        P,
        D,
        SQRT,
        STATIC,
        VELOCITY,
        ACCELERATION
    }

    private Coefficients coefficients;
    private double staticDeadband = linearStaticDeadband;
    private boolean angularController;

    private double lastError;
    private boolean firstRun = true;
    private long lastTimestamp;
    private final LongSupplier nanoTime;

    /** Holds the gains used by the available controller components. */
    public static class Coefficients {
        /** A NaN gain is not included and cannot be used by a calculation. */
        public double kP = Double.NaN;
        public double kD = Double.NaN;
        public double kS = Double.NaN;
        public double kSqrt = Double.NaN;
        public double kV = Double.NaN;
        public double kA = Double.NaN;

        /** Creates a coefficient set with no gains included. */
        public Coefficients() {}

        public Coefficients p(double kP) { this.kP = kP; return this; }
        public Coefficients d(double kD) { this.kD = kD; return this; }
        public Coefficients s(double kS) { this.kS = kS; return this; }
        public Coefficients sqrt(double kSqrt) { this.kSqrt = kSqrt; return this; }
        public Coefficients vel(double kV) { this.kV = kV; return this; }
        public Coefficients accel(double kA) { this.kA = kA; return this; }
    }

    /** Creates a controller using the supplied coefficients. */
    public FeedbackController(Coefficients coefficients) {
        this(coefficients, System::nanoTime);
    }

    FeedbackController(Coefficients coefficients, LongSupplier nanoTime) {
        this.coefficients = Objects.requireNonNull(coefficients, "coefficients");
        this.nanoTime = Objects.requireNonNull(nanoTime, "nanoTime");
        this.lastTimestamp = nanoTime.getAsLong();
    }

    /** Replaces the coefficients used by future calculations. */
    public void setCoefficients(Coefficients coefficients) {
        this.coefficients = Objects.requireNonNull(coefficients, "coefficients");
    }

    /** @return the coefficients currently used by this controller. */
    public Coefficients getCoefficients() { return coefficients; }

    public void setLinearStaticDeadband(double linearStaticDeadband) {
        this.linearStaticDeadband = linearStaticDeadband;
        if (!angularController) { staticDeadband = linearStaticDeadband; }
    }

    public void setAngularStaticDeadband(double angularStaticDeadband) {
        this.angularStaticDeadband = angularStaticDeadband;
        if (angularController) { staticDeadband = angularStaticDeadband; }
    }

    /**
     * Configures this controller to wrap errors to [-PI, PI] and use the angular static deadband.
     */
    public void setAngularController() {
        angularController = true;
        staticDeadband = angularStaticDeadband;
    }

    /** Resets the derivative history before beginning a new movement. */
    public void reset() {
        firstRun = true;
        lastTimestamp = nanoTime.getAsLong();
    }

    /** Starts a calculation containing the proportional term. */
    public Calculation p() { return calculation().p(); }

    /** Starts a calculation containing an internally calculated derivative term. */
    public Calculation d() { return calculation().d(); }

    /** Starts a calculation containing a derivative term from a supplied error rate. */
    public Calculation d(double errorRate) { return calculation().d(errorRate); }

    /** Starts a calculation containing signed square-root feedback. */
    public Calculation sqrt() { return calculation().sqrt(); }

    /** Starts a calculation containing static compensation in the error's direction. */
    public Calculation s() { return calculation().s(); }

    /** Starts a calculation containing static compensation in an explicit direction. */
    public Calculation s(double desiredDirection) { return calculation().s(desiredDirection); }

    /** Starts a calculation containing velocity feedforward. */
    public Calculation vel(double desiredVelocity) { return calculation().vel(desiredVelocity); }

    /** Starts a calculation containing acceleration feedforward. */
    public Calculation accel(double desiredAcceleration) {
        return calculation().accel(desiredAcceleration);
    }

    /** Starts a calculation containing an arbitrary feedforward value. */
    public Calculation f(double feedforward) { return calculation().f(feedforward); }

    private Calculation calculation() {
        return new Calculation();
    }

    private Scalar calculate(Calculation calculation, double error) {
        calculation.validateCoefficients();
        long currentNano = nanoTime.getAsLong();
        double deltaTime = (currentNano - lastTimestamp) / 1_000_000_000.0;
        boolean timeAnomaly = deltaTime < 1E-6 || deltaTime > 0.15;
        double actualError = angularController ? Angle.wrap(error) : error;

        if (firstRun) {
            lastError = actualError;
            timeAnomaly = true;
            firstRun = false;
        }

        double errorRate = calculation.measuredErrorRate
                ? calculation.errorRate
                : timeAnomaly ? 0.0 : (actualError - lastError) / deltaTime;
        double response = 0.0;

        if (calculation.has(Component.P)) {
            response += coefficients.kP * actualError;
        }
        if (calculation.has(Component.D)) {
            response += coefficients.kD * errorRate;
        }
        if (calculation.has(Component.SQRT)) {
            response += coefficients.kSqrt
                    * Math.copySign(Math.sqrt(Math.abs(actualError)), actualError);
        }
        if (calculation.has(Component.STATIC)) {
            double direction = calculation.explicitStaticDirection
                    ? calculation.staticDirection : actualError;
            if (Math.abs(actualError) - staticDeadband > 1e-12 && direction != 0.0) {
                response += Math.copySign(coefficients.kS, direction);
            }
        }
        if (calculation.has(Component.VELOCITY)) {
            response += coefficients.kV * calculation.desiredVelocity;
        }
        if (calculation.has(Component.ACCELERATION)) {
            response += coefficients.kA * calculation.desiredAcceleration;
        }
        response += calculation.feedforward;

        lastTimestamp = currentNano;
        lastError = actualError;
        return Scalar.of(response);
    }

    private static double requireCoefficient(double coefficient, String coefficientName,
                                             String componentName) {
        if (Double.isNaN(coefficient)) {
            throw new IllegalStateException("Cannot use " + componentName + "() without "
                    + coefficientName + " in the controller coefficients");
        }
        if (!Double.isFinite(coefficient)) {
            throw new IllegalStateException(coefficientName + " must be finite");
        }
        return coefficient;
    }

    /**
     * An independent selection of terms to evaluate on the next call to
     * {@link #calculate(double)}.
     * Re-selecting a term replaces its supplied value instead of applying the term twice.
     */
    public final class Calculation {
        private final EnumSet<Component> components = EnumSet.noneOf(Component.class);
        private boolean measuredErrorRate;
        private double errorRate;
        private boolean explicitStaticDirection;
        private double staticDirection;
        private double desiredVelocity;
        private double desiredAcceleration;
        private double feedforward;

        private Calculation() {}

        private boolean has(Component component) { return components.contains(component); }

        private void validateCoefficients() {
            if (has(Component.P)) { requireCoefficient(coefficients.kP, "kP", "p"); }
            if (has(Component.D)) { requireCoefficient(coefficients.kD, "kD", "d"); }
            if (has(Component.SQRT)) {
                requireCoefficient(coefficients.kSqrt, "kSqrt", "sqrt");
            }
            if (has(Component.STATIC)) { requireCoefficient(coefficients.kS, "kS", "s"); }
            if (has(Component.VELOCITY)) {
                requireCoefficient(coefficients.kV, "kV", "vel");
            }
            if (has(Component.ACCELERATION)) {
                requireCoefficient(coefficients.kA, "kA", "accel");
            }
        }

        /** Includes proportional feedback. */
        public Calculation p() {
            requireCoefficient(coefficients.kP, "kP", "p");
            components.add(Component.P);
            return this;
        }

        /** Includes derivative feedback using the controller's calculated error rate. */
        public Calculation d() {
            requireCoefficient(coefficients.kD, "kD", "d");
            components.add(Component.D);
            measuredErrorRate = false;
            errorRate = 0.0;
            return this;
        }

        /** Includes derivative feedback using a supplied error rate. */
        public Calculation d(double errorRate) {
            requireCoefficient(coefficients.kD, "kD", "d");
            components.add(Component.D);
            measuredErrorRate = true;
            this.errorRate = errorRate;
            return this;
        }

        /** Includes signed square-root feedback. */
        public Calculation sqrt() {
            requireCoefficient(coefficients.kSqrt, "kSqrt", "sqrt");
            components.add(Component.SQRT);
            return this;
        }

        /** Includes static compensation in the error's direction. */
        public Calculation s() {
            requireCoefficient(coefficients.kS, "kS", "s");
            components.add(Component.STATIC);
            explicitStaticDirection = false;
            staticDirection = 0.0;
            return this;
        }

        /** Includes static compensation in an explicit direction; zero disables the term. */
        public Calculation s(double desiredDirection) {
            requireCoefficient(coefficients.kS, "kS", "s");
            components.add(Component.STATIC);
            explicitStaticDirection = true;
            staticDirection = desiredDirection;
            return this;
        }

        /** Includes velocity feedforward. */
        public Calculation vel(double desiredVelocity) {
            requireCoefficient(coefficients.kV, "kV", "vel");
            components.add(Component.VELOCITY);
            this.desiredVelocity = desiredVelocity;
            return this;
        }

        /** Includes acceleration feedforward. */
        public Calculation accel(double desiredAcceleration) {
            requireCoefficient(coefficients.kA, "kA", "accel");
            components.add(Component.ACCELERATION);
            this.desiredAcceleration = desiredAcceleration;
            return this;
        }

        /** Includes an arbitrary feedforward value. */
        public Calculation f(double feedforward) {
            this.feedforward = feedforward;
            return this;
        }

        /** Evaluates the selected terms for the supplied target-minus-current error. */
        public Scalar calculate(double error) {
            return FeedbackController.this.calculate(this, error);
        }
    }
}
