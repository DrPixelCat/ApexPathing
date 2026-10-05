package localizers.util;

/**
 * Reusable linear Kalman mathematics. The caller supplies the motion model, process covariance,
 * observation model and measurement variance. Timing, units, robust weighting and noise learning
 * belong to the estimator using this core. Scalar corrections can assimilate any linear observation.
 */
public final class KalmanFilter {
    private final int size;
    private double[] state;
    private double[][] covariance;

    public KalmanFilter(int stateSize) {
        if (stateSize < 1) { throw new IllegalArgumentException("State size must be positive"); }
        size = stateSize;
        clear();
    }

    public void clear() { state = new double[size]; covariance = new double[size][size]; }
    public double getState(int component) { return state[component]; }
    public double[] getState() { return state.clone(); }
    public double getCovariance(int row, int column) { return covariance[row][column]; }
    public double[][] getCovariance() { return copy(covariance); }

    public void reset(double[] initialState, double[][] initialCovariance) {
        validateVector(initialState);
        validateCovariance(initialCovariance);
        state = initialState.clone();
        covariance = copy(initialCovariance);
    }

    /** Replaces uncertainty without changing the state, for model-specific initialization policies. */
    public void setCovariance(double[][] value) {
        validateCovariance(value);
        covariance = copy(value);
    }

    /** x = Fx, P = FPF^T + Q. Q must be positive semidefinite. */
    public void predict(double[][] transition, double[][] processCovariance) {
        validateMatrix(transition);
        validateCovariance(processCovariance);
        double[] nextState = multiply(transition, state);
        double[][] nextCovariance = multiply(multiply(transition, covariance), transpose(transition));
        for (int i = 0; i < size; i++) {
            for (int j = 0; j < size; j++) { nextCovariance[i][j] += processCovariance[i][j]; }
        }
        stabilize(nextCovariance);
        validateVector(nextState);
        validateCovariance(nextCovariance);
        state = nextState; covariance = nextCovariance;
    }

    public double innovation(double measurement, double[] observation) {
        validateMeasurement(measurement, observation);
        double predicted = 0;
        for (int i = 0; i < size; i++) { predicted += observation[i] * state[i]; }
        return measurement - predicted;
    }

    public double innovationVariance(double[] observation, double measurementVariance) {
        validateVector(observation);
        validateVariance(measurementVariance);
        double[] ph = multiply(covariance, observation);
        double variance = measurementVariance;
        for (int i = 0; i < size; i++) { variance += observation[i] * ph[i]; }
        if (!Double.isFinite(variance) || variance <= 0) {
            throw new IllegalArgumentException("Innovation variance must be finite and positive");
        }
        return variance;
    }

    /** Assimilates a direct observation of one state component. */
    public void correctComponent(double measurement, int component, double measurementVariance) {
        double[] observation = new double[size];
        observation[component] = 1;
        correct(measurement, observation, measurementVariance);
    }

    /** Scalar linear observation z = Hx + noise, with Joseph-form covariance correction. */
    public void correct(double measurement, double[] observation, double measurementVariance) {
        double residual = innovation(measurement, observation);
        double variance = innovationVariance(observation, measurementVariance);
        double[] gain = multiply(covariance, observation);
        double[] nextState = state.clone();
        double[][] transform = new double[size][size];
        for (int i = 0; i < size; i++) {
            gain[i] /= variance;
            nextState[i] += gain[i] * residual;
            for (int j = 0; j < size; j++) {
                transform[i][j] = (i == j ? 1 : 0) - gain[i] * observation[j];
            }
        }
        double[][] nextCovariance = multiply(multiply(transform, covariance), transpose(transform));
        for (int i = 0; i < size; i++) {
            for (int j = 0; j < size; j++) {
                nextCovariance[i][j] += gain[i] * measurementVariance * gain[j];
            }
        }
        stabilize(nextCovariance);
        validateVector(nextState);
        validateCovariance(nextCovariance);
        state = nextState; covariance = nextCovariance;
    }

    private void validateMeasurement(double measurement, double[] observation) {
        if (!Double.isFinite(measurement)) { throw new IllegalArgumentException("Measurement must be finite"); }
        validateVector(observation);
    }

    private static void validateVariance(double variance) {
        if (!Double.isFinite(variance) || variance <= 0) {
            throw new IllegalArgumentException("Measurement variance must be finite and positive");
        }
    }

    private void validateVector(double[] value) {
        if (value == null || value.length != size) { throw new IllegalArgumentException("State dimension mismatch"); }
        for (double v : value) {
            if (!Double.isFinite(v)) { throw new IllegalArgumentException("Vector must be finite"); }
        }
    }

    private void validateMatrix(double[][] value) {
        if (value == null || value.length != size) { throw new IllegalArgumentException("Matrix dimension mismatch"); }
        for (double[] row : value) { validateVector(row); }
    }

    private void validateCovariance(double[][] value) {
        validateMatrix(value);
        for (int i = 0; i < size; i++) {
            if (value[i][i] < 0) { throw new IllegalArgumentException("Covariance diagonal cannot be negative"); }
            for (int j = 0; j < i; j++) {
                double tolerance = 1e-12 * Math.max(1e-24, Math.max(Math.abs(value[i][j]), Math.abs(value[j][i])));
                if (Math.abs(value[i][j] - value[j][i]) > tolerance) {
                    throw new IllegalArgumentException("Covariance must be symmetric");
                }
            }
        }
        requirePositiveSemidefinite(value);
    }

    /** Pivoted LDL check also accepts singular positive semidefinite matrices. */
    private static void requirePositiveSemidefinite(double[][] covariance) {
        double[][] residual = copy(covariance);
        double scale = 0;
        for (double[] row : residual) {
            for (double value : row) { scale = Math.max(scale, Math.abs(value)); }
        }
        double tolerance = 1e-12 * Math.max(scale, 1e-24);
        for (int k = 0; k < residual.length; k++) {
            int pivot = k;
            for (int i = k + 1; i < residual.length; i++) {
                if (residual[i][i] > residual[pivot][pivot]) { pivot = i; }
            }
            double[] row = residual[k]; residual[k] = residual[pivot]; residual[pivot] = row;
            for (int i = 0; i < residual.length; i++) {
                double value = residual[i][k]; residual[i][k] = residual[i][pivot]; residual[i][pivot] = value;
            }
            double diagonal = residual[k][k];
            if (diagonal < -tolerance) { throw new IllegalArgumentException("Covariance must be positive semidefinite"); }
            if (diagonal <= tolerance) {
                for (int i = k; i < residual.length; i++) {
                    for (int j = k; j < residual.length; j++) {
                        if (Math.abs(residual[i][j]) > tolerance) {
                            throw new IllegalArgumentException("Covariance must be positive semidefinite");
                        }
                    }
                }
                return;
            }
            for (int i = k + 1; i < residual.length; i++) {
                for (int j = k + 1; j < residual.length; j++) {
                    residual[i][j] -= residual[i][k] * residual[k][j] / diagonal;
                }
            }
        }
    }

    private static void stabilize(double[][] value) {
        for (int i = 0; i < value.length; i++) {
            value[i][i] = Math.max(1e-24, value[i][i]);
            for (int j = 0; j < i; j++) { value[i][j] = value[j][i] = (value[i][j] + value[j][i]) / 2; }
        }
    }

    private static double[][] copy(double[][] value) {
        double[][] result = new double[value.length][];
        for (int i = 0; i < value.length; i++) { result[i] = value[i].clone(); }
        return result;
    }

    private static double[] multiply(double[][] matrix, double[] vector) {
        double[] result = new double[vector.length];
        for (int i = 0; i < result.length; i++) {
            for (int j = 0; j < result.length; j++) { result[i] += matrix[i][j] * vector[j]; }
        }
        return result;
    }

    private static double[][] multiply(double[][] left, double[][] right) {
        int n = left.length;
        double[][] result = new double[n][n];
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                for (int k = 0; k < n; k++) { result[i][j] += left[i][k] * right[k][j]; }
            }
        }
        return result;
    }

    private static double[][] transpose(double[][] value) {
        double[][] result = new double[value.length][value.length];
        for (int i = 0; i < value.length; i++) {
            for (int j = 0; j < value.length; j++) { result[i][j] = value[j][i]; }
        }
        return result;
    }
}
