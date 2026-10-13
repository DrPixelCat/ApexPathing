package localizers.util;

import geometry.Matrix;

/**
 * Reusable linear Kalman mathematics. The caller supplies the motion model, process covariance,
 * observation model and measurement variance. Timing, units, robust weighting and noise learning
 * belong to the estimator using this core. Scalar corrections can assimilate any linear observation.
 * State and gain are column matrices, and observation weights form a row matrix. Measurements,
 * innovations and their variances are scalars. Arrays are used at input/output boundaries only.
 *
 * @author Joel H
 */
public final class KalmanFilter {
    /** Number of state components. */
    private final int size;
    /** Current state estimates as a size x 1 column matrix. */
    private Matrix<Double> state;
    /** Uncertainty and cross-covariance of the state estimates. */
    private Matrix<Double> covariance;

    /**
     * Creates a new multi-state Kalman Filter
     *
     * @param stateSize The number of states that will be measured
     * @throws IllegalArgumentException if the size isn't positive
     */
    public KalmanFilter(int stateSize) {
        if (stateSize < 1) { throw new IllegalArgumentException("State size must be positive"); }
        size = stateSize;
        clear();
    }

    /**
     * Resets the state and covariance matrices. Dimensions are kept the same.
     */
    public void clear() { state = Matrix.of(new double[size][1]); covariance = Matrix.of(new double[size][size]); }

    /**
     * Gets the selected filtered state
     *
     * @param component The index of the state
     * @return The filtered state
     * @throws ArrayIndexOutOfBoundsException if the component is outside {@code [0, size - 1]}
     */
    public double getState(int component) { return state.get(component, 0); }

    /**
     * Gets the full array of filtered state estimates
     *
     * @return An array containing all filtered states
     */
    public double[] getState() { return state.transpose().toDoubleArray()[0]; }

    /**
     * Gets an entry from the state covariance matrix.
     * Diagonal entries are variances; off-diagonal entries are covariances.
     *
     * @param row The row of the covariance matrix as an array index. Possible values are {@code [0, states - 1]}
     * @param column The column of the covariance matrix as an array index. Possible values are {@code [0, states - 1]}
     * @return The entry from the state covariance matrix, either variance or covariance.
     * @throws ArrayIndexOutOfBoundsException if either index is outside {@code [0, size - 1]}
     */
    public double getCovariance(int row, int column) { return covariance.get(row, column); }

    /**
     * Gets the full covariance matrix as a 2D array
     *
     * @return The state covariance matrix
     */
    public double[][] getCovariance() { return covariance.toDoubleArray(); }

    /**
     * Resets the Kalman Filter to its initial state and covariance
     *
     * @param initialState The initial state estimates the filter will start with
     * @param initialCovariance The initial covariance matrix the filter will start with
     * @throws IllegalArgumentException if dimensions or entries are invalid, or the covariance is
     *         not symmetric within tolerance and positive semidefinite
     */
    public void reset(double[] initialState, double[][] initialCovariance) {
        Matrix<Double> nextState = checkedRow(Matrix.of(initialState)).transpose();
        Matrix<Double> nextCovariance = checkedCovariance(Matrix.of(initialCovariance));
        state = nextState;
        covariance = nextCovariance;
    }

    /**
     * Replaces uncertainty without changing the state, for model-specific initialization policies.
     *
     * @param value The size x size covariance matrix
     * @throws IllegalArgumentException if dimensions or entries are invalid, or the covariance is
     *         not symmetric within tolerance and positive semidefinite
     */
    public void setCovariance(double[][] value) {
        covariance = checkedCovariance(Matrix.of(value));
    }

    /** Predicts the state and its uncertainty using the supplied motion model.
     * Computes x = Fx and P = FPF^T + Q. Q must be positive semidefinite.
     *
     * @param transition the state-transition matrix for the prediction interval
     * @param processCovariance the uncertainty added by the motion model during that interval; must
     * be symmetric and positive semidefinite
     * @throws IllegalArgumentException if dimensions or entries are invalid, the process covariance
     *         is invalid, or a matrix calculation produces a non-finite entry
     */
    public void predict(double[][] transition, double[][] processCovariance) {
        Matrix<Double> model = Matrix.of(transition);
        validateMatrix(model);
        Matrix<Double> process = checkedCovariance(Matrix.of(processCovariance));
        Matrix<Double> nextState = model.multiply(state);
        Matrix<Double> nextCovariance = symmetrize(model.multiply(covariance).multiply(model.transpose()).add(process));
        state = nextState; covariance = nextCovariance;
    }

    /**
     * Calculates the innovation: the actual measurement minus the measurement
     * predicted from the current state.
     *
     * @param measurement the actual sensor measurement
     * @param observation the weights mapping state components to the predicted measurement
     * @return the difference between the actual and predicted measurements
     * @throws IllegalArgumentException if the measurement or observation is invalid, or the
     *         predicted measurement is non-finite
     */
    public double innovation(double measurement, double[] observation) {
        validateMeasurement(measurement);
        Matrix<Double> model = checkedRow(Matrix.of(observation));
        return innovation(measurement, model);
    }

    /**
     * Calculates the variance of {@link #innovation(double, double[])} by
     * combining predicted measurement uncertainty with measurement noise
     *
     * @param observation The weights mapping state components to the predicted measurement
     * @param measurementVariance The variance of the raw measurements
     * @return The variance of the innovation
     * @throws IllegalArgumentException if the observation has an invalid dimension or non-finite
     * elements, or the measurement variance is not nonnegative and finite
     */
    public double innovationVariance(double[] observation, double measurementVariance) {
        Matrix<Double> model = checkedRow(Matrix.of(observation));
        validateVariance(measurementVariance);
        Matrix<Double> projectedCovariance = covariance.multiply(model.transpose());
        return innovationVariance(model, projectedCovariance, measurementVariance);
    }

    /**
     * Corrects the entire state estimate using the measurement of a single component
     * by assimilating a direct observation of that component.
     *
     * @param measurement The current measurement
     * @param component The component of the state matrix belonging to {@code component}
     * @param measurementVariance The variance of the component
     * @throws ArrayIndexOutOfBoundsException if the component is outside {@code [0, size - 1]}
     * @throws IllegalArgumentException if the measurement or variance is invalid, the innovation
     *         variance is zero, or a matrix calculation produces a non-finite entry
     */
    public void correctComponent(double measurement, int component, double measurementVariance) {
        double[] observation = new double[size];
        observation[component] = 1;
        correct(measurement, observation, measurementVariance);
    }

    /**
     * Corrects the entire state estimate using a measurement of a linear combination of state
     * components. Updates both the state and its covariance.
     * Uses the scalar linear observation z = Hx + noise, with Joseph-form covariance correction.
     *
     * @param measurement The current measurement
     * @param observation The weights mapping state components to the predicted measurement
     * @param measurementVariance The measurement noise variance as a nonnegative, finite number;
     * zero represents an exact observation
     * @throws IllegalArgumentException If any inputs are invalid or the innovation variance is zero.
     * All inputs must be finite.
     */
    public void correct(double measurement, double[] observation, double measurementVariance) {
        validateMeasurement(measurement);
        Matrix<Double> model = checkedRow(Matrix.of(observation));
        validateVariance(measurementVariance);
        double residual = innovation(measurement, model);
        Matrix<Double> projectedCovariance = covariance.multiply(model.transpose()); // PH^T
        double variance = innovationVariance(model, projectedCovariance, measurementVariance);
        if (variance == 0) { throw new IllegalArgumentException("Correction requires positive innovation variance"); }
        Matrix<Double> gain = projectedCovariance.map(value -> value / variance);
        Matrix<Double> nextState = state.add(gain.multiply(residual));
        Matrix<Double> transform = Matrix.identity(size).subtract(gain.multiply(model));
        Matrix<Double> measurementNoise = gain.multiply(measurementVariance).multiply(gain.transpose());
        Matrix<Double> nextCovariance = symmetrize(
                transform.multiply(covariance).multiply(transform.transpose()).add(measurementNoise));
        state = nextState; covariance = nextCovariance;
    }

    /**
     * Computes the scalar innovation y = z - Hx.
     *
     * @param measurement The actual measurement z
     * @param observation The 1 x size observation matrix H
     * @return The measured value minus the predicted value
     */
    private double innovation(double measurement, Matrix<Double> observation) {
        return measurement - observation.multiply(state).get(0, 0);
    }

    /**
     * Computes the scalar innovation variance S = H(PH^T) + R.
     *
     * @param observation The 1 x size observation matrix H
     * @param projectedCovariance The precomputed size x 1 matrix PH^T
     * @param measurementVariance The measurement noise variance R
     * @return The nonnegative innovation variance
     * @throws IllegalArgumentException if the result is negative or non-finite
     */
    private double innovationVariance(Matrix<Double> observation, Matrix<Double> projectedCovariance,
                                      double measurementVariance) {
        double variance = observation.multiply(projectedCovariance).get(0, 0) + measurementVariance;
        if (!Double.isFinite(variance) || variance < 0) {
            throw new IllegalArgumentException("Innovation variance must be finite and nonnegative");
        }
        return variance;
    }

    /**
     * Throws an error if a measurement is invalid.
     *
     * @param measurement The measurement to be inspected
     * @throws IllegalArgumentException if the measurement is non-finite
     */
    private static void validateMeasurement(double measurement) {
        if (!Double.isFinite(measurement)) { throw new IllegalArgumentException("Measurement must be finite"); }
    }

    /**
     * Requires a nonnegative, finite measurement variance.
     * @param variance The variance value
     * @throws IllegalArgumentException if variance is negative or not finite
     */
    private static void validateVariance(double variance) {
        if (!Double.isFinite(variance) || variance < 0) {
            throw new IllegalArgumentException("Measurement variance must be finite and nonnegative");
        }
    }

    /**
     * Checks a 1 x size row matrix; transpose it for a state column.
     *
     * @param value The row matrix
     * @return The validated matrix
     * @throws IllegalArgumentException if the row's state dimension is incorrect or an entry is non-finite
     */
    private Matrix<Double> checkedRow(Matrix<Double> value) {
        validateVector(value);
        return value;
    }

    /**
     * Throws an error if a vector's state dimension is incorrect or a number in the matrix is
     * non-finite.
     *
     * @param value The vector
     * @throws IllegalArgumentException if the vector's state dimension is incorrect or a number in
     * the matrix is non-finite
     */
    private void validateVector(Matrix<Double> value) {
        if (value == null || value.getRows() != 1 || value.getColumns() != size) {
            throw new IllegalArgumentException("State dimension mismatch");
        }
        for (double v : value) {
            if (!Double.isFinite(v)) { throw new IllegalArgumentException("Vector must be finite"); }
        }
    }


    /**
     * Throws an error if a matrix's dimension doesn't match the Kalman filter's dimension
     *
     * @param value The size x size matrix
     * @throws IllegalArgumentException if the state dimension of value is wrong
     */
    private void validateMatrix(Matrix<Double> value) {
        if (value == null || value.getRows() != size || value.getColumns() != size) {
            throw new IllegalArgumentException("Matrix dimension mismatch");
        }
    }

    /**
     * Checks if the supplied value can properly represent uncertainty in the filter's state.
     *
     * @param value The covariance matrix
     * @return A symmetric copy of the validated matrix
     * @throws IllegalArgumentException if the matrix is null, has incorrect dimensions,
     *         has a negative diagonal entry, exceeds the symmetry tolerance, or is
     *         not positive semidefinite
     */
    private Matrix<Double> checkedCovariance(Matrix<Double> value) {
        validateMatrix(value);
        for (int i = 0; i < size; i++) {
            if (value.get(i, i) < 0) { throw new IllegalArgumentException("Covariance diagonal cannot be negative"); }
            for (int j = 0; j < i; j++) {
                double scale = Math.max(Math.max(Math.abs(value.get(i, j)), Math.abs(value.get(j, i))),
                        Math.max(Math.abs(value.get(i, i)), Math.abs(value.get(j, j))));
                if (scale > 0 && Math.abs(value.get(i, j) / scale - value.get(j, i) / scale) > 1e-12) {
                    throw new IllegalArgumentException("Covariance must be symmetric");
                }
            }
        }
        Matrix<Double> result = symmetrize(value);
        requirePositiveSemidefinite(result);
        return result;
    }

    /**
     * Checks whether the covariance matrix is positive semidefinite using pivoted elimination.
     * Singular matrices are allowed. Checks run in correlation units to make the tolerance
     * independent of the state units.
     *
     * @param covariance The covariance matrix
     * @throws IllegalArgumentException if covariance is not positive semidefinite within tolerance
     */
    private void requirePositiveSemidefinite(Matrix<Double> covariance) {
        final double tolerance = 1e-12;
        Matrix<Double> residual = normalizeCovariance(covariance, tolerance);

        for (int step = 0; step < size; step++) {
            residual = pivotLargestDiagonal(residual, step);
            double diagonal = residual.get(step, step);

            if (diagonal < -tolerance) {
                throw new IllegalArgumentException("Covariance must be positive semidefinite");
            }
            if (diagonal <= tolerance) {
                requireZeroBlock(residual, step, tolerance);
                return;
            }

            residual = eliminatePivot(residual, step, diagonal);
        }
    }

    /**
     * Converts positive diagonal entries to {@code 1} and normalizes off-diagonal entries to
     * correlations within the range {@code [-1, 1]}, allowing the supplied tolerance.
     * Zero-variance rows and columns must contain only zeros and remain zero.
     *
     * @param covariance The covariance matrix
     * @param tolerance The allowed tolerance above/below the accepted range
     * @return The normalized covariance matrix
     * @throws IllegalArgumentException if a zero variance has nonzero covariance, or a normalized
     *         correlation exceeds the accepted range; this check alone does not establish
     *         positive semidefiniteness
     */
    private Matrix<Double> normalizeCovariance(Matrix<Double> covariance, double tolerance) {
        return covariance.mapIndexed((row, column, value) -> {
            double rowScale = Math.sqrt(covariance.get(row, row));
            double columnScale = Math.sqrt(covariance.get(column, column));
            if (rowScale == 0 || columnScale == 0) {
                if (value != 0) { throw new IllegalArgumentException("Zero variance requires zero covariance"); }
                return value;
            }
            double normalized = row == column ? 1 : value / rowScale / columnScale;
            if (Math.abs(normalized) > 1 + tolerance) {
                throw new IllegalArgumentException("Covariance must be positive semidefinite");
            }
            return normalized;
        });
    }

    /**
     * Moves the largest diagonal at or after this step into the pivot position.
     * Swaps both rows and columns to preserve symmetry and the represented relationships.
     *
     * @param residual The matrix remaining during elimination
     * @param step The index of the next pivot
     * @return The reordered matrix, or the original if the largest diagonal is already in place
     */
    private Matrix<Double> pivotLargestDiagonal(Matrix<Double> residual, int step) {
        int largest = step;
        for (int row = step + 1; row < size; row++) {
            if (residual.get(row, row) > residual.get(largest, largest)) { largest = row; }
        }
        if (largest == step) { return residual; }

        final int pivot = largest;
        return residual.mapIndexed((row, column, value) -> {
            int sourceRow = row;
            if (row == step) { sourceRow = pivot; }
            else if (row == pivot) { sourceRow = step; }

            int sourceColumn = column;
            if (column == step) { sourceColumn = pivot; }
            else if (column == pivot) { sourceColumn = step; }

            return residual.get(sourceRow, sourceColumn);
        });
    }

    /**
     * Checks that the remaining lower-right block is zero within the tolerance.
     * This permits a zero pivot only when no unexplained covariance remains in that block.
     *
     * @param residual The matrix remaining during elimination
     * @param start The first row and column of the block to check
     * @param tolerance The largest accepted absolute entry
     * @throws IllegalArgumentException if any entry in the block exceeds the tolerance
     */
    private void requireZeroBlock(Matrix<Double> residual, int start, double tolerance) {
        for (int row = start; row < size; row++) {
            for (int column = start; column < size; column++) {
                if (Math.abs(residual.get(row, column)) > tolerance) {
                    throw new IllegalArgumentException("Covariance must be positive semidefinite");
                }
            }
        }
    }

    /**
     * Removes the pivot's contribution from the remaining lower-right block.
     * Entries in the processed rows and columns are retained.
     *
     * @param residual The matrix with the pivot already in place
     * @param step The pivot's row and column index
     * @param diagonal The positive pivot value
     * @return The updated matrix, or the original if there is no remaining block
     */
    private Matrix<Double> eliminatePivot(Matrix<Double> residual, int step, double diagonal) {
        if (step + 1 == size) { return residual; }
        return residual.mapIndexed((row, column, value) -> {
            if (row <= step || column <= step) { return value; }
            double factor = residual.get(row, step) / diagonal;
            return value - factor * residual.get(step, column);
        });
    }

    /**
     * Remove roundoff asymmetry without adding uncertainty or rounding equal subnormal entries.
     * Unequal entries on opposite sides of the diagonal are replaced by their average.
     *
     * @param covariance The covariance matrix to symmetrize
     * @return A symmetric copy with the same diagonal entries
     */
    private Matrix<Double> symmetrize(Matrix<Double> covariance) {
        return covariance.zip(covariance.transpose(), (a, b) -> a.equals(b) ? a : a / 2 + b / 2);
    }
}
