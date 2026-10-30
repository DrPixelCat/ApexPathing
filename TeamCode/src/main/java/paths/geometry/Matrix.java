package paths.geometry;

import java.lang.reflect.Array;
import java.util.Arrays;
import java.util.Iterator;
import java.util.NoSuchElementException;
import java.util.Objects;
import java.util.function.BinaryOperator;
import java.util.function.IntFunction;
import java.util.function.UnaryOperator;

/**
 * A nonempty, rectangular matrix with user-defined element operations.
 * Arrays are copied, but element objects are shared. Use immutable elements for immutable values.
 * Null entries and non-finite {@link Double} or {@link Float} entries are rejected.
 *
 * <p>Operations return new matrices. Instance operations retain this matrix's arithmetic configuration.
 * Supply {@link Arithmetic} for the standard arithmetic methods, or pass lambdas to methods such as
 * {@link #map(UnaryOperator)} and {@link #zip(Matrix, BinaryOperator)}.
 * The {@code of} factories configure standard double arithmetic automatically.
 *
 * <p>A one-dimensional input creates a row matrix; transpose it to create a column matrix.
 * For example, {@code new Matrix<>(new Integer[] {1, 2, 3}).map(x -> x * x)} creates
 * the row {@code [1, 4, 9]}. Iteration visits entries row by row.
 *
 * @param <T> the element type
 * @author Sohum Arora - 22985 Paraducks
 */
public class Matrix<T> implements Iterable<T> {
    private static final Arithmetic<Double> DOUBLES = new Arithmetic<>(
            0.0, 1.0, Double::sum, (a, b) -> a - b, (a, b) -> a * b, (a, b) -> a / b);

    private final Object[][] data;
    private final int rows;
    private final int cols;
    private final Arithmetic<T> arithmetic;

    /**
     * An element operation that receives zero-based row and column indices.
     * @param <T> the element type
     */
    @FunctionalInterface
    public interface IndexedOperator<T> {
        /**
         * Computes a replacement entry.
         * @param row the row index
         * @param column the column index
         * @param value the current entry
         * @return the replacement entry
         */
        T apply(int row, int column, T value);
    }

    /**
     * Defines the identities and operations used by the standard arithmetic methods.
     * @param <T> the element type
     */
    public static final class Arithmetic<T> {
        private final T zero;
        private final T one;
        private final BinaryOperator<T> add;
        private final BinaryOperator<T> subtract;
        private final BinaryOperator<T> multiply;
        private final BinaryOperator<T> divide;

        /**
         * Configures arithmetic for an element type.
         * @param zero the additive identity
         * @param one the multiplicative identity
         * @param add computes left + right
         * @param subtract computes left - right
         * @param multiply computes left * right
         * @param divide computes left / right
         */
        public Arithmetic(T zero, T one, BinaryOperator<T> add, BinaryOperator<T> subtract,
                          BinaryOperator<T> multiply, BinaryOperator<T> divide) {
            this.zero = checkedEntry(zero);
            this.one = checkedEntry(one);
            this.add = Objects.requireNonNull(add, "Addition operation");
            this.subtract = Objects.requireNonNull(subtract, "Subtraction operation");
            this.multiply = Objects.requireNonNull(multiply, "Multiplication operation");
            this.divide = Objects.requireNonNull(divide, "Division operation");
        }
    }

    /**
     * Copies a rectangular array without configuring arithmetic.
     * @param values the entries indexed by row and column
     * @throws IllegalArgumentException if the array is empty, ragged, or contains invalid entries
     */
    public Matrix(T[][] values) { this(values, null); }

    /**
     * Copies a rectangular array and configures its arithmetic operations.
     * @param values the entries indexed by row and column
     * @param arithmetic the arithmetic operations, or null to use only lambda overloads
     * @throws IllegalArgumentException if the array is empty, ragged, or contains invalid entries
     */
    public Matrix(T[][] values, Arithmetic<T> arithmetic) {
        if (values == null || values.length == 0 || values[0] == null || values[0].length == 0) {
            throw new IllegalArgumentException("Matrix must be nonempty and rectangular");
        }
        rows = values.length;
        cols = values[0].length;
        this.arithmetic = arithmetic;
        data = new Object[rows][cols];
        for (int i = 0; i < rows; i++) {
            if (values[i] == null || values[i].length != cols) {
                throw new IllegalArgumentException("Matrix must be rectangular");
            }
            for (int j = 0; j < cols; j++) { data[i][j] = checkedEntry(values[i][j]); }
        }
    }

    /**
     * Copies a one-dimensional array into a 1 x n row matrix without configuring arithmetic.
     * @param row the row entries
     * @throws IllegalArgumentException if the array is null, empty, or contains invalid entries
     */
    public Matrix(T[] row) { this(row, null); }

    /**
     * Copies a one-dimensional array into a 1 x n row matrix.
     * @param row the row entries
     * @param arithmetic the arithmetic operations, or null to use only lambda overloads
     * @throws IllegalArgumentException if the array is null, empty, or contains invalid entries
     */
    public Matrix(T[] row, Arithmetic<T> arithmetic) {
        if (row == null || row.length == 0) { throw new IllegalArgumentException("Row must be nonempty"); }
        rows = 1;
        cols = row.length;
        this.arithmetic = arithmetic;
        data = new Object[rows][cols];
        for (int j = 0; j < cols; j++) { data[0][j] = checkedEntry(row[j]); }
    }

    /** Internal result constructor; the freshly allocated storage never escapes. */
    private Matrix(Object[][] values, int rows, int cols, Arithmetic<T> arithmetic) {
        this.data = values;
        this.rows = rows;
        this.cols = cols;
        this.arithmetic = arithmetic;
        for (Object[] row : values) {
            for (Object value : row) { checkedEntry(value); }
        }
    }

    /**
     * Copies primitive doubles and configures standard floating-point arithmetic.
     * @param values the entries indexed by row and column
     * @return a double matrix
     * @throws IllegalArgumentException if the array is null, empty, ragged, or contains non-finite entries
     */
    public static Matrix<Double> of(double[][] values) {
        if (values == null) { throw new IllegalArgumentException("Matrix cannot be null"); }
        Double[][] boxed = new Double[values.length][];
        for (int i = 0; i < values.length; i++) {
            if (values[i] == null) { throw new IllegalArgumentException("Matrix must be rectangular"); }
            boxed[i] = new Double[values[i].length];
            for (int j = 0; j < values[i].length; j++) { boxed[i][j] = values[i][j]; }
        }
        return new Matrix<>(boxed, DOUBLES);
    }

    /**
     * Copies primitive doubles into a 1 x n row matrix with standard arithmetic.
     * @param row the row entries
     * @return a double row matrix
     * @throws IllegalArgumentException if the array is null, empty, or contains non-finite entries
     */
    public static Matrix<Double> of(double[] row) {
        if (row == null) { throw new IllegalArgumentException("Row cannot be null"); }
        Double[] boxed = new Double[row.length];
        for (int i = 0; i < row.length; i++) { boxed[i] = row[i]; }
        return new Matrix<>(boxed, DOUBLES);
    }

    /**
     * Applies an operation to each entry.
     * @param operation computes each replacement entry
     * @return a matrix with the same dimensions and transformed entries
     */
    public Matrix<T> map(UnaryOperator<T> operation) {
        Objects.requireNonNull(operation, "Element operation");
        Object[][] result = new Object[rows][cols];
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) { result[i][j] = operation.apply(get(i, j)); }
        }
        return new Matrix<>(result, rows, cols, arithmetic);
    }

    /**
     * Applies an operation to each entry using its row and column indices.
     * @param operation computes each replacement entry
     * @return a matrix with the same dimensions and transformed entries
     */
    public Matrix<T> mapIndexed(IndexedOperator<T> operation) {
        Objects.requireNonNull(operation, "Indexed element operation");
        Object[][] result = new Object[rows][cols];
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) { result[i][j] = operation.apply(i, j, get(i, j)); }
        }
        return new Matrix<>(result, rows, cols, arithmetic);
    }

    /**
     * Combines corresponding entries of equally sized matrices.
     * @param other the right-hand matrix
     * @param operation combines this matrix's entry with the other matrix's entry
     * @return a matrix with the combined entries
     * @throws IllegalArgumentException if the dimensions differ
     */
    public Matrix<T> zip(Matrix<T> other, BinaryOperator<T> operation) {
        Objects.requireNonNull(other, "Other matrix");
        Objects.requireNonNull(operation, "Element operation");
        if (rows != other.rows || cols != other.cols) {
            throw new IllegalArgumentException("Dimension mismatch for elementwise operation");
        }
        Object[][] result = new Object[rows][cols];
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) { result[i][j] = operation.apply(get(i, j), other.get(i, j)); }
        }
        return new Matrix<>(result, rows, cols, arithmetic);
    }

    /**
     * Adds corresponding entries using configured arithmetic.
     * @param other a matrix with the same dimensions
     * @return the sum
     * @throws UnsupportedOperationException if arithmetic is not configured
     * @throws IllegalArgumentException if the dimensions differ
     */
    public Matrix<T> add(Matrix<T> other) { return zip(other, arithmetic().add); }

    /**
     * Subtracts corresponding entries using configured arithmetic.
     * @param other a matrix with the same dimensions
     * @return this matrix minus the other matrix
     * @throws UnsupportedOperationException if arithmetic is not configured
     * @throws IllegalArgumentException if the dimensions differ
     */
    public Matrix<T> subtract(Matrix<T> other) { return zip(other, arithmetic().subtract); }

    /**
     * Multiplies each entry by a scalar using configured arithmetic.
     * @param scalar the right-hand multiplier
     * @return the scaled matrix
     * @throws UnsupportedOperationException if arithmetic is not configured
     */
    public Matrix<T> multiply(T scalar) { return multiply(scalar, arithmetic().multiply); }

    /**
     * Multiplies each entry by a scalar using a lambda.
     * @param scalar the right-hand multiplier
     * @param product combines each entry with the scalar
     * @return the scaled matrix
     */
    public Matrix<T> multiply(T scalar, BinaryOperator<T> product) {
        checkedEntry(scalar);
        Objects.requireNonNull(product, "Multiplication operation");
        return map(value -> product.apply(value, scalar));
    }

    /**
     * Performs matrix multiplication using configured arithmetic.
     * @param other a matrix whose row count equals this matrix's column count
     * @return a matrix with this row count and the other matrix's column count
     * @throws UnsupportedOperationException if arithmetic is not configured
     * @throws IllegalArgumentException if the inner dimensions differ
     */
    public Matrix<T> multiply(Matrix<T> other) {
        Arithmetic<T> operations = arithmetic();
        return multiply(other, operations.add, operations.multiply, operations.zero);
    }

    /**
     * Performs matrix multiplication using lambdas. Each entry accumulates
     * {@code sum(value, product(left, right))}, starting from {@code zero}.
     * @param other a matrix whose row count equals this matrix's column count
     * @param sum combines accumulated products
     * @param product multiplies a left entry by a right entry
     * @param zero the additive identity
     * @return a matrix with this row count and the other matrix's column count
     * @throws IllegalArgumentException if the inner dimensions differ
     */
    public Matrix<T> multiply(Matrix<T> other, BinaryOperator<T> sum, BinaryOperator<T> product, T zero) {
        Objects.requireNonNull(other, "Other matrix");
        Objects.requireNonNull(sum, "Addition operation");
        Objects.requireNonNull(product, "Multiplication operation");
        checkedEntry(zero);
        if (cols != other.rows) { throw new IllegalArgumentException("Dimension mismatch for multiplication"); }
        Object[][] result = new Object[rows][other.cols];
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < other.cols; j++) {
                T value = zero;
                for (int k = 0; k < cols; k++) { value = sum.apply(value, product.apply(get(i, k), other.get(k, j))); }
                result[i][j] = value;
            }
        }
        return new Matrix<>(result, rows, other.cols, arithmetic);
    }

    /**
     * Multiplies by a column vector using configured arithmetic.
     * @param vector the column entries; length must equal this matrix's column count
     * @param arrayFactory creates an output array, for example {@code Double[]::new}
     * @return the result as an array with one entry per matrix row
     * @throws UnsupportedOperationException if arithmetic is not configured
     * @throws IllegalArgumentException if dimensions or the output array length are incorrect
     */
    public T[] multiply(T[] vector, IntFunction<T[]> arrayFactory) {
        Objects.requireNonNull(arrayFactory, "Array factory");
        Matrix<T> product = multiply(new Matrix<>(vector).transpose());
        T[] result = arrayFactory.apply(rows);
        if (result == null || result.length != rows) { throw new IllegalArgumentException("Array factory must match row count"); }
        for (int i = 0; i < rows; i++) { result[i] = product.get(i, 0); }
        return result;
    }

    /**
     * Swaps rows and columns.
     * @return the transposed matrix
     */
    public Matrix<T> transpose() {
        Object[][] result = new Object[cols][rows];
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) { result[j][i] = get(i, j); }
        }
        return new Matrix<>(result, cols, rows, arithmetic);
    }

    /**
     * Inverts a 1 x 1 or 2 x 2 matrix using configured arithmetic.
     * Division must support fractional results for a general inverse.
     * @return the inverse matrix
     * @throws UnsupportedOperationException if arithmetic is missing or the size is unsupported
     * @throws IllegalArgumentException if singular, ill-conditioned under standard double arithmetic,
     *         or an operation produces an invalid entry
     */
    public Matrix<T> inverse() {
        if (rows != cols) { throw new UnsupportedOperationException("Inverse requires a square matrix"); }
        if (rows > 2) { throw new UnsupportedOperationException("Inverse only supported for 1x1 and 2x2"); }
        Arithmetic<T> op = arithmetic();
        if (rows == 1) {
            if (get(0, 0).equals(op.zero)) { throw new IllegalArgumentException("Matrix is singular"); }
            return new Matrix<>(new Object[][] {{op.divide.apply(op.one, get(0, 0))}}, 1, 1, arithmetic);
        }
        T a = get(0, 0), b = get(0, 1), c = get(1, 0), d = get(1, 1);
        T determinant = checkedEntry(op.subtract.apply(op.multiply.apply(a, d), op.multiply.apply(b, c)));
        boolean singular = determinant.equals(op.zero);
        if (arithmetic == DOUBLES) {
            double scale = Math.max(Math.max(Math.abs((Double) a), Math.abs((Double) b)),
                    Math.max(Math.abs((Double) c), Math.abs((Double) d)));
            singular |= Math.abs((Double) determinant) <= 1e-12 * scale * scale;
        }
        if (singular) { throw new IllegalArgumentException("Matrix is singular or ill-conditioned"); }
        Object[][] result = {
                {op.divide.apply(d, determinant), op.divide.apply(op.subtract.apply(op.zero, b), determinant)},
                {op.divide.apply(op.subtract.apply(op.zero, c), determinant), op.divide.apply(a, determinant)}
        };
        return new Matrix<>(result, 2, 2, arithmetic);
    }

    /**
     * Creates a square double matrix with ones on the diagonal and zeros elsewhere.
     * @param size the positive row and column count
     * @return the identity matrix with standard double arithmetic
     * @throws IllegalArgumentException if size is not positive
     */
    public static Matrix<Double> identity(int size) { return identity(size, DOUBLES); }

    /**
     * Creates an identity matrix using the supplied zero and one values.
     * @param <T> the element type
     * @param size the positive row and column count
     * @param arithmetic the identities and operations for the element type
     * @return the identity matrix with the supplied arithmetic
     * @throws IllegalArgumentException if size is not positive
     */
    public static <T> Matrix<T> identity(int size, Arithmetic<T> arithmetic) {
        if (size < 1) { throw new IllegalArgumentException("Matrix size must be positive"); }
        Objects.requireNonNull(arithmetic, "Arithmetic operations");
        Object[][] result = new Object[size][size];
        for (int i = 0; i < size; i++) {
            for (int j = 0; j < size; j++) { result[i][j] = i == j ? arithmetic.one : arithmetic.zero; }
        }
        return new Matrix<>(result, size, size, arithmetic);
    }

    /**
     * Creates the outer product: {@code result[i][j] = left[i] * right[j]}.
     * @param left the entries defining the rows
     * @param right the entries defining the columns
     * @return a matrix with standard double arithmetic
     * @throws IllegalArgumentException if either array is null or empty, or an entry or result is non-finite
     */
    public static Matrix<Double> outerProduct(double[] left, double[] right) {
        return of(left).transpose().multiply(of(right));
    }

    /**
     * Creates an outer product using a lambda; vector lengths may differ.
     * @param <T> the element type
     * @param left the entries defining the rows
     * @param right the entries defining the columns
     * @param product combines a left entry with a right entry
     * @return the outer product without an arithmetic configuration
     * @throws IllegalArgumentException if either array is null or empty, or an entry or result is invalid
     */
    public static <T> Matrix<T> outerProduct(T[] left, T[] right, BinaryOperator<T> product) {
        Matrix<T> column = new Matrix<>(left).transpose();
        Matrix<T> row = new Matrix<>(right);
        Objects.requireNonNull(product, "Multiplication operation");
        Object[][] result = new Object[column.rows][row.cols];
        for (int i = 0; i < column.rows; i++) {
            for (int j = 0; j < row.cols; j++) { result[i][j] = product.apply(column.get(i, 0), row.get(0, j)); }
        }
        return new Matrix<>(result, column.rows, row.cols, null);
    }

    /**
     * Copies entries into a typed array. Element objects are shared; array storage is independent.
     * @param rowFactory creates a row array, for example {@code Integer[]::new}
     * @return the entries indexed by row and column
     * @throws IllegalArgumentException if the factory returns null or the wrong row length
     */
    @SuppressWarnings("unchecked") // The factory supplies the runtime component type, not Object[].
    public T[][] toArray(IntFunction<T[]> rowFactory) {
        Objects.requireNonNull(rowFactory, "Row array factory");
        T[] firstRow = rowFactory.apply(cols);
        if (firstRow == null || firstRow.length != cols) { throw new IllegalArgumentException("Array factory must match column count"); }
        T[][] result = (T[][]) Array.newInstance(firstRow.getClass(), rows);
        for (int i = 0; i < rows; i++) {
            // Clone the template so even a factory that reuses an array cannot alias the output.
            result[i] = firstRow.clone();
            for (int j = 0; j < cols; j++) { result[i][j] = get(i, j); }
        }
        return result;
    }

    /**
     * Copies numeric entries into a primitive double array using {@link Number#doubleValue()}.
     * @return the entries indexed by row and column; conversion may lose precision or overflow
     * @throws UnsupportedOperationException if any entry is not a number
     */
    public double[][] toDoubleArray() {
        double[][] result = new double[rows][cols];
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                Object value = data[i][j];
                if (!(value instanceof Number)) { throw new UnsupportedOperationException("Double export requires numeric entries"); }
                result[i][j] = ((Number) value).doubleValue();
            }
        }
        return result;
    }

    /**
     * Gets an entry using zero-based indices.
     * @param row the row index
     * @param column the column index
     * @return the entry
     * @throws ArrayIndexOutOfBoundsException if either index is outside the matrix
     */
    @SuppressWarnings("unchecked")
    public T get(int row, int column) { return (T) data[row][column]; }

    /**
     * Gets the row count.
     * @return the number of rows
     */
    public int getRows() { return rows; }

    /**
     * Gets the column count.
     * @return the number of columns
     */
    public int getColumns() { return cols; }

    /**
     * Iterates entries from left to right within each row, then proceeds to the next row.
     * Each iterator has its own position. Removal is unsupported.
     * @return an iterator over the entries
     */
    @Override
    public Iterator<T> iterator() {
        return new Iterator<T>() {
            private int row;
            private int column;

            @Override
            public boolean hasNext() { return row < rows; }

            @Override
            public T next() {
                if (!hasNext()) { throw new NoSuchElementException("No more matrix entries"); }
                T value = get(row, column);
                if (++column == cols) {
                    column = 0;
                    row++;
                }
                return value;
            }
        };
    }

    /**
     * Compares dimensions and entries using their exact equals methods.
     * Arithmetic configurations are ignored. Only matrices of the same concrete class compare equal.
     * @param other the object to compare
     * @return true if dimensions and entries match
     */
    @Override
    public boolean equals(Object other) {
        if (this == other) { return true; }
        if (other == null || getClass() != other.getClass()) { return false; }
        Matrix<?> matrix = (Matrix<?>) other;
        if (rows != matrix.rows || cols != matrix.cols) { return false; }
        for (int i = 0; i < rows; i++) {
            if (!Arrays.equals(data[i], matrix.data[i])) { return false; }
        }
        return true;
    }

    /**
     * Hashes the dimensions and entry values used by {@link #equals(Object)}.
     * @return the value hash code
     */
    @Override
    public int hashCode() {
        int result = 31 * rows + cols;
        for (Object[] row : data) { result = 31 * result + Arrays.hashCode(row); }
        return result;
    }

    private Arithmetic<T> arithmetic() {
        if (arithmetic == null) {
            throw new UnsupportedOperationException("Supply Arithmetic in the constructor or use a lambda operation overload");
        }
        return arithmetic;
    }

    private static <T> T checkedEntry(T value) {
        if (value == null) { throw new IllegalArgumentException("Matrix entries cannot be null"); }
        if ((value instanceof Double && !Double.isFinite((Double) value))
                || (value instanceof Float && !Float.isFinite((Float) value))) {
            throw new IllegalArgumentException("Matrix entries must be finite");
        }
        return value;
    }
}
