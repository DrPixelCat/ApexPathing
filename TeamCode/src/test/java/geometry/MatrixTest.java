package geometry;

import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.NoSuchElementException;

import static org.junit.Assert.*;

public class MatrixTest {
    @Test public void indexedMappingUsesCoordinatesAndRetainsArithmeticWithoutChangingTheSource() {
        Matrix<Double> source = Matrix.of(new double[][] {{1, 2, 3}, {4, 5, 6}});
        Matrix<Double> mapped = source.mapIndexed((row, column, value) -> value + 10 * row + column);
        assertArrayEquals(new double[] {1, 3, 5}, mapped.toDoubleArray()[0], 0);
        assertArrayEquals(new double[] {14, 16, 18}, mapped.toDoubleArray()[1], 0);
        assertArrayEquals(new double[] {2, 6, 10}, mapped.multiply(2.0).toDoubleArray()[0], 0);
        assertArrayEquals(new double[] {4, 5, 6}, source.toDoubleArray()[1], 0);
        assertThrows(NullPointerException.class, () -> source.mapIndexed(null));
        assertThrows(IllegalArgumentException.class, () -> source.mapIndexed((row, column, value) -> null));
        assertThrows(IllegalArgumentException.class,
                () -> source.mapIndexed((row, column, value) -> Double.POSITIVE_INFINITY));
    }

    @Test public void iterationVisitsEntriesInRowOrderWithoutExposingStorage() {
        Matrix<Integer> matrix = new Matrix<>(new Integer[][] {{1, 2, 3}, {4, 5, 6}});
        List<Integer> visited = new ArrayList<>();
        for (Integer value : matrix) { visited.add(value); }
        assertEquals(Arrays.asList(1, 2, 3, 4, 5, 6), visited);
        visited.clear();
        matrix.forEach(visited::add);
        assertEquals(Arrays.asList(1, 2, 3, 4, 5, 6), visited);

        Iterator<Integer> iterator = matrix.iterator();
        assertEquals(Integer.valueOf(1), iterator.next());
        assertThrows(UnsupportedOperationException.class, iterator::remove);
        assertEquals(Integer.valueOf(1), matrix.get(0, 0));
    }

    @Test public void iteratorsAreIndependentAndFollowTheExhaustionContract() {
        Matrix<String> column = new Matrix<>(new String[] {"a", "b"}).transpose();
        Iterator<String> first = column.iterator();
        Iterator<String> second = column.iterator();
        assertTrue(first.hasNext());
        assertTrue(first.hasNext());
        assertEquals("a", first.next());
        assertEquals("b", first.next());
        assertFalse(first.hasNext());
        assertThrows(NoSuchElementException.class, first::next);
        assertEquals("a", second.next());
        assertEquals("b", second.next());
        assertFalse(second.hasNext());
        Iterator<Integer> single = new Matrix<>(new Integer[] {7}).iterator();
        assertEquals(Integer.valueOf(7), single.next());
        assertFalse(single.hasNext());
        assertThrows(NoSuchElementException.class, single::next);
    }

    @Test public void equalityComparesDimensionsAndExactEntryValues() {
        Matrix<Integer> first = new Matrix<>(new Integer[][] {{1, 2}, {3, 4}});
        Matrix<Integer> second = new Matrix<>(new Integer[][] {{1, 2}, {3, 4}});
        Matrix<Number> third = new Matrix<Number>(new Integer[][] {{1, 2}, {3, 4}});
        assertEquals(first, first);
        assertEquals(first, second);
        assertEquals(second, first);
        assertEquals(second, third);
        assertEquals(first, third);
        assertNotEquals(first, null);
        assertNotEquals(first, new Integer[][] {{1, 2}, {3, 4}});
        assertNotEquals(first, new Matrix<>(new Integer[][] {{1, 2}, {3, 5}}));
        assertNotEquals(first, new Matrix<>(new Integer[] {1, 2, 3, 4}));
        assertNotEquals(first, new Matrix<>(new Integer[][] {{1}, {2}, {3}, {4}}));
        assertNotEquals(new Matrix<>(new Integer[] {1}), Matrix.of(new double[] {1}));
        assertNotEquals(Matrix.of(new double[] {1}), Matrix.of(new double[] {1 + 1e-14}));
    }

    @Test public void equalValuesIgnoreArithmeticConfigurationAndWorkInHashCollections() {
        Matrix.Arithmetic<Integer> arithmetic = new Matrix.Arithmetic<>(
                0, 1, Integer::sum, (a, b) -> a - b, (a, b) -> a * b, (a, b) -> a / b);
        Matrix<Integer> configured = new Matrix<>(new Integer[] {1, 2, 3}, arithmetic);
        Matrix<Integer> unconfigured = new Matrix<>(new Integer[] {1, 2, 3});
        assertEquals(configured, unconfigured);
        assertEquals(configured.hashCode(), unconfigured.hashCode());
        HashSet<Matrix<Integer>> matrices = new HashSet<>();
        matrices.add(configured);
        assertTrue(matrices.contains(unconfigured));
        assertFalse(matrices.add(unconfigured));
        assertEquals(1, matrices.size());
        assertEquals(Matrix.of(new double[] {1, 2}), new Matrix<>(new Double[] {1.0, 2.0}));
    }

    @Test public void rowConstructorCreatesOneByNAndCopiesItsInput() {
        Integer[] values = {1, 2, 3};
        Matrix<Integer> row = new Matrix<>(values);
        values[0] = 99;
        assertEquals(1, row.getRows());
        assertEquals(3, row.getColumns());
        assertArrayEquals(new Integer[] {1, 2, 3}, row.toArray(Integer[]::new)[0]);
        assertEquals(Integer.valueOf(3), row.transpose().get(2, 0));
        Matrix<Double> primitiveRow = Matrix.of(new double[] {4, 5});
        assertEquals(1, primitiveRow.getRows());
        assertEquals(2, primitiveRow.getColumns());
        assertArrayEquals(new double[] {8, 10}, primitiveRow.multiply(2.0).toDoubleArray()[0], 0);
    }

    @Test public void lambdasOperateOnNonnumericEntriesWithoutArithmeticConfiguration() {
        Matrix<String> words = new Matrix<>(new String[] {"a", "bb"});
        Matrix<String> upper = words.map(String::toUpperCase);
        Matrix<String> joined = upper.zip(words, (a, b) -> a + ":" + b);
        assertArrayEquals(new String[] {"A:a", "BB:bb"}, joined.toArray(String[]::new)[0]);
        assertArrayEquals(new String[] {"a", "bb"}, words.toArray(String[]::new)[0]);
        assertArrayEquals(new String[] {"a!", "bb!"},
                words.multiply("!", (a, b) -> a + b).toArray(String[]::new)[0]);
        assertThrows(UnsupportedOperationException.class, () -> words.add(words));
    }

    @Test public void matrixMultiplicationCanUseBooleanSumAndProductLambdas() {
        Matrix<Boolean> left = new Matrix<>(new Boolean[][] {{true, false, true}, {false, true, false}});
        Matrix<Boolean> right = new Matrix<>(new Boolean[][] {{false, true}, {true, false}, {true, false}});
        Matrix<Boolean> result = left.multiply(right, (a, b) -> a || b, (a, b) -> a && b, false);
        assertArrayEquals(new Boolean[] {true, true}, result.toArray(Boolean[]::new)[0]);
        assertArrayEquals(new Boolean[] {true, false}, result.toArray(Boolean[]::new)[1]);
    }

    @Test public void configuredIntegerArithmeticSurvivesChainedOperations() {
        Matrix.Arithmetic<Integer> integers = new Matrix.Arithmetic<>(
                0, 1, Integer::sum, (a, b) -> a - b, (a, b) -> a * b, (a, b) -> a / b);
        Matrix<Integer> values = new Matrix<>(new Integer[][] {{1, 2, 3}, {4, 5, 6}}, integers);
        Matrix<Integer> result = values.multiply(values.transpose());
        assertArrayEquals(new Integer[] {14, 32}, result.toArray(Integer[]::new)[0]);
        assertArrayEquals(new Integer[] {32, 77}, result.toArray(Integer[]::new)[1]);
        Matrix<Integer> transformed = values.map(a -> a + 1).multiply(2).subtract(values).add(values);
        assertArrayEquals(new Integer[] {4, 6, 8}, transformed.toArray(Integer[]::new)[0]);
        assertArrayEquals(new Integer[] {10, 12, 14}, transformed.toArray(Integer[]::new)[1]);
        assertArrayEquals(values.toArray(Integer[]::new)[0],
                Matrix.identity(2, integers).multiply(values).toArray(Integer[]::new)[0]);
    }

    @Test public void constructorAndExportsDoNotExposeArrayStorage() {
        String[][] input = {{"a", "b"}, {"c", "d"}};
        Matrix<String> values = new Matrix<>(input);
        input[0][0] = "changed";
        input[1] = new String[] {"changed", "changed"};
        String[][] exported = values.toArray(String[]::new);
        exported[0][0] = "changed";
        exported[1] = new String[] {"changed", "changed"};
        assertEquals("a", values.get(0, 0));
        assertEquals("d", values.get(1, 1));
        String[] sharedTemplate = new String[2];
        String[][] independentRows = values.toArray(length -> sharedTemplate);
        independentRows[0][0] = "changed";
        assertEquals("c", independentRows[1][0]);
        assertNull(sharedTemplate[0]);
    }

    @Test public void operationsMayReturnOtherImplementationsOfTheDeclaredElementType() {
        Matrix<Number> numbers = new Matrix<Number>(new Integer[] {1, 2});
        Matrix<Number> halves = numbers.map(value -> value.doubleValue() / 2);
        assertArrayEquals(new Number[] {0.5, 1.0}, halves.toArray(Number[]::new)[0]);
    }

    @Test public void genericOuterProductUsesItsLambda() {
        Matrix<String> product = Matrix.outerProduct(new String[] {"a", "b"},
                new String[] {"1", "2", "3"}, (a, b) -> a + b);
        assertArrayEquals(new String[] {"a1", "a2", "a3"}, product.toArray(String[]::new)[0]);
        assertArrayEquals(new String[] {"b1", "b2", "b3"}, product.toArray(String[]::new)[1]);
    }

    @Test public void invalidShapesAndOperationResultsAreRejected() {
        assertThrows(IllegalArgumentException.class, () -> new Matrix<>((Integer[]) null));
        assertThrows(IllegalArgumentException.class, () -> new Matrix<>(new Integer[0]));
        assertThrows(IllegalArgumentException.class, () -> new Matrix<>(new Integer[][] {{1}, {2, 3}}));
        assertThrows(IllegalArgumentException.class, () -> new Matrix<>(new Integer[] {null}));
        Matrix<Double> row = Matrix.of(new double[] {1, 2});
        assertThrows(IllegalArgumentException.class, () -> row.map(value -> Double.NaN));
        assertThrows(IllegalArgumentException.class, () -> row.map(value -> null));
        assertThrows(IllegalArgumentException.class, () -> row.zip(Matrix.identity(2), Double::sum));
        assertThrows(IllegalArgumentException.class, () -> row.multiply(row));
        assertThrows(IllegalArgumentException.class, () -> row.toArray(length -> new Double[1]));
        assertThrows(IllegalArgumentException.class, () -> Matrix.identity(0));
    }

    @Test public void inverseRetainsDoubleResultsAndSingularityChecks() {
        Matrix<Double> inverse = Matrix.of(new double[][] {{4, 7}, {2, 6}}).inverse();
        assertArrayEquals(new double[] {0.6, -0.7}, inverse.toDoubleArray()[0], 1e-12);
        assertArrayEquals(new double[] {-0.2, 0.4}, inverse.toDoubleArray()[1], 1e-12);
        assertEquals(0.25, Matrix.of(new double[] {4}).inverse().get(0, 0), 0);
        assertThrows(IllegalArgumentException.class, () -> Matrix.of(new double[] {0}).inverse());
        assertThrows(IllegalArgumentException.class,
                () -> Matrix.of(new double[][] {{1, 2}, {2, 4}}).inverse());
        assertThrows(IllegalArgumentException.class,
                () -> Matrix.of(new double[][] {{1, 1}, {1, 1 + 1e-14}}).inverse());
    }

    @Test public void outerProductSupportsDifferentVectorLengths() {
        Matrix<Double> product = Matrix.outerProduct(new double[] {2, -3}, new double[] {4, 0, -5});
        assertArrayEquals(new double[] {8, 0, -10}, product.toDoubleArray()[0], 0);
        assertArrayEquals(new double[] {-12, 0, 15}, product.toDoubleArray()[1], 0);
    }

    @Test public void outerProductAndExportDoNotExposeMutableStorage() {
        double[] left = {2, 3};
        double[] right = {4, 5};
        Matrix<Double> product = Matrix.outerProduct(left, right);
        left[0] = 99;
        right[1] = 99;
        double[][] snapshot = product.toDoubleArray();
        snapshot[0][0] = 99;
        snapshot[1] = new double[] {99, 99};
        assertArrayEquals(new double[] {8, 10}, product.toDoubleArray()[0], 0);
        assertArrayEquals(new double[] {12, 15}, product.toDoubleArray()[1], 0);
    }

    @Test public void outerProductRejectsEmptyNonfiniteAndOverflowingInputs() {
        assertThrows(IllegalArgumentException.class, () -> Matrix.outerProduct(null, new double[] {1}));
        assertThrows(IllegalArgumentException.class, () -> Matrix.outerProduct(new double[] {1}, null));
        assertThrows(IllegalArgumentException.class, () -> Matrix.outerProduct(new double[0], new double[] {1}));
        assertThrows(IllegalArgumentException.class, () -> Matrix.outerProduct(new double[] {1}, new double[0]));
        assertThrows(IllegalArgumentException.class,
                () -> Matrix.outerProduct(new double[] {Double.NaN}, new double[] {1}));
        assertThrows(IllegalArgumentException.class,
                () -> Matrix.outerProduct(new double[] {1}, new double[] {Double.POSITIVE_INFINITY}));
        assertThrows(IllegalArgumentException.class,
                () -> Matrix.outerProduct(new double[] {Double.MAX_VALUE}, new double[] {2}));
    }

    @Test public void rectangularMultiplicationAndTransposeMatchKnownResults() {
        Matrix<Double> left = Matrix.of(new double[][] {{1, 2, 3}, {4, 5, 6}});
        Matrix<Double> right = Matrix.of(new double[][] {{7, 8}, {9, 10}, {11, 12}});
        Matrix<Double> product = left.multiply(right);
        assertArrayEquals(new double[] {58, 64}, product.toDoubleArray()[0], 0);
        assertArrayEquals(new double[] {139, 154}, product.toDoubleArray()[1], 0);
        assertArrayEquals(new Double[] {6.0, 15.0}, left.multiply(new Double[] {1.0, 1.0, 1.0}, Double[]::new));
        Matrix<Double> transposed = left.transpose();
        assertArrayEquals(new double[] {1, 4}, transposed.toDoubleArray()[0], 0);
        assertArrayEquals(new double[] {2, 5}, transposed.toDoubleArray()[1], 0);
        assertArrayEquals(new double[] {3, 6}, transposed.toDoubleArray()[2], 0);
    }
}
