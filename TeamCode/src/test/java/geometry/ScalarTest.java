package geometry;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotSame;

public class ScalarTest {
    private static final double EPSILON = 1e-9;

    @Test
    public void arithmeticReturnsExpectedValues() {
        Scalar value = Scalar.of(-3);

        assertEquals(1, value.plus(Scalar.of(4)).get(), EPSILON);
        assertEquals(1, value.plus(4).get(), EPSILON);
        assertEquals(-5, value.minus(Scalar.of(2)).get(), EPSILON);
        assertEquals(-5, value.minus(2).get(), EPSILON);
        assertEquals(-6, value.times(2).get(), EPSILON);
        assertEquals(-12, value.times(Scalar.of(4)).get(), EPSILON);
        assertEquals(-1.5, value.div(2).get(), EPSILON);
        assertEquals(-0.75, value.div(Scalar.of(4)).get(), EPSILON);
        assertEquals(3, value.abs().get(), EPSILON);
        assertEquals(5, value.hypot(Scalar.of(4)).get(), EPSILON);
        assertEquals(3, value.mirror().get(), EPSILON);
    }

    @Test
    public void copyReturnsAnIndependentScalarWithTheSameValue() {
        Scalar original = Scalar.of(2.5);
        Scalar copy = original.copy();

        assertNotSame(original, copy);
        assertEquals(original.get(), copy.get(), EPSILON);
    }

    @Test
    public void distanceSupportsTypedScalars() {
        Dist distance = Dist.fromIn(12);

        assertEquals(18, distance.times(Scalar.of(1.5)).getIn(), EPSILON);
        assertEquals(4, distance.div(Scalar.of(3)).getIn(), EPSILON);
    }
}
