package controllers;

import org.junit.Test;

import java.util.function.LongSupplier;

import paths.geometry.Scalar;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertThrows;

public class FeedbackControllerTest {
    private static final double EPSILON = 1e-9;

    @Test
    public void selectedTermsAreComposed() {
        FeedbackController controller = controller(
                allCoefficients());

        Scalar response = controller.p().d(-0.5).s().sqrt()
                .vel(0.25).accel(0.5).f(0.75).calculate(4.0);

        assertEquals(22.65, response.get(), EPSILON);
    }

    @Test
    public void calculationsOnlyIncludeSelectedTerms() {
        FeedbackController controller = controller(
                allCoefficients());

        assertEquals(4.0, controller.p().calculate(2.0).get(), EPSILON);
        assertEquals(10.0, controller.sqrt().calculate(4.0).get(), EPSILON);
        assertEquals(5.5, controller.vel(0.5).f(2.5).calculate(100.0).get(), EPSILON);
    }

    @Test
    public void mutableCalculationsRemainIndependentAndIdempotent() {
        FeedbackController controller = controller(allCoefficients());
        FeedbackController.Calculation proportional = controller.p();
        FeedbackController.Calculation velocity = controller.vel(1.0);

        assertEquals(4.0, proportional.calculate(2.0).get(), EPSILON);
        assertEquals(6.0, velocity.calculate(2.0).get(), EPSILON);
        assertEquals(20.0, controller.p().p().vel(1.0).vel(2.0)
                .f(3.0).f(4.0).calculate(2.0).get(), EPSILON);
    }

    @Test
    public void staticDirectionOverloadHonorsDirectionZeroAndDeadband() {
        FeedbackController controller = controller(
                new FeedbackController.Coefficients().s(0.4));

        assertEquals(-0.4, controller.s(-10).calculate(1.0).get(), EPSILON);
        assertEquals(0.0, controller.s(0).calculate(1.0).get(), EPSILON);
        assertEquals(0.0, controller.s(-10).calculate(0.24).get(), EPSILON);
        assertEquals(-0.4, controller.s().calculate(-1.0).get(), EPSILON);
    }

    @Test
    public void staticCompensationHonorsLinearAndAngularDeadbandBoundaries() {
        FeedbackController linear = controller(new FeedbackController.Coefficients().s(0.3));
        assertEquals(0.3, linear.s().calculate(0.26).get(), EPSILON);
        assertEquals(0.0, linear.s().calculate(0.25).get(), EPSILON);
        assertEquals(0.0, linear.s().calculate(0.24).get(), EPSILON);
        assertEquals(-0.3, linear.s().calculate(-0.26).get(), EPSILON);

        FeedbackController angular = controller(new FeedbackController.Coefficients().s(0.3));
        angular.setAngularController();
        assertEquals(0.3, angular.s().calculate(Math.toRadians(0.76)).get(), EPSILON);
        assertEquals(0.0, angular.s().calculate(Math.toRadians(0.75)).get(), EPSILON);
        assertEquals(-0.3, angular.s().calculate(Math.toRadians(-0.76)).get(), EPSILON);
        assertEquals(0.0, angular.s().calculate(Math.toRadians(-0.75)).get(), EPSILON);
    }

    @Test
    public void proportionalFeedbackStillWorksInsideStaticDeadband() {
        FeedbackController controller = controller(
                new FeedbackController.Coefficients().p(2.0).d(0.0).s(0.3));

        assertEquals(0.2, controller.p().d().s().calculate(0.1).get(), EPSILON);
    }

    @Test
    public void deadbandUpdatesApplyToTheCurrentControllerMode() {
        FeedbackController linear = controller(new FeedbackController.Coefficients().s(0.3));
        linear.setLinearStaticDeadband(2.0);
        assertEquals(0.0, linear.s().calculate(1.0).get(), EPSILON);

        FeedbackController angular = controller(new FeedbackController.Coefficients().s(0.3));
        angular.setAngularController();
        angular.setAngularStaticDeadband(0.5);
        assertEquals(0.0, angular.s().calculate(0.4).get(), EPSILON);
    }

    @Test
    public void zeroStaticDirectionDisablesOnlyStaticCompensation() {
        FeedbackController controller = controller(
                new FeedbackController.Coefficients().p(2.0).d(0.5).s(0.3));

        assertEquals(1.0, controller.p().d(-2.0).s(0.0).calculate(1.0).get(), EPSILON);
        assertEquals(2.3, controller.p().d(0.0).s().calculate(1.0).get(), EPSILON);
    }

    @Test
    public void internalDerivativeUsesElapsedTimeAndHasNoFirstRunKick() {
        MutableNanoTime clock = new MutableNanoTime();
        FeedbackController controller = new FeedbackController(
                new FeedbackController.Coefficients().d(2.0), clock);

        clock.advanceSeconds(0.02);
        assertEquals(0.0, controller.d().calculate(1.0).get(), EPSILON);
        clock.advanceSeconds(0.02);
        assertEquals(100.0, controller.d().calculate(2.0).get(), EPSILON);
    }

    @Test
    public void angularControllerWrapsFeedbackAndUsesAngularDeadband() {
        FeedbackController controller = controller(
                new FeedbackController.Coefficients().p(1.0).s(0.4));
        controller.setAngularController();

        assertEquals(-Math.toRadians(10), controller.p().calculate(Math.toRadians(350)).get(),
                EPSILON);
        assertEquals(0.0, controller.s().calculate(Math.toRadians(0.74)).get(), EPSILON);
        assertEquals(0.4, controller.s().calculate(Math.toRadians(0.76)).get(), EPSILON);
    }

    @Test
    public void scalarAdjustmentsRemainScalarOperations() {
        FeedbackController controller = controller(
                new FeedbackController.Coefficients().p(2.0));

        Scalar response = controller.p().calculate(3.0).plus(1.0).minus(Scalar.of(2.0));

        assertEquals(5.0, response.get(), EPSILON);
    }

    @Test
    public void componentRequiresItsCoefficient() {
        FeedbackController controller = controller(new FeedbackController.Coefficients().p(2.0));

        assertEquals(4.0, controller.p().calculate(2.0).get(), EPSILON);
        IllegalStateException error = assertThrows(IllegalStateException.class, controller::d);
        assertEquals("Cannot use d() without kD in the controller coefficients",
                error.getMessage());

        FeedbackController empty = controller(new FeedbackController.Coefficients());
        assertThrows(IllegalStateException.class, empty::p);
        assertThrows(IllegalStateException.class, empty::d);
        assertThrows(IllegalStateException.class, empty::sqrt);
        assertThrows(IllegalStateException.class, empty::s);
        assertThrows(IllegalStateException.class, () -> empty.s(1.0));
        assertThrows(IllegalStateException.class, () -> empty.vel(1.0));
        assertThrows(IllegalStateException.class, () -> empty.accel(1.0));
    }

    @Test
    public void calculationRevalidatesCoefficientsWhenTheyAreReplaced() {
        FeedbackController controller = controller(new FeedbackController.Coefficients().p(2.0));
        FeedbackController.Calculation calculation = controller.p();
        controller.setCoefficients(new FeedbackController.Coefficients().d(1.0));

        IllegalStateException error = assertThrows(
                IllegalStateException.class, () -> calculation.calculate(2.0));
        assertEquals("Cannot use p() without kP in the controller coefficients",
                error.getMessage());
    }

    private static FeedbackController controller(FeedbackController.Coefficients coefficients) {
        return new FeedbackController(coefficients, () -> 0L);
    }

    private static FeedbackController.Coefficients allCoefficients() {
        return new FeedbackController.Coefficients()
                .p(2.0).d(3.0).s(0.4).sqrt(5.0).vel(6.0).accel(7.0);
    }

    private static final class MutableNanoTime implements LongSupplier {
        private long value;

        @Override
        public long getAsLong() { return value; }

        void advanceSeconds(double seconds) { value += (long) (seconds * 1_000_000_000L); }
    }
}
