package tuning.follower.characterization;

import feedforward.MotorPowerModel;
import org.junit.Test;
import static org.junit.Assert.*;

public class MotorAuthorityRegressionTest {
    private ValidatedMotorAuthorityRegression data(double slope, double validationBias, double validationRange) {
        ValidatedMotorAuthorityRegression r = new ValidatedMotorAuthorityRegression();
        for (boolean heldOut : new boolean[]{false, true}) {
            for (int i = 1; i <= 30; i++) {
                double v = i * (heldOut ? validationRange : 1.5);
                for (double power : new double[]{.4, .7, 1}) {
                    r.add(heldOut, v, (150 + slope * v) * power - 30 - .5 * v
                            + (heldOut ? validationBias : 0), power);
                }
            }
        }
        return r;
    }
    @Test public void identifiesDutySpeedCouplingAndHoldingPowerWithoutPlantConstants() {
        MotorPowerModel m = data(-1.5, 0, 1.5).fit();
        assertEquals(150, m.gain0, 1e-7);
        assertEquals(-1.5, m.gainSlope, 1e-7);
        assertEquals(30, m.drag0, 1e-7);
        assertEquals(.5, m.dragSlope, 1e-7);
        assertEquals(0, m.acceleration(40, m.holdingPower(40)), 1e-7);
        assertTrue(m.kA(40) > m.kA(10));
    }
    @Test public void acceptsConventionalConstantAccelerationGainMotor() {
        MotorPowerModel m = data(0, 0, 1.5).fit();
        assertEquals(0, m.gainSlope, 1e-7);
        assertEquals(m.kA(0), m.kA(40), 1e-7);
    }
    @Test public void rejectsIndependentValidationBiasAndMissingSpeedCoverage() {
        assertThrows(IllegalStateException.class, () -> data(-1.5, 20, 1.5).fit());
        assertThrows(IllegalStateException.class, () -> data(-1.5, 0, .6).fit());
    }
    @Test public void validationCoverageRequiresTheHighSpeedRangeUsedByFit() {
        assertTrue(data(-1.5, 0, 1.5).validationCovered());
        assertFalse("Enough samples in a short range must still request another held-out run",
                data(-1.5, 0, .6).validationCovered());
    }
    @Test public void rejectsUnidentifiableSinglePowerMeasurement() {
        ValidatedMotorAuthorityRegression r = new ValidatedMotorAuthorityRegression();
        for (int i = 1; i <= 30; i++) {
            r.add(false, i, 100 - i, .5); r.add(true, i, 100 - i, .5);
        }
        assertThrows(IllegalStateException.class, r::fit);
    }
    @Test public void acquisitionFitNeedsNoValidationSamplesAndReportsMeasuredResidual() {
        MotorAuthorityRegression r = new MotorAuthorityRegression();
        for (int i = 1; i <= 30; i++) {
            double v = i * 1.5;
            for (double power : new double[]{.65, 1}) {
                r.add(v, (150 - 1.5 * v) * power - 30 - .5 * v, power);
            }
        }
        MotorPowerModel model = r.fit();
        assertEquals(150, model.gain0, 1e-7);
        assertEquals(-1.5, model.gainSlope, 1e-7);
        assertEquals(42.75, model.maxVelocity, 1e-7);
        assertEquals(0, model.validationRms, 1e-7);
    }
    @Test public void invalidAndInsufficientAcquisitionCannotCreateModel() {
        MotorAuthorityRegression r = new MotorAuthorityRegression();
        for (int i = 0; i < 100; i++) r.add(Double.NaN, 1, .65);
        assertThrows(IllegalStateException.class, r::fit);
        for (int i = 1; i <= 30; i++) r.add(i, 100 - i, .65);
        assertThrows(IllegalStateException.class, r::fit);
    }

    @Test public void acquisitionResidualIsDiagnosticRatherThanValidationGate() {
        MotorAuthorityRegression r = new MotorAuthorityRegression();
        for (int i = 1; i <= 30; i++) {
            double v = i * 1.5;
            for (double power : new double[]{.65, 1}) {
                r.add(v, (150 - 1.5 * v) * power - 30 - .5 * v + 10 * Math.sin(i * 2), power);
            }
        }
        MotorPowerModel model = r.fit();
        assertTrue("Acquisition error is recorded, not used as held-out acceptance", model.validationRms > .04);
        assertEquals(model.validationRms, MotorPowerModel.fromJson(model.toJson()).validationRms, 0);
    }

}
