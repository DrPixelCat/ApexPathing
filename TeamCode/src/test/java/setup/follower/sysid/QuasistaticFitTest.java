package setup.follower.sysid;

import org.junit.Test;
import static org.junit.Assert.*;

public class QuasistaticFitTest {
    private SysIdRegression line(double ks, double kv, int directions) {
        SysIdRegression samples = new SysIdRegression();
        for (int run = 0; run < directions; run++) {
            int direction = run == 0 ? 1 : -1;
            for (int i = 0; i < 30; i++) {
                double speed = kv > .05 ? .2 + .1 * i : 2 + i;
                samples.add(run, i * .1, direction * speed, direction * 4,
                        direction * (ks + kv * speed));
            }
        }
        return samples;
    }

    @Test public void movingRampsFitTheInterceptAndSlopeWithoutChangingKA() {
        SysIdRegression samples = line(.18, .02, 2);
        SysIdRegression.Fit fit = QuasistaticFit.fit(samples, 2, 2, .123);
        assertTrue(fit.valid);
        assertEquals(.18, fit.kS, 1e-12);
        assertEquals(.02, fit.kV, 1e-12);
        assertEquals(.123, fit.kA, 0);
        assertEquals(1, fit.rSquared, 1e-12);
        assertTrue(QuasistaticFit.fit(line(.05, .1, 1), 1, .2, 0).valid);
    }

    @Test public void rampAcquisitionRecoversARepresentativeLinearFeedforwardPlant() {
        // Independent first-order plant. It never changes FTCodeSim's motor model.
        double ks = .18, kv = .02, ka = .001, dt = .005;
        SysIdRegression samples = new SysIdRegression();
        for (int run = 0; run < 2; run++) {
            double velocity = 0, elapsed = 0, velocityIntegral = 0, powerIntegral = 0;
            double initialVelocity = 0;
            int direction = run == 0 ? 1 : -1;
            for (int tick = 1; tick <= 1200; tick++) {
                double time = tick * dt;
                double power = Math.abs(FeedforwardTuner.trialPower(FeedforwardTuner.Axis.DRIVE, run, time - dt));
                double nextVelocity = Math.max(0, velocity + (power - ks - kv * velocity) / ka * dt);
                if (velocity >= .5 && nextVelocity >= .5) {
                    elapsed += dt;
                    velocityIntegral += (velocity + nextVelocity) * .5 * dt;
                    powerIntegral += power * dt;
                    if (elapsed >= .1) {
                        samples.add(run, time, direction * velocityIntegral / elapsed,
                                direction * (nextVelocity - initialVelocity) / elapsed,
                                direction * powerIntegral / elapsed);
                        elapsed = velocityIntegral = powerIntegral = 0;
                        initialVelocity = nextVelocity;
                    }
                } else {
                    elapsed = velocityIntegral = powerIntegral = 0;
                    initialVelocity = nextVelocity;
                }
                velocity = nextVelocity;
            }
        }
        SysIdRegression.Fit fit = QuasistaticFit.fit(samples, 2, 2, ka);
        assertTrue(fit.valid);
        // A slow ramp still carries a small inertial offset; do not claim an exact kS measurement.
        assertEquals(ks, fit.kS, .01);
        assertEquals(kv, fit.kV, .0002);
        for (double heldOutSpeed : new double[]{5, 15, 25}) {
            assertEquals(ks + kv * heldOutSpeed, fit.kS + fit.kV * heldOutSpeed, .01);
        }
    }

    @Test public void missingDirectionsAndFrozenReadingsCannotProduceAValidFit() {
        assertFalse(QuasistaticFit.fit(line(.18, .02, 1), 2, 2, 0).valid);
        SysIdRegression frozen = new SysIdRegression();
        for (int run = 0; run < 2; run++) {
            int direction = run == 0 ? 1 : -1;
            for (int i = 0; i < 30; i++) {
                frozen.add(run, i * .1, direction * 10, 0, direction * (.2 + i * .01));
            }
        }
        assertFalse(QuasistaticFit.fit(frozen, 2, 2, 0).valid);
    }

    @Test public void nonlinearAndDirectionDependentResponsesAreRejected() {
        SysIdRegression curved = new SysIdRegression(), asymmetric = new SysIdRegression();
        for (int run = 0; run < 2; run++) {
            int direction = run == 0 ? 1 : -1;
            for (int i = 0; i < 30; i++) {
                double speed = 2 + i;
                curved.add(run, i * .1, direction * speed, 0, direction * (.2 + .0007 * speed * speed));
                asymmetric.add(run, i * .1, direction * speed, 0,
                        direction * (.15 + .01 * speed + run * .1));
            }
        }
        assertFalse(QuasistaticFit.fit(curved, 2, 2, 0).valid);
        assertFalse(QuasistaticFit.fit(asymmetric, 2, 2, 0).valid);
        assertFalse(QuasistaticFit.fit(line(-.1, .02, 2), 2, 2, 0).valid);
    }

    @Test public void aGoodLocalLineDoesNotProveTheStockNonlinearPlantMatchesFeedforward() {
        SysIdRegression stock = new SysIdRegression();
        for (int run = 0; run < 2; run++) {
            int direction = run == 0 ? 1 : -1;
            for (int i = 0; i < 30; i++) {
                double speed = 10 + .5 * i;
                double power = 49 / (183 - (150.0 / 65) * speed);
                stock.add(run, i * .1, direction * speed, 0, direction * power);
            }
        }
        SysIdRegression.Fit fit = QuasistaticFit.fit(stock, 2, 2, 0);
        assertTrue("A narrow interval can look linear", fit.valid);
        double heldOutPower = 49 / (183 - (150.0 / 65) * 50);
        assertTrue("Held-out speed exposes the stock model mismatch",
                Math.abs(heldOutPower - fit.kS - fit.kV * 50) > .1);
    }
}
