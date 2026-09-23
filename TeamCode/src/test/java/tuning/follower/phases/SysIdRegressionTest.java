package tuning.follower.phases;

import org.junit.Test;
import static org.junit.Assert.*;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

public class SysIdRegressionTest {
    private SysIdRegression plant(double ks, double kv, double ka) {
        SysIdRegression data = new SysIdRegression();
        for (int run = 0; run < 4; run++) {
            double direction = run % 2 == 0 ? 1 : -1;
            for (int i = 0; i < 40; i++) {
                double v = direction * (2 + i * .4);
                double a = direction * (run < 2 ? 2 : 20 * Math.exp(-i * .1));
                data.add(run, .1 * (i + 1), v, a, ks * direction + kv * v + ka * a);
            }
        }
        return data;
    }
    @Test public void identifiesAllThreeGainsInBothDirections() {
        SysIdRegression.Fit fit = plant(.06, .012, .006).fit(2);
        assertTrue(fit.valid);
        assertEquals(.06, fit.kS, 1e-10);
        assertEquals(.012, fit.kV, 1e-10);
        assertEquals(.006, fit.kA, 1e-10);
        assertEquals(1, fit.rSquared, 1e-10);
    }
    @Test public void rejectsMissingDirectionAndUnidentifiableData() {
        SysIdRegression data = plant(.06, .012, .006);
        data.samples.removeIf(s -> s.run == 3);
        assertFalse(data.fit(2).valid);
        data = new SysIdRegression();
        for (int run = 0; run < 4; run++) {
            for (int i = 0; i < 40; i++) { data.add(run, i * .1, i + 1, 2, .1 + .01 * i); }
        }
        assertFalse(data.fit(2).valid);
    }
    @Test public void refitsNearZeroFrictionAtPhysicalBoundary() {
        SysIdRegression data = plant(-.001, .012, .006);
        SysIdRegression.Fit fit = data.fit(2);
        assertTrue(fit.valid);
        assertEquals(0, fit.kS, 0);
        assertTrue(fit.kV > 0 && fit.kA > 0);
        double clampedError = 0, refittedError = 0;
        for (SysIdRegression.Sample s : data.samples) {
            clampedError += Math.pow(.012 * s.velocity + .006 * s.acceleration - s.power, 2);
            refittedError += Math.pow(fit.predict(s) - s.power, 2);
        }
        assertTrue(refittedError < clampedError);
    }
    @Test public void rejectsInvalidSamplesAndNegativeGains() {
        SysIdRegression data = new SysIdRegression();
        assertFalse(data.add(0, 0, Double.NaN, 1, .5));
        assertFalse(data.add(0, 0, 1, Double.POSITIVE_INFINITY, .5));
        assertFalse(data.add(0, 0, 1, 1, 1.1));
        assertFalse(plant(.06, .012, -.006).fit(2).valid);
        assertTrue(plant(.06, .012, 0).fit(2).valid);
    }
    @Test public void writesXChartPngsForEveryTest() throws Exception {
        SysIdRegression data = plant(.06, .012, .006);
        SysIdRegression.Fit fit = data.fit(2);
        Path directory = java.nio.file.Paths.get("build/sysid-graphs");
        Files.createDirectories(directory);
        Path csv = directory.resolve("drive_fit.csv");
        List<String> lines = new ArrayList<>();
        lines.add("test,time_s,velocity,acceleration,applied_power,fitted_power,residual,kS,kV,kA,r_squared,valid");
        for (SysIdRegression.Sample s : data.samples) {
            lines.add(s.run + "," + s.time + "," + s.velocity + "," + s.acceleration + "," + s.power
                    + "," + fit.predict(s) + "," + (s.power-fit.predict(s)) + "," + fit.kS + "," + fit.kV + "," + fit.kA + "," + fit.rSquared + ",true");
        }
        Files.write(csv, lines);
        SysIdGraphs.render(csv.toString(), "DRIVE");
        for (int run = 0; run < 4; run++) {
            for (String kind : new String[]{"power", "motion", "residual"}) {
                byte[] image = Files.readAllBytes(directory.resolve("drive_fit_" + run + "_" + kind + ".png"));
                assertEquals(1000, java.nio.ByteBuffer.wrap(image, 16, 4).getInt()); assertEquals(460, java.nio.ByteBuffer.wrap(image, 20, 4).getInt());
            }
        }
    }
}
