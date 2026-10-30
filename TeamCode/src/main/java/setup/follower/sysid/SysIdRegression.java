package setup.follower.sysid;

import java.util.ArrayList;
import java.util.List;

/** Fits power = kS sign(v) + kV v + kA a from signed, time-averaged windows. */
final class SysIdRegression {
    static final class Sample {
        final int run;
        final double time, velocity, acceleration, power;
        Sample(int run, double time, double velocity, double acceleration, double power) {
            this.run = run; this.time = time; this.velocity = velocity;
            this.acceleration = acceleration; this.power = power;
        }
    }
    static final class Fit {
        final double kS, kV, kA, rSquared, rms, maxRunBias;
        final boolean valid;
        Fit(double[] k, double rSquared, double rms, double maxRunBias, boolean valid) {
            kS = k[0]; kV = k[1]; kA = k[2];
            this.rSquared = rSquared; this.rms = rms;
            this.maxRunBias = maxRunBias; this.valid = valid;
        }
        double predict(Sample s) {
            return kS * Math.signum(s.velocity) + kV * s.velocity + kA * s.acceleration;
        }
    }
    final List<Sample> samples = new ArrayList<>();
    boolean add(int run, double time, double velocity, double acceleration, double power) {
        if (run < 0 || run > 3 || !Double.isFinite(time) || !Double.isFinite(velocity)
                || !Double.isFinite(acceleration) || !Double.isFinite(power)
                || Math.abs(power) > 1 || Math.abs(velocity) < 1e-6
                || Math.signum(velocity) != (run % 2 == 0 ? 1.0 : -1.0)
                || (run >= 2 && acceleration * velocity < 0.0)) { return false; }
        samples.add(new Sample(run, time, velocity, acceleration, power));
        return true;
    }
    Fit fit(double minimumSpan) {
        double[][] m = new double[3][4];
        int[] counts = new int[4];
        double[] runResiduals = new double[4];
        double min = Double.POSITIVE_INFINITY, max = 0, mean = 0;
        for (Sample s : samples) {
            double[] x = {Math.signum(s.velocity), s.velocity, s.acceleration};
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) { m[i][j] += x[i] * x[j]; }
                m[i][3] += x[i] * s.power;
            }
            counts[s.run]++; mean += s.power;
            min = Math.min(min, Math.abs(s.velocity)); max = Math.max(max, Math.abs(s.velocity));
        }
        // Normalize columns before elimination so angular and linear units have equal conditioning.
        double[] scale = new double[3];
        for (int i = 0; i < 3; i++) {
            scale[i] = Math.sqrt(m[i][i]);
            if (!(scale[i] > 0)) { return invalid(); }
        }
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) { m[i][j] /= scale[i] * scale[j]; }
            m[i][3] /= scale[i];
        }
        for (int i = 0; i < 3; i++) {
            int pivot = i;
            for (int j = i + 1; j < 3; j++) { if (Math.abs(m[j][i]) > Math.abs(m[pivot][i])) { pivot = j; } }
            double[] row = m[i]; m[i] = m[pivot]; m[pivot] = row;
            if (Math.abs(m[i][i]) < 1e-8) { return invalid(); }
            double d = m[i][i];
            for (int j = i; j < 4; j++) { m[i][j] /= d; }
            for (int r = 0; r < 3; r++) {
                if (r == i) { continue; }
                d = m[r][i];
                for (int j = i; j < 4; j++) { m[r][j] -= d * m[i][j]; }
            }
        }
        double[] k = {m[0][3] / scale[0], m[1][3] / scale[1], m[2][3] / scale[2]};
        // A frictionless plant can put the unconstrained intercept below zero through noise
        // or model mismatch. Refit the physical kS=0 boundary rather than clamp the old fit.
        // The same excitation, excitation and gain-sign checks still apply to the new model.
        if (k[0] < 0 && k[1] > 0 && k[2] >= 0) {
            double vv = 0, va = 0, aa = 0, vp = 0, ap = 0;
            for (Sample s : samples) {
                vv += s.velocity * s.velocity;
                va += s.velocity * s.acceleration;
                aa += s.acceleration * s.acceleration;
                vp += s.velocity * s.power;
                ap += s.acceleration * s.power;
            }
            double determinant = vv * aa - va * va;
            if (determinant > 1e-8 * vv * aa) {
                k = new double[] {0, (vp * aa - ap * va) / determinant,
                        (ap * vv - vp * va) / determinant};
            }
        }
        mean /= samples.size();
        double error = 0, total = 0;
        Fit candidate = new Fit(k, 0, 0, 0, false);
        for (Sample s : samples) {
            double residual = s.power - candidate.predict(s);
            error += residual * residual;
            runResiduals[s.run] += residual;
            total += Math.pow(s.power - mean, 2);
        }
        double r2 = total > 0 ? 1 - error / total : Double.NaN;
        boolean valid = k[0] >= 0 && k[0] < .9 && k[1] > 0 && k[2] >= 0
                && Double.isFinite(k[0]) && Double.isFinite(k[1]) && Double.isFinite(k[2])
                && max - min >= minimumSpan;
        double maxRunBias = 0.0;
        for (int run = 0; run < counts.length; run++) {
            maxRunBias = Math.max(maxRunBias,
                    Math.abs(runResiduals[run] / Math.max(1, counts[run])));
            valid &= counts[run] >= 5;
        }
        return new Fit(k, r2, Math.sqrt(error / samples.size()), maxRunBias, valid);
    }
    private Fit invalid() { return new Fit(new double[]{Double.NaN, Double.NaN, Double.NaN},
            Double.NaN, Double.NaN, Double.NaN, false); }
}
