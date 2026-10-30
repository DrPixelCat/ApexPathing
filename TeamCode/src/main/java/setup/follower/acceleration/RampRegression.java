package setup.follower.acceleration;

import java.util.ArrayList;
import java.util.List;

/** Moving-friction regression, optionally separating inertial power with paired ramps. */
final class RampRegression {
    static final class Sample {
        final double velocity, power, acceleration, powerVelocity;
        boolean excluded;
        Sample(double velocity, double power, double acceleration) {
            this(velocity, power, acceleration, velocity * power);
        }
        Sample(double velocity, double power, double acceleration, double powerVelocity) {
            this.velocity = velocity; this.power = power; this.acceleration = acceleration;
            this.powerVelocity = powerVelocity;
        }
    }

    static final class Fit {
        final int count;
        final double kS, kV, rSquared, span;
        double kA;
        double linearKV, kPowerVelocity;
        Fit(int count, double kS, double kV, double rSquared, double span) {
            this.count = count; this.kS = kS; this.kV = kV;
            this.rSquared = rSquared; this.span = span;
            this.linearKV = kV;
        }
        double residual(Sample s) {
            return s.power - kS - linearKV*s.velocity - kA*s.acceleration
                    - kPowerVelocity*s.powerVelocity;
        }
        boolean valid(double minimumSpan) {
            return count >= 20 && span >= minimumSpan && Double.isFinite(kS)
                    && Double.isFinite(kV) && kS >= 0 && kS < .9 && kV > 0
                    && Double.isFinite(rSquared) && rSquared >= .9;
        }
    }

    final List<Sample> samples = new ArrayList<>();
    private final double minimumVelocity;
    private boolean modelInertia;
    private boolean modelPowerVelocity;
    private boolean fixedStatic, motionStarted;
    private double stationaryPower = Double.NaN;
    RampRegression(double minimumVelocity) { this.minimumVelocity = minimumVelocity; }

    /** Before motion, track the greatest power resisted by friction; afterwards fit only kV. */
    boolean addRisingSample(double velocity, double power) {
        fixedStatic = true;
        if (!Double.isFinite(velocity) || !Double.isFinite(power) || power < 0 || power > .9) {
            return false;
        }
        if (!motionStarted) {
            if (Math.abs(velocity) < minimumVelocity) {
                stationaryPower = Double.isNaN(stationaryPower) ? power : Math.max(stationaryPower, power);
                return false;
            }
            if (velocity < minimumVelocity) { return false; }
            motionStarted = true;
        }
        return add(velocity, power);
    }

    boolean hasStartedMoving() { return motionStarted; }

    private Fit fitFixedStatic() {
        int n = 0;
        double vv = 0, vp = 0, meanPower = 0;
        double min = Double.POSITIVE_INFINITY, max = Double.NEGATIVE_INFINITY;
        for (Sample s : samples) {
            if (s.excluded) { continue; }
            n++; vv += s.velocity*s.velocity;
            vp += s.velocity*(s.power-stationaryPower);
            meanPower += s.power;
            min = Math.min(min, s.velocity); max = Math.max(max, s.velocity);
        }
        if (!motionStarted || n < 2 || !(vv > 0) || !Double.isFinite(stationaryPower)) {
            return new Fit(n, stationaryPower, Double.NaN, Double.NaN, 0);
        }
        double kv = vp/vv, residual = 0, total = 0;
        meanPower /= n;
        for (Sample s : samples) {
            if (s.excluded) { continue; }
            double error = s.power-stationaryPower-kv*s.velocity;
            residual += error*error;
            total += (s.power-meanPower)*(s.power-meanPower);
        }
        return new Fit(n, stationaryPower, kv, total > 0 ? 1-residual/total : Double.NaN, max-min);
    }

    boolean add(double velocity, double power) {
        if (!Double.isFinite(velocity) || !Double.isFinite(power)
                || velocity < minimumVelocity || power <= 0 || power > .9) { return false; }
        samples.add(new Sample(velocity, power, 0));
        return true;
    }

    boolean add(double velocity, double power, double acceleration) {
        modelInertia = true;
        if (!Double.isFinite(velocity) || !Double.isFinite(power)
                || !Double.isFinite(acceleration) || velocity < minimumVelocity
                || power < 0 || power > .9) { return false; }
        samples.add(new Sample(velocity, power, acceleration));
        return true;
    }

    /** The product must be integrated before averaging; mean(p*v) != mean(p)*mean(v). */
    boolean add(double velocity, double power, double acceleration, double powerVelocity) {
        modelPowerVelocity = true;
        if (!Double.isFinite(powerVelocity) || !Double.isFinite(velocity)
                || !Double.isFinite(power) || !Double.isFinite(acceleration)
                || velocity < minimumVelocity || power < 0 || power > .9) { return false; }
        samples.add(new Sample(velocity, power, acceleration, powerVelocity));
        return true;
    }

    Fit fit() {
        if (fixedStatic) { return fitFixedStatic(); }
        if (modelPowerVelocity) { return fitPowerVelocity(); }
        if (modelInertia) { return fitInertia(); }
        int n = 0;
        double x = 0, y = 0, min = Double.POSITIVE_INFINITY, max = Double.NEGATIVE_INFINITY;
        for (Sample s : samples) {
            if (s.excluded) { continue; }
            n++; x += s.velocity; y += s.power;
            min = Math.min(min, s.velocity); max = Math.max(max, s.velocity);
        }
        if (n < 2) { return new Fit(n, Double.NaN, Double.NaN, Double.NaN, 0); }
        x /= n; y /= n;
        double xx = 0, xy = 0, yy = 0;
        for (Sample s : samples) {
            if (s.excluded) { continue; }
            double dx = s.velocity - x, dy = s.power - y;
            xx += dx * dx; xy += dx * dy; yy += dy * dy;
        }
        double slope = xy / xx;
        return new Fit(n, y - slope * x, slope, xy * xy / (xx * yy), max - min);
    }

    /**
     * Identify p = S + B*v + A*a + C*(p*v). This includes ordinary linear feedforward
     * (C=0) and duty-dependent drag, without confusing either drag or inertia with S.
     * kV remains a least-squares constant approximation to the identified holding-power
     * curve over the measured speeds, with the independently identified S held fixed.
     */
    private Fit fitPowerVelocity() {
        int n = 0, accelerating = 0, braking = 0;
        double[] mean = new double[3];
        double pMean = 0, min = Double.POSITIVE_INFINITY, max = 0;
        for (Sample s : samples) {
            if (s.excluded) { continue; }
            n++; mean[0] += s.velocity; mean[1] += s.acceleration;
            mean[2] += s.powerVelocity; pMean += s.power;
            if (s.acceleration > 0) { accelerating++; }
            if (s.acceleration < 0) { braking++; }
            min = Math.min(min, s.velocity); max = Math.max(max, s.velocity);
        }
        Fit invalid = new Fit(n, Double.NaN, Double.NaN, Double.NaN, max-min);
        // At least five windows per direction for a four-coefficient model; total
        // count, speed span, rank and residual checks still apply independently.
        if (accelerating < 5 || braking < 5) { return invalid; }
        for (int i=0; i<3; i++) { mean[i] /= n; }
        pMean /= n;
        double[][] normal = new double[3][4];
        double pp = 0;
        for (Sample s : samples) {
            if (s.excluded) { continue; }
            double[] x = {s.velocity-mean[0], s.acceleration-mean[1], s.powerVelocity-mean[2]};
            double p = s.power-pMean;
            pp += p*p;
            for (int i=0; i<3; i++) {
                for (int j=0; j<3; j++) { normal[i][j] += x[i]*x[j]; }
                normal[i][3] += x[i]*p;
            }
        }
        // Both drag terms dissipate energy: B >= 0 and C >= 0. Fit the active
        // sets explicitly rather than letting noise create negative drag that
        // cancels positive drag. BIC prefers a simpler model unless the extra
        // term measurably improves the fit. No expected S enters this choice.
        Fit best = invalid;
        double bestScore = Double.POSITIVE_INFINITY;
        for (int mask : new int[] {3, 6, 7}) {
            double[] coefficient = solve(normal, mask);
            if (coefficient == null) { continue; }
            double b = coefficient[0], a = coefficient[1], c = coefficient[2];
            if (b < 0 || a < 0 || c < 0 || !(pp > 0)) { continue; }
            double ks = pMean-b*mean[0]-a*mean[1]-c*mean[2];
            if (!Double.isFinite(ks) || ks < 0 || b+c*ks <= 0) { continue; }
            double squaredError = 0, vv = 0, vp = 0;
            boolean usable = true;
            for (Sample s : samples) {
                if (s.excluded) { continue; }
                double denominator = 1-c*s.velocity;
                if (!(denominator > .05)) { usable = false; break; }
                double holdingPower = (ks+b*s.velocity)/denominator;
                vv += s.velocity*s.velocity;
                vp += s.velocity*(holdingPower-ks);
                double error = s.power-ks-b*s.velocity-a*s.acceleration-c*s.powerVelocity;
                squaredError += error*error;
            }
            if (!usable) { continue; }
            double score = n*Math.log(Math.max(squaredError/n, 1e-20))
                    + (Integer.bitCount(mask)+1)*Math.log(n);
            if (score < bestScore) {
                bestScore = score;
                best = new Fit(n, ks, vp/vv, 1-squaredError/pp, max-min);
                best.kA = a; best.linearKV = b; best.kPowerVelocity = c;
            }
        }
        return best;
    }

    /** Centered, column-scaled least squares for one active set of drag terms. */
    private static double[] solve(double[][] normal, int mask) {
        int size = Integer.bitCount(mask);
        int[] index = new int[size];
        double[] scale = new double[size];
        for (int i=0, j=0; i<3; i++) {
            if ((mask & (1 << i)) == 0) { continue; }
            index[j] = i;
            scale[j] = Math.sqrt(normal[i][i]);
            if (!(scale[j] > 0)) { return null; }
            j++;
        }
        double[][] matrix = new double[size][size+1];
        for (int i=0; i<size; i++) {
            for (int j=0; j<size; j++) {
                matrix[i][j] = normal[index[i]][index[j]]/(scale[i]*scale[j]);
            }
            matrix[i][size] = normal[index[i]][3]/scale[i];
        }
        for (int col=0; col<size; col++) {
            int pivot = col;
            for (int row=col+1; row<size; row++) {
                if (Math.abs(matrix[row][col]) > Math.abs(matrix[pivot][col])) { pivot = row; }
            }
            if (Math.abs(matrix[pivot][col]) < 1e-6) { return null; }
            double[] swap = matrix[col]; matrix[col] = matrix[pivot]; matrix[pivot] = swap;
            double divisor = matrix[col][col];
            for (int j=col; j<=size; j++) { matrix[col][j] /= divisor; }
            for (int row=0; row<size; row++) {
                if (row == col) { continue; }
                double factor = matrix[row][col];
                for (int j=col; j<=size; j++) { matrix[row][j] -= factor*matrix[col][j]; }
            }
        }
        double[] result = new double[3];
        for (int i=0; i<size; i++) {
            result[index[i]] = matrix[i][size]/scale[i];
        }
        return result;
    }

    /** Fit moving power = kS + kV*v + kA*a; both ramp directions identify the intercept. */
    private Fit fitInertia() {
        int n = 0, accelerating = 0, braking = 0;
        double v = 0, a = 0, p = 0, min = Double.POSITIVE_INFINITY, max = 0;
        for (Sample s : samples) {
            if (s.excluded) { continue; }
            n++; v += s.velocity; a += s.acceleration; p += s.power;
            if (s.acceleration > 0) { accelerating++; }
            if (s.acceleration < 0) { braking++; }
            min = Math.min(min, s.velocity); max = Math.max(max, s.velocity);
        }
        Fit invalid = new Fit(n, Double.NaN, Double.NaN, Double.NaN, max - min);
        if (accelerating < 10 || braking < 10) { return invalid; }
        v /= n; a /= n; p /= n;
        double vv = 0, aa = 0, va = 0, vp = 0, ap = 0, pp = 0;
        for (Sample s : samples) {
            if (s.excluded) { continue; }
            double dv = s.velocity-v, da = s.acceleration-a, dp = s.power-p;
            vv += dv*dv; aa += da*da; va += dv*da; vp += dv*dp; ap += da*dp; pp += dp*dp;
        }
        double determinant = vv*aa-va*va;
        if (determinant <= 1e-6 * vv*aa || vv <= 0 || aa <= 0 || pp <= 0) { return invalid; }
        double kv = (vp*aa-ap*va)/determinant;
        double ka = (ap*vv-vp*va)/determinant;
        if (!Double.isFinite(ka) || ka < 0) { return invalid; }
        double ks = p-kv*v-ka*a;
        double residual = 0;
        for (Sample s : samples) {
            if (s.excluded) { continue; }
            double error = s.power-ks-kv*s.velocity-ka*s.acceleration;
            residual += error*error;
        }
        Fit result = new Fit(n, ks, kv, 1-residual/pp, max-min);
        result.kA = ka;
        return result;
    }

    /** Operator-requested exclusion only; never silently deletes an inconvenient measurement. */
    boolean excludeWorst() {
        Fit fit = fit();
        if (fit.count <= 20 || !Double.isFinite(fit.kV)) { return false; }
        Sample worst = null;
        double largest = -1;
        for (Sample s : samples) {
            if (s.excluded) { continue; }
            double residual = Math.abs(fit.residual(s));
            if (residual > largest) { largest = residual; worst = s; }
        }
        if (worst == null) { return false; }
        worst.excluded = true;
        return true;
    }

    void restore() { for (Sample s : samples) { s.excluded = false; } }
}
