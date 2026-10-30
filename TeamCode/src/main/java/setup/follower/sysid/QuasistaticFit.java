package setup.follower.sysid;

/** Fits power = kS + kV * speed from moving windows of slow power ramps. */
final class QuasistaticFit {
    private QuasistaticFit() { }

    /** Combines direction-normalized samples; acceleration is not fitted in this stage. */
    static SysIdRegression.Fit fit(SysIdRegression data, int directions, double minimumSpan, double existingKA) {
        int[] counts = new int[directions];
        double[] minimum = new double[directions], maximum = new double[directions];
        java.util.Arrays.fill(minimum, Double.POSITIVE_INFINITY);
        double meanVelocity = 0, meanPower = 0;
        int count = 0;
        for (SysIdRegression.Sample sample : data.samples) {
            if (sample.run >= directions) { continue; }
            counts[sample.run]++;
            double speed = Math.abs(sample.velocity);
            minimum[sample.run] = Math.min(minimum[sample.run], speed);
            maximum[sample.run] = Math.max(maximum[sample.run], speed);
            meanVelocity += speed;
            meanPower += Math.abs(sample.power);
            count++;
        }
        for (int run = 0; run < directions; run++) {
            if (counts[run] < 10 || maximum[run] - minimum[run] < minimumSpan) {
                return invalid(existingKA);
            }
        }
        meanVelocity /= count;
        meanPower /= count;
        double velocityVariance = 0, covariance = 0, powerVariance = 0;
        for (SysIdRegression.Sample sample : data.samples) {
            if (sample.run >= directions) { continue; }
            double velocityOffset = Math.abs(sample.velocity) - meanVelocity;
            double powerOffset = Math.abs(sample.power) - meanPower;
            velocityVariance += velocityOffset * velocityOffset;
            covariance += velocityOffset * powerOffset;
            powerVariance += powerOffset * powerOffset;
        }
        if (velocityVariance < 1e-9 || powerVariance < 1e-9) { return invalid(existingKA); }
        double kv = covariance / velocityVariance;
        double ks = meanPower - kv * meanVelocity;
        double squaredError = 0;
        double[] residuals = new double[directions];
        for (SysIdRegression.Sample sample : data.samples) {
            if (sample.run >= directions) { continue; }
            double residual = Math.abs(sample.power) - ks - kv * Math.abs(sample.velocity);
            squaredError += residual * residual;
            residuals[sample.run] += residual;
        }
        double rSquared = 1 - squaredError / powerVariance;
        double rms = Math.sqrt(squaredError / count), maxRunBias = 0;
        for (int run = 0; run < directions; run++) {
            maxRunBias = Math.max(maxRunBias, Math.abs(residuals[run] / counts[run]));
        }
        boolean valid = Double.isFinite(ks) && ks >= 0 && ks < .9 && Double.isFinite(kv) && kv > 0
                && rSquared >= .98 && rms <= .02 && maxRunBias <= .02;
        return new SysIdRegression.Fit(new double[]{ks, kv, existingKA}, rSquared, rms, maxRunBias, valid);
    }

    private static SysIdRegression.Fit invalid(double existingKA) {
        return new SysIdRegression.Fit(new double[]{Double.NaN, Double.NaN, existingKA},
                Double.NaN, Double.NaN, Double.NaN, false);
    }
}
