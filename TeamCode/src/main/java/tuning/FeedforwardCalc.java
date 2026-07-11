package tuning;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import controllers.PDSController.PDSCoefficients;

/**
 *
 */
public class FeedforwardCalc {
    public static final class Sample {
        final double power;
        final double velocity;
        final double acceleration;

        Sample(double power, double velocity, double acceleration) {
            this.power = power;
            this.velocity = velocity;
            this.acceleration = acceleration;
        }
    }

    public static final class Result {
        public final double kS;
        public final double kV;
        public final double kA;
        public final double rSquared;
        public final int sampleCount;

        Result(double kS, double kV, double kA, double rSquared, int sampleCount) {
            this.kS = kS;
            this.kV = kV;
            this.kA = kA;
            this.rSquared = rSquared;
            this.sampleCount = sampleCount;
        }

        public boolean isUsable() {
            return sampleCount >= 40 && finite(kS) && finite(kV) && finite(kA) &&
                    kS >= 0.0 && kS < 0.5 && kV > 0.0 && kA > 0.0 && rSquared >= 0.75;
        }

        /** Initial position-loop PDS gains for the fitted second-order plant. */
        public PDSCoefficients positionGains(double settlingTimeSeconds) {
            double damping = 0.90;
            double naturalFrequency = 4.0 / (damping * settlingTimeSeconds);
            double kP = kA * naturalFrequency * naturalFrequency;
            double kD = Math.max(0.0,
                    (2.0 * damping * naturalFrequency * kA) - kV);
            return new PDSCoefficients(kP, kD, kS, 0.0);
        }

        /** Velocity error gain for the requested closed-loop time constant. */
        public double velocityGain(double timeConstantSeconds) {
            return Math.max(0.0, (kA / timeConstantSeconds) - kV);
        }
    }

    private final List<Sample> samples = new ArrayList<>();

    public void add(double power, double velocity, double acceleration,
                    double minimumVelocity) {
        if (!finite(power) || !finite(velocity) || !finite(acceleration) ||
                Math.abs(velocity) < minimumVelocity || Math.abs(power) < 0.02) {
            return;
        }
        samples.add(new Sample(power, velocity, acceleration));
    }

    public int size() { return samples.size(); }

    public Result solve() {
        if (samples.size() < 3) return new Result(Double.NaN, Double.NaN, Double.NaN, 0, samples.size());

        double[] weights = new double[samples.size()];
        for (int i = 0; i < weights.length; i++) weights[i] = 1.0;

        double[] beta = null;
        for (int iteration = 0; iteration < 5; iteration++) {
            beta = fitModel(weights);
            if (beta == null) {
                return new Result(Double.NaN, Double.NaN, Double.NaN, 0, samples.size());
            }

            List<Double> residualMagnitudes = new ArrayList<>();
            for (Sample sample : samples) {
                residualMagnitudes.add(Math.abs(sample.power - predict(beta, sample)));
            }
            double scale = Math.max(1e-4, median(residualMagnitudes) * 1.4826);
            double huberLimit = 1.5 * scale;
            for (int i = 0; i < samples.size(); i++) {
                double residual = Math.abs(samples.get(i).power - predict(beta, samples.get(i)));
                weights[i] = residual <= huberLimit ? 1.0 : huberLimit / residual;
            }
        }

        double mean = 0.0;
        for (Sample sample : samples) mean += sample.power;
        mean /= samples.size();
        double residualSum = 0.0;
        double totalSum = 0.0;
        for (Sample sample : samples) {
            double residual = sample.power - predict(beta, sample);
            residualSum += residual * residual;
            double centered = sample.power - mean;
            totalSum += centered * centered;
        }
        double rSquared = totalSum <= 1e-12 ? 0.0 : 1.0 - (residualSum / totalSum);
        return new Result(Math.abs(beta[0]), beta[1], beta[2], rSquared, samples.size());
    }

    public double speedAt(double percentile) {
        List<Double> values = new ArrayList<>();
        for (Sample sample : samples) values.add(Math.abs(sample.velocity));
        return percentile(values, percentile);
    }

    public double accelAt(double percentile) {
        List<Double> values = new ArrayList<>();
        for (Sample sample : samples) values.add(Math.abs(sample.acceleration));
        return percentile(values, percentile);
    }

    private double[] fitModel(double[] weights) {
        double[][] normal = new double[3][3];
        double[] rhs = new double[3];
        for (int i = 0; i < samples.size(); i++) {
            Sample sample = samples.get(i);
            double[] x = {Math.signum(sample.velocity), sample.velocity, sample.acceleration};
            double weight = weights[i];
            for (int row = 0; row < 3; row++) {
                rhs[row] += weight * x[row] * sample.power;
                for (int col = 0; col < 3; col++) {
                    normal[row][col] += weight * x[row] * x[col];
                }
            }
        }
        for (int i = 0; i < 3; i++) normal[i][i] += 1e-9;
        return solve3x3(normal, rhs);
    }

    private static double predict(double[] beta, Sample sample) {
        return (beta[0] * Math.signum(sample.velocity)) +
                (beta[1] * sample.velocity) + (beta[2] * sample.acceleration);
    }

    private static double[] solve3x3(double[][] matrix, double[] vector) {
        double[][] augmented = new double[3][4];
        for (int row = 0; row < 3; row++) {
            System.arraycopy(matrix[row], 0, augmented[row], 0, 3);
            augmented[row][3] = vector[row];
        }
        for (int pivot = 0; pivot < 3; pivot++) {
            int best = pivot;
            for (int row = pivot + 1; row < 3; row++) {
                if (Math.abs(augmented[row][pivot]) > Math.abs(augmented[best][pivot])) best = row;
            }
            double[] swap = augmented[pivot]; augmented[pivot] = augmented[best]; augmented[best] = swap;
            if (Math.abs(augmented[pivot][pivot]) < 1e-12) return null;
            double divisor = augmented[pivot][pivot];
            for (int col = pivot; col < 4; col++) augmented[pivot][col] /= divisor;
            for (int row = 0; row < 3; row++) {
                if (row == pivot) continue;
                double factor = augmented[row][pivot];
                for (int col = pivot; col < 4; col++) augmented[row][col] -= factor * augmented[pivot][col];
            }
        }
        return new double[]{augmented[0][3], augmented[1][3], augmented[2][3]};
    }

    private static double median(List<Double> values) { return percentile(values, 0.5); }

    private static double percentile(List<Double> values, double percentile) {
        if (values.isEmpty()) return 0.0;
        Collections.sort(values);
        int index = (int) Math.round(Math.max(0.0, Math.min(1.0, percentile)) * (values.size() - 1));
        return values.get(index);
    }

    private static boolean finite(double value) { return Double.isFinite(value); }
}
