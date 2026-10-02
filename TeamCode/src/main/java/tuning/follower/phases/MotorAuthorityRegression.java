package tuning.follower.phases;

import java.util.ArrayList;
import java.util.List;
import core.MotorPowerModel;

/** Fit on acquisition runs; judge prediction on an entirely separate power sequence. */
public final class MotorAuthorityRegression {
    public static final class Sample {
        public final double velocity, acceleration, power, powerVelocity;
        public Sample(double velocity, double acceleration, double power, double powerVelocity) {
            this.velocity = velocity; this.acceleration = acceleration; this.power = power;
            this.powerVelocity = powerVelocity;
        }
    }
    private final List<Sample> training = new ArrayList<>(), validation = new ArrayList<>();
    boolean validationCovered() {
        double acquired = training.stream().mapToDouble(s -> s.velocity).max().orElse(0);
        double speed = Math.min(acquired, validation.stream().mapToDouble(s -> s.velocity).max().orElse(0));
        if (speed <= 0) { return false; }
        int[] count = new int[3]; int total = 0;
        for (Sample s : validation) {
            if (s.velocity <= speed) { count[Math.min(2, (int) (3 * s.velocity / speed))]++; total++; }
        }
        return total >= 18 && count[0] >= 2 && count[1] >= 2 && count[2] >= 2;
    }
    public void add(boolean heldOut, double velocity, double acceleration, double power) {
        add(heldOut, velocity, acceleration, power, power * velocity);
    }
    public void add(boolean heldOut, double velocity, double acceleration, double power, double powerVelocity) {
        if (Double.isFinite(velocity) && Double.isFinite(acceleration) && Double.isFinite(power)
                && Double.isFinite(powerVelocity)
                && velocity > 0 && power > .05 && power <= 1.001) {
            (heldOut ? validation : training).add(new Sample(velocity, acceleration, power, powerVelocity));
        }
    }
    public MotorPowerModel fit() {
        if (training.size() < 20 || validation.size() < 12) { throw new IllegalStateException("Too few moving measurements"); }
        double scale = training.stream().mapToDouble(s -> s.velocity).max().orElse(0);
        double[][] matrix = new double[4][5];
        for (Sample s : training) {
            double[] x = {s.power, s.powerVelocity / scale, 1, s.velocity / scale};
            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++) { matrix[i][j] += x[i] * x[j]; }
                matrix[i][4] += x[i] * s.acceleration;
            }
        }
        for (int i = 0; i < 4; i++) {
            int pivot = i;
            for (int j = i + 1; j < 4; j++) { if (Math.abs(matrix[j][i]) > Math.abs(matrix[pivot][i])) { pivot = j; } }
            double[] swap = matrix[i]; matrix[i] = matrix[pivot]; matrix[pivot] = swap;
            if (Math.abs(matrix[i][i]) < 1e-7) { throw new IllegalStateException("Power/speed response is not identifiable"); }
            double divisor = matrix[i][i];
            for (int j = i; j < 5; j++) { matrix[i][j] /= divisor; }
            for (int row = 0; row < 4; row++) {
                if (row == i) { continue; }
                double factor = matrix[row][i];
                for (int j = i; j < 5; j++) { matrix[row][j] -= factor * matrix[i][j]; }
            }
        }
        double gain0 = matrix[0][4], gainSlope = matrix[1][4] / scale;
        double drag0 = -matrix[2][4], dragSlope = -matrix[3][4] / scale;
        double validationMax = validation.stream().mapToDouble(s -> s.velocity).max().orElse(0);
        if (validationMax < .85 * scale) { throw new IllegalStateException("Validation did not cover high-speed response"); }
        double supportedSpeed = Math.min(scale, validationMax);
        double squared = 0, peak = 0;
        int supportedSamples = 0;
        int[] count = new int[3]; double[] bandSquared = new double[3];
        for (Sample s : validation) {
            // Validation can reach farther than usable acquisition windows after a timing gap.
            // Judge only their shared range, which also bounds the saved speed ceiling.
            if (s.velocity > supportedSpeed) { continue; }
            supportedSamples++;
            double gain = gain0 + gainSlope * s.velocity;
            if (gain <= 0) { throw new IllegalStateException("Nonpositive measured acceleration authority"); }
            double covariance = s.powerVelocity - s.power * s.velocity;
            double error = (s.acceleration + drag0 + dragSlope * s.velocity - gainSlope * covariance) / gain - s.power;
            squared += error * error; peak = Math.max(peak, Math.abs(error));
            int band = Math.min(2, (int) (3 * s.velocity / supportedSpeed));
            count[band]++; bandSquared[band] += error * error;
        }
        for (int band = 0; band < 3; band++) {
            if (count[band] < 2 || Math.sqrt(bandSquared[band] / count[band]) > .05) {
                throw new IllegalStateException("Validation failed in speed band " + (band + 1));
            }
        }
        if (supportedSamples < 12) { throw new IllegalStateException("Too few validation samples inside the acquired range"); }
        double rms = Math.sqrt(squared / supportedSamples);
        if (rms > .04 || peak > .10) { throw new IllegalStateException("Held-out power prediction failed"); }
        return new MotorPowerModel(gain0, gainSlope, drag0, dragSlope,
                .95 * Math.min(scale, validationMax), rms);
    }
}
