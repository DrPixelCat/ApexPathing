package setup.follower.characterization;

import java.util.ArrayList;
import java.util.List;
import feedforward.MotorPowerModel;

/** Fits measured acquisition data. Independent prediction validation belongs in test programs. */
public final class MotorAuthorityRegression {
    private static final class Sample {
        final double velocity, acceleration, power, powerVelocity;
        Sample(double velocity, double acceleration, double power, double powerVelocity) {
            this.velocity = velocity; this.acceleration = acceleration; this.power = power;
            this.powerVelocity = powerVelocity;
        }
    }
    private final List<Sample> training = new ArrayList<>();
    public void add(double velocity, double acceleration, double power) {
        add(velocity, acceleration, power, power * velocity);
    }
    public void add(double velocity, double acceleration, double power, double powerVelocity) {
        if (Double.isFinite(velocity) && Double.isFinite(acceleration) && Double.isFinite(power)
                && Double.isFinite(powerVelocity) && velocity > 0 && power > .05 && power <= 1.001) {
            training.add(new Sample(velocity, acceleration, power, powerVelocity));
        }
    }
    public MotorPowerModel fit() {
        if (training.size() < 20) { throw new IllegalStateException("Too few moving measurements"); }
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
        double squared = 0;
        for (Sample s : training) {
            double gain = gain0 + gainSlope * s.velocity;
            if (gain <= 0) { throw new IllegalStateException("Nonpositive measured acceleration authority"); }
            double covariance = s.powerVelocity - s.power * s.velocity;
            double error = (s.acceleration + drag0 + dragSlope * s.velocity - gainSlope * covariance) / gain - s.power;
            squared += error * error;
        }
        // The legacy JSON field validationRms now records acquisition residual only.
        return new MotorPowerModel(gain0, gainSlope, drag0, dragSlope,
                .95 * scale, Math.sqrt(squared / training.size()));
    }
}
