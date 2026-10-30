package setup.follower.characterization;

import java.util.ArrayList;
import java.util.List;
import feedforward.MotorPowerModel;

/** Test-only held-out prediction gates; no validation motions or gates run on the robot. */
public final class ValidatedMotorAuthorityRegression {
    public static final class Sample {
        public final double velocity, acceleration, power, powerVelocity;
        public Sample(double velocity, double acceleration, double power, double powerVelocity) {
            this.velocity = velocity; this.acceleration = acceleration; this.power = power;
            this.powerVelocity = powerVelocity;
        }
    }
    private final MotorAuthorityRegression acquisition = new MotorAuthorityRegression();
    private final List<Sample> training = new ArrayList<>(), validation = new ArrayList<>();
    boolean validationCovered() {
        double acquired = training.stream().mapToDouble(s -> s.velocity).max().orElse(0);
        double validationMax = validation.stream().mapToDouble(s -> s.velocity).max().orElse(0);
        // Repeat held-out acquisition when a dropped frame leaves its high-speed range
        // below the same coverage requirement enforced by fit().
        if (validationMax < .85 * acquired) { return false; }
        double speed = Math.min(acquired, validationMax);
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
            if (!heldOut) { acquisition.add(velocity, acceleration, power, powerVelocity); }
        }
    }
    public MotorPowerModel fit() {
        MotorPowerModel acquired = acquisition.fit();
        double scale = training.stream().mapToDouble(s -> s.velocity).max().orElse(0);
        double gain0 = acquired.gain0, gainSlope = acquired.gainSlope;
        double drag0 = acquired.drag0, dragSlope = acquired.dragSlope;
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
