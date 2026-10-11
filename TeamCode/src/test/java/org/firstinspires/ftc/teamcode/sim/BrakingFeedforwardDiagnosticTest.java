package org.firstinspires.ftc.teamcode.sim;

import trajectory.generators.PathProfileGenerator;

import feedforward.MotionCharacterization;
import follower.FollowerConstants;
import geometry.AngleUnit;
import geometry.DistUnit;
import geometry.GeometryFactory;
import org.codeblooded.ftcodesim.ascope.boundaries.MotionVector;
import org.codeblooded.ftcodesim.hardware.devices.SimMotor;
import org.junit.Test;
import org.knowm.xchart.BitmapEncoder;
import org.knowm.xchart.XYChart;
import org.knowm.xchart.XYChartBuilder;
import org.knowm.xchart.style.Styler;
import paths.heading.InterpolationStyle;
import paths.movements.Path;
import trajectory.profile.FFLut;
import trajectory.profile.MotionParameters;

import java.io.File;
import java.io.PrintWriter;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import static org.junit.Assert.assertTrue;

/** Input-only experiment: fit signed braking from stock plant measurements, then test held-out states and paths. */
public class BrakingFeedforwardDiagnosticTest {
    private static final double DT = .02;

    @Test public void measuredSignedBrakingImprovesMotionWithoutFeedback() throws Exception {
        File directory = new File("build/path-power-graphs/braking-diagnostic");
        Files.createDirectories(directory.toPath());
        // Fit separate authority for powers with and against travel. No plant coefficients are read or changed.
        double[][] normal = new double[8][9];
        try (PrintWriter csv = new PrintWriter(new File(directory, "braking-probes.csv"))) {
            csv.println("role,direction,speed_in_s,power,acceleration_in_s2");
            for (double speed : new double[]{8, 18, 32, 46, 51}) {
                for (double power : new double[]{.03, .12, .28, -.03, -.12, -.28}) {
                    double a = probe(speed, power, 1);
                    csv.printf(Locale.US, "training,1,%.9f,%.9f,%.9f%n", speed, power, a);
                    double[] x = features(speed, power);
                    for (int i = 0; i < 8; i++) {
                        for (int j = 0; j < 8; j++) { normal[i][j] += x[i] * x[j]; }
                        normal[i][8] += x[i] * a;
                    }
                }
            }
            double[] model = solve(normal);
            double maximumValidationError = 0;
            for (double direction : new double[]{1, -1}) {
                for (double speed : new double[]{12, 24, 39, 49}) {
                    for (double power : new double[]{.07, .21, -.07, -.21}) {
                        double a = probe(speed, power, direction);
                        csv.printf(Locale.US, "validation,%.0f,%.9f,%.9f,%.9f%n", direction, speed, power, a);
                        double predicted = 0;
                        double[] x = features(speed, power);
                        for (int i = 0; i < x.length; i++) { predicted += x[i] * model[i]; }
                        maximumValidationError = Math.max(maximumValidationError, Math.abs(predicted - a));
                    }
                }
            }
            System.out.printf(Locale.US, "BRAKING MODEL: heldOutMaxAccelError=%.9f coefficients=%s%n",
                    maximumValidationError, java.util.Arrays.toString(model));
            assertTrue("Held-out signed acceleration probes", maximumValidationError < .1);
            for (double direction : new double[]{1, -1}) {
                Run baseline = run(directory, 0, model, direction);
                Run previous = run(directory, 1, model, direction);
                Run corrected = run(directory, 2, model, direction);
                System.out.printf(Locale.US, "BRAKING COMPARISON: direction=%.0f baselineRMSE=%.6f correctedRMSE=%.6f " +
                                "baselinePeak=%.6f correctedPeak=%.6f%n", direction,
                        baseline.rmse(), corrected.rmse(), baseline.peak, corrected.peak);
                System.out.printf(Locale.US, "BRAKING CENTERING: direction=%.0f previousRMSE=%.6f centeredRMSE=%.6f " +
                                "previousBias=%.6f centeredBias=%.6f centeredPeak=%.6f%n", direction,
                        previous.rmse(), corrected.rmse(), previous.bias(), corrected.bias(), corrected.peak);
                assertTrue("Measured braking must reduce error in both travel directions",
                        corrected.rmse() < .5 * baseline.rmse());
                assertTrue("Centered braking must reduce RMSE versus the previous measured model",
                        corrected.rmse() < .3 * previous.rmse());
                assertTrue("Braking must track around the target without a substantial one-sided bias",
                        Math.abs(corrected.bias()) < .6);
                XYChart chart = new XYChartBuilder().width(1000).height(460)
                        .title("Braking Feedforward Experiment (" + (direction > 0 ? "forward" : "reverse") + ")")
                        .xAxisTitle("Actual distance along line (in)").yAxisTitle("Speed (in/s)").build();
                chart.getStyler().setLegendPosition(Styler.LegendPosition.OutsideE);
                chart.getStyler().setMarkerSize(0);
                chart.addSeries("Profile target", corrected.distance, corrected.target);
                chart.addSeries("Current braking model", baseline.distance, baseline.actual);
                chart.addSeries("Previous measured braking", previous.distance, previous.actual);
                chart.addSeries("Centered measured braking", corrected.distance, corrected.actual);
                BitmapEncoder.saveBitmap(chart, new File(directory, direction > 0 ? "Forward_Braking_Velocity" : "Reverse_Braking_Velocity")
                        .getAbsolutePath(), BitmapEncoder.BitmapFormat.PNG);
                chart.getStyler().setXAxisMin(62.0);
                chart.getStyler().setXAxisMax(80.0);
                BitmapEncoder.saveBitmap(chart, new File(directory, direction > 0 ? "Forward_Braking_Velocity_Detail" : "Reverse_Braking_Velocity_Detail")
                        .getAbsolutePath(), BitmapEncoder.BitmapFormat.PNG);
                writePowerGraphs(directory, corrected, direction);
            }
        }
    }

    private static double[] features(double speed, double power) {
        double positive = Math.max(0, power), negative = Math.min(0, power);
        return new double[]{positive, positive * speed, negative, negative * speed,
                power > 0 ? 1 : 0, power > 0 ? speed : 0, power < 0 ? 1 : 0, power < 0 ? speed : 0};
    }

    /** Reacquire the same stock-plant forward-axis calibration for a real follower run. */
    static double[] measuredForwardCoefficients() {
        double[][] normal = new double[8][9];
        for (double speed : new double[]{8, 18, 32, 46, 51}) {
            for (double power : new double[]{.03, .12, .28, -.03, -.12, -.28}) {
                double a = probe(speed, power, 1);
                double[] x = features(speed, power);
                for (int i = 0; i < 8; i++) {
                    for (int j = 0; j < 8; j++) { normal[i][j] += x[i] * x[j]; }
                    normal[i][8] += x[i] * a;
                }
            }
        }
        return solve(normal);
    }

    private static double probe(double speed, double power, double direction) {
        ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
        StockSimTestSupport.setSpeed(hardware, direction * speed);
        for (SimMotor motor : StockSimTestSupport.motors(hardware)) { motor.setPower(direction * power); }
        double dt = .0001;
        StockSimTestSupport.step(hardware, dt);
        return direction * (hardware.drivetrain.velocity.x - direction * speed) / dt;
    }

    private static Run run(File directory, int mode, double[] model, double direction) throws Exception {
        FollowerConstants constants = SimulatorTuning.snapshot();
        GeometryFactory geometry = new GeometryFactory().setDistUnit(DistUnit.IN).setAngleUnit(AngleUnit.DEG);
        Path path = geometry.holonomicPath(geometry.pose(-40, 0, 0), geometry.pose(40, 0, 0))
                .interpolateWith(InterpolationStyle.CONSTANT_START_HEADING).quickBuild();
        FFLut profile = new FFLut(new PathProfileGenerator(constants, path, drivetrains.models.DriveModels.mecanum(constants)).generate());
        ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
        hardware.drivetrain.setPosition(new MotionVector(ApexSimulation.FIELD_CENTER_INCHES - direction * 40,
                ApexSimulation.FIELD_CENTER_INCHES, 0));
        SimMotor[] motors = StockSimTestSupport.motors(hardware);
        Run result = new Run();
        // Round duty with a small phase lead for the held command. Starting at
        // zero always rounds down, systematically delaying braking by up to a tick.
        double reverseDutyDebt = mode == 2 ? .6 : 0;
        String name = (direction > 0 ? "forward" : "reverse")
                + (mode == 2 ? "-centered" : mode == 1 ? "-measured" : "-baseline");
        try (PrintWriter csv = new PrintWriter(new File(directory, name + ".csv"))) {
            csv.println("elapsed_s,path_distance_in,target_velocity_in_s,actual_velocity_in_s,target_accel_in_s2,power," +
                    "predicted_mean_power,predicted_pulse_power,actual_signed_power,expected_motor_power,command_power");
            for (int tick = 0; tick < 400; tick++) {
                double distance = direction * (hardware.drivetrain.position.x - ApexSimulation.FIELD_CENTER_INCHES) + 40;
                if (distance >= 80) { break; }
                MotionParameters target = profile.getKinematicParams(Math.max(0, distance));
                double v = target.getTangentialVel(), a = target.getTangentialAccel();
                double actual = direction * hardware.drivetrain.velocity.x;
                double power, predictedMean;
                // Small reference lead compensates the held-command delay. Taper
                // it near zero speed so the preview cannot stop the plant short.
                double lead = mode == 2 ? DT / 8 * Math.min(1, v / 12) : 0;
                MotionParameters preview = lead > 0
                        ? profile.getKinematicParams(Math.min(80, distance + v * lead)) : target;
                double feedforwardAccel = preview.getTangentialAccel();
                double modelVelocity = Math.max(0, v + feedforwardAccel * lead);
                if (mode != 0 && feedforwardAccel < -1e-6 && modelVelocity > 1e-6) {
                    // Invert the independently measured signed-power branches at the reference state.
                    double forwardGain = model[0] + model[1] * modelVelocity, reverseGain = model[2] + model[3] * modelVelocity;
                    double forwardOffset = model[4] + model[5] * modelVelocity, reverseOffset = model[6] + model[7] * modelVelocity;
                    double forwardPower = (feedforwardAccel - forwardOffset) / forwardGain;
                    double reversePower = (feedforwardAccel - reverseOffset) / reverseGain;
                    if (forwardPower >= .02) { power = predictedMean = forwardPower; }
                    else if (reversePower <= -.02) { power = predictedMean = reversePower; }
                    else {
                        // The plant has a discontinuity at zero/reverse power. Choose a
                        // feedforward duty cycle between two measured regimes at 50 Hz.
                        double forwardAccel = forwardOffset + forwardGain * .02;
                        double reverseAccel = reverseOffset - reverseGain * .02;
                        double duty = Math.max(0, Math.min(1, (forwardAccel - feedforwardAccel) / (forwardAccel - reverseAccel)));
                        predictedMean = (1 - duty) * .02 - duty * .02;
                        reverseDutyDebt += duty;
                        if (reverseDutyDebt >= 1) { power = -.02; reverseDutyDebt -= 1; }
                        else { power = .02; }
                    }
                } else {
                    double legacy = constants.translationalFeedforwardKS + constants.translationalKV * v + constants.translationalKA * a;
                    power = predictedMean = direction * constants.characterizedPower(MotionCharacterization.Axis.FORWARD,
                            direction * v, direction * a, direction * legacy);
                }
                double requestedPulse = power;
                power = Math.max(-1, Math.min(1, requestedPulse));
                for (SimMotor motor : motors) { motor.setPower(direction * power); }
                double applied = 0, peakCommand = 0;
                for (SimMotor motor : motors) {
                    applied += direction * motor.getPower() / motors.length;
                    peakCommand = Math.max(peakCommand, Math.abs(motor.getPower()));
                }
                csv.printf(Locale.US, "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f%n",
                        tick * DT, distance, v, actual, a, power, predictedMean, requestedPulse, applied, Math.abs(requestedPulse), peakCommand);
                result.distance.add(distance); result.target.add(v); result.actual.add(actual);
                result.predictedMean.add(predictedMean); result.applied.add(applied);
                result.expectedPeak.add(Math.abs(requestedPulse)); result.appliedPeak.add(peakCommand);
                assertTrue("The predicted pulse must fit the available motor power", Math.abs(requestedPulse) <= 1);
                assertTrue("Applied motor power must match the predicted pulse",
                        Math.abs(applied - requestedPulse) < 1e-12);
                if (a < -5) {
                    result.squared += Math.pow(actual - v, 2); result.samples++;
                    result.sum += actual - v;
                    result.peak = Math.max(result.peak, Math.abs(actual - v));
                }
                for (int i = 0; i < 4; i++) { StockSimTestSupport.step(hardware, .005); }
            }
        }
        assertTrue("Expected braking samples", result.samples > 10);
        double finalDistance = direction * (hardware.drivetrain.position.x - ApexSimulation.FIELD_CENTER_INCHES) + 40;
        assertTrue("Feedforward must traverse the full line without stopping short: " + finalDistance,
                finalDistance >= 80);
        return result;
    }

    private static void writePowerGraphs(File directory, Run run, double direction) throws Exception {
        String prefix = direction > 0 ? "Forward" : "Reverse";
        XYChart peak = chart("Centered Braking (" + prefix + "): Expected vs Applied Peak Motor Power", "Normalized peak motor power");
        peak.addSeries("Expected pulse peak", run.distance, run.expectedPeak);
        peak.addSeries("Applied motor peak", run.distance, run.appliedPeak);
        BitmapEncoder.saveBitmap(peak, new File(directory, prefix + "_Braking_Power").getAbsolutePath(), BitmapEncoder.BitmapFormat.PNG);
        XYChart signed = chart("Centered Braking (" + prefix + "): Predicted vs Applied Signed Power", "Signed power in travel direction");
        signed.addSeries("Predicted mean", run.distance, run.predictedMean);
        signed.addSeries("Applied pulse", run.distance, run.applied);
        signed.addSeries("Applied 100 ms mean", run.distance, rollingMean(run.applied));
        signed.addSeries("Predicted 100 ms mean", run.distance, rollingMean(run.predictedMean));
        BitmapEncoder.saveBitmap(signed, new File(directory, prefix + "_Braking_Signed_Power").getAbsolutePath(), BitmapEncoder.BitmapFormat.PNG);
        signed.getStyler().setXAxisMin(62.0);
        signed.getStyler().setXAxisMax(80.0);
        BitmapEncoder.saveBitmap(signed, new File(directory, prefix + "_Braking_Signed_Power_Detail").getAbsolutePath(), BitmapEncoder.BitmapFormat.PNG);
        double squared = 0;
        for (int i = 0; i < run.appliedPeak.size(); i++) {
            squared += Math.pow(run.appliedPeak.get(i) - run.expectedPeak.get(i), 2);
        }
        System.out.printf(Locale.US, "CENTERED POWER PREDICTION: direction=%.0f peakCommandRMSE=%.9f samples=%d%n",
                direction, Math.sqrt(squared / run.appliedPeak.size()), run.appliedPeak.size());
    }

    private static XYChart chart(String title, String yTitle) {
        XYChart chart = new XYChartBuilder().width(1000).height(460).title(title)
                .xAxisTitle("Actual distance along line (in)").yAxisTitle(yTitle).build();
        chart.getStyler().setLegendPosition(Styler.LegendPosition.OutsideE);
        chart.getStyler().setMarkerSize(0);
        return chart;
    }

    private static List<Double> rollingMean(List<Double> data) {
        List<Double> result = new ArrayList<>();
        for (int i = 0; i < data.size(); i++) {
            double sum = 0;
            int start = Math.max(0, i - 4);
            for (int j = start; j <= i; j++) { sum += data.get(j); }
            result.add(sum / (i - start + 1));
        }
        return result;
    }

    private static double[] solve(double[][] matrix) {
        int n = matrix.length;
        for (int i = 0; i < n; i++) {
            int pivot = i;
            for (int j = i + 1; j < n; j++) { if (Math.abs(matrix[j][i]) > Math.abs(matrix[pivot][i])) { pivot = j; } }
            double[] swap = matrix[i]; matrix[i] = matrix[pivot]; matrix[pivot] = swap;
            if (Math.abs(matrix[i][i]) < 1e-9) { throw new IllegalStateException("Unidentifiable braking response"); }
            double scale = matrix[i][i];
            for (int j = i; j <= n; j++) { matrix[i][j] /= scale; }
            for (int row = 0; row < n; row++) {
                if (row == i) { continue; }
                double factor = matrix[row][i];
                for (int j = i; j <= n; j++) { matrix[row][j] -= factor * matrix[i][j]; }
            }
        }
        double[] result = new double[n];
        for (int i = 0; i < n; i++) { result[i] = matrix[i][n]; }
        return result;
    }

    private static final class Run {
        final List<Double> distance = new ArrayList<>(), target = new ArrayList<>(), actual = new ArrayList<>();
        final List<Double> predictedMean = new ArrayList<>(), applied = new ArrayList<>();
        final List<Double> expectedPeak = new ArrayList<>(), appliedPeak = new ArrayList<>();
        double squared, peak, sum;
        int samples;
        double rmse() { return Math.sqrt(squared / samples); }
        double bias() { return sum / samples; }
    }
}
