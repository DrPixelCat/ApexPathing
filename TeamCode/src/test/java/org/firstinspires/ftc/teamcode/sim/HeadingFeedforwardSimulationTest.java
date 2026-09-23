package org.firstinspires.ftc.teamcode.sim;

import controllers.FeedbackController;
import core.ApexStorage;
import drivetrains.BaseDrivetrain;
import org.firstinspires.ftc.teamcode.apexpathing.Constants;
import org.firstinspires.ftc.teamcode.apexpathing.FollowerTuner;
import org.junit.Test;
import org.knowm.xchart.*;
import org.knowm.xchart.style.Styler;

import java.io.*;
import java.nio.file.*;
import java.util.*;

import static org.junit.Assert.assertTrue;

/** Open-loop gain acceptance: no heading feedback, translation, allocator or wall-clock pacing. */
public class HeadingFeedforwardSimulationTest {
    private static final double CONTROL_DT = 0.020;
    private static final double PHYSICS_DT = 0.005;
    // Explicit engineering acceptance limits, not values fitted to make current gains pass.
    private static final double MAX_RATE_RMSE = 0.50; // rad/s, independently for each phase
    private static final double MAX_STOP_RATE = 0.25; // rad/s, 200 ms after target stops

    @Test public void headingFeedforwardTracksBothDirectionsAndBrakes() throws Exception {
        core.FollowerConstants tuning = SimulatorTuning.snapshot();
        double kS = gain("APEX_HEADING_FF_KS", tuning.angularFeedforwardKS);
        double kV = gain("APEX_HEADING_FF_KV", tuning.angularKV);
        double kA = gain("APEX_HEADING_FF_KA", tuning.angularKA);
        Path directory = Paths.get(System.getProperty("user.dir"), "build", "heading-feedforward");
        Files.createDirectories(directory);
        List<String> failures = new ArrayList<>();
        String oldStorage = System.getProperty(ApexStorage.DIRECTORY_PROPERTY);
        String oldUnlock = System.getProperty(FollowerTuner.UNLOCK_PHASES_PROPERTY);
        try (PrintWriter summary = new PrintWriter(Files.newBufferedWriter(directory.resolve("summary.txt")))) {
            summary.printf(Locale.US, "kS=%.12f kV=%.12f kA=%.12f%n", kS, kV, kA);
            summary.println("Fixed 20 ms control / 5 ms physics. No feedback. Rates in rad/s.");
            summary.println("Acceptance: each phase RMSE <= 0.50; residual rate at stop+0.20s <= 0.25; no clipping.");
            for (double direction : new double[] {1, -1}) {
                for (double peak : new double[] {1, 2, 3}) {
                    for (double acceleration : new double[] {4, 8}) {
                        String name = (direction > 0 ? "ccw" : "cw") + "-v" + (int) peak + "-a" + (int) acceleration;
                        runCase(directory, name, direction, peak, acceleration, kS, kV, kA, summary, failures);
                    }
                }
            }
            summary.println(failures.isEmpty() ? "PASS" : "FAIL: " + String.join("; ", failures));
        } finally {
            restore(ApexStorage.DIRECTORY_PROPERTY, oldStorage);
            restore(FollowerTuner.UNLOCK_PHASES_PROPERTY, oldUnlock);
        }
        System.out.println("Heading feedforward report: " + directory.toAbsolutePath());
        assertTrue("Heading feedforward acceptance failed. See " + directory.resolve("summary.txt")
                + ": " + String.join("; ", failures), failures.isEmpty());
    }

    private static void runCase(Path directory, String name, double direction, double peak,
                                double acceleration, double kS, double kV, double kA,
                                PrintWriter summary, List<String> failures) throws Exception {
        ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
        BaseDrivetrain<?> drive = new Constants().drivetrainConstants().build(hardware.hardwareMap);
        FeedbackController ff = new FeedbackController(new FeedbackController.Coefficients().vel(kV).accel(kA));
        double ramp = peak / acceleration, cruiseEnd = ramp + 0.6, stop = cruiseEnd + ramp;
        double[] squared = new double[3];
        int[] counts = new int[3];
        double omega = 0, heading = 0, targetHeading = 0, stopRate = Double.NaN;
        int clipped = 0;
        List<Double> times = new ArrayList<>(), targets = new ArrayList<>(), actuals = new ArrayList<>();
        try (PrintWriter csv = new PrintWriter(Files.newBufferedWriter(directory.resolve(name + ".csv")))) {
            csv.println("time_s,phase,target_omega_rad_s,actual_omega_rad_s,target_alpha_rad_s2,actual_alpha_rad_s2,requested_power,applied_power,target_heading_rad,actual_heading_rad");
            for (int tick = 0; tick * CONTROL_DT < stop + 0.6 - 1e-9; tick++) {
                double t = tick * CONTROL_DT;
                int phase = t < ramp - 1e-9 ? 0 : t < cruiseEnd - 1e-9 ? 1 : t < stop - 1e-9 ? 2 : 3;
                double target = targetRate(t, direction, peak, ramp, cruiseEnd, stop);
                double alpha = direction * (phase == 0 ? acceleration : phase == 2 ? -acceleration : 0);
                // Match HolonomicPathFollower's angular feedforward formula at full time scale.
                double power = ff.vel(target).accel(alpha)
                        .f(Math.abs(target) > 1e-6 ? Math.copySign(kS, target) : 0).calculate(0).get();
                if (Math.abs(power) > 1) clipped++;
                double applied = Math.max(-1, Math.min(1, power));
                drive.moveWithVectors(0, 0, applied);
                double previousOmega = omega;
                for (int sub = 0; sub < 4; sub++) {
                    ApexSimulation.stepPhysics(hardware, PHYSICS_DT);
                }
                omega = hardware.drivetrain.velocity.theta;
                heading = hardware.drivetrain.position.theta;
                double end = t + CONTROL_DT;
                double endTarget = targetRate(end, direction, peak, ramp, cruiseEnd, stop);
                targetHeading += (target + endTarget) * 0.5 * CONTROL_DT;
                if (phase < 3) {
                    squared[phase] += Math.pow(omega - endTarget, 2);
                    counts[phase]++;
                }
                if (Double.isNaN(stopRate) && end >= stop + 0.2 - 1e-9) stopRate = Math.abs(omega);
                times.add(end); targets.add(endTarget); actuals.add(omega);
                csv.printf(Locale.US, "%.6f,%s,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f%n",
                        end, new String[] {"accelerate", "cruise", "brake", "stopped"}[phase],
                        endTarget, omega, alpha, (omega - previousOmega) / CONTROL_DT,
                        power, applied, targetHeading, heading);
            }
        } finally { drive.moveWithVectors(0, 0, 0); }
        String[] phases = {"acceleration", "cruise", "braking"};
        summary.print(name);
        for (int i = 0; i < 3; i++) {
            double rmse = Math.sqrt(squared[i] / counts[i]);
            summary.printf(Locale.US, " %s_RMSE=%.4f", phases[i], rmse);
            if (!Double.isFinite(rmse) || rmse > MAX_RATE_RMSE) failures.add(name + " " + phases[i]);
        }
        summary.printf(Locale.US, " stop_rate=%.4f final_heading_error=%.4f clipped=%d%n", stopRate, targetHeading-heading, clipped);
        if (!Double.isFinite(stopRate) || stopRate > MAX_STOP_RATE) failures.add(name + " stop");
        if (clipped != 0) failures.add(name + " clipping");
        XYChart chart = new XYChartBuilder().width(1000).height(460).title(name + " heading feedforward only")
                .xAxisTitle("Time (s)").yAxisTitle("Angular velocity (rad/s)").build();
        chart.getStyler().setLegendPosition(Styler.LegendPosition.OutsideE);
        chart.getStyler().setMarkerSize(0);
        chart.addSeries("Target", times, targets); chart.addSeries("Actual", times, actuals);
        BitmapEncoder.saveBitmap(chart, directory.resolve(name).toString(), BitmapEncoder.BitmapFormat.PNG);
    }

    private static double targetRate(double t, double direction, double peak, double ramp, double cruiseEnd, double stop) {
        return direction * (t < ramp ? peak * t / ramp : t < cruiseEnd ? peak : t < stop ? peak * (stop-t)/ramp : 0);
    }
    private static double gain(String key, double fallback) {
        String value = System.getenv(key);
        double gain = value == null ? fallback : Double.parseDouble(value);
        if (!Double.isFinite(gain) || gain < 0) throw new IllegalArgumentException(key + " must be finite and nonnegative");
        return gain;
    }
    private static void restore(String key, String value) {
        if (value == null) System.clearProperty(key); else System.setProperty(key, value);
    }
}
