package org.firstinspires.ftc.teamcode.sim;

import follower.Follower;
import follower.FollowerConstants;
import geometry.AngleUnit;
import geometry.Pose;
import localizers.Pinpoint;
import org.firstinspires.ftc.teamcode.apexpathing.Constants;
import org.junit.Test;
import storage.ApexStorage;
import tuning.follower.characterization.ValidatedMotorAuthorityRegression;

import java.lang.reflect.Field;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicLong;

import static org.junit.Assert.assertEquals;

/** Matched angular excitation distinguishes plant response, filtering, and window arithmetic. */
public class AngularCharacterizationMeasurementTest {
    private static final double DT = .02;

    @Test public void compareNativeAndFilteredWindowsOnIdenticalExcitation() throws Exception {
        compare(false, false);
    }

    @Test public void compareAfterAngularFilterHasAdaptedDuringEarlierPhases() throws Exception {
        compare(true, false);
    }

    @Test public void compareAtRecordedFailingWorkflowCadence() throws Exception {
        compare(true, true);
    }

    private void compare(boolean warmFilter, boolean recordedCadence) throws Exception {
        String directory = System.getProperty(ApexStorage.DIRECTORY_PROPERTY);
        Field singleton = FollowerConstants.class.getDeclaredField("instance");
        singleton.setAccessible(true);
        Object original = singleton.get(null);
        Path output = Paths.get("build/angular-characterization-measurements/"
                + (recordedCadence ? "recorded-cadence" : warmFilter ? "after-idle" : "fresh")).toAbsolutePath();
        Files.createDirectories(output);
        List<String> trace = new ArrayList<>(), windows = new ArrayList<>();
        trace.add("direction,run,time_s,power,native_velocity,filtered_velocity,physical_acceleration");
        windows.add("direction,run,held_out,time_s,power,native_velocity,filtered_velocity,native_endpoint_acceleration,native_ols_acceleration,filtered_endpoint_acceleration,filtered_ols_acceleration,native_power_velocity,filtered_power_velocity");
        try {
            System.setProperty(ApexStorage.DIRECTORY_PROPERTY,
                    Files.createTempDirectory("angular-characterization-").toString());
            singleton.set(null, null);
            for (int direction : new int[] {1, -1}) {
                ValidatedMotorAuthorityRegression[] variants = new ValidatedMotorAuthorityRegression[4];
                for (int i = 0; i < variants.length; i++) variants[i] = new ValidatedMotorAuthorityRegression();
                ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
                AtomicLong clock = new AtomicLong(1_000_000_000L);
                ((SimApexPinpoint) hardware.hardwareMap.get(Pinpoint.Driver.class,
                        ApexSimulation.PINPOINT)).setClock(clock::get);
                Follower follower = new Follower(new Constants(), hardware.hardwareMap, true, clock::get);
                follower.setPose(Pose.zero()); follower.disableControllers(); follower.update(false);
                if (warmFilter) {
                    // The angular estimator sees little motion during forward/strafe acquisition.
                    for (int tick = 0; tick < 2000; tick++) {
                        clock.addAndGet(20_000_000L); follower.update(false);
                    }
                }
                System.out.println("warm=" + warmFilter + " angular Q="
                        + follower.getLocalizer().getHeadingKalmanTuning().processVariance);
                for (int run = 0; run < 6; run++) {
                    hardware.drivetrain.velocity = new org.codeblooded.ftcodesim.ascope.boundaries.MotionVector(0, 0, 0);
                    hardware.drivetrain.updateWheelRollVelocities();
                    clock.addAndGet(20_000_000L);
                    follower.setPose(Pose.zero()); follower.disableControllers(); follower.update(false);
                    double previousNative = 0, previousFiltered = 0, previousPower = 0;
                    Window window = new Window(0, 0);
                    long runStarted = clock.get();
                    double previousTime = 0;
                    for (double time : times(run, recordedCadence)) {
                        double dt = time - previousTime, remaining = dt;
                        while (remaining > 1e-9) {
                            double step = Math.min(.005, remaining);
                            ApexSimulation.stepPhysics(hardware, step); remaining -= step;
                        }
                        clock.set(runStarted + Math.round(time * 1e9)); follower.update(false);
                        double nativeV = direction * follower.getLocalizer().getRawVel().getHeading(AngleUnit.RAD);
                        double filteredV = direction * follower.getVelocity().getHeading(AngleUnit.RAD);
                        trace.add(direction + "," + run + "," + time + "," + previousPower + ","
                                + nativeV + "," + filteredV + "," + direction * hardware.drivetrain.acceleration.theta);
                        // A common native movement gate gives all variants exactly the same windows.
                        if (nativeV > .1 && previousNative > .1) {
                            window.add(previousNative, nativeV, previousFiltered, filteredV, previousPower, dt);
                            if (window.time >= .15) {
                                double nv = window.nativeArea / window.time, fv = window.filteredArea / window.time;
                                double ne = (nativeV - window.startNative) / window.time;
                                double fe = (filteredV - window.startFiltered) / window.time;
                                double no = window.slope(window.nativeSum, window.nativeTimeSum);
                                double fo = window.slope(window.filteredSum, window.filteredTimeSum);
                                double p = window.powerArea / window.time;
                                double npv = window.nativePowerArea / window.time, fpv = window.filteredPowerArea / window.time;
                                variants[0].add(run >= 4, nv, ne, p, npv);
                                variants[1].add(run >= 4, nv, no, p, npv);
                                variants[2].add(run >= 4, fv, fe, p, fpv);
                                variants[3].add(run >= 4, fv, fo, p, fpv);
                                windows.add(direction + "," + run + "," + (run >= 4) + "," + time + "," + p
                                        + "," + nv + "," + fv + "," + ne + "," + no + "," + fe + "," + fo + "," + npv + "," + fpv);
                                window = new Window(nativeV, filteredV);
                            }
                        } else window = new Window(nativeV, filteredV);
                        double power = run < 2 ? .65 : run < 4 ? 1 : time < .75 ? .55 : .95;
                        follower.getDrivetrain().moveWithVectors(0, 0, direction * power);
                        if (Math.abs(power - previousPower) > .02) window = new Window(nativeV, filteredV);
                        previousNative = nativeV; previousFiltered = filteredV; previousPower = power;
                        previousTime = time;
                    }
                    follower.stop();
                }
                String[] names = {"native endpoint", "native OLS", "filtered endpoint", "filtered OLS"};
                for (int i = 0; i < variants.length; i++) {
                    try {
                        feedforward.MotorPowerModel fit = variants[i].fit();
                        System.out.println(direction + " " + names[i] + " PASS rms=" + fit.validationRms
                                + " coefficients=" + fit.gain0 + "," + fit.gainSlope + "," + fit.drag0 + "," + fit.dragSlope);
                        // Independent holding probes: a good fit must describe the actual plant,
                        // not just another transient from the same velocity estimator.
                        for (double speed : new double[] {3, 7.5}) {
                            hardware.drivetrain.velocity = new org.codeblooded.ftcodesim.ascope.boundaries.MotionVector(0, 0, direction * speed);
                            hardware.drivetrain.updateWheelRollVelocities();
                            follower.getDrivetrain().moveWithVectors(0, 0, direction * fit.holdingPower(speed));
                            ApexSimulation.stepPhysics(hardware, .0001);
                            double acceleration = (direction * hardware.drivetrain.velocity.theta - speed) / .0001;
                            System.out.println(names[i] + " holding probe at " + speed + " rad/s: " + acceleration + " rad/s^2");
                            if (i == 0) assertEquals("Native fit must hold imposed physical speed", 0, acceleration, .05);
                        }
                        follower.stop();
                    } catch (IllegalStateException | IllegalArgumentException failure) {
                        System.out.println(direction + " " + names[i] + " REJECT " + failure.getMessage());
                        if (i == 0) throw failure;
                    }
                }
            }
        } finally {
            Files.write(output.resolve("samples.csv"), trace);
            Files.write(output.resolve("windows.csv"), windows);
            singleton.set(null, original);
            if (directory == null) System.clearProperty(ApexStorage.DIRECTORY_PROPERTY);
            else System.setProperty(ApexStorage.DIRECTORY_PROPERTY, directory);
        }
    }

    private static List<Double> times(int run, boolean recorded) throws Exception {
        List<Double> times = new ArrayList<>();
        if (!recorded) {
            for (int tick = 1; tick <= 100; tick++) times.add(tick * DT);
        } else {
            // Timings from the rejected October 6 workflow; no recorded velocities are replayed.
            try (java.io.BufferedReader reader = new java.io.BufferedReader(new java.io.InputStreamReader(
                    AngularCharacterizationMeasurementTest.class.getResourceAsStream(
                            "/simulator/angular-characterization-cadence.csv"), java.nio.charset.StandardCharsets.UTF_8))) {
                reader.readLine();
                for (String line; (line = reader.readLine()) != null;) {
                    String[] fields = line.split(",");
                    if (Integer.parseInt(fields[0]) == run) times.add(Double.parseDouble(fields[1]));
                }
            }
        }
        return times;
    }

    private static final class Window {
        final double startNative, startFiltered;
        double time, nativeArea, filteredArea, powerArea, nativePowerArea, filteredPowerArea;
        double timeSum, timeSquared, nativeSum, filteredSum, nativeTimeSum, filteredTimeSum;
        int count = 1;
        Window(double nativeV, double filteredV) {
            startNative = nativeV; startFiltered = filteredV; nativeSum = nativeV; filteredSum = filteredV;
        }
        void add(double pn, double n, double pf, double f, double power, double dt) {
            time += dt;
            nativeArea += .5 * (pn + n) * dt; filteredArea += .5 * (pf + f) * dt;
            powerArea += power * dt;
            nativePowerArea += power * .5 * (pn + n) * dt;
            filteredPowerArea += power * .5 * (pf + f) * dt;
            count++; timeSum += time; timeSquared += time * time;
            nativeSum += n; filteredSum += f; nativeTimeSum += time * n; filteredTimeSum += time * f;
        }
        double slope(double sum, double timeVelocity) {
            return (count * timeVelocity - timeSum * sum) / (count * timeSquared - timeSum * timeSum);
        }
    }
}
