package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.assertEquals;

import com.qualcomm.robotcore.eventloop.opmode.SimLinearOpModeBridge;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.codeblooded.ftcodesim.hardware.devices.SimMotor;
import org.codeblooded.ftcodesim.ascope.boundaries.MotionVector;
import org.firstinspires.ftc.teamcode.apexpathing.AutoTest;
import org.junit.Test;
import org.knowm.xchart.BitmapEncoder;
import org.knowm.xchart.BitmapEncoder.BitmapFormat;
import org.knowm.xchart.XYChart;
import org.knowm.xchart.XYChartBuilder;
import org.knowm.xchart.style.Styler;

import java.lang.reflect.Method;
import java.io.File;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;

import core.FollowerConstants;
import core.ApexStorage;
import controllers.FeedbackController.Coefficients;
import paths.movements.Path;

public class AutoTestSimulationTest {
    private static final double MAX_TOTAL_MOVEMENT_SECONDS = 18.0;

    @Test(timeout = 130_000L)
    public void autoTestCompletesEveryMovement() throws Exception {
        runAutoTest("true".equalsIgnoreCase(System.getenv("APEX_USE_SAVED_TUNER_CONSTANTS")));
    }

    @Test(timeout = 130_000L)
    public void storedAutoTestCompletesEveryMovement() throws Exception { runAutoTest(false, true); }

    Path runAutoTest(boolean useSavedConstants) throws Exception { return runAutoTest(useSavedConstants, false); }

    Path runAutoTest(boolean useSavedConstants, boolean stored) throws Exception {
        configureStableConstants(useSavedConstants);
        ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
        List<String> frames = new CopyOnWriteArrayList<>();
        ApexSimTelemetry telemetry = new ApexSimTelemetry(frames::add);
        telemetry.setMsTransmissionInterval(0);

        AutoTest auto = new AutoTest();
        auto.hardwareMap = hardware.hardwareMap;
        auto.telemetry = telemetry;
        auto.gamepad1 = new Gamepad();
        auto.gamepad2 = new Gamepad();

        SimLinearOpModeBridge.Session session = SimLinearOpModeBridge.initialize(auto, () -> { });
        try {
            pump(session, auto, telemetry, hardware, 100);
            long initializationDeadline = System.nanoTime() + 10_000_000_000L;
            while (auto.getOutboundPath() == null &&
                    System.nanoTime() < initializationDeadline) {
                pump(session, auto, telemetry, hardware, 20);
            }
            Path outbound = auto.getOutboundPath();
            assertTrue("Auto Test did not finish building its outbound path", outbound != null);
            if (stored) {
                java.lang.reflect.Field routeField = AutoTest.class.getDeclaredField("path");
                routeField.setAccessible(true);
                Object route = routeField.get(auto);
                for (java.lang.reflect.Field field : route.getClass().getDeclaredFields()) {
                    field.setAccessible(true);
                    Object movement = field.get(route);
                    if (movement instanceof Path) {
                        ((Path) movement).setBuildMode(Path.BuildMode.MEMORY);
                        ((Path) movement).precomputeTrajectory();
                    } else if (movement instanceof paths.movements.Turn) {
                        ((paths.movements.Turn) movement).setBuildMode(Path.BuildMode.MEMORY);
                        ((paths.movements.Turn) movement).precomputeTrajectory();
                    }
                }
            }
            long routeStartedNanos = System.nanoTime();
            SimLinearOpModeBridge.start(session);
            long deadline = System.nanoTime() + 100_000_000_000L;
            while (!latest(frames).contains("Current check COMPLETE") &&
                    !latest(frames).contains("Current check FAILED")) {
                if (System.nanoTime() >= deadline) { break; }
                pump(session, auto, telemetry, hardware, 20);
            }

            String frame = latest(frames);
            double routeRunSeconds = (System.nanoTime() - routeStartedNanos) * 1e-9;
            File velocityTrace = new File(auto.getOutboundVelocityCsvPath());
            if (velocityTrace.isFile()) {
                writeVelocityTrackingGraph(velocityTrace, useSavedConstants);
            }
            assertFalse("Auto Test failed after its outbound path:\n" + frame,
                    frame.contains("Current check FAILED"));
            assertTrue("Auto Test did not complete every movement:\n" + frame,
                    frame.contains("Current check COMPLETE"));
            System.out.println("AUTO TEST COMMAND DEMAND: " + auto.getCommandDemandReport());
            System.out.println("AUTO TEST TIMING: " + auto.getMovementTimingReport());
            System.out.println("AUTO TEST EXTERNAL RUNTIME: " + routeRunSeconds + " s");
            System.out.println("AUTO TEST LOOP: " + auto.getAverageLoopMilliseconds() + " ms");
            validateVelocityTrace(new File(auto.getOutboundVelocityCsvPath()));
            // The saved corner fixture builds additional diagnostics on the host event loop.
            double maximumLoopMs = useSavedConstants ? 30.0 : 23.0;
            assertTrue("Auto Test loop exceeded fixture timing range: " +
                            auto.getAverageLoopMilliseconds(),
                    auto.getAverageLoopMilliseconds() >= 17.0 &&
                            auto.getAverageLoopMilliseconds() <= maximumLoopMs);
            // Keep the existing saved-calibration and desktop-fixture timing budgets.
            double maximumMovementSeconds = useSavedConstants ? 20.0
                    : MAX_TOTAL_MOVEMENT_SECONDS;
            assertTrue("Auto Test following regressed: external=" + routeRunSeconds +
                            "s, internal=" + auto.getMovementTimingReport(),
                    routeRunSeconds <= maximumMovementSeconds);
            {
                File velocityCsv = new File(auto.getOutboundVelocityCsvPath());
                assertTrue("Outbound trajectory CSV was not created: path=" +
                                auto.getOutboundVelocityCsvPath() + ", error=" +
                                auto.getOutboundVelocityCsvError(),
                        velocityCsv.isFile() && velocityCsv.length() > 0L);
            }
            return outbound;
        } finally {
            SimLinearOpModeBridge.stop(session);
        }
    }

    private static void pump(SimLinearOpModeBridge.Session session, AutoTest auto,
                             ApexSimTelemetry telemetry, ApexSimulation.Hardware hardware,
                             long milliseconds) throws Exception {
        long deadline = System.nanoTime() + milliseconds * 1_000_000L;
        while (System.nanoTime() < deadline) {
            long now = System.nanoTime();
            double remaining = Math.max(0, Math.min(0.05,
                    (now - hardware.lastPhysicsUpdateNanos) * 1e-9));
            hardware.lastPhysicsUpdateNanos = now;
            while (remaining > 1e-9) {
                double dt = Math.min(0.005, remaining);
                stepPhysics(hardware, dt);
                remaining -= dt;
            }
            SimLinearOpModeBridge.eventLoopIteration(session, auto.gamepad1, auto.gamepad2);
            telemetry.update();
            Thread.sleep(5);
        }
    }

    private static void stepPhysics(ApexSimulation.Hardware hardware, double dt) throws Exception {
        ApexSimulation.stepPhysics(hardware, dt);
    }

    private static String latest(List<String> frames) {
        return frames.isEmpty() ? "" : frames.get(frames.size() - 1);
    }

    /** Validate the controller's actual signals, not just whether a CSV file exists. */
    private static void validateVelocityTrace(File csv) throws Exception {
        List<String> lines = Files.readAllLines(csv.toPath());
        String[] headers = lines.get(0).split(",");
        java.util.Map<String, Integer> columns = new java.util.HashMap<>();
        for (int i = 0; i < headers.length; i++) { columns.put(headers[i], i); }
        int acceleratingSamples = 0;
        double squaredError = 0;
        double bias = 0;
        double peak = 0;
        int brakingSamples = 0;
        double brakingSquaredError = 0, brakingBias = 0, brakingPeak = 0;
        double previousElapsed = Double.NaN, previousRawVelocity = Double.NaN;
        double previousTargetVelocity = Double.NaN;
        double previousAcceleration = Double.NaN;
        for (int i = 1; i < lines.size(); i++) {
            String[] row = lines.get(i).split(",");
            double elapsed = Double.parseDouble(row[columns.get("elapsed_s")]);
            double target = Double.parseDouble(row[columns.get("controller_target_in_s")]);
            double plotted = Double.parseDouble(row[columns.get("kalman_velocity_in_s")]);
            if (target == 0) { continue; } // Endpoint has taken over.
            double measured = Double.parseDouble(row[columns.get("controller_measured_in_s")]);
            assertEquals("CSV must show the feedback measurement", plotted, measured, 1e-5);
            double profileTarget = Double.parseDouble(row[columns.get("target_velocity_in_s")]);
            assertEquals("Follower must consume the constrained profile without a speed floor",
                    profileTarget, target, 1e-5);
            double acceleration = Double.parseDouble(row[columns.get("target_accel_in_s2")]);
            double rawVelocity = Double.parseDouble(row[columns.get("raw_velocity_in_s")]);
            double dt = elapsed - previousElapsed;
            if (acceleration < -20.0 && previousAcceleration < -20.0
                    && target < previousTargetVelocity
                    && dt > 0.01 && dt < 0.06) {
                // One sample can include heading allocation and estimator projection.
                // Reject a large rebound while the profile continues to decelerate.
                assertTrue("Speed rebounded during sustained braking at " + elapsed + "s",
                        rawVelocity - previousRawVelocity < 1.5);
            }
            previousElapsed = elapsed;
            previousRawVelocity = rawVelocity;
            previousTargetVelocity = target;
            previousAcceleration = acceleration;
            if (acceleration < -5) {
                double error = measured - target;
                brakingSquaredError += error * error;
                brakingBias += error;
                brakingPeak = Math.max(brakingPeak, error);
                brakingSamples++;
            }
            if (acceleration > 5) {
                double error = measured - target;
                squaredError += error * error;
                bias += error;
                peak = Math.max(peak, error);
                acceleratingSamples++;
            }
        }
        assertTrue("Expected accelerating trace samples", acceleratingSamples > 10);
        assertTrue("Expected braking trace samples", brakingSamples > 10);
        System.out.println("ACCELERATION TRACKING: samples=" + acceleratingSamples +
                " bias=" + bias / acceleratingSamples + " RMSE=" +
                Math.sqrt(squaredError / acceleratingSamples) + " peakOverspeed=" + peak);
        System.out.println("BRAKING TRACKING: samples=" + brakingSamples +
                " bias=" + brakingBias / brakingSamples + " RMSE=" +
                Math.sqrt(brakingSquaredError / brakingSamples) + " peakOverspeed=" + brakingPeak);
    }

    /** Writes the simulated outbound target-versus-measured velocity trace beside profile graphs. */
    private static void writeVelocityTrackingGraph(File trace,
                                                   boolean useSavedConstants) throws Exception {
        List<Double> elapsedSeconds = new ArrayList<>();
        List<Double> targetVelocity = new ArrayList<>();
        List<Double> actualVelocity = new ArrayList<>();
        List<Double> rawVelocity = new ArrayList<>();
        List<String> lines = Files.readAllLines(trace.toPath());
        if (lines.size() < 2) { return; }
        String[] headers = lines.get(0).split(",");
        java.util.Map<String, Integer> columns = new java.util.HashMap<>();
        for (int i = 0; i < headers.length; i++) { columns.put(headers[i], i); }
        for (int i = 1; i < lines.size(); i++) {
            String[] row = lines.get(i).split(",");
            elapsedSeconds.add(Double.parseDouble(row[columns.get("elapsed_s")]));
            targetVelocity.add(Double.parseDouble(row[columns.get("controller_target_in_s")]));
            actualVelocity.add(Double.parseDouble(row[columns.get("kalman_velocity_in_s")]));
            rawVelocity.add(Double.parseDouble(row[columns.get("raw_velocity_in_s")]));
        }
        if (elapsedSeconds.isEmpty()) { return; }
        XYChart chart = new XYChartBuilder().width(1000).height(460)
                .title("AutoTest Outbound Velocity Tracking ("
                        + (useSavedConstants ? "saved profile" : "simulator fixture") + ")")
                .xAxisTitle("Elapsed Time (s)")
                .yAxisTitle("Velocity (in/s)").build();
        chart.getStyler().setLegendPosition(Styler.LegendPosition.OutsideE);
        chart.getStyler().setMarkerSize(0);
        chart.getStyler().setPlotGridLinesVisible(true);
        chart.addSeries("Target velocity", elapsedSeconds, targetVelocity);
        chart.addSeries("Actual velocity (Kalman)", elapsedSeconds, actualVelocity);
        chart.addSeries("Raw velocity", elapsedSeconds, rawVelocity);
        String graphDirectory = System.getenv("APEX_AUTOTEST_GRAPH_DIR");
        File directory = graphDirectory == null
                ? new File(System.getProperty("user.dir")) : new File(graphDirectory);
        if (!directory.exists() && !directory.mkdirs()) {
            throw new IllegalStateException("Could not create graph directory " + directory);
        }
        File output = new File(directory, "AutoTest_Velocity_Tracking");
        BitmapEncoder.saveBitmap(chart, output.getAbsolutePath(), BitmapFormat.PNG);
    }

    private static void configureStableConstants(boolean useSavedConstants) {
        String dataDirectory = System.getenv("APEX_AUTOTEST_DATA_DIR");
        if (dataDirectory != null) {
            System.setProperty(ApexStorage.DIRECTORY_PROPERTY, dataDirectory);
        } else if (System.getProperty(ApexStorage.DIRECTORY_PROPERTY) == null) {
            File directory = new File(System.getProperty("user.dir"), "build/ftcodesim-data");
            System.setProperty(ApexStorage.DIRECTORY_PROPERTY, directory.getAbsolutePath());
        }
        FollowerConstants constants = FollowerConstants.getInstance();
        if (useSavedConstants) {
            constants.reload();
            if ("true".equalsIgnoreCase(System.getenv("APEX_BENCH_FEEDFORWARD"))) {
                constants.translationalFeedforwardKS = 0.189891228443;
                constants.translationalKV = 0.004178027423;
                constants.translationalKA = 0.007254097165;
            }
        } else {
            SimulatorTuning.apply(constants);
        }
        String gain = System.getenv("APEX_TRANSLATION_FEEDBACK_GAIN");
        if (gain != null) { constants.velocityFeedbackGain = Double.parseDouble(gain); }
        String tolerance = System.getenv("APEX_BRAKING_OVERSPEED_TOLERANCE_IN");
        if (tolerance != null) {
            constants.brakingOverspeedToleranceIn = Double.parseDouble(tolerance);
        } else if (!useSavedConstants) { constants.brakingOverspeedToleranceIn = 3.0; }
        String cornerTolerance = System.getenv("APEX_BRAKING_CORNER_TOLERANCE_IN");
        if (cornerTolerance != null) {
            constants.brakingCornerToleranceIn = Double.parseDouble(cornerTolerance);
        }
        String pulsePower = System.getenv("APEX_BRAKING_PULSE_POWER");
        if (pulsePower != null) {
            constants.brakingPulsePower = Double.parseDouble(pulsePower);
        } else if (!useSavedConstants) { constants.brakingPulsePower = 0.30; }
        String pulseCooldown = System.getenv("APEX_BRAKING_PULSE_COOLDOWN_CYCLES");
        if (pulseCooldown != null) {
            constants.brakingPulseCooldownCycles = Integer.parseInt(pulseCooldown);
        } else if (!useSavedConstants) { constants.brakingPulseCooldownCycles = 1; }
        String neutralMargin = System.getenv("APEX_BRAKING_NEUTRAL_MARGIN");
        if (neutralMargin != null) {
            constants.brakingNeutralPowerMargin = Double.parseDouble(neutralMargin);
        } else if (!useSavedConstants) { constants.brakingNeutralPowerMargin = 0.02; }
        String forwardAccelLimit = System.getenv("APEX_FORWARD_ACCEL_LIMIT_IN");
        if (forwardAccelLimit != null) {
            constants.forwardAccelLimitIn = Double.parseDouble(forwardAccelLimit);
        }
        String cornerSpeedScale = System.getenv("APEX_PROFILE_CORNER_SPEED_SCALE");
        if (cornerSpeedScale != null) {
            constants.profileCornerSpeedScale = Double.parseDouble(cornerSpeedScale);
        }
        String profileKV = System.getenv("APEX_PROFILE_VELOCITY_POWER_COEFFICIENT");
        if (profileKV != null) {
            constants.profileVelocityPowerCoefficient = Double.parseDouble(profileKV);
        }
    }
}
