package org.firstinspires.ftc.teamcode.sim;

import util.DeltaTime;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.SimLinearOpModeBridge;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import follower.Follower;
import follower.FollowerConstants;
import paths.geometry.AngleUnit;
import paths.geometry.Pose;
import localizers.LocalizationConstants;
import org.firstinspires.ftc.teamcode.apexpathing.Constants;
import org.junit.Test;
import storage.ApexStorage;
import setup.follower.TunerContext;
import setup.follower.sysid.FeedforwardTuner;

import java.lang.reflect.Field;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

import static org.junit.Assert.*;

/** Fresh production SysId followed by unchanged-gain manual validation on stock FTCodeSim. */
public class FreshSysIdManualValidationTest {
    @Test(timeout = 180000)
    public void freshlyIdentifiedGainsRunBothManualAxes() throws Exception {
        String directory = System.getProperty(ApexStorage.DIRECTORY_PROPERTY);
        String unlock = System.getProperty("apex.simulation.unlockTunerPhases");
        Field singleton = FollowerConstants.class.getDeclaredField("instance");
        singleton.setAccessible(true);
        Object previousConstants = singleton.get(null);
        singleton.set(null, null);
        Path output = Paths.get("build/fresh-sysid-manual-validation").toAbsolutePath();
        Files.createDirectories(output);
        System.setProperty(ApexStorage.DIRECTORY_PROPERTY, Files.createTempDirectory("fresh-sysid-").toString());
        System.setProperty("apex.simulation.unlockTunerPhases", "true");
        SimLinearOpModeBridge.Session session = null;
        try {
            ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
            AtomicReference<TunerContext> contextRef = new AtomicReference<>();
            AtomicReference<FeedforwardTuner> phaseRef = new AtomicReference<>();
            AtomicReference<String> frame = new AtomicReference<>("");
            AtomicInteger phaseIndex = new AtomicInteger(-1);
            Field profileField = FeedforwardTuner.class.getDeclaredField("manualProfile");
            Field timerField = FeedforwardTuner.class.getDeclaredField("timer");
            Field stageField = FeedforwardTuner.class.getDeclaredField("stage");
            Field directionField = FeedforwardTuner.class.getDeclaredField("manualDirection");
            for (Field f : new Field[] {profileField, timerField, stageField, directionField}) { f.setAccessible(true); }
            List<String> trace = new ArrayList<>();
            trace.add("axis,direction,time_s,target_velocity,measured_velocity,physical_velocity,motor_power");
            List<String> results = new ArrayList<>();
            ApexSimTelemetry telemetry = new ApexSimTelemetry(value -> {
                frame.set(value);
                FeedforwardTuner phase = phaseRef.get();
                TunerContext context = contextRef.get();
                if (phaseIndex.get() < 2 || phase == null || context == null) { return; }
                try {
                    if (profileField.get(phase) == null || !((Enum<?>) stageField.get(phase)).name().equals("RUNNING")) { return; }
                    double t = ((ElapsedTime) timerField.get(phase)).seconds();
                    int direction = directionField.getInt(phase);
                    boolean heading = phaseIndex.get() == 3;
                    double peak = direction * (heading ? 1.5 : 24);
                    double target = t < 1 ? peak * t : t < 2 ? peak : t < 3 ? peak * (3 - t) : 0;
                    Follower follower = context.getFollower();
                    double measured = heading ? follower.getVelocity().getHeading(AngleUnit.RAD)
                            : follower.getVelocity().getVec().rotate(follower.getPose().getHeading().times(-1)).getX().getIn();
                    double physical = heading ? hardware.drivetrain.velocity.theta
                            : hardware.drivetrain.velocity.x * Math.cos(hardware.drivetrain.position.theta)
                                    + hardware.drivetrain.velocity.y * Math.sin(hardware.drivetrain.position.theta);
                    synchronized (trace) {
                        trace.add((heading ? "HEADING" : "DRIVE") + "," + direction + "," + t + ","
                                + target + "," + measured + "," + physical + "," + follower.getDrivetrain().getLastFrPower());
                    }
                } catch (IllegalAccessException e) { throw new IllegalStateException(e); }
            });
            telemetry.setMsTransmissionInterval(0);
            LinearOpMode opMode = new LinearOpMode() {
                @Override public void runOpMode() throws InterruptedException {
                    Constants robot = new Constants();
                    Follower seed = Follower.create(robot, hardwareMap, true);
                    LocalizationConstants localization = LocalizationConstants.empty();
                    localization.capture("DEFAULT", robot.localizerConstants(), seed.getLocalizer(),
                            LocalizationConstants.Status.ACCEPTED, LocalizationConstants.Status.ACCEPTED);
                    try { localization.save(); } catch (java.io.IOException e) { throw new IllegalStateException(e); }
                    // Advance the plant immediately before each sensor/control frame. A second
                    // thread catching up physics can expose partially updated poses to localization.
                    TunerContext context = new TunerContext(this) {
                        private long previousPhysics = System.nanoTime();
                        @Override public void beginFrame() {
                            previousPhysics = advanceRealtime(hardware, previousPhysics);
                            super.beginFrame();
                        }
                    };
                    context.setFollower(Follower.create(robot, hardwareMap, true));
                    contextRef.set(context);
                    waitForStart();
                    for (int i = 0; i < 4 && opModeIsActive(); i++) {
                        context.positionRobotForSimulation(Pose.zero());
                        FeedforwardTuner phase = new FeedforwardTuner(context,
                                i % 2 == 0 ? FeedforwardTuner.Axis.DRIVE : FeedforwardTuner.Axis.HEADING);
                        phaseRef.set(phase); phaseIndex.set(i);
                        if (!phase.run(this)) { return; }
                        if (i < 2) {
                            try { context.saveConstants(); } catch (Exception e) { throw new IllegalStateException(e); }
                        }
                    }
                    phaseIndex.set(4);
                    while (opModeIsActive()) { sleep(20); }
                }
            };
            opMode.hardwareMap = hardware.hardwareMap; opMode.telemetry = telemetry;
            opMode.gamepad1 = new Gamepad(); opMode.gamepad2 = new Gamepad();
            session = SimLinearOpModeBridge.initialize(opMode, () -> {});
            SimLinearOpModeBridge.start(session);
            int lastPhase = -1, completed = 0;
            String lastResult = "";
            long lastPress = 0, pulseEnd = 0, previous = System.nanoTime();
            char pulse = ' ';
            long deadline = previous + 160_000_000_000L;
            while (System.nanoTime() < deadline && phaseIndex.get() < 4) {
                long now = System.nanoTime();
                int index = phaseIndex.get();
                String value = frame.get();
                if (index != lastPhase) { lastPhase = index; completed = 0; lastResult = ""; }
                assertFalse("Fresh fit failed: " + value, value.contains("acquisition failed"));
                if (index >= 2 && (value.contains("Trial Completed") || value.contains("Trial Aborted"))) {
                    String result = value.substring(value.indexOf("Trial "));
                    result = result.substring(0, result.indexOf('\n'));
                    String expectedTrial = index == 2 ? (completed == 0 ? "forward" : "backward")
                            : (completed == 0 ? "CCW" : "CW");
                    // A phase change can briefly leave the preceding phase's last telemetry visible.
                    if (completed < 2 && result.equals("Trial Completed " + expectedTrial + " trial")
                            && !result.equals(lastResult)) {
                        lastResult = result; completed++;
                        results.add(value); System.out.println("MANUAL RESULT phase=" + index + "\n" + value);
                    }
                }
                if (now >= pulseEnd && now - lastPress > 450_000_000L) {
                    char desired = ' ';
                    if (index >= 0 && index < 2 && value.contains("Press A")) { desired = 'A'; }
                    if (index >= 2) {
                        if (value.contains("Mode Automatic")) { desired = 'R'; }
                        else if (value.contains("Mode Manual") || value.contains("Press A to save")) { desired = 'A'; }
                        else if (value.contains("Robot can be driven with sticks")) {
                            desired = completed >= 2 ? 'A' : 'X';
                        }
                    }
                    if (desired != ' ') {
                        pulse = desired; lastPress = now; pulseEnd = now + 60_000_000L;
                        if (desired == 'X') { contextRef.get().positionRobotForSimulation(Pose.zero()); }
                    }
                }
                opMode.gamepad1.a = now < pulseEnd && pulse == 'A';
                opMode.gamepad1.x = now < pulseEnd && pulse == 'X';
                opMode.gamepad1.dpad_right = now < pulseEnd && pulse == 'R';
                SimLinearOpModeBridge.eventLoopIteration(session, opMode.gamepad1, opMode.gamepad2);
                Thread.sleep(5);
            }
            synchronized (trace) { Files.write(output.resolve("manual-traces.csv"), trace, StandardCharsets.UTF_8); }
            Files.write(output.resolve("results.txt"), results, StandardCharsets.UTF_8);
            Files.write(output.resolve("fresh-constants.json"), contextRef.get().constants.toJson().toString(4).getBytes(StandardCharsets.UTF_8));
            System.out.println("DIAGNOSTICS " + output);
            assertEquals("Workflow did not finish: " + frame.get(), 4, phaseIndex.get());
            assertEquals(4, results.size());
            for (String result : results) { assertTrue(result, result.contains("Trial Completed")); }
            graphTraces(output, trace);
            probeCruiseHoldingPower(output, contextRef.get().constants);
            runDisplacementProfiles(output, hardware, contextRef.get().getFollower());
        } finally {
            if (session != null) { SimLinearOpModeBridge.stop(session); }
            singleton.set(null, previousConstants);
            if (directory == null) { System.clearProperty(ApexStorage.DIRECTORY_PROPERTY); }
            else { System.setProperty(ApexStorage.DIRECTORY_PROPERTY, directory); }
            if (unlock == null) { System.clearProperty("apex.simulation.unlockTunerPhases"); }
            else { System.setProperty("apex.simulation.unlockTunerPhases", unlock); }
        }
    }

    /** Same trapezoid as the manual tuner, indexed by measured progress instead of time. */
    private static void runDisplacementProfiles(Path output, ApexSimulation.Hardware hardware,
                                                Follower follower) throws Exception {
        List<String> trace = new ArrayList<>(), results = new ArrayList<>();
        trace.add("axis,direction,time_s,displacement,target_velocity,target_acceleration,measured_velocity,physical_velocity,power,phase");
        results.add("axis,direction,acceleration_mean_error,cruise_mean_error_after_target_crossing,cruise_mean_error_entire_phase,physical_cruise_mean_error_after_crossing,cruise_scored_s,max_power,final_displacement,outcome");
        follower.disableControllers();
        try {
            for (boolean heading : new boolean[] {false, true}) {
                double peak = heading ? 1.5 : 24, acceleration = peak;
                double rampDistance = peak * peak / (2 * acceleration), total = 4 * rampDistance;
                FollowerConstants gains = follower.getConstants();
                double ks = heading ? gains.angularFeedforwardKS : gains.translationalFeedforwardKS;
                double kv = heading ? gains.angularKV : gains.translationalKV;
                double ka = heading ? gains.angularKA : gains.translationalKA;
                for (int direction : new int[] {1, -1}) {
                    follower.stop(); follower.setPose(Pose.zero());
                    // Let the stock robot settle; resetting odometry alone does not remove velocity.
                    long previous = System.nanoTime(), settleEnd = previous + 1_000_000_000L;
                    while (System.nanoTime() < settleEnd) {
                        previous = advanceRealtime(hardware, previous);
                        follower.update(false); Thread.sleep(5);
                    }
                    follower.setPose(Pose.zero());
                    double lastHeading = follower.getPose().getHeading().getRad(), angularDisplacement = 0;
                    long start = System.nanoTime(); previous = start;
                    double previousTime = 0, previousProgress = 0, previousMeasured = 0, previousPhysical = 0;
                    double accelArea = 0, accelTime = 0, cruiseArea = 0, cruiseTime = 0;
                    double physicalCruiseArea = 0, allCruiseArea = 0, allCruiseTime = 0, maxPower = 0;
                    boolean cruiseReached = false;
                    String outcome = "timeout";
                    double progress = 0;
                    while (DeltaTime.seconds(System.nanoTime() - start) < 8) {
                        previous = advanceRealtime(hardware, previous);
                        follower.update(false);
                        double time = DeltaTime.seconds(System.nanoTime() - start);
                        double currentHeading = follower.getPose().getHeading().getRad();
                        angularDisplacement += Math.atan2(Math.sin(currentHeading - lastHeading), Math.cos(currentHeading - lastHeading));
                        lastHeading = currentHeading;
                        progress = direction * (heading ? angularDisplacement : follower.getPose().getVec().getX().getIn());
                        assertTrue("Invalid localization", follower.getLocalizer().isLastMeasurementValid());
                        double measured = heading ? follower.getVelocity().getHeading(AngleUnit.RAD)
                                : follower.getVelocity().getVec().rotate(follower.getPose().getHeading().times(-1)).getX().getIn();
                        double physical = heading ? hardware.drivetrain.velocity.theta
                                : hardware.drivetrain.velocity.x * Math.cos(hardware.drivetrain.position.theta)
                                + hardware.drivetrain.velocity.y * Math.sin(hardware.drivetrain.position.theta);
                        double target = direction * spatialVelocity(progress, peak, acceleration, total);
                        double targetAcceleration = direction * (progress < rampDistance ? acceleration
                                : progress < total - rampDistance ? 0 : progress < total ? -acceleration : 0);
                        String phase = progress < rampDistance ? "acceleration" : progress < total - rampDistance ? "cruise" : "braking";
                        // Split measurement intervals at spatial boundaries; braking never contributes.
                        double dt = time - previousTime;
                        if (dt > 0 && progress > previousProgress) {
                            double lo = Math.max(previousProgress, 0), hi = Math.min(progress, rampDistance);
                            if (hi > lo) {
                                double f0 = (lo - previousProgress) / (progress - previousProgress);
                                double f1 = (hi - previousProgress) / (progress - previousProgress);
                                double v0 = previousMeasured + f0 * (measured - previousMeasured);
                                double v1 = previousMeasured + f1 * (measured - previousMeasured);
                                double duration = (f1 - f0) * dt;
                                accelArea += .5 * duration * (v0 - direction * spatialVelocity(lo, peak, acceleration, total)
                                        + v1 - direction * spatialVelocity(hi, peak, acceleration, total));
                                accelTime += duration;
                            }
                            lo = Math.max(previousProgress, rampDistance); hi = Math.min(progress, total - rampDistance);
                            if (hi > lo) {
                                double f0 = (lo - previousProgress) / (progress - previousProgress);
                                double f1 = (hi - previousProgress) / (progress - previousProgress);
                                double v0 = previousMeasured + f0 * (measured - previousMeasured);
                                double v1 = previousMeasured + f1 * (measured - previousMeasured);
                                double duration = (f1 - f0) * dt;
                                allCruiseArea += .5 * duration * (v0 + v1 - 2 * direction * peak);
                                allCruiseTime += duration;
                                if (!cruiseReached) {
                                    if (direction * v0 >= peak) { cruiseReached = true; }
                                    else if (direction * v1 >= peak) {
                                        f0 += (f1 - f0) * (peak - direction * v0) / (direction * (v1 - v0));
                                        cruiseReached = true;
                                    }
                                }
                                if (cruiseReached) {
                                    v0 = previousMeasured + f0 * (measured - previousMeasured);
                                    double p0 = previousPhysical + f0 * (physical - previousPhysical);
                                    double p1 = previousPhysical + f1 * (physical - previousPhysical);
                                    duration = (f1 - f0) * dt;
                                    cruiseArea += .5 * duration * (v0 + v1 - 2 * direction * peak);
                                    physicalCruiseArea += .5 * duration * (p0 + p1 - 2 * direction * peak);
                                    cruiseTime += duration;
                                }
                            }
                        }
                        double command = ks * (target == 0 ? Math.signum(targetAcceleration) : Math.signum(target))
                                + kv * target + ka * targetAcceleration;
                        assertTrue("Unexpected power clipping", Math.abs(command) < .85);
                        maxPower = Math.max(maxPower, Math.abs(command));
                        trace.add((heading ? "HEADING" : "DRIVE") + "," + direction + "," + time + "," + progress
                                + "," + target + "," + targetAcceleration + "," + measured + "," + physical + "," + command + "," + phase);
                        if (progress >= total - (heading ? .015 : .25)) { outcome = "completed"; break; }
                        follower.getDrivetrain().moveWithVectors(heading ? 0 : command, 0, heading ? command : 0);
                        previousTime = time; previousProgress = progress; previousMeasured = measured; previousPhysical = physical;
                        Thread.sleep(20);
                    }
                    follower.stop();
                    assertTrue("Profile never reached cruise", allCruiseTime > 0);
                    String result = (heading ? "HEADING" : "DRIVE") + "," + direction + "," + accelArea / accelTime
                            + "," + (cruiseTime > 0 ? cruiseArea / cruiseTime : Double.NaN) + "," + allCruiseArea / allCruiseTime
                            + "," + (cruiseTime > 0 ? physicalCruiseArea / cruiseTime : Double.NaN) + "," + cruiseTime
                            + "," + maxPower + "," + progress + "," + outcome;
                    results.add(result); System.out.println("DISPLACEMENT RESULT " + result);
                }
            }
        } finally { follower.stop(); follower.enableControllers(); }
        Files.write(output.resolve("displacement-traces.csv"), trace, StandardCharsets.UTF_8);
        Files.write(output.resolve("displacement-results.csv"), results, StandardCharsets.UTF_8);
    }

    private static double spatialVelocity(double progress, double peak, double acceleration, double total) {
        return Math.min(peak, Math.sqrt(Math.max(0, 2 * acceleration * Math.min(progress, total - progress))));
    }

    private static long advanceRealtime(ApexSimulation.Hardware hardware, long previous) {
        long now = System.nanoTime();
        double remaining = DeltaTime.seconds(now - previous);
        while (remaining > 1e-9) {
            double dt = Math.min(.005, remaining); StockSimTestSupport.step(hardware, dt); remaining -= dt;
        }
        return now;
    }

    private static void graphTraces(Path output, List<String> trace) throws Exception {
        for (String axis : new String[] {"DRIVE", "HEADING"}) {
            org.knowm.xchart.XYChart chart = new org.knowm.xchart.XYChartBuilder()
                    .width(850).height(480).title("Fresh SysId: " + axis + " manual velocity profile")
                    .xAxisTitle("Time (s)").yAxisTitle("Speed in trial direction (" + (axis.equals("DRIVE") ? "in/s" : "rad/s") + ")").build();
            double peak = axis.equals("DRIVE") ? 24 : 1.5;
            chart.addSeries("Target", new double[] {0, 1, 2, 3}, new double[] {0, peak, peak, 0});
            for (int direction : new int[] {1, -1}) {
                List<Double> times = new ArrayList<>(), measured = new ArrayList<>(), physical = new ArrayList<>();
                for (String line : trace.subList(1, trace.size())) {
                    String[] fields = line.split(",");
                    if (!fields[0].equals(axis) || Integer.parseInt(fields[1]) != direction) { continue; }
                    times.add(Double.parseDouble(fields[2])); measured.add(direction * Double.parseDouble(fields[4]));
                    physical.add(direction * Double.parseDouble(fields[5]));
                }
                String label = axis.equals("DRIVE") ? (direction > 0 ? "Forward" : "Backward") : (direction > 0 ? "CCW" : "CW");
                chart.addSeries(label + " measured", times, measured);
                chart.addSeries(label + " simulator", times, physical);
            }
            chart.getStyler().setMarkerSize(0);
            chart.getStyler().setLegendPosition(org.knowm.xchart.style.Styler.LegendPosition.InsideNW);
            org.knowm.xchart.BitmapEncoder.saveBitmap(chart, output.resolve(axis.toLowerCase() + "-velocity").toString(),
                    org.knowm.xchart.BitmapEncoder.BitmapFormat.PNG);
        }
    }

    private static void probeCruiseHoldingPower(Path output, FollowerConstants constants) throws Exception {
        ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
        org.codeblooded.ftcodesim.hardware.devices.SimMotor[] motors = StockSimTestSupport.motors(hardware);
        double drive = motors[0].config.modelCoefficients[0] * motors[0].config.voltageSensor.getVoltage()
                * hardware.drivetrain.config.wheelRadius;
        double emf = motors[0].config.modelCoefficients[1];
        double drag = 49; // The unchanged stock DECODE setup in ApexSimulation.
        double holding = drag / (drive - emf * 24);
        double fitted = constants.translationalFeedforwardKS + constants.translationalKV * 24;
        List<String> probes = new ArrayList<>();
        probes.add("case,target_velocity_in_s,power,actual_acceleration_in_s2");
        for (boolean corrected : new boolean[] {false, true}) {
            hardware.drivetrain.setPosition(new org.codeblooded.ftcodesim.ascope.boundaries.MotionVector(
                    ApexSimulation.FIELD_CENTER_INCHES, ApexSimulation.FIELD_CENTER_INCHES, 0));
            StockSimTestSupport.setSpeed(hardware, 24);
            double power = corrected ? holding : fitted;
            for (org.codeblooded.ftcodesim.hardware.devices.SimMotor motor : motors) { motor.setPower(power); }
            StockSimTestSupport.step(hardware, .0001);
            double acceleration = (hardware.drivetrain.velocity.x - 24) / .0001;
            assertEquals(power * (drive - emf * 24) - drag, acceleration, 1e-7);
            if (corrected) { assertEquals(0, acceleration, 1e-7); }
            probes.add((corrected ? "stock holding power" : "fresh fitted cruise power") + ",24," + power + "," + acceleration);
        }
        Files.write(output.resolve("cruise-power-probes.csv"), probes, StandardCharsets.UTF_8);
        System.out.println("CRUISE POWER PROBES " + probes);
    }
}
