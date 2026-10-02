package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import core.ApexConstants;
import core.ApexStorage;
import core.Follower;
import core.FollowerConstants;
import drivetrains.BaseDrivetrainConstants;
import drivetrains.Mecanum;
import drivetrains.Motor;
import feedforward.MotionParameters;
import geometry.Angle;
import geometry.Dist;
import geometry.GeometryFactory;
import geometry.Pose;
import geometry.Vector;
import localizers.BaseLocalizer;
import localizers.BaseLocalizerConstants;
import org.codeblooded.ftcodesim.hardware.devices.SimMotor;
import org.codeblooded.fit.MotorModel;
import org.firstinspires.ftc.teamcode.apexpathing.FollowerTuner;
import org.json.JSONObject;
import org.junit.Test;
import org.knowm.xchart.BitmapEncoder;
import org.knowm.xchart.XYChart;
import org.knowm.xchart.XYChartBuilder;
import org.knowm.xchart.style.Styler;
import paths.builders.PathBuilder;
import paths.constraint.PathConstraint;
import paths.constraint.TranslationalConstraint;
import paths.heading.InterpolationStyle;
import paths.movements.Path;

import java.io.File;
import java.lang.reflect.Field;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Test-only A/B experiment. CURRENT runs the actual production Follower. The alternatives
 * drive the same four SimMotors with the discussed scalar law; production code is unchanged.
 * An unbounded straight-line bench integrates wheel velocity, avoiding simulated field walls.
 * Pose is exact; velocity has a specified, deterministic sensor lag/noise rather than wall time.
 * Endpoint capture and spatial derivative clocks are intentionally outside this experiment.
 */
public class StraightDecelerationComparisonTest {
    private static final double LENGTH = 360, STOP = 300;
    private static final double[] CHANGES = {0, 60, 90, 150, 180, 240, 270};
    private static final double[] SPEEDS = {42, 18, 40, 12, 36, 20, 42};
    private static final double[] BOUNDARIES = {60, 150, 240};
    private static final double LINEAR_KS = .06, LINEAR_KV = .94 / 75, LINEAR_KA = .94 / 150;

    private enum Policy { CURRENT, CONTINUOUS_SAME_LUT, CONTINUOUS_CONSTANT_ACCEL }
    private enum Fixture { LINEAR_MECANUM, LINEAR_TANK, DEFAULT_MECANUM }

    @Test public void accelerationMustPassBeforeInteriorBrakingIsCompared() throws Exception {
        File output = new File("build/straight-braking-comparison").getAbsoluteFile();
        assertTrue(output.isDirectory() || output.mkdirs());
        Field singleton = FollowerConstants.class.getDeclaredField("instance");
        singleton.setAccessible(true);
        Object previous = singleton.get(null);
        String previousStorage = System.getProperty(ApexStorage.DIRECTORY_PROPERTY);
        String previousUnlock = System.getProperty(FollowerTuner.UNLOCK_PHASES_PROPERTY);
        List<String> report = new ArrayList<>();
        report.add("# Straight-line deceleration comparison\n");
        report.add("Production follower versus test-only continuous braking. Three interior slowdowns: "
                + "42→18, 40→12, and 36→20 in/s at 60, 150, and 240 inches. "
                + "The robot is simulated from rest to 300 inches; endpoint braking is excluded.\n");
        report.add("Same generated profile, gains, motor model, initial state, and deterministic sensor "
                + "signal for each paired run. CONTINUOUS_SAME_LUT isolates the braking law; "
                + "CONTINUOUS_CONSTANT_ACCEL additionally changes displacement interpolation. "
                + "All speed errors use one common constant-acceleration reference.\n");
        report.add("DEFAULT_MECANUM uses src/test/resources/straight-deceleration-baseline.json, "
                + "a snapshot of the current saved tuning on 2026-10-01. No runtime tuning file is overwritten.\n");
        report.add("An unbounded wheel-velocity bench has no field collisions, lateral slip, steering "
                + "demand, or battery variation. DEFAULT_MECANUM includes the simulator's one-dimensional "
                + "chassis friction; LINEAR fixtures are explicitly known affine plants. "
                + "These are controller experiments, not robot validation.\n");
        try {
            System.setProperty(ApexStorage.DIRECTORY_PROPERTY, new File(output, "isolated-storage").toString());
            for (Fixture fixture : Fixture.values()) {
                FollowerConstants constants = seed(fixture);
                if (fixture != Fixture.DEFAULT_MECANUM) verifyLinearPlant(fixture);
                report.add("## " + fixture + "\n");
                // No braking evidence is interpreted until open-loop and closed-loop acceleration pass.
                double openLoop = rampError(fixture, constants, output, "initial");
                report.add(String.format(Locale.US, "Initial feedforward-only 6→36 in/s ramp RMS: %.3f in/s.\n", openLoop));
                if (openLoop > 2.5 && fixture == Fixture.DEFAULT_MECANUM) {
                    report.add("Initial acceleration model failed. Recharacterized **test-local** kS/kV/kA "
                            + "using holding speeds and positive-acceleration probes only; saved constants are untouched.\n");
                    characterize(fixture, constants);
                    openLoop = rampError(fixture, constants, output, "calibrated");
                    report.add(String.format(Locale.US, "Calibrated feedforward-only acceleration RMS: %.3f in/s.\n", openLoop));
                }
                Files.write(new File(output, fixture + "-constants.json").toPath(),
                        constants.toJson().toString(2).getBytes(StandardCharsets.UTF_8));
                assertTrue("Acceleration feedforward failed for " + fixture + ": " + openLoop, openLoop <= 2.5);
                singleton.set(null, constants);
                Path path = course(fixture);
                verifyCourse(path);
                writeProfile(output, fixture, path);
                double[] boundaries = effectiveBoundaries(knots(path));
                report.add(String.format(Locale.US, "Generated slowdown boundaries: %.3f, %.3f, %.3f in. "
                        + "Apex samples constraints onto profile knots, so the requested boundaries are rounded forward. "
                        + "Boundary error is scored at the generated knots.\n", boundaries[0], boundaries[1], boundaries[2]));

                // Validate initial and repeated acceleration legs for EVERY policy before comparing brakes.
                List<Run> runs = new ArrayList<>();
                for (double dt : new double[]{.01, .02, .04}) {
                    for (int sensorCase = 0; sensorCase < 3; sensorCase++) {
                        for (Policy policy : Policy.values()) {
                            singleton.set(null, constants);
                            Run run = run(fixture, constants, path, policy, dt, sensorCase);
                            runs.add(run);
                            writeTrace(output, run);
                        }
                    }
                }
                report.add(String.format(Locale.US, "Test-local gains: kS=%.6f, kV=%.6f, kA=%.6f, "
                        + "velocity gain=%.6f; limits %.1f in/s, %.1f in/s².\n", constants.translationalFeedforwardKS,
                        constants.translationalKV, constants.translationalKA, constants.velocityFeedbackGain,
                        constants.forwardVelLimitIn, constants.forwardAccelLimitIn));
                report.add("| Policy | Loop ms | Sensor case | Accel RMS | Brake RMS | Brake bias | Peak over | "
                        + "Peak under | Boundary over | Boundary under | Power variation/s | Max power step | Accel gate | Brake target |\n"
                        + "|---|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---|");
                boolean accelerationPassed = true;
                for (Run run : runs) {
                    boolean gate = run.accelRms() <= 4.2 && run.accelPeak <= 8.4 && run.completed;
                    accelerationPassed &= gate;
                    boolean brakeTarget = run.brakeRms() <= 3 && run.brakePeak <= 6 && run.brakeUnderspeed <= 6
                            && run.boundaryPeak <= 3 && run.boundaryUnderspeed <= 3;
                    report.add(String.format(Locale.US, "| %s | %.0f | %s | %.3f | %.3f | %.3f | %.3f | %.3f | %.3f | %.3f | %.3f | %.3f | %s | %s |",
                            run.policy, run.dt * 1000, sensorLabel(run.sensorCase), run.accelRms(), run.brakeRms(),
                            run.brakeSigned / run.brakeDuration, run.brakePeak, run.brakeUnderspeed,
                            run.boundaryPeak, run.boundaryUnderspeed, run.powerVariation / run.brakeDuration,
                            run.powerStep, gate ? "PASS" : "FAIL", brakeTarget ? "MET" : "MISSED"));
                    if (run.dt == .02 && run.sensorCase == 1) graph(output, fixture, runs, run);
                }
                report.add("\nAcceleration gate: RMS ≤4.2 in/s and peak absolute error ≤8.4 in/s, "
                        + "after the first 3 inches and outside endpoint capture. Includes every positive-acceleration "
                        + "leg, so poor recovery acceleration cannot masquerade as better braking.\n");
                double maxAccelRms = 0, maxAccelPeak = 0;
                for (Run run : runs) {
                    maxAccelRms = Math.max(maxAccelRms, run.accelRms());
                    maxAccelPeak = Math.max(maxAccelPeak, run.accelPeak);
                }
                report.add(String.format(Locale.US, "Worst acceleration RMS: %.3f in/s; "
                        + "worst absolute acceleration tracking error: %.3f in/s.\n", maxAccelRms, maxAccelPeak));
                Files.write(new File(output, "results.md").toPath(), report, StandardCharsets.UTF_8);
                assertTrue("Acceleration tracking failed; braking conclusion withheld for " + fixture, accelerationPassed);
                for (Run run : runs) assertTrue("Expected exactly three braking intervals for " + run.name(), run.brakeIntervals == 3);
                report.add("Braking acceptance targets (diagnostic, not asserted): RMS ≤3 in/s, "
                        + "peak absolute error ≤6 in/s, absolute speed error at each slowdown boundary ≤3 in/s. "
                        + "Power variation/s is total absolute power change during deceleration divided by braking duration; "
                        + "it measures command activity, not physical jerk.\n");
                responseProbes(output, fixture, constants, report);
                System.out.printf(Locale.US, "%s acceleration gates passed; comparison artifacts: %s%n", fixture, output);
            }
            Files.write(new File(output, "results.md").toPath(), report, StandardCharsets.UTF_8);
        } finally {
            Files.write(new File(output, "results.md").toPath(), report, StandardCharsets.UTF_8);
            singleton.set(null, previous);
            restoreProperty(ApexStorage.DIRECTORY_PROPERTY, previousStorage);
            restoreProperty(FollowerTuner.UNLOCK_PHASES_PROPERTY, previousUnlock);
        }
    }

    private static FollowerConstants seed(Fixture fixture) throws Exception {
        boolean tank = fixture == Fixture.LINEAR_TANK;
        FollowerConstants c;
        File saved = new File("src/test/resources/straight-deceleration-baseline.json");
        if (fixture == Fixture.DEFAULT_MECANUM) {
            c = FollowerConstants.fromJson(new JSONObject(new String(Files.readAllBytes(saved.toPath()), StandardCharsets.UTF_8)));
        } else {
            c = FollowerConstants.fromJson(new JSONObject().put("drivetrainType", tank ? "TANK" : "MECANUM"));
            c.translationalFeedforwardKS = LINEAR_KS;
            c.translationalKV = LINEAR_KV;
            c.translationalKA = LINEAR_KA;
            c.deriveVelocityFeedbackGains();
            c.forwardVelLimitIn = c.strafeVelLimitIn = 65;
            c.forwardAccelLimitIn = c.strafeAccelLimitIn = 60;
            c.angularVelLimitRad = 5;
            c.angularAccelLimitRad = 10;
        }
        return c;
    }

    private static ApexSimulation.Hardware hardware(Fixture fixture) {
        ApexSimulation.Hardware h = fixture == Fixture.LINEAR_TANK
                ? ApexSimulation.createTankHardware() : ApexSimulation.createHardware();
        if (fixture != Fixture.DEFAULT_MECANUM) {
            double radius = radius(fixture);
            for (String name : h.drivetrain.motorNames) {
                SimMotor m = (SimMotor) h.hardwareMap.get(DcMotorEx.class, name);
                // Explicit terms avoid relying on the library's changing coefficient layout.
                m.config.motorModel = new MotorModel((v, d, b) -> d * b,
                        (v, d, b) -> -v, (v, d, b) -> -Math.signum(v));
                double[] coefficients = {1 / (LINEAR_KA * radius * m.config.voltageSensor.getVoltage()), LINEAR_KV / LINEAR_KA,
                        LINEAR_KS / (LINEAR_KA * radius)};
                m.config.modelCoefficients = coefficients;
                m.config.zeroPowerBrakeCoefficients = coefficients.clone();
            }
        }
        return h;
    }

    private static double radius(Fixture f) { return f == Fixture.LINEAR_TANK ? 2 : 1.889765; }

    private static void verifyLinearPlant(Fixture fixture) {
        Bench bench = new Bench(fixture);
        for (double v : new double[]{9, 21, 39}) for (double a : new double[]{0, 15, 45}) {
            bench.setSpeed(v);
            bench.power(LINEAR_KS + LINEAR_KV * v + LINEAR_KA * a);
            bench.step(.001);
            double actual = (bench.speed() - v) / .001;
            assertTrue("Known plant: " + fixture + " v=" + v + " a=" + a + " actual=" + actual,
                    Math.abs(actual - a) < 1e-4);
        }
    }

    private static Path course(Fixture fixture) {
        GeometryFactory g = new GeometryFactory();
        PathBuilder<?> b = fixture == Fixture.LINEAR_TANK
                ? g.tankPath(g.pose(0, 0, 0), g.pose(LENGTH, 0, 0))
                : g.holonomicPath(g.pose(0, 0, 0), g.pose(LENGTH, 0, 0));
        b.interpolateWith(fixture == Fixture.LINEAR_TANK ? InterpolationStyle.TANGENT_FORWARD
                : InterpolationStyle.CONSTANT_START_HEADING);
        for (int i = 0; i < CHANGES.length; i++) {
            b.addConstraint(new TranslationalConstraint(CHANGES[i] / LENGTH,
                    PathConstraint.Type.VELOCITY, Dist.fromIn(SPEEDS[i])));
        }
        return b.buildToMemory();
    }

    private static void verifyCourse(Path path) {
        assertTrue("Course must have a generated profile", path.isProfiled());
        int intervals = 0;
        boolean braking = false;
        for (int i = 0; i < STOP * 4; i++) {
            double a = path.getFeedforwardLut().getFFParams(i / 4.0).getTangentialAccel();
            if (a < -5 && !braking) intervals++;
            braking = a < -5;
        }
        assertTrue("Expected at least three interior deceleration segments", intervals >= 3);
    }

    private static Run run(Fixture fixture, FollowerConstants c, Path path, Policy policy,
                           double dt, int sensorCase) throws Exception {
        Bench bench = new Bench(fixture);
        BenchLocalizer localizer = new BenchLocalizer(bench, sensorCase);
        ApexConstants robot = new ApexConstants() {
            @Override public BaseDrivetrainConstants<?> drivetrainConstants() {
                if (fixture == Fixture.LINEAR_TANK) return new TankSimulationConstants().drivetrainConstants();
                return new Mecanum.Constants().setFrontLeftMotor(new Motor(ApexSimulation.FRONT_LEFT_MOTOR))
                        .setFrontRightMotor(new Motor(ApexSimulation.FRONT_RIGHT_MOTOR))
                        .setBackLeftMotor(new Motor(ApexSimulation.BACK_LEFT_MOTOR))
                        .setBackRightMotor(new Motor(ApexSimulation.BACK_RIGHT_MOTOR)).setMaxPower(1);
            }
            @Override public BaseLocalizerConstants<?> localizerConstants() { return localizer.configValue; }
        };
        Follower follower = new Follower(robot, bench.h.hardwareMap);
        follower.follow(path);
        MotionParameters[] knots = knots(path);
        double[] boundaries = effectiveBoundaries(knots);
        Run result = new Run(fixture, policy, dt, sensorCase);
        double time = 0;
        boolean wasBraking = false;
        double previousPower = 0;
        int frame = 0, nextBoundary = 0;
        try {
            while (bench.x < STOP && time < 40) {
                double step = sensorCase == 2 ? dt * new double[]{.7, 1.3, 1.1, .9}[frame % 4] : dt;
                localizer.sample(step, frame);
                // Score every policy against the same physically consistent displacement reference.
                MotionParameters reference = constantAcceleration(knots, bench.x);
                MotionParameters sampled = policy == Policy.CONTINUOUS_CONSTANT_ACCEL
                        ? reference : path.getFeedforwardLut().getFFParams(bench.x);
                double target = sampled.getTangentialVel(), a = sampled.getTangentialAccel();
                double referenceAccel = reference.getTangentialAccel();
                double measured = localizer.getVel().getX().getIn();
                double feedforward, feedback;
                if (policy == Policy.CURRENT) {
                    follower.update();
                    target = follower.getTrackingVelocityTarget();
                    feedforward = follower.getTrackingFeedforward();
                    feedback = follower.getTrackingVelocityFeedback();
                } else {
                    double applied = a;
                    if (a < 0 && target > 1e-6) {
                        double scale = Math.max(0, Math.min(1, measured / target));
                        applied *= scale * scale;
                    }
                    double sign = Math.abs(target) > 1e-6 ? Math.signum(target) : Math.signum(applied);
                    feedforward = c.translationalFeedforwardKS * sign + c.translationalKV * target
                            + c.translationalKA * applied;
                    feedback = c.velocityFeedbackGain * (target - measured);
                    bench.power(Math.max(-1, Math.min(1, feedforward + feedback)));
                }
                double power = bench.power();
                double truth = bench.speed();
                double error = truth - target;
                double nominalError = truth - reference.getTangentialVel();
                if (referenceAccel > 5 && bench.x > 3) {
                    result.accelSquared += nominalError * nominalError * step;
                    result.accelDuration += step;
                    result.accelPeak = Math.max(result.accelPeak, Math.abs(nominalError));
                }
                if (referenceAccel < -5) {
                    if (!wasBraking) result.brakeIntervals++;
                    result.brakeSquared += nominalError * nominalError * step;
                    result.brakeDuration += step;
                    result.brakePeak = Math.max(result.brakePeak, nominalError);
                    result.brakeUnderspeed = Math.max(result.brakeUnderspeed, -nominalError);
                    result.brakeSigned += nominalError * step;
                    if (wasBraking) {
                        result.powerVariation += Math.abs(power - previousPower);
                        result.powerStep = Math.max(result.powerStep, Math.abs(power - previousPower));
                    }
                }
                result.rows.add(new double[]{time, bench.x, reference.getTangentialVel(), target, a,
                        truth, measured, power, feedforward, feedback, error, nominalError, step});
                previousPower = power;
                wasBraking = referenceAccel < -5;
                double previousX = bench.x, previousV = truth;
                bench.step(step);
                while (nextBoundary < boundaries.length && bench.x >= boundaries[nextBoundary]) {
                    double fraction = (boundaries[nextBoundary] - previousX) / (bench.x - previousX);
                    double speedAtBoundary = previousV + fraction * (bench.speed() - previousV);
                    result.boundaryPeak = Math.max(result.boundaryPeak, speedAtBoundary - SPEEDS[2 * nextBoundary + 1]);
                    result.boundaryUnderspeed = Math.max(result.boundaryUnderspeed,
                            SPEEDS[2 * nextBoundary + 1] - speedAtBoundary);
                    result.boundarySpeeds[nextBoundary] = speedAtBoundary;
                    nextBoundary++;
                }
                time += step;
                frame++;
            }
            result.completed = bench.x >= STOP;
            assertTrue("Acceleration and braking samples required: " + result.name(),
                    result.accelDuration > .2 && result.brakeDuration > .2);
            return result;
        } finally { follower.stop(); }
    }

    private static MotionParameters[] knots(Path path) {
        geometry.PathPoint[] points = path.getGeneratedPoints();
        MotionParameters[] knots = new MotionParameters[points.length];
        for (int i = 0; i < knots.length; i++) knots[i] = path.getFeedforwardLut().getFFParams(
                path.getParametricPath().getLengthIn() - points[i].getDistanceToEndIn());
        return knots;
    }

    private static double[] effectiveBoundaries(MotionParameters[] knots) {
        double[] result = new double[BOUNDARIES.length];
        for (int i = 0; i < result.length; i++) {
            result[i] = Double.NaN;
            for (MotionParameters p : knots) {
                if (p.getDistAlongCurve() >= BOUNDARIES[i] - 1e-8
                        && p.getTangentialVel() <= SPEEDS[2 * i + 1] + 1e-8) {
                    result[i] = p.getDistAlongCurve(); break;
                }
            }
            assertTrue("Slowdown must reach the specified low speed", Double.isFinite(result[i]));
        }
        return result;
    }

    private static MotionParameters constantAcceleration(MotionParameters[] knots, double s) {
        if (s <= 0) return knots[0];
        for (int i = 0; i < knots.length - 1; i++) {
            MotionParameters p = knots[i], q = knots[i + 1];
            if (s < q.getDistAlongCurve()) {
                double ds = q.getDistAlongCurve() - p.getDistAlongCurve();
                if (ds <= 1e-9) continue;
                double a = (q.getTangentialVel() * q.getTangentialVel()
                        - p.getTangentialVel() * p.getTangentialVel()) / (2 * ds);
                MotionParameters result = new MotionParameters().setTangentialAccel(a)
                        .setTangentialVel(Math.sqrt(Math.max(0, p.getTangentialVel() * p.getTangentialVel()
                                + 2 * a * (s - p.getDistAlongCurve()))));
                result.setDistAlongCurve(s);
                return result;
            }
        }
        return knots[knots.length - 1];
    }

    private static double rampError(Fixture fixture, FollowerConstants c, File output, String stage) throws Exception {
        Bench bench = new Bench(fixture);
        bench.setSpeed(6);
        List<String> csv = new ArrayList<>();
        csv.add("time_s,target_in_s,true_in_s,power");
        double squared = 0;
        for (int i = 0; i < 50; i++) {
            double target = 6 + 30 * i * .02;
            double speed = bench.speed();
            squared += (speed - target) * (speed - target);
            double power = c.translationalFeedforwardKS + c.translationalKV * target + c.translationalKA * 30;
            csv.add(String.format(Locale.US, "%.3f,%.6f,%.6f,%.6f", i * .02, target, speed, power));
            bench.power(Math.max(-1, Math.min(1, power)));
            bench.step(.02);
        }
        Files.write(new File(output, fixture + "-acceleration-" + stage + ".csv").toPath(), csv, StandardCharsets.UTF_8);
        return Math.sqrt(squared / 50);
    }

    private static void characterize(Fixture fixture, FollowerConstants c) {
        Bench bench = new Bench(fixture);
        double n = 0, sx = 0, sy = 0, sxx = 0, sxy = 0;
        for (double v = 6; v <= 42; v += 6) {
            double u = powerFor(bench, v, 0);
            n++; sx += v; sy += u; sxx += v * v; sxy += v * u;
        }
        c.translationalKV = (n * sxy - sx * sy) / (n * sxx - sx * sx);
        c.translationalFeedforwardKS = (sy - c.translationalKV * sx) / n;
        double numerator = 0, denominator = 0;
        for (double v = 6; v <= 42; v += 6) for (double a : new double[]{15, 30, 45}) {
            numerator += a * (powerFor(bench, v, a) - c.translationalFeedforwardKS - c.translationalKV * v);
            denominator += a * a;
        }
        c.translationalKA = numerator / denominator;
        c.deriveVelocityFeedbackGains();
    }

    private static double powerFor(Bench bench, double velocity, double acceleration) {
        double low = 0, high = 1;
        for (int i = 0; i < 40; i++) {
            double power = (low + high) / 2;
            bench.setSpeed(velocity);
            bench.power(power);
            bench.step(.001);
            double actual = (bench.speed() - velocity) / .001;
            if (actual > acceleration) high = power; else low = power;
        }
        return (low + high) / 2;
    }

    private static void responseProbes(File output, Fixture fixture, FollowerConstants c,
                                       List<String> report) throws Exception {
        // Run only AFTER every complete-run acceleration gate has passed. No gain is changed here.
        Bench bench = new Bench(fixture);
        List<String> csv = new ArrayList<>();
        csv.add("speed_in_s,commanded_acceleration_in_s2,power,actual_acceleration_in_s2");
        report.add("Signed motor-response probes at 24 in/s, after acceleration validation:\n"
                + "\n| Requested acceleration | Power | Actual acceleration |\n|---:|---:|---:|");
        for (double v : new double[]{12, 24, 36, 42}) for (double a : new double[]{0, 30, 60, -30, -60, -120}) {
            double power = c.translationalFeedforwardKS + c.translationalKV * v + c.translationalKA * a;
            power = Math.max(-1, Math.min(1, power));
            bench.setSpeed(v);
            bench.power(power);
            bench.step(.001);
            double actual = (bench.speed() - v) / .001;
            csv.add(String.format(Locale.US, "%.6f,%.6f,%.6f,%.6f", v, a, power, actual));
            if (v == 24) report.add(String.format(Locale.US, "| %.1f | %.4f | %.1f |", a, power, actual));
        }
        report.add("");
        Files.write(new File(output, fixture + "-signed-response.csv").toPath(), csv, StandardCharsets.UTF_8);
    }

    private static final class Bench {
        final ApexSimulation.Hardware h;
        final double radius;
        final double naturalDeceleration;
        final SimMotor[] motors = new SimMotor[4];
        double x;
        Bench(Fixture fixture) {
            h = hardware(fixture); radius = radius(fixture);
            naturalDeceleration = fixture == Fixture.DEFAULT_MECANUM ? 150 * .06 / .94 : 0;
            for (int i = 0; i < motors.length; i++) motors[i] = (SimMotor) h.hardwareMap.get(
                    DcMotorEx.class, h.drivetrain.motorNames[i]);
        }
        double speed() { double v = 0; for (SimMotor m : motors) v += m.getVelocity() * radius; return v / 4; }
        void setSpeed(double v) { for (SimMotor m : motors) m.setRollVelocity(v / radius); }
        void power(double u) { for (SimMotor m : motors) m.setPower(u); }
        double power() { double u = 0; for (SimMotor m : motors) u += m.getPower(); return u / 4; }
        void step(double dt) {
            int steps = (int) Math.ceil(dt / .001);
            double substep = dt / steps;
            for (int i = 0; i < steps; i++) {
                double before = speed();
                for (SimMotor m : motors) m.update(substep);
                if (naturalDeceleration > 0) {
                    double acceleration = (speed() - before) / substep;
                    // One-dimensional specialization of SimulatedMecanum's chassis friction.
                    if (Math.abs(before) < .05 && Math.abs(acceleration) < naturalDeceleration) setSpeed(0);
                    else setSpeed(speed() - Math.signum(before) * naturalDeceleration * substep);
                }
                x += (before + speed()) * .5 * substep;
            }
        }
    }

    private static final class BenchConfig implements BaseLocalizerConstants<BenchConfig> {
        final BenchLocalizer localizer;
        BenchConfig(BenchLocalizer localizer) { this.localizer = localizer; }
        @Override public BaseLocalizer<?> build(HardwareMap hardwareMap) { return localizer; }
    }
    private static final class BenchLocalizer extends BaseLocalizer<BenchConfig> {
        final Bench bench;
        final int sensorCase;
        final BenchConfig configValue;
        double filtered;
        BenchLocalizer(Bench bench, int sensorCase) {
            super(null); this.bench = bench; this.sensorCase = sensorCase;
            configValue = new BenchConfig(this);
        }
        void sample(double dt, int frame) {
            double measured = bench.speed() + (sensorCase == 2 ? .2 * Math.sin(frame * 1.71) : 0);
            filtered += (sensorCase == 0 ? 1 : 1 - Math.exp(-dt / .04)) * (measured - filtered);
            pose = new Pose(Vector.of(bench.x, 0, geometry.DistUnit.IN), Angle.zero());
            velocity = new Pose(Vector.of(filtered, 0, geometry.DistUnit.IN), Angle.zero());
            rawVelocity = new Pose(Vector.of(measured, 0, geometry.DistUnit.IN), Angle.zero());
        }
        @Override public void update() { }
        @Override public void setPose(Pose newPose) { bench.x = newPose.getX().getIn(); pose = newPose; }
    }

    private static final class Run {
        final Fixture fixture; final Policy policy; final double dt; final int sensorCase;
        final List<double[]> rows = new ArrayList<>();
        double accelSquared, accelDuration, accelPeak, brakeSquared, brakeDuration, brakePeak;
        double boundaryPeak, boundaryUnderspeed, brakeUnderspeed, brakeSigned, powerVariation, powerStep;
        final double[] boundarySpeeds = new double[3];
        int brakeIntervals; boolean completed;
        Run(Fixture fixture, Policy policy, double dt, int sensorCase) {
            this.fixture = fixture; this.policy = policy; this.dt = dt; this.sensorCase = sensorCase;
        }
        double accelRms() { return Math.sqrt(accelSquared / accelDuration); }
        double brakeRms() { return Math.sqrt(brakeSquared / brakeDuration); }
        String name() { return fixture + "-" + policy + "-" + (int) (dt * 1000) + "ms-" + sensorCase; }
    }

    private static String sensorLabel(int i) { return new String[]{"exact", "40ms lag", "lag+noise+jitter"}[i]; }
    private static void restoreProperty(String key, String previous) {
        if (previous == null) System.clearProperty(key); else System.setProperty(key, previous);
    }
    private static void writeTrace(File output, Run run) throws Exception {
        List<String> csv = new ArrayList<>();
        csv.add("time_s,distance_in,profile_target_in_s,controller_target_in_s,profile_accel_in_s2,"
                + "true_speed_in_s,measured_speed_in_s,power,feedforward,feedback,controller_error_in_s,profile_error_in_s,dt_s");
        for (double[] row : run.rows) {
            StringBuilder line = new StringBuilder();
            for (double v : row) { if (line.length() > 0) line.append(','); line.append(String.format(Locale.US, "%.8f", v)); }
            csv.add(line.toString());
        }
        Files.write(new File(output, run.name() + ".csv").toPath(), csv, StandardCharsets.UTF_8);
        List<String> boundaryCsv = new ArrayList<>();
        boundaryCsv.add("slowdown_index,target_speed_in_s,actual_speed_in_s,error_in_s");
        for (int i = 0; i < run.boundarySpeeds.length; i++) boundaryCsv.add(String.format(Locale.US,
                "%d,%.8f,%.8f,%.8f", i + 1, SPEEDS[2 * i + 1], run.boundarySpeeds[i],
                run.boundarySpeeds[i] - SPEEDS[2 * i + 1]));
        Files.write(new File(output, run.name() + "-boundaries.csv").toPath(), boundaryCsv, StandardCharsets.UTF_8);
    }
    private static void writeProfile(File output, Fixture fixture, Path path) throws Exception {
        List<String> csv = new ArrayList<>(); csv.add("distance_in,velocity_in_s,acceleration_in_s2");
        for (MotionParameters p : knots(path)) csv.add(String.format(Locale.US, "%.8f,%.8f,%.8f",
                p.getDistAlongCurve(), p.getTangentialVel(), p.getTangentialAccel()));
        Files.write(new File(output, fixture + "-profile.csv").toPath(), csv, StandardCharsets.UTF_8);
    }
    private static void graph(File output, Fixture fixture, List<Run> runs, Run trigger) throws Exception {
        if (trigger.policy != Policy.CONTINUOUS_CONSTANT_ACCEL) return;
        for (boolean power : new boolean[]{false, true}) {
            XYChart chart = new XYChartBuilder().width(1200).height(500)
                    .title(fixture + " — 20 ms loop, 40 ms velocity lag")
                    .xAxisTitle("Distance along straight line (in)")
                    .yAxisTitle(power ? "Motor power" : "Speed (in/s)").build();
            chart.getStyler().setLegendPosition(Styler.LegendPosition.InsideNW);
            chart.getStyler().setDefaultSeriesRenderStyle(org.knowm.xchart.XYSeries.XYSeriesRenderStyle.Line);
            for (Run run : runs) if (run.dt == .02 && run.sensorCase == 1) {
                double[] x = new double[run.rows.size()], y = new double[x.length];
                for (int i = 0; i < x.length; i++) { x[i] = run.rows.get(i)[1]; y[i] = run.rows.get(i)[power ? 7 : 5]; }
                chart.addSeries(run.policy.toString(), x, y);
                if (!power && run.policy == Policy.CURRENT) {
                    double[] target = new double[x.length];
                    for (int i = 0; i < x.length; i++) target[i] = run.rows.get(i)[2];
                    chart.addSeries("Shared profile reference", x, target);
                }
            }
            BitmapEncoder.saveBitmap(chart, new File(output, fixture + (power ? "-power" : "-velocity")).toString(),
                    BitmapEncoder.BitmapFormat.PNG);
        }
    }
}
