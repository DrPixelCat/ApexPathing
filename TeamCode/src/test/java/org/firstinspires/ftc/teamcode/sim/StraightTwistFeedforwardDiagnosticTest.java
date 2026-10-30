package org.firstinspires.ftc.teamcode.sim;

import feedforward.MotionPowerCalculator;
import drivetrains.models.DriveModels;
import follower.FollowerConstants;
import paths.geometry.Angle;
import paths.geometry.DistUnit;
import paths.geometry.Vector;
import org.codeblooded.ftcodesim.ascope.boundaries.MotionVector;
import org.codeblooded.ftcodesim.hardware.devices.SimMotor;
import org.firstinspires.ftc.teamcode.apexpathing.Constants;
import org.junit.Test;
import org.knowm.xchart.BitmapEncoder;
import org.knowm.xchart.XYChart;
import org.knowm.xchart.XYChartBuilder;
import org.knowm.xchart.style.Styler;

import java.io.File;
import java.io.PrintWriter;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import static org.junit.Assert.*;

/** Controlled stock-plant probes: reference states, no follower, allocator, feedback or wall-clock pacing. */
public class StraightTwistFeedforwardDiagnosticTest {
    private static final double DT = .0001;

    @Test public void isolateModelSwitchesAndCombinedAxisResponse() throws Exception {
        File directory = new File("build/path-power-graphs/straight-twist-diagnosis").getAbsoluteFile();
        Files.createDirectories(directory.toPath());
        FollowerConstants constants = SimulatorTuning.snapshot();
        MotionPowerCalculator calculator = DriveModels.mecanum(constants).powerCalculator();
        ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
        new Constants().drivetrainConstants().build(hardware.hardwareMap); // Normal motor brake configuration.
        SimMotor[] motors = StockSimTestSupport.motors(hardware);
        try (PrintWriter csv = new PrintWriter(Files.newBufferedWriter(new File(directory, "controlled-probes.csv").toPath()));
             PrintWriter summary = new PrintWriter(Files.newBufferedWriter(new File(directory, "summary.txt").toPath()))) {
            csv.println("case,vx_in_s,vy_in_s,omega_rad_s,power_x,power_y,power_turn," +
                    "left_front,right_front,left_back,right_back,actual_ax_in_s2,actual_ay_in_s2,actual_alpha_rad_s2");
            summary.println("Stock FTCodeSim; 0.1 ms probes at imposed reference states. No gain or plant changes.");
            summary.println("Motor commands are independently read and verified. No allocator or feedback is present.");

            // Separate angular acceleration from concurrent linear speed at identical omega/power.
            List<Double> speeds = new ArrayList<>(), isolated = new ArrayList<>(), combined = new ArrayList<>();
            double omega = .47;
            double turn = calculator.calculate(vector(1, 0), 0, 0, omega, 0, 0).heading;
            for (double speed : new double[]{0, 8, 16, 24, 32}) {
                Vector drive = calculator.calculate(vector(1, 0), speed, 0, 0, 0, 0).translation();
                Probe p = probe(hardware, motors, speed, 0, omega, drive.getX().getIn(), 0, turn);
                row(csv, "cruise-angular-coupling", speed, 0, omega, p);
                speeds.add(speed); combined.add(p.alpha);
                isolated.add(probe(hardware, motors, 0, 0, omega, 0, 0, turn).alpha);
                summary.printf(Locale.US, "angular cruise vx=%.1f omega=%.3f turn=%.6f ax=%.6f alpha=%.6f%n",
                        speed, omega, turn, p.ax, p.alpha);
            }
            writeChart(directory, "Angular_Coupling", "Same Angular Feedforward at Different Travel Speeds",
                    "Forward speed (in/s)", "Actual angular acceleration (rad/s2)", speeds,
                    "Rotation only", isolated, "Translation + rotation", combined);

            // Tiny negative alpha selects the legacy branch, not the measured response.
            double accelerating = calculator.calculate(vector(1, 0), 0, 0, omega, .001, 0).heading;
            double cruise = calculator.calculate(vector(1, 0), 0, 0, omega, 0, 0).heading;
            double braking = calculator.calculate(vector(1, 0), 0, 0, omega, -.001, 0).heading;
            for (double alpha : new double[]{.001, 0, -.001, -.2}) {
                double power = calculator.calculate(vector(1, 0), 0, 0, omega, alpha, 0).heading;
                for (double vx : new double[]{0, 16}) {
                    double x = vx == 0 ? 0 : calculator.calculate(vector(1, 0), vx, 0, 0, 0, 0).translation().getX().getIn();
                    Probe p = probe(hardware, motors, vx, 0, omega, x, 0, power);
                    row(csv, "angular-phase-switch-alpha=" + alpha, vx, 0, omega, p);
                    summary.printf(Locale.US, "angular phase vx=%.1f requestedAlpha=%.6f turn=%.6f actualAlpha=%.6f%n",
                            vx, alpha, power, p.alpha);
                }
            }
            summary.printf(Locale.US, "angular phase discontinuity accelerating=%.6f cruise=%.6f braking=%.6f drop=%.1f%%%n",
                    accelerating, cruise, braking, 100 * (cruise - braking) / cruise);

            // Independent strafe drag activates even for a tiny body-axis component.
            List<Double> headings = new ArrayList<>(), lateral = new ArrayList<>(), expectedLateral = new ArrayList<>();
            for (double heading : new double[]{0, .001, .1, 10, 30, 45, 90}) {
                double theta = Math.toRadians(heading);
                Vector direction = vector(Math.cos(theta), -Math.sin(theta));
                Vector drive = calculator.calculate(direction, 16, 0, 0, 0, 0).translation();
                Probe matching = probe(hardware, motors, 16 * Math.cos(theta), -16 * Math.sin(theta), 0,
                        drive.getX().getIn(), drive.getY().getIn(), 0);
                row(csv, "translation-matching-heading=" + heading, 16 * Math.cos(theta), -16 * Math.sin(theta), 0, matching);
                Vector field = drive.rotate(Angle.fromRad(theta));
                Probe lagging = probe(hardware, motors, 16, 0, 0, field.getX().getIn(), field.getY().getIn(), 0);
                row(csv, "translation-actual-heading-zero-target=" + heading, 16, 0, 0, lagging);
                summary.printf(Locale.US,
                        "translation targetHeading=%.3f bodyX=%.6f bodyY=%.6f matchingAlongAccel=%.6f " +
                                "matchingCrossAccel=%.6f laggingAx=%.6f laggingAy=%.6f%n",
                        heading, drive.getX().getIn(), drive.getY().getIn(),
                        matching.ax * Math.cos(theta) - matching.ay * Math.sin(theta),
                        matching.ax * Math.sin(theta) + matching.ay * Math.cos(theta), lagging.ax, lagging.ay);
                headings.add(heading); lateral.add(drive.getY().getIn()); expectedLateral.add(0.0);
            }
            writeChart(directory, "Tiny_Heading_Strafe_Power", "Strafe Feedforward for a Straight Field Path",
                    "Target heading (deg)", "Body strafe power", headings,
                    "Current feedforward", lateral, "At zero heading", expectedLateral);

            // A local response inversion, fitted only to probe results, confirms missing coupling.
            // It is diagnostic, not a persisted calibration or a controller using live error.
            double speed = 16;
            double x = calculator.calculate(vector(1, 0), speed, 0, 0, 0, 0).translation().getX().getIn();
            Probe baseline = probe(hardware, motors, speed, 0, omega, x, 0, cruise);
            double[] command = {x, 0, cruise};
            double[][] jacobian = new double[3][4];
            double delta = .005;
            double[] base = {baseline.ax, baseline.ay, baseline.alpha};
            for (int axis = 0; axis < 3; axis++) {
                double[] perturbed = command.clone(); perturbed[axis] += delta;
                Probe p = probe(hardware, motors, speed, 0, omega, perturbed[0], perturbed[1], perturbed[2]);
                double[] response = {p.ax, p.ay, p.alpha};
                for (int row = 0; row < 3; row++) { jacobian[row][axis] = (response[row] - base[row]) / delta; }
            }
            for (int row = 0; row < 3; row++) { jacobian[row][3] = -base[row]; }
            double[] correction = solve(jacobian);
            Probe resolved = probe(hardware, motors, speed, 0, omega,
                    command[0] + correction[0], command[1] + correction[1], command[2] + correction[2]);
            row(csv, "coupled-probe-inversion", speed, 0, omega, resolved);
            summary.printf(Locale.US,
                    "local coupled inversion baselineX=%.6f baselineTurn=%.6f correctedX=%.6f correctedY=%.6f correctedTurn=%.6f " +
                            "baselineAx=%.6f baselineAlpha=%.6f correctedAx=%.6f correctedAy=%.6f correctedAlpha=%.6f%n",
                    x, cruise, resolved.x, resolved.y, resolved.turn, baseline.ax, baseline.alpha,
                    resolved.ax, resolved.ay, resolved.alpha);
            assertTrue("A measured coupled inverse must remove the angular residual at the same state",
                    Math.abs(resolved.alpha) < Math.abs(baseline.alpha) * .1);
            assertTrue("A measured coupled inverse must remove the longitudinal residual at the same state",
                    Math.abs(resolved.ax) < Math.abs(baseline.ax) * .1);
        } finally {
            for (SimMotor motor : motors) { motor.setPower(0); }
        }
        System.out.println("STRAIGHT TWIST DIAGNOSIS: " + directory);
        System.out.println(new String(Files.readAllBytes(new File(directory, "summary.txt").toPath()), java.nio.charset.StandardCharsets.UTF_8));
    }

    private static Probe probe(ApexSimulation.Hardware hardware, SimMotor[] motors, double vx, double vy, double omega,
                                double x, double y, double turn) {
        hardware.drivetrain.setPosition(new MotionVector(ApexSimulation.FIELD_CENTER_INCHES, ApexSimulation.FIELD_CENTER_INCHES, 0));
        hardware.drivetrain.velocity = new MotionVector(vx, vy, omega);
        hardware.drivetrain.updateWheelRollVelocities();
        double[] power = {x - y - turn, x + y + turn, x + y - turn, x - y + turn};
        for (int i = 0; i < 4; i++) {
            assertTrue("Probe must remain below clipping", Math.abs(power[i]) <= 1);
            motors[i].setPower(power[i]);
            assertEquals("The commanded wheel demand must reach the motor", power[i], motors[i].getPower(), 1e-5);
        }
        ApexSimulation.stepPhysics(hardware, DT);
        Probe p = new Probe(); p.x = x; p.y = y; p.turn = turn; p.wheels = power;
        p.ax = (hardware.drivetrain.velocity.x - vx) / DT;
        p.ay = (hardware.drivetrain.velocity.y - vy) / DT;
        p.alpha = (hardware.drivetrain.velocity.theta - omega) / DT;
        assertTrue(Double.isFinite(p.ax) && Double.isFinite(p.ay) && Double.isFinite(p.alpha));
        return p;
    }

    private static void row(PrintWriter csv, String name, double vx, double vy, double omega, Probe p) {
        csv.printf(Locale.US, "%s,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f%n",
                name, vx, vy, omega, p.x, p.y, p.turn, p.wheels[0], p.wheels[1], p.wheels[2], p.wheels[3], p.ax, p.ay, p.alpha);
    }

    private static Vector vector(double x, double y) { return Vector.of(x, y, DistUnit.IN); }
    private static final class Probe { double x, y, turn, ax, ay, alpha; double[] wheels; }

    private static double[] solve(double[][] matrix) {
        for (int column = 0; column < 3; column++) {
            int pivot = column;
            for (int row = column + 1; row < 3; row++) {
                if (Math.abs(matrix[row][column]) > Math.abs(matrix[pivot][column])) { pivot = row; }
            }
            double[] swap = matrix[column]; matrix[column] = matrix[pivot]; matrix[pivot] = swap;
            assertTrue("Coupled probe response must be invertible", Math.abs(matrix[column][column]) > 1e-6);
            double scale = matrix[column][column];
            for (int i = column; i < 4; i++) { matrix[column][i] /= scale; }
            for (int row = 0; row < 3; row++) {
                if (row == column) { continue; }
                double factor = matrix[row][column];
                for (int i = column; i < 4; i++) { matrix[row][i] -= factor * matrix[column][i]; }
            }
        }
        return new double[]{matrix[0][3], matrix[1][3], matrix[2][3]};
    }

    private static void writeChart(File directory, String name, String title, String xTitle, String yTitle,
                                   List<Double> x, String firstName, List<Double> first,
                                   String secondName, List<Double> second) throws Exception {
        XYChart chart = new XYChartBuilder().width(1000).height(460).title(title).xAxisTitle(xTitle).yAxisTitle(yTitle).build();
        chart.getStyler().setLegendPosition(Styler.LegendPosition.OutsideE); chart.getStyler().setMarkerSize(0);
        chart.addSeries(firstName, x, first); chart.addSeries(secondName, x, second);
        BitmapEncoder.saveBitmap(chart, new File(directory, name).getAbsolutePath(), BitmapEncoder.BitmapFormat.PNG);
    }
}
