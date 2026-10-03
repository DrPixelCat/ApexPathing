package org.firstinspires.ftc.teamcode.sim;

import follower.FollowerConstants;
import org.codeblooded.ftcodesim.ascope.boundaries.MotionVector;
import org.codeblooded.ftcodesim.hardware.devices.SimMotor;
import org.junit.Test;
import org.knowm.xchart.BitmapEncoder;
import org.knowm.xchart.XYChart;
import org.knowm.xchart.XYChartBuilder;
import org.knowm.xchart.style.Styler;

import java.io.File;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;

import static org.junit.Assert.assertEquals;

/** Measures the stock plant's authority; never changes its equations or coefficients. */
public class StockAccelerationDiagnosticTest {
    @Test public void diagnoseAndGraphForwardAuthority() throws Exception {
        measureForwardAccelerationEnvelope();
        holdPlateauOnStockPlantWithAndWithoutFeedforwardBias();
        File output = new File("build/stock-ftcodesim-retuned-20261002/acceleration-diagnosis");
        XYChart authority = new XYChartBuilder().width(660).height(460)
                .title("Acceleration available at full motor power")
                .xAxisTitle("Forward speed (in/s)").yAxisTitle("Acceleration (in/s²)").build();
        List<Double> velocities = new ArrayList<>(), predicted = new ArrayList<>(), actual = new ArrayList<>();
        List<String> probeRows = Files.readAllLines(new File(output,"forward-authority.csv").toPath());
        for (String line : probeRows.subList(1,probeRows.size())) {
            String[] row = line.split(",");
            if (Double.parseDouble(row[1]) != 1) continue;
            velocities.add(Double.parseDouble(row[0])); predicted.add(Double.parseDouble(row[2]));
            actual.add(Double.parseDouble(row[3]));
        }
        authority.addSeries("Apex fitted prediction",velocities,predicted);
        authority.addSeries("Stock plant measurement",velocities,actual);
        authority.getStyler().setYAxisMin(0.0);
        authority.getStyler().setLegendPosition(Styler.LegendPosition.InsideNE);
        XYChart holding = new XYChartBuilder().width(660).height(460)
                .title("Plateau hold: unchanged stock plant")
                .xAxisTitle("Elapsed time (s)").yAxisTitle("Forward velocity (in/s)").build();
        List<String> rows = Files.readAllLines(new File(output,"plateau-hold.csv").toPath());
        for (String name : new String[]{"fitted holding power","measured holding power"}) {
            List<Double> times = new ArrayList<>(), speeds = new ArrayList<>();
            for (String line : rows.subList(1,rows.size())) {
                String[] row = line.split(",");
                if (!row[0].equals(name)) continue;
                times.add(Double.parseDouble(row[1])); speeds.add(Double.parseDouble(row[3]));
            }
            holding.addSeries(name,times,speeds);
        }
        double target = SimulatorTuning.snapshot().forwardVelLimitIn;
        holding.addSeries("Target 52.88 in/s",Arrays.asList(0.0,1.8),Arrays.asList(target,target));
        holding.getStyler().setMarkerSize(0);
        holding.getStyler().setLegendPosition(Styler.LegendPosition.InsideSE);
        File left = new File(output,"Full_Power_Authority"), right = new File(output,"Plateau_Hold");
        BitmapEncoder.saveBitmap(authority,left.getAbsolutePath(),BitmapEncoder.BitmapFormat.PNG);
        BitmapEncoder.saveBitmap(holding,right.getAbsolutePath(),BitmapEncoder.BitmapFormat.PNG);
    }

    private void holdPlateauOnStockPlantWithAndWithoutFeedforwardBias() throws Exception {
        FollowerConstants c = SimulatorTuning.snapshot();
        List<String> rows = new ArrayList<>();
        rows.add("case,time_s,target_in_s,actual_in_s,power");
        for (boolean corrected : new boolean[]{false,true}) {
            ApexSimulation.Hardware h = ApexSimulation.createHardware();
            SimMotor[] motors = StockSimTestSupport.motors(h);
            h.drivetrain.setPosition(new MotionVector(ApexSimulation.FIELD_CENTER_INCHES-50,
                    ApexSimulation.FIELD_CENTER_INCHES,0));
            StockSimTestSupport.setSpeed(h,44);
            double target = c.forwardVelLimitIn;
            double drive = motors[0].config.modelCoefficients[0]
                    * motors[0].config.voltageSensor.getVoltage() * h.drivetrain.config.wheelRadius;
            double emf = motors[0].config.modelCoefficients[1];
            // Input-only diagnostic: use the holding power established by the stock-state probes.
            // Nothing about the motor/chassis plant or production follower is replaced.
            double basePower = corrected ? 49/(drive-emf*target)
                    : c.translationalFeedforwardKS+c.translationalKV*target;
            for (int i=0;i<1800;i++) {
                double velocity = h.drivetrain.velocity.x;
                double power = Math.max(-1,Math.min(1,basePower+c.velocityFeedbackGain*(target-velocity)));
                for (SimMotor motor : motors) motor.setPower(power);
                if (i%20==0) rows.add(String.format(Locale.US,"%s,%.3f,%.9f,%.9f,%.9f",
                        corrected ? "measured holding power" : "fitted holding power",i*.001,target,velocity,power));
                ApexSimulation.stepPhysics(h,.001);
            }
            System.out.printf(Locale.US,"PLATEAU %s target=%.6f finalVelocity=%.6f basePower=%.6f%n",
                    corrected ? "measured holding power" : "fitted holding power",target,h.drivetrain.velocity.x,basePower);
        }
        File output = new File("build/stock-ftcodesim-retuned-20261002/acceleration-diagnosis");
        if (!output.isDirectory() && !output.mkdirs()) throw new IllegalStateException("Cannot create " + output);
        Files.write(new File(output,"plateau-hold.csv").toPath(),rows,StandardCharsets.UTF_8);
    }

    private void measureForwardAccelerationEnvelope() throws Exception {
        FollowerConstants c = SimulatorTuning.snapshot();
        ApexSimulation.Hardware h = ApexSimulation.createHardware();
        SimMotor[] motors = StockSimTestSupport.motors(h);
        System.out.println("STOCK MOTOR COEFFICIENTS " + Arrays.toString(motors[0].config.modelCoefficients));
        double motorDrive = motors[0].config.modelCoefficients[0]
                * motors[0].config.voltageSensor.getVoltage() * h.drivetrain.config.wheelRadius;
        double emf = motors[0].config.modelCoefficients[1];
        double drag = 49;
        List<String> rows = new ArrayList<>();
        rows.add("velocity_in_s,power,apex_predicted_accel_in_s2,stock_measured_accel_in_s2");
        for (double velocity : new double[]{10,20,30,40,50,c.forwardVelLimitIn}) {
            double holding = c.translationalFeedforwardKS + c.translationalKV * velocity;
            for (double power : new double[]{holding, Math.min(1,holding+c.translationalKA*60),1}) {
                h.drivetrain.setPosition(new MotionVector(ApexSimulation.FIELD_CENTER_INCHES,
                        ApexSimulation.FIELD_CENTER_INCHES,0));
                StockSimTestSupport.setSpeed(h,velocity);
                for (SimMotor motor : motors) motor.setPower(power);
                ApexSimulation.stepPhysics(h,0.0001);
                double actual = (h.drivetrain.velocity.x-velocity)/0.0001;
                double predicted = (power-holding)/c.translationalKA;
                // Check the loaded model's powered-forward equation against actual library stepping.
                assertEquals(power*(motorDrive-emf*velocity)-drag,actual,1e-7);
                rows.add(String.format(Locale.US,"%.9f,%.9f,%.9f,%.9f",velocity,power,predicted,actual));
            }
            System.out.printf(Locale.US,
                    "AUTHORITY v=%.3f fittedHolding=%.6f actualHolding=%.6f predictedMaxA=%.3f actualMaxA=%.3f%n",
                    velocity,holding,drag/(motorDrive-emf*velocity),
                    (1-holding)/c.translationalKA,motorDrive-emf*velocity-drag);
        }
        File output = new File("build/stock-ftcodesim-retuned-20261002/acceleration-diagnosis");
        if (!output.isDirectory() && !output.mkdirs()) throw new IllegalStateException("Cannot create " + output);
        Files.write(new File(output,"forward-authority.csv").toPath(),rows,StandardCharsets.UTF_8);
    }
}
