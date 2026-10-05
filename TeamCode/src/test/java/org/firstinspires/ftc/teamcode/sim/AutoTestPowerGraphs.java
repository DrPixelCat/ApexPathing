package org.firstinspires.ftc.teamcode.sim;

import org.knowm.xchart.BitmapEncoder;
import org.knowm.xchart.XYChart;
import org.knowm.xchart.XYChartBuilder;
import org.knowm.xchart.style.Styler;

import java.io.File;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/** Distance-aligned feedforward prediction versus delivered peak absolute wheel command. */
public final class AutoTestPowerGraphs {
    private AutoTestPowerGraphs() { }

    /** Render a robot or simulator AutoTest CSV on the desktop test classpath. */
    public static void main(String[] args) throws Exception {
        if (args.length < 1 || args.length > 2) {
            throw new IllegalArgumentException("Usage: AutoTestPowerGraphs <trace.csv> [output-directory]");
        }
        File trace = new File(args[0]);
        write(trace, args.length == 2 ? new File(args[1]) : trace.getAbsoluteFile().getParentFile(),
                "AutoTest_Power_Tracking");
    }

    public static File write(File trace, File directory, String name) throws Exception {
        return write(trace, directory, name, "AutoTest Outbound");
    }

    public static File write(File trace, File directory, String name, String title) throws Exception {
        List<String> lines = Files.readAllLines(trace.toPath());
        if (lines.size() < 2) { throw new IllegalArgumentException("Empty power trace: " + trace); }
        Map<String, Integer> columns = new HashMap<>();
        String[] headers = lines.get(0).split(",");
        for (int i = 0; i < headers.length; i++) { columns.put(headers[i], i); }
        String[] required = {"path_distance_in", "expected_motor_power", "command_power"};
        for (String column : required) {
            if (!columns.containsKey(column)) {
                throw new IllegalArgumentException("Missing " + column + "; record a fresh AutoTest trace.");
            }
        }
        List<Double> distance = new ArrayList<>(), expected = new ArrayList<>(), actual = new ArrayList<>();
        List<Double> residual = new ArrayList<>();
        double sum = 0, squared = 0, peak = 0;
        for (int i = 1; i < lines.size(); i++) {
            String[] row = lines.get(i).split(",");
            double s = Double.parseDouble(row[columns.get("path_distance_in")]);
            double prediction = Double.parseDouble(row[columns.get("expected_motor_power")]);
            double command = Double.parseDouble(row[columns.get("command_power")]);
            if (!Double.isFinite(s) || !Double.isFinite(prediction) || !Double.isFinite(command)) {
                throw new IllegalArgumentException("Nonfinite power trace at row " + (i + 1));
            }
            double error = command - prediction;
            distance.add(s); expected.add(prediction); actual.add(command); residual.add(error);
            sum += error; squared += error * error; peak = Math.max(peak, Math.abs(error));
        }
        if (!directory.isDirectory() && !directory.mkdirs()) {
            throw new IllegalStateException("Could not create graph directory " + directory);
        }
        XYChart power = chart(title + ": Expected vs Actual Motor Power", "Normalized peak motor power");
        power.addSeries("Expected (feedforward)", distance, expected);
        power.addSeries("Actual (motor command)", distance, actual);
        power.getStyler().setYAxisMin(0.0);
        XYChart error = chart(title + ": Power Prediction Error", "Actual minus expected power");
        error.addSeries("Prediction error", distance, residual);
        error.addSeries("Zero error", distance, java.util.Collections.nCopies(distance.size(), 0.0));
        File output = new File(directory, name + ".png");
        BitmapEncoder.saveBitmap(power, new File(directory, name).getAbsolutePath(), BitmapEncoder.BitmapFormat.PNG);
        BitmapEncoder.saveBitmap(error, new File(directory, name + "_Error").getAbsolutePath(), BitmapEncoder.BitmapFormat.PNG);
        System.out.printf(java.util.Locale.US,
                "POWER PREDICTION: samples=%d bias=%.6f RMSE=%.6f peakAbsoluteError=%.6f graph=%s%n",
                distance.size(), sum / distance.size(), Math.sqrt(squared / distance.size()), peak, output);
        return output;
    }

    private static XYChart chart(String title, String yTitle) {
        XYChart chart = new XYChartBuilder().width(1000).height(460).title(title)
                .xAxisTitle("Distance along path (in)").yAxisTitle(yTitle).build();
        chart.getStyler().setLegendPosition(Styler.LegendPosition.OutsideE);
        chart.getStyler().setMarkerSize(0);
        chart.getStyler().setPlotGridLinesVisible(true);
        return chart;
    }
}
