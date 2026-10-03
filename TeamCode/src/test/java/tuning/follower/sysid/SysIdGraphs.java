package tuning.follower.sysid;

import java.io.File;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.List;
import org.knowm.xchart.BitmapEncoder;
import org.knowm.xchart.XYChart;
import org.knowm.xchart.XYChartBuilder;
import org.knowm.xchart.XYSeries;
import org.knowm.xchart.style.Styler;

/** XChart PNGs using the same rendering and styling as PathProfileConvergenceTest. */
public final class SysIdGraphs {
    private SysIdGraphs() { }
    public static void main(String[] args) throws Exception {
        if (args.length != 2) { throw new IllegalArgumentException("Usage: SysIdGraphs <fit.csv> <DRIVE|HEADING>"); }
        System.out.println(render(args[0], args[1]));
    }
    public static String render(String csvPath, String axis) throws Exception {
        List<double[]> rows = new ArrayList<>();
        List<String> lines = Files.readAllLines(new File(csvPath).toPath());
        for (int i = 1; i < lines.size(); i++) {
            String[] fields = lines.get(i).split(",");
            double[] row = new double[11];
            for (int j = 0; j < row.length; j++) { row[j] = Double.parseDouble(fields[j]); }
            rows.add(row);
        }
        String prefix = csvPath.substring(0, csvPath.length() - 4);
        String[] names = {"Quasistatic forward", "Quasistatic reverse", "Dynamic forward", "Dynamic reverse"};
        for (int run = 0; run < 4; run++) {
            List<double[]> selected = new ArrayList<>();
            for (double[] row : rows) { if ((int) row[0] == run) { selected.add(row); } }
            if (selected.isEmpty()) { continue; }
            double[] t = column(selected, 1);
            XYChart power = chart(axis + " SysId - " + names[run] + " - Power fit");
            power.setYAxisTitle("Normalized motor power");
            power.addSeries("Measured power", t, column(selected, 4));
            double[] predicted = column(selected, 5);
            if (finite(predicted)) { power.addSeries("Fitted power", t, predicted); }
            XYChart motion = chart(axis + " SysId - " + names[run] + " - Motion");
            motion.setYAxisGroupTitle(0, axis.equals("DRIVE") ? "Velocity (in/s)" : "Velocity (rad/s)");
            motion.setYAxisGroupTitle(1, axis.equals("DRIVE") ? "Acceleration (in/s^2)" : "Acceleration (rad/s^2)");
            motion.getStyler().setYAxisGroupPosition(1, Styler.YAxisPosition.Right);
            motion.addSeries("Measured velocity", t, column(selected, 2));
            motion.addSeries("Measured acceleration", t, column(selected, 3)).setYAxisGroup(1);
            BitmapEncoder.saveBitmap(power, prefix + "_" + run + "_power", BitmapEncoder.BitmapFormat.PNG);
            BitmapEncoder.saveBitmap(motion, prefix + "_" + run + "_motion", BitmapEncoder.BitmapFormat.PNG);
            double[] residual = column(selected, 6);
            if (finite(residual)) {
                XYChart error = chart(axis + " SysId - " + names[run] + " - Residual");
                error.setYAxisTitle("Measured minus fitted power");
                error.addSeries("Residual", t, residual);
                BitmapEncoder.saveBitmap(error, prefix + "_" + run + "_residual", BitmapEncoder.BitmapFormat.PNG);
            }
        }
        return prefix + "_*_power.png / *_motion.png / *_residual.png";
    }
    private static boolean finite(double[] data) {
        for (double value : data) { if (!Double.isFinite(value)) { return false; } }
        return true;
    }
    private static double[] column(List<double[]> rows, int index) {
        double[] result = new double[rows.size()];
        for (int i = 0; i < rows.size(); i++) { result[i] = rows.get(i)[index]; }
        return result;
    }
    private static XYChart chart(String title) {
        XYChart chart = new XYChartBuilder().width(1000).height(460).title(title).xAxisTitle("Time (s)").build();
        chart.getStyler().setLegendPosition(Styler.LegendPosition.OutsideE);
        chart.getStyler().setMarkerSize(0);
        chart.getStyler().setPlotGridLinesVisible(true);
        chart.getStyler().setPlotContentSize(0.92);
        chart.getStyler().setChartPadding(12);
        chart.getStyler().setXAxisDecimalPattern("0.##");
        chart.getStyler().setXAxisMaxLabelCount(8);
        chart.getStyler().setXAxisTickLabelsFormattingFunction(value ->
                Math.abs(value * 2 - Math.rint(value * 2)) < 1e-6
                        ? String.format(java.util.Locale.US, "%.1f", value) : "" );
        chart.getStyler().setYAxisDecimalPattern("0.###");
        chart.getStyler().setDefaultSeriesRenderStyle(XYSeries.XYSeriesRenderStyle.Line);
        chart.getStyler().setLegendSeriesLineLength(44);
        return chart;
    }
}
