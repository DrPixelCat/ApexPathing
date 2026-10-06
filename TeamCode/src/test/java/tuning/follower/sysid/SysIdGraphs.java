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
import org.knowm.xchart.style.markers.SeriesMarkers;

/** XChart PNGs using the same rendering and styling as PathProfileConvergenceTest. */
public final class SysIdGraphs {
    private SysIdGraphs() { }
    public static void main(String[] args) throws Exception {
        if (args.length != 2 && args.length != 3) { throw new IllegalArgumentException("Usage: SysIdGraphs <fit.csv> <DRIVE|HEADING> [KV]"); }
        System.out.println(args.length == 3 && args[2].equals("KV")
                ? renderQuasistaticKV(args[0], args[1]) : render(args[0], args[1]));
    }

    /** Diagnostic simple regressions of raw quasistatic power against velocity, one per direction. */
    public static String renderQuasistaticKV(String csvPath, String axis) throws Exception {
        List<double[]> rows = readRows(csvPath);
        XYChart plot = new XYChartBuilder().width(1100).height(650)
                .title(axis + " kV diagnostic: quasistatic samples only")
                .xAxisTitle(axis.equals("DRIVE") ? "Signed velocity (in/s)" : "Signed velocity (rad/s)")
                .yAxisTitle("Applied motor power (acceleration term not removed)").build();
        plot.getStyler().setDefaultSeriesRenderStyle(XYSeries.XYSeriesRenderStyle.Scatter);
        plot.getStyler().setMarkerSize(6);
        plot.getStyler().setLegendPosition(Styler.LegendPosition.OutsideE);
        plot.getStyler().setChartPadding(16);
        plot.getStyler().setXAxisDecimalPattern("0.##");
        plot.getStyler().setYAxisDecimalPattern("0.##");
        List<String> summary = new ArrayList<>();
        summary.add("axis,direction,samples,intercept,kv_slope,r_squared");
        for (int run = 0; run < 2; run++) {
            List<double[]> selected = new ArrayList<>();
            for (double[] row : rows) { if ((int) row[0] == run) { selected.add(row); } }
            if (selected.size() < 2) { continue; }
            double sumV = 0, sumP = 0, minV = Double.POSITIVE_INFINITY, maxV = Double.NEGATIVE_INFINITY;
            for (double[] row : selected) {
                sumV += row[2]; sumP += row[4];
                minV = Math.min(minV, row[2]); maxV = Math.max(maxV, row[2]);
            }
            double meanV = sumV / selected.size(), meanP = sumP / selected.size();
            double vv = 0, vp = 0, total = 0;
            for (double[] row : selected) {
                vv += (row[2] - meanV) * (row[2] - meanV);
                vp += (row[2] - meanV) * (row[4] - meanP);
                total += (row[4] - meanP) * (row[4] - meanP);
            }
            if (vv == 0) { continue; }
            double slope = vp / vv, intercept = meanP - slope * meanV, squaredError = 0;
            for (double[] row : selected) { squaredError += Math.pow(row[4] - intercept - slope * row[2], 2); }
            double r2 = total > 0 ? 1 - squaredError / total : Double.NaN;
            String direction = run == 0 ? "Forward" : "Reverse";
            plot.addSeries(direction + " quasistatic samples", column(selected, 2), column(selected, 4));
            plot.addSeries(String.format(java.util.Locale.US, "%s fit: kV = %.6f; R² = %.5f", direction, slope, r2),
                    new double[] {minV, maxV}, new double[] {intercept + slope * minV, intercept + slope * maxV})
                    .setXYSeriesRenderStyle(XYSeries.XYSeriesRenderStyle.Line).setMarker(SeriesMarkers.NONE).setLineWidth(2);
            summary.add(axis + "," + direction + "," + selected.size() + "," + intercept + "," + slope + "," + r2);
        }
        String prefix = csvPath.substring(0, csvPath.length() - 4) + "_kv_quasistatic";
        BitmapEncoder.saveBitmap(plot, prefix + "_regression", BitmapEncoder.BitmapFormat.PNG);
        Files.write(new File(prefix + "_fits.csv").toPath(), summary);
        return prefix + "_regression.png";
    }
    public static String render(String csvPath, String axis) throws Exception {
        List<double[]> rows = readRows(csvPath);
        String prefix = csvPath.substring(0, csvPath.length() - 4);
        renderRegressionScatter(rows, prefix, axis);
        String[] names = {"Quasistatic forward", "Quasistatic reverse", "Dynamic forward", "Dynamic reverse"};
        for (int run = 0; run < names.length; run++) {
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
        return prefix + "_*_power.png / *_motion.png / *_residual.png / _regression_scatter.png / _regression_parity.png";
    }

    private static List<double[]> readRows(String csvPath) throws Exception {
        List<double[]> rows = new ArrayList<>();
        List<String> lines = Files.readAllLines(new File(csvPath).toPath());
        for (int i = 1; i < lines.size(); i++) {
            String[] fields = lines.get(i).split(",");
            double[] row = new double[11];
            for (int j = 0; j < row.length; j++) { row[j] = Double.parseDouble(fields[j]); }
            rows.add(row);
        }
        return rows;
    }

    /** Project the three-term fit into velocity/power space by removing its acceleration term. */
    private static void renderRegressionScatter(List<double[]> rows, String prefix, String axis) throws Exception {
        if (rows.isEmpty()) { return; }
        double[] first = rows.get(0);
        double ks = first[7], kv = first[8], ka = first[9], r2 = first[10];
        if (!finite(new double[] {ks, kv, ka, r2})) { return; }
        String coefficients = String.format(java.util.Locale.US,
                "kS = %.5f    kV = %.6f    kA = %.6f    Full-fit R² = %.5f", ks, kv, ka, r2);
        XYChart scatter = new XYChartBuilder().width(1100).height(650)
                .title(axis + " SysId regression | " + coefficients)
                .xAxisTitle(axis.equals("DRIVE") ? "Signed velocity (in/s)" : "Signed velocity (rad/s)")
                .yAxisTitle("Applied power - kA × acceleration").build();
        XYChart parity = new XYChartBuilder().width(1100).height(650)
                .title(axis + " SysId power fit | " + coefficients)
                .xAxisTitle("Predicted power: kS × sign(v) + kV × v + kA × a")
                .yAxisTitle("Measured applied power").build();
        for (XYChart plot : new XYChart[] {scatter, parity}) {
            plot.getStyler().setDefaultSeriesRenderStyle(XYSeries.XYSeriesRenderStyle.Scatter);
            plot.getStyler().setMarkerSize(5);
            plot.getStyler().setLegendPosition(Styler.LegendPosition.OutsideE);
            plot.getStyler().setChartPadding(16);
            plot.getStyler().setXAxisDecimalPattern("0.##");
            plot.getStyler().setYAxisDecimalPattern("0.##");
        }
        double minV = Double.POSITIVE_INFINITY, maxV = Double.NEGATIVE_INFINITY;
        double minP = Double.POSITIVE_INFINITY, maxP = Double.NEGATIVE_INFINITY;
        String[] names = {"Quasistatic forward", "Quasistatic reverse", "Dynamic forward", "Dynamic reverse"};
        for (int run = 0; run < names.length; run++) {
            List<double[]> selected = new ArrayList<>();
            for (double[] row : rows) { if ((int) row[0] == run) { selected.add(row); } }
            if (selected.isEmpty()) { continue; }
            double[] velocity = column(selected, 2), adjusted = new double[selected.size()];
            for (int i = 0; i < selected.size(); i++) {
                double[] row = selected.get(i);
                adjusted[i] = row[4] - ka * row[3];
                minV = Math.min(minV, row[2]); maxV = Math.max(maxV, row[2]);
                minP = Math.min(minP, Math.min(row[4], row[5]));
                maxP = Math.max(maxP, Math.max(row[4], row[5]));
            }
            scatter.addSeries(names[run], velocity, adjusted);
            parity.addSeries(names[run], column(selected, 5), column(selected, 4));
        }
        if (maxV > 0) {
            scatter.addSeries("Forward: kS + kV × v", new double[] {0, maxV},
                    new double[] {ks, ks + kv * maxV}).setXYSeriesRenderStyle(XYSeries.XYSeriesRenderStyle.Line).setMarker(SeriesMarkers.NONE)
                    .setLineWidth(2.0f);
        }
        if (minV < 0) {
            scatter.addSeries("Reverse: -kS + kV × v", new double[] {minV, 0},
                    new double[] {-ks + kv * minV, -ks}).setXYSeriesRenderStyle(XYSeries.XYSeriesRenderStyle.Line).setMarker(SeriesMarkers.NONE)
                    .setLineWidth(2.0f);
        }
        parity.addSeries("Perfect agreement", new double[] {minP, maxP}, new double[] {minP, maxP})
                .setXYSeriesRenderStyle(XYSeries.XYSeriesRenderStyle.Line).setMarker(SeriesMarkers.NONE);
        BitmapEncoder.saveBitmap(scatter, prefix + "_regression_scatter", BitmapEncoder.BitmapFormat.PNG);
        BitmapEncoder.saveBitmap(parity, prefix + "_regression_parity", BitmapEncoder.BitmapFormat.PNG);
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
