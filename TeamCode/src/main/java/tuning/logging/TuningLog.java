package tuning.logging;

import java.io.Closeable;
import java.io.File;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.LinkedHashMap;
import java.util.Locale;
import java.util.Map;
import java.util.UUID;

import core.Follower;
import core.FollowerConstants;
import drivetrains.BaseDrivetrain;
import geometry.Pose;
import localizers.BaseLocalizer;
import localizers.util.AdaptiveKalmanFilter;

/** One folder per OpMode run. All tables share a monotonic time origin. */
public final class TuningLog implements Closeable {
    private final File directory;
    private final Map<String, CsvLog> files = new LinkedHashMap<>();
    private final long startNanos = System.nanoTime();
    private long previousSampleNanos;
    private long sequence;

    public TuningLog(File parent, String name) {
        String stamp = new SimpleDateFormat("yyyyMMdd-HHmmss-SSS", Locale.US).format(new Date());
        directory = new File(parent, name + "-" + stamp + "-" + UUID.randomUUID().toString().substring(0, 8));
        if (!directory.mkdirs()) { throw new IllegalStateException("Cannot create CSV folder: " + directory); }
    }

    public File getDirectory() { return directory; }
    public double seconds() { return (System.nanoTime() - startNanos) * 1e-9; }

    public void write(String table, String[] header, Object... values) {
        CsvLog csv = files.get(table);
        if (csv == null) {
            try {
                csv = new CsvLog(new File(directory, table + ".csv"), header);
                files.put(table, csv);
            } catch (IOException e) {
                throw new IllegalStateException("Cannot open tuning CSV " + table, e);
            }
        }
        csv.row(values);
    }

    public void event(String phase, String event, String detail) {
        write("events", new String[]{"time_s", "phase", "event", "detail"}, seconds(), phase, event, detail);
        flush();
    }

    public void coefficients(FollowerConstants c, String phase, String stage) {
        write("coefficients", new String[]{"time_s", "phase", "stage", "axis", "kS", "kV", "kA"},
                seconds(), phase, stage, "translation", c.translationalCoeffs.kS, c.translationalKV, c.translationalKA);
        write("coefficients", new String[]{"time_s", "phase", "stage", "axis", "kS", "kV", "kA"},
                seconds(), phase, stage, "angular", c.angularCoeffs.kS, c.angularKV, c.angularKA);
        flush();
    }

    public void sample(Follower follower, String phase, String mode, double batteryVolts) {
        long now = System.nanoTime();
        double dt = previousSampleNanos == 0 ? Double.NaN : (now - previousSampleNanos) * 1e-9;
        previousSampleNanos = now;
        double time = (now - startNanos) * 1e-9;
        long id = sequence++;
        BaseLocalizer<?> l = follower.getLocalizer();
        Pose p = l.getPose(), raw = l.getRawVel(), filtered = l.getVel();
        Pose rawA = l.getRawAccel(), filteredA = l.getAccel();
        BaseDrivetrain<?> d = follower.getDrivetrain();
        FollowerConstants c = follower.getConstants();
        write("motion", new String[]{"sample", "time_s", "loop_dt_s", "phase", "mode", "localizer",
                        "filter", "battery_v", "x_in", "y_in", "heading_rad", "raw_vx_in_s", "raw_vy_in_s",
                        "raw_omega_rad_s", "filtered_vx_in_s", "filtered_vy_in_s", "filtered_omega_rad_s",
                        "raw_ax_in_s2", "raw_ay_in_s2", "raw_alpha_rad_s2", "filtered_ax_in_s2", "filtered_ay_in_s2",
                        "filtered_alpha_rad_s2", "fl_power", "fr_power", "bl_power", "br_power", "max_power",
                        "translation_kS", "translation_kV", "translation_kA", "angular_kS", "angular_kV", "angular_kA"},
                id, time, dt, phase, mode, l.getClass().getSimpleName(), l.getVelocityFilterMode(), batteryVolts,
                p.getX().getIn(), p.getY().getIn(), p.getHeading().getRad(),
                raw.getX().getIn(), raw.getY().getIn(), raw.getHeading().getRad(),
                filtered.getX().getIn(), filtered.getY().getIn(), filtered.getHeading().getRad(),
                rawA.getX().getIn(), rawA.getY().getIn(), rawA.getHeading().getRad(),
                filteredA.getX().getIn(), filteredA.getY().getIn(), filteredA.getHeading().getRad(),
                d.getLastFlPower(), d.getLastFrPower(), d.getLastBlPower(), d.getLastBrPower(), d.getConstants().maxPower,
                c.translationalCoeffs.kS, c.translationalKV, c.translationalKA, c.angularCoeffs.kS, c.angularKV, c.angularKA);

        AdaptiveKalmanFilter.Diagnostics[] diagnostics = l.getKalmanDiagnostics();
        String[] axes = {"x", "y", "heading"};
        double[] rv = {raw.getX().getIn(), raw.getY().getIn(), raw.getHeading().getRad()};
        double[] fv = {filtered.getX().getIn(), filtered.getY().getIn(), filtered.getHeading().getRad()};
        double[] ra = {rawA.getX().getIn(), rawA.getY().getIn(), rawA.getHeading().getRad()};
        double[] fa = {filteredA.getX().getIn(), filteredA.getY().getIn(), filteredA.getHeading().getRad()};
        for (int axis = 0; axis < diagnostics.length; axis++) {
            AdaptiveKalmanFilter.Diagnostics k = diagnostics[axis];
            if (k == null) { continue; }
            write("kalman", new String[]{"sample", "time_s", "phase", "axis", "raw_velocity", "filtered_velocity",
                            "raw_acceleration", "filtered_acceleration", "initialized", "adapting", "bootstrap_complete",
                            "filter_reports_tuned", "filter_samples", "filter_dt_s", "R", "Q", "innovation",
                            "innovation_variance", "nis", "smoothed_nis", "velocity_variance", "acceleration_variance",
                            "inside_gate", "time_anomaly"},
                    id, time, phase, axes[axis], rv[axis], fv[axis], ra[axis], fa[axis], k.initialized,
                    k.adapting, k.bootstrapComplete, k.tuned, k.samples, k.dt, k.measurementVariance,
                    k.processVariance, k.innovation, k.innovationVariance, k.nis, k.smoothedNis,
                    k.velocityVariance, k.accelerationVariance, k.insideGate, k.timeAnomaly);
        }
    }

    public void flush() { for (CsvLog csv : files.values()) { csv.flush(); } }

    @Override public void close() {
        IOException failure = null;
        for (CsvLog csv : files.values()) {
            try { csv.close(); } catch (IOException e) { failure = e; }
        }
        if (failure != null) { throw new IllegalStateException("Cannot close tuning CSV files", failure); }
    }
}
