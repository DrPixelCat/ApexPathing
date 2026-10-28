package setup;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import drivetrains.BaseDrivetrain;
import geometry.Pose;
import localizers.BaseLocalizer;
import localizers.BaseLocalizerConstants;
import localizers.Pinpoint;
import localizers.ThreeWheel;
import localizers.TwoWheel;
import localizers.util.AdaptiveKalmanFilter;
import org.json.JSONObject;
import java.util.ArrayList;
import java.util.List;
import java.util.TreeSet;

/** Android hardware adapter. Motion is limited to an explicit selected-motor hold test. */
public class SdkSetupHardware implements SetupHardware {
    protected final HardwareMap map;
    protected BaseDrivetrain<?> drivetrain;
    protected BaseLocalizer<?> localizer;
    protected BaseLocalizerConstants<?> config;
    protected RobotProfile profile;

    private DcMotorEx testMotor;
    private DcMotorSimple.Direction priorDirection;
    private DcMotor.RunMode priorMode;
    private DcMotor.ZeroPowerBehavior priorZeroPowerBehavior;

    @Override public MotorTestReading testMotor(String name, boolean reversed) {
        DcMotorEx selected = map.get(DcMotorEx.class, name);
        DcMotorSimple.Direction direction = reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD;
        if (selected != testMotor || selected.getDirection() != direction) {
            stop();
            priorDirection = selected.getDirection(); priorMode = selected.getMode();
            // Returning to reset mode would discard the encoder data captured by this test.
            if (priorMode == DcMotor.RunMode.STOP_AND_RESET_ENCODER) { priorMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER; }
            priorZeroPowerBehavior = selected.getZeroPowerBehavior();
            testMotor = selected;
            selected.setPower(0);
            selected.setDirection(direction);
            selected.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            selected.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        testMotor.setPower(0.5);
        double voltage = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : map.voltageSensor) {
            double reading = sensor.getVoltage();
            if (Double.isFinite(reading) && reading > 0) { voltage = Math.min(voltage, reading); }
        }
        return new MotorTestReading(testMotor.getPower(), testMotor.getCurrentPosition(), testMotor.getVelocity(),
                Double.isFinite(voltage) ? voltage : Double.NaN);
    }

    private void stopTestMotor() {
        if (testMotor == null) { return; }
        testMotor.setPower(0);
        try { testMotor.setMode(priorMode); }
        finally {
            try { testMotor.setDirection(priorDirection); }
            finally { testMotor.setZeroPowerBehavior(priorZeroPowerBehavior); }
        }
        testMotor = null;
    }

    public SdkSetupHardware(HardwareMap map) { this.map = map; }
    @Override public List<String> devices(String role) {
        Class<? extends HardwareDevice> type = role.equals("motor") ? DcMotorEx.class
                : role.equals("imu") ? IMU.class : Pinpoint.Driver.class;
        TreeSet<String> names = new TreeSet<>();
        // Enumeration must not initialize every sensor while the operator is still choosing it.
        for (HardwareDevice device : map.unsafeIterable()) {
            if (type.isInstance(device)) { names.addAll(map.getNamesOf(device)); }
        }
        return new ArrayList<>(names);
    }

    @Override public void prepare(RobotProfile next) {
        stop();
        profile = next.copy();
        profile.validate();
        drivetrain = SetupConfigurations.drivetrain(profile).build(map);
        drivetrain.stop();
        config = SetupConfigurations.localizer(profile);
        localizer = config.build(map);
        SetupConfigurations.applyFilters(profile, localizer);
        learn(false);
    }
    public BaseLocalizer<?> getLocalizer() { return localizer; }

    @Override public SetupSnapshot update() {
        stop();
        localizer.update();
        SetupSnapshot s = new SetupSnapshot();
        s.pose = pose(localizer.getPose()); s.rawPose = pose(localizer.getPose());
        s.velocity = pose(localizer.getVel()); s.rawVelocity = pose(localizer.getRawVel());
        s.acceleration = pose(localizer.getAccel()); s.rawAcceleration = pose(localizer.getRawAccel());
        s.valid = !(localizer instanceof Pinpoint) || ((Pinpoint) localizer).isSetupSampleValid();
        for (double coordinate : s.rawPose) { s.valid &= Double.isFinite(coordinate); }
        if (localizer instanceof Pinpoint) { s.ticks = ((Pinpoint) localizer).getPodTicks(); }
        else if (localizer instanceof TwoWheel) { s.ticks = ((TwoWheel) localizer).getPodTicks(); }
        else { s.ticks = ((ThreeWheel) localizer).getPodTicks(); }
        s.pods = new double[s.ticks.length];
        String[] signs = profile.threeWheel() ? new String[]{"forwardReverse", "rightReverse", "strafeReverse"}
                : new String[]{"forwardReverse", "strafeReverse"};
        for (int i = 0; i < s.ticks.length; i++) { s.pods[i] = s.ticks[i] * (profile.flag(signs[i]) ? -1.0 : 1.0) / profile.number("ticksPerInch"); }
        AdaptiveKalmanFilter.KalmanTuning[] tuning = {localizer.getXKalmanTuning(), localizer.getYKalmanTuning(), localizer.getHeadingKalmanTuning()};
        for (int i = 0; i < tuning.length; i++) {
            s.noise[i * 2] = tuning[i].measurementVariance;
            s.noise[i * 2 + 1] = tuning[i].processVariance;
        }
        addDiagnostics(s);
        double voltage = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : map.voltageSensor) {
            double reading = sensor.getVoltage();
            if (Double.isFinite(reading) && reading > 0) { voltage = Math.min(voltage, reading); }
        }
        s.battery = Double.isFinite(voltage) ? voltage : Double.NaN;
        s.diagnostics.put("fl_power", drivetrain.getLastFlPower()); s.diagnostics.put("fr_power", drivetrain.getLastFrPower());
        s.diagnostics.put("bl_power", drivetrain.getLastBlPower()); s.diagnostics.put("br_power", drivetrain.getLastBrPower());
        return s;
    }

    protected void addDiagnostics(SetupSnapshot s) {
        AdaptiveKalmanFilter.Diagnostics[] axes = localizer.getKalmanDiagnostics();
        for (int i = 0; i < axes.length; i++) {
            AdaptiveKalmanFilter.Diagnostics k = axes[i];
            if (k == null) { continue; }
            s.diagnostics.put(i + "_nis", k.nis); s.diagnostics.put(i + "_smoothed_nis", k.smoothedNis);
            s.diagnostics.put(i + "_innovation", k.innovation); s.diagnostics.put(i + "_innovation_variance", k.innovationVariance);
            s.diagnostics.put(i + "_velocity_variance", k.velocityVariance); s.diagnostics.put(i + "_acceleration_variance", k.accelerationVariance);
            s.diagnostics.put(i + "_adapting", k.adapting ? 1.0 : 0.0); s.diagnostics.put(i + "_bootstrap_complete", k.bootstrapComplete ? 1.0 : 0.0);
            s.diagnostics.put(i + "_filter_reports_tuned", k.tuned ? 1.0 : 0.0); s.diagnostics.put(i + "_time_anomaly", k.timeAnomaly ? 1.0 : 0.0);
        }
    }
    protected static double[] pose(Pose p) { return new double[]{p.getX().getIn(), p.getY().getIn(), p.getHeading().getRad()}; }
    @Override public void learn(boolean enabled) {
        if (localizer != null) { localizer.setKalmanAutoTuning(enabled, enabled && profile.holonomic(), enabled); }
    }
    @Override public JSONObject captureFilter() { return SetupConfigurations.captureFilters(localizer); }
    @Override public void stop() {
        try { stopTestMotor(); } finally { if (drivetrain != null) { drivetrain.stop(); } }
    }
}
