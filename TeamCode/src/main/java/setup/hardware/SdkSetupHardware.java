package setup.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import drivetrains.BaseDrivetrain;
import paths.geometry.Pose;
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
    @Override public double steeringAngleDegrees(String name) {
        AnalogInput encoder = map.get(AnalogInput.class, name);
        double voltage = encoder.getVoltage(), maximum = encoder.getMaxVoltage();
        if (!Double.isFinite(voltage) || !Double.isFinite(maximum) || maximum <= 0 || voltage < 0 || voltage > maximum) {
            throw new IllegalStateException("Steering encoder voltage is invalid");
        }
        return voltage * 360 / maximum;
    }
    @Override public List<String> devices(String role) {
        Class<? extends HardwareDevice> type = role.equals("motor") ? DcMotorEx.class
                : role.equals("imu") ? IMU.class : role.equals("servo") ? Servo.class
                : role.equals("crservo") ? CRServo.class : role.equals("analog") ? AnalogInput.class : Pinpoint.Driver.class;
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
        for (SetupField field : SetupField.forProfile(profile, this)) {
            if (!field.role.isEmpty() && !field.accepts(profile.get(field.key))) {
                throw new IllegalArgumentException("Configure the selected " + field.label + " in the Robot Controller first");
            }
        }
        // Localization setup must not deploy dual-drive actuators or run steering controllers.
        // Normal OpModes build the saved drivetrain; hold tests initialize just the selected motor.
        SetupConfigurations.drivetrain(profile).validate();
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
        s.pose = pose(localizer.getPose()); s.rawPose = pose(localizer.getRawPose());
        s.velocity = pose(localizer.getVel()); s.rawVelocity = pose(localizer.getRawVel());
        s.acceleration = pose(localizer.getAccel()); s.rawAcceleration = pose(localizer.getRawAccel());
        s.valid = localizer.isLastMeasurementValid();
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
        for (String key : profile.motorKeys()) { s.diagnostics.put(key + "_power", 0.0); }
        return s;
    }

    protected void addDiagnostics(SetupSnapshot s) {
        double[][] covariance = localizer.getKinematicCovariance();
        for (int i = 0; i < covariance.length; i++) {
            for (int j = 0; j < covariance[i].length; j++) { s.diagnostics.put("kinematic_covariance_" + i + "_" + j, covariance[i][j]); }
        }
        localizers.util.PoseEstimator.Tuning tuning = localizer.getPoseKalmanTuning();
        AdaptiveKalmanFilter.KalmanTuning[] axes = {tuning.x, tuning.y, tuning.heading};
        for (int i = 0; i < axes.length; i++) {
            s.diagnostics.put(i + "_pose_R", axes[i].measurementVariance);
            s.diagnostics.put(i + "_pose_Q", axes[i].processVariance);
            s.diagnostics.put(i + "_pose_adapting", localizer.isPoseAutoTuning(i) ? 1.0 : 0.0);
        }
        s.diagnostics.put("measurement_valid", localizer.isLastMeasurementValid() ? 1.0 : 0.0);
    }
    protected static double[] pose(Pose p) { return new double[]{p.getX().getIn(), p.getY().getIn(), p.getHeading().getRad()}; }
    @Override public void learn(boolean enabled) {
        if (localizer != null) {
            localizer.setKalmanAutoTuning(enabled, enabled && profile.holonomic(), enabled);
            localizer.setPoseAutoTuning(enabled, enabled && profile.holonomic(), enabled);
        }
    }
    @Override public JSONObject captureFilter() {
        JSONObject result = SetupConfigurations.captureFilters(localizer);
        localizers.LocalizationConstants nativeCalibration = localizers.LocalizationConstants.empty();
        nativeCalibration.capture("DEFAULT", config, localizer, localizers.LocalizationConstants.Status.ACCEPTED,
                localizers.LocalizationConstants.Status.ACCEPTED);
        try { result.put("poseCalibration", nativeCalibration.toJson()); }
        catch (Exception e) { throw new IllegalStateException("Could not capture the complete estimator", e); }
        return result;
    }
    @Override public void stop() {
        try { stopTestMotor(); }
        finally { if (drivetrain != null) { drivetrain.stop(); } }
    }
}
