package setup;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import drivetrains.BaseDrivetrainConstants;
import drivetrains.Mecanum;
import drivetrains.Motor;
import drivetrains.Tank;
import geometry.Dist;
import geometry.DistUnit;
import geometry.Vector;
import localizers.BaseLocalizer;
import localizers.BaseLocalizerConstants;
import localizers.Pinpoint;
import localizers.ThreeWheel;
import localizers.TwoWheel;
import localizers.util.AdaptiveKalmanFilter;
import org.json.JSONObject;

/** Converts the operator's profile into the same configuration objects used by normal OpModes. */
public final class SetupConfigurations {
    private SetupConfigurations() { }
    private static Motor motor(RobotProfile p, String name) {
        Motor motor = new Motor(p.get(name));
        return p.flag(name + "Reverse") ? motor.reverse() : motor;
    }
    public static BaseDrivetrainConstants<?> drivetrain(RobotProfile p) {
        p.validate();
        if (p.holonomic()) {
            return new Mecanum.Constants().setFrontLeftMotor(motor(p, "fl")).setFrontRightMotor(motor(p, "fr"))
                    .setBackLeftMotor(motor(p, "bl")).setBackRightMotor(motor(p, "br")).setRobotCentric(true);
        }
        Tank.Constants result = new Tank.Constants().setFrontLeftMotor(motor(p, "fl")).setFrontRightMotor(motor(p, "fr"));
        if (p.get("drive").equals("TANK_4")) { result.setBackLeftMotor(motor(p, "bl")).setBackRightMotor(motor(p, "br")); }
        return result;
    }
    public static BaseLocalizerConstants<?> localizer(RobotProfile p) {
        p.validate();
        if (p.pinpoint()) {
            return new Pinpoint.Constants().setName(p.get("device"))
                    .setOffsets(p.number("offsetX"), p.number("offsetY"), DistUnit.IN)
                    .setEncoderResolution(Dist.fromMm(p.number("ticksPerInch") / 25.4))
                    .setEncoderDirections(p.flag("forwardReverse") ? Pinpoint.EncoderDirection.REVERSED : Pinpoint.EncoderDirection.FORWARD,
                            p.flag("strafeReverse") ? Pinpoint.EncoderDirection.REVERSED : Pinpoint.EncoderDirection.FORWARD);
        }
        Vector offsets = Vector.of(p.number("offsetX"), p.number("offsetY"), DistUnit.IN);
        if (p.threeWheel()) {
            return new ThreeWheel.Constants().setForwardLeftPodName(p.get("forward")).setForwardRightPodName(p.get("right"))
                    .setStrafePodName(p.get("strafe")).setTicksPerInch(p.number("ticksPerInch")).setOffsets(offsets)
                    .setEncoderDirections(p.flag("forwardReverse"), p.flag("rightReverse"), p.flag("strafeReverse"));
        }
        return new TwoWheel.Constants().setForwardPodName(p.get("forward")).setStrafePodName(p.get("strafe"))
                .setIMUName(p.get("imu")).setHubOrientation(new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.valueOf(p.get("logo")),
                        RevHubOrientationOnRobot.UsbFacingDirection.valueOf(p.get("usb"))))
                .setTicksPerInch(p.number("ticksPerInch")).setOffsets(offsets)
                .setEncoderDirections(p.flag("forwardReverse"), p.flag("strafeReverse"));
    }

    public static void applyFilters(RobotProfile p, BaseLocalizer<?> localizer) {
        JSONObject velocity = p.filter.optJSONObject("velocity");
        if (velocity == null) { return; }
        localizer.setVelocityFilterMode(BaseLocalizer.VelocityFilterMode.ADAPTIVE_KALMAN);
        localizer.setXKalmanTuning(read(velocity, "x"));
        localizer.setYKalmanTuning(read(velocity, "y"));
        localizer.setHeadingKalmanTuning(read(velocity, "heading"));
        localizer.setIsTuning(false);
        JSONObject poseCalibration = p.filter.optJSONObject("poseCalibration");
        if (poseCalibration != null) {
            localizers.LocalizationConstants saved = localizers.LocalizationConstants.empty();
            saved.restore(poseCalibration);
            if (!saved.applyFilter("DEFAULT", localizerConfig(p), localizer)) {
                throw new IllegalArgumentException("Saved pose-estimator settings are invalid");
            }
        }
        localizer.setPoseAutoTuning(false);
    }

    private static BaseLocalizerConstants<?> localizerConfig(RobotProfile p) { return localizer(p); }

    /** Keeps the advanced localization tuner and guided setup on one saved robot profile. */
    public static RobotProfile withGeometry(RobotProfile source, BaseLocalizerConstants<?> config) {
        RobotProfile p = source.copy();
        if (config instanceof Pinpoint.Constants) {
            Pinpoint.Constants c = (Pinpoint.Constants) config;
            p.set("offsetX", Double.toString(c.offsets.getX().getIn())); p.set("offsetY", Double.toString(c.offsets.getY().getIn()));
            if (c.customEncoderResolution.getIn() > 0) { p.set("ticksPerInch", Double.toString(c.customEncoderResolution.getMm() * 25.4)); }
            p.set("forwardReverse", Boolean.toString(c.xPodDirection == Pinpoint.EncoderDirection.REVERSED));
            p.set("strafeReverse", Boolean.toString(c.yPodDirection == Pinpoint.EncoderDirection.REVERSED));
        } else if (config instanceof TwoWheel.Constants) {
            TwoWheel.Constants c = (TwoWheel.Constants) config;
            p.set("ticksPerInch", Double.toString(c.ticksPerInch));
            p.set("offsetX", Double.toString(c.offsets.getX().getIn())); p.set("offsetY", Double.toString(c.offsets.getY().getIn()));
            p.set("forwardReverse", Boolean.toString(c.forwardPodReversed)); p.set("strafeReverse", Boolean.toString(c.strafePodReversed));
        } else if (config instanceof ThreeWheel.Constants) {
            ThreeWheel.Constants c = (ThreeWheel.Constants) config;
            p.set("ticksPerInch", Double.toString(c.ticksPerInch));
            p.set("offsetX", Double.toString(c.offsets.getX().getIn())); p.set("offsetY", Double.toString(c.offsets.getY().getIn()));
            p.set("forwardReverse", Boolean.toString(c.forwardLeftPodReversed)); p.set("rightReverse", Boolean.toString(c.forwardRightPodReversed));
            p.set("strafeReverse", Boolean.toString(c.strafePodReversed));
        } else { throw new IllegalArgumentException("This localizer is not part of the guided profile"); }
        return p;
    }
    private static AdaptiveKalmanFilter.KalmanTuning read(JSONObject json, String axis) {
        try {
            JSONObject value = json.getJSONObject(axis);
            double r = value.getDouble("R"), q = value.getDouble("Q");
            if (!Double.isFinite(r) || !Double.isFinite(q) || r <= 0 || q <= 0) { throw new IllegalArgumentException("Invalid filter noise"); }
            return new AdaptiveKalmanFilter.KalmanTuning(r, q);
        } catch (Exception e) { throw new IllegalArgumentException("Saved filter settings are invalid", e); }
    }
    public static JSONObject captureFilters(BaseLocalizer<?> l) {
        try {
            return new JSONObject().put("velocity", new JSONObject().put("x", noise(l.getXKalmanTuning()))
                    .put("y", noise(l.getYKalmanTuning())).put("heading", noise(l.getHeadingKalmanTuning())));
        } catch (Exception e) { throw new IllegalStateException("Could not record filter settings", e); }
    }
    private static JSONObject noise(AdaptiveKalmanFilter.KalmanTuning tuning) throws org.json.JSONException {
        return new JSONObject().put("R", tuning.measurementVariance).put("Q", tuning.processVariance);
    }
}
