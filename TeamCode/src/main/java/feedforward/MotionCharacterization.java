package feedforward;

import org.json.JSONObject;

/** Immutable, independently validated responses for each axis and travel direction. */
public final class MotionCharacterization {
    public enum Axis { FORWARD, STRAFE, ANGULAR }
    public static final double PROFILE_POWER_LIMIT = .88;
    private final MotorPowerModel[][] models;
    public final double availableMotorPower;
    public final MecanumWheelModel mecanumWheelModel;

    public MotionCharacterization(MotorPowerModel[][] models, boolean holonomic) {
        this(models, holonomic, 1);
    }
    public MotionCharacterization(MotorPowerModel[][] models, boolean holonomic, double availableMotorPower) {
        this(models,holonomic,availableMotorPower,null);
    }
    public MotionCharacterization(MotorPowerModel[][] models, boolean holonomic, double availableMotorPower,
                                  MecanumWheelModel mecanumWheelModel) {
        if (!Double.isFinite(availableMotorPower) || availableMotorPower <= 0 || availableMotorPower > 1) {
            throw new IllegalArgumentException("Invalid calibrated motor-power limit");
        }
        this.availableMotorPower = availableMotorPower;
        if(mecanumWheelModel!=null && !holonomic) throw new IllegalArgumentException("Wheel model requires a mecanum calibration");
        this.mecanumWheelModel=mecanumWheelModel;
        if (models == null || models.length != Axis.values().length) {
            throw new IllegalArgumentException("Characterization requires three axis rows");
        }
        for (MotorPowerModel[] row : models) {
            if (row == null || row.length != 2) {
                throw new IllegalArgumentException("Each axis requires positive and negative direction models");
            }
        }
        this.models = new MotorPowerModel[3][2];
        for (Axis axis : Axis.values()) {
            for (int direction = 0; direction < 2; direction++) {
                MotorPowerModel model = models[axis.ordinal()][direction];
                if (model == null && (axis != Axis.STRAFE || holonomic)) {
                    throw new IllegalArgumentException("Missing " + axis + " characterization");
                }
                this.models[axis.ordinal()][direction] = model;
            }
        }
    }
    public double profilePowerLimit() { return PROFILE_POWER_LIMIT * availableMotorPower; }
    public MotorPowerModel model(Axis axis, double direction) {
        return models[axis.ordinal()][direction < 0 ? 1 : 0];
    }
    public double velocityLimit(Axis axis) {
        return velocityLimit(axis, profilePowerLimit());
    }
    /** Hard measured domain, independent of the acceleration-dependent power budget. */
    public double maximumMeasuredVelocity(Axis axis) {
        MotorPowerModel positive = model(axis, 1), negative = model(axis, -1);
        return positive == null || negative == null ? 0
                : Math.min(positive.maxVelocity, negative.maxVelocity);
    }
    /** Computes the measured cruising ceiling for the requested normalized power budget. */
    public double velocityLimit(Axis axis, double powerLimit) {
        if (!Double.isFinite(powerLimit) || powerLimit <= 0 || powerLimit > 1) {
            throw new IllegalArgumentException("Profile power limit must be finite and in (0, 1].");
        }
        double availablePower = Math.min(powerLimit, availableMotorPower);
        double result = Double.POSITIVE_INFINITY;
        for (int direction = 0; direction < 2; direction++) {
            MotorPowerModel m = models[axis.ordinal()][direction];
            if (m == null) { return 0; }
            double low = 0, high = m.maxVelocity;
            for (int i = 0; i < 30; i++) {
                double mid = (low + high) / 2;
                if (m.holdingPower(mid) <= availablePower) { low = mid; }
                else { high = mid; }
            }
            result = Math.min(result, low);
        }
        return result;
    }
    /** Preserve the existing signed braking feedforward; the new map covers powered ramp/cruise. */
    public double power(Axis axis, double velocity, double acceleration, double legacy) {
        if (Math.abs(velocity) < 1e-6 && Math.abs(acceleration) < 1e-6) { return 0; }
        MotorPowerModel m = model(axis, Math.abs(velocity) > 1e-6 ? velocity : acceleration);
        if (m == null || velocity * acceleration < -1e-6
                || Math.abs(velocity) > m.maxVelocity + 1e-6) { return legacy; }
        return Math.copySign(m.power(Math.abs(velocity), Math.abs(acceleration)),
                Math.abs(velocity) > 1e-6 ? velocity : acceleration);
    }
    public JSONObject toJson() {
        try {
            JSONObject json = new JSONObject().put("version", 1).put("availableMotorPower", availableMotorPower);
            if(mecanumWheelModel!=null) json.put("mecanumWheelModel",mecanumWheelModel.toJson());
            for (Axis axis : Axis.values()) {
                if (model(axis, 1) != null) {
                    json.put(axis.name(), new JSONObject().put("positive", model(axis, 1).toJson())
                            .put("negative", model(axis, -1).toJson()));
                }
            }
            return json;
        } catch (Exception e) { throw new IllegalStateException("Cannot save characterization", e); }
    }
    public static MotionCharacterization fromJson(JSONObject json, boolean holonomic) {
        try {
            if (json.getInt("version") != 1) { throw new IllegalArgumentException("Unknown characterization version"); }
            MotorPowerModel[][] models = new MotorPowerModel[3][2];
            for (Axis axis : Axis.values()) {
                JSONObject values = json.optJSONObject(axis.name());
                if (values != null) {
                    models[axis.ordinal()][0] = MotorPowerModel.fromJson(values.getJSONObject("positive"));
                    models[axis.ordinal()][1] = MotorPowerModel.fromJson(values.getJSONObject("negative"));
                }
            }
            JSONObject wheel=json.optJSONObject("mecanumWheelModel");
            if(json.has("mecanumWheelModel") && wheel==null) throw new IllegalArgumentException("Wheel calibration must be an object");
            return new MotionCharacterization(models, holonomic, json.optDouble("availableMotorPower", 1),
                    wheel==null?null:MecanumWheelModel.fromJson(wheel));
        } catch (Exception e) { throw new IllegalArgumentException("Invalid motion characterization", e); }
    }
}
