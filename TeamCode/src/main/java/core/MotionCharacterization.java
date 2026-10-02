package core;

import org.json.JSONObject;

/** Immutable, independently validated responses for each axis and travel direction. */
public final class MotionCharacterization {
    public enum Axis { FORWARD, STRAFE, ANGULAR }
    public static final double PROFILE_POWER_LIMIT = .88;
    private final MotorPowerModel[][] models;
    public final double availableMotorPower;

    public MotionCharacterization(MotorPowerModel[][] models, boolean holonomic) {
        this(models, holonomic, 1);
    }
    public MotionCharacterization(MotorPowerModel[][] models, boolean holonomic, double availableMotorPower) {
        if (!Double.isFinite(availableMotorPower) || availableMotorPower <= 0 || availableMotorPower > 1) {
            throw new IllegalArgumentException("Invalid calibrated motor-power limit");
        }
        this.availableMotorPower = availableMotorPower;
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
        double result = Double.POSITIVE_INFINITY;
        for (int direction = 0; direction < 2; direction++) {
            MotorPowerModel m = models[axis.ordinal()][direction];
            if (m == null) { return 0; }
            double low = 0, high = m.maxVelocity;
            for (int i = 0; i < 30; i++) {
                double mid = (low + high) / 2;
                if (m.holdingPower(mid) <= profilePowerLimit()) { low = mid; }
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
            return new MotionCharacterization(models, holonomic, json.optDouble("availableMotorPower", 1));
        } catch (Exception e) { throw new IllegalArgumentException("Invalid motion characterization", e); }
    }
}
