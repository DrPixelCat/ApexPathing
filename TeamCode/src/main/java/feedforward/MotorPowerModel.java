package feedforward;

import org.json.JSONObject;

/** Measured moving-axis response: a = (gain0 + gainSlope*v)*u - (drag0 + dragSlope*v).
 * Velocity and acceleration use inches or radians; power is normalized motor duty.
 * This describes measured response, never substitutes for a simulator or a physical plant. */
public final class MotorPowerModel {
    public final double gain0, gainSlope, drag0, dragSlope, maxVelocity, validationRms;

    public MotorPowerModel(double gain0, double gainSlope, double drag0, double dragSlope,
                           double maxVelocity, double validationRms) {
        this.gain0 = gain0; this.gainSlope = gainSlope;
        this.drag0 = drag0; this.dragSlope = dragSlope;
        this.maxVelocity = maxVelocity; this.validationRms = validationRms;
        if (!Double.isFinite(gain0) || !Double.isFinite(gainSlope)
                || !Double.isFinite(drag0) || !Double.isFinite(dragSlope)
                || !Double.isFinite(maxVelocity) || maxVelocity <= 0
                || !Double.isFinite(validationRms) || validationRms < 0 || validationRms > .04
                || gain(0) <= 0 || gain(maxVelocity) <= 0
                || holdingPower(0) < 0 || holdingPower(maxVelocity) >= 1
                || dragSlope * gain0 - drag0 * gainSlope < -1e-6) {
            throw new IllegalArgumentException("Invalid or unvalidated motor authority");
        }
    }

    public double gain(double speed) { return gain0 + gainSlope * speed; }
    public double holdingPower(double speed) { return (drag0 + dragSlope * speed) / gain(speed); }
    public double kA(double speed) { return 1.0 / gain(speed); }
    public double power(double speed, double acceleration) {
        return holdingPower(speed) + kA(speed) * acceleration;
    }
    public double acceleration(double speed, double power) {
        return gain(speed) * power - drag0 - dragSlope * speed;
    }
    public JSONObject toJson() {
        try {
            return new JSONObject().put("gain0", gain0).put("gainSlope", gainSlope)
                    .put("drag0", drag0).put("dragSlope", dragSlope)
                    .put("maxVelocity", maxVelocity).put("validationRms", validationRms);
        } catch (Exception e) { throw new IllegalStateException("Cannot save motor authority", e); }
    }
    public static MotorPowerModel fromJson(JSONObject json) {
        try {
            return new MotorPowerModel(json.getDouble("gain0"), json.getDouble("gainSlope"),
                    json.getDouble("drag0"), json.getDouble("dragSlope"),
                    json.getDouble("maxVelocity"), json.getDouble("validationRms"));
        } catch (Exception e) { throw new IllegalArgumentException("Invalid motor authority", e); }
    }
}
