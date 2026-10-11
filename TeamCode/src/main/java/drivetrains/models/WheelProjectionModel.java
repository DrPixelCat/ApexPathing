package drivetrains.models;

/** Immutable body-command to wheel-power geometry shared by planning and motor mixing. */
public final class WheelProjectionModel {
    private final double[] forward, strafe;
    private final double turn;

    public WheelProjectionModel(double[] forward, double[] strafe, double turn) {
        if (forward == null || strafe == null || forward.length < 3 || forward.length != strafe.length
                || !Double.isFinite(turn) || turn == 0)
            throw new IllegalArgumentException("Provide matching projection arrays of length >= 3 and a nonzero turn coefficient");
        this.forward = forward.clone(); this.strafe = strafe.clone(); this.turn = turn;
        for (int i = 0; i < forward.length; i++) {
            if (!Double.isFinite(forward[i]) || !Double.isFinite(strafe[i]))
                throw new IllegalArgumentException("Wheel projections must be finite");
        }
    }
    public int size() { return forward.length; }
    public double[] getForwardProjections() { return forward.clone(); }
    public double[] getStrafeProjections() { return strafe.clone(); }
    public double wheelPower(int index, double x, double y, double rotation) {
        return forward[index] * x + strafe[index] * y + turn * rotation;
    }
    public double demand(double x, double y, double rotation) {
        double peak = 0;
        for (int i = 0; i < size(); i++) peak = Math.max(peak, Math.abs(wheelPower(i, x, y, rotation)));
        return peak;
    }
    /** Extra fixed motor slots receive zero. */
    public void mix(double x, double y, double rotation, double[] output) {
        if (output.length < size()) throw new IllegalArgumentException("Provide one output slot per wheel");
        for (int i = 0; i < output.length; i++) output[i] = i < size() ? wheelPower(i, x, y, rotation) : 0;
    }
    /** Intersect each wheel's affine constraint |offset + slope*a| <= limit. */
    public double maximumAcceleration(double[] coefficients, double min, double max, double limit) {
        double low = min, high = max;
        for (int i = 0; i < size(); i++) {
            double offset = wheelPower(i, coefficients[0], coefficients[2], coefficients[4]);
            double slope = wheelPower(i, coefficients[1], coefficients[3], coefficients[5]);
            if (slope == 0) {
                if (Math.abs(offset) > limit) return Double.NaN;
            } else {
                double first = (-limit - offset) / slope, last = (limit - offset) / slope;
                low = Math.max(low, Math.min(first, last)); high = Math.min(high, Math.max(first, last));
            }
        }
        return low <= high ? high : Double.NaN;
    }
}
