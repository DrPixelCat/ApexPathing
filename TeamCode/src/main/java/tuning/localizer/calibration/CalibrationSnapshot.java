package tuning.localizer.calibration;


import geometry.Pose;

/** Immutable pose and raw-channel observation captured at a trial boundary. */
public final class CalibrationSnapshot {
    public final Pose pose;
    private final int[] rawTicks;

    public int getChannelCount() { return rawTicks.length; }
    public int getRawTick(int channel) { return rawTicks[channel]; }
    public int[] getRawTicks() { return rawTicks.clone(); }

    public CalibrationSnapshot(Pose pose, int[] rawTicks) {
        this.pose = pose;
        this.rawTicks = rawTicks == null ? new int[0] : rawTicks.clone();
    }
}
