package localizers;

/** A raw pod distance (or parallel-pod difference) used in an externally measured point turn. */
public final class PodCalibrationReading {
    public final String name;
    public final double travelIn;
    public final double configuredOffsetIn;
    public final double sign;

    public PodCalibrationReading(String name, double travelIn, double configuredOffsetIn, double sign) {
        this.name = name;
        this.travelIn = travelIn;
        this.configuredOffsetIn = configuredOffsetIn;
        this.sign = sign;
    }

    public double measuredOffset(double startTravelIn, double measuredSweepRad) {
        return sign * (travelIn - startTravelIn) / measuredSweepRad;
    }
}
