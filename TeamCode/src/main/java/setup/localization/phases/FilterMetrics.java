package setup.localization.phases;

import paths.geometry.Pose;

/** Compact evidence gathered during filter tuning. */
final class FilterMetrics {
    private double rawSquared;
    private double filteredSquared;
    private double stopSquared;
    private double rawAngularSquared, filteredAngularSquared, stopAngularSquared;
    private int stationarySamples;
    private int stopSamples;
    private int invalidSamples;

    void sample(Pose raw, Pose filtered, boolean stationary, boolean stopping, boolean valid) {
        if (!valid) {
            invalidSamples++;
            return;
        }
        double rawMagnitude = magnitude(raw);
        double filteredMagnitude = magnitude(filtered);
        if (stationary) {
            rawSquared += rawMagnitude * rawMagnitude;
            filteredSquared += filteredMagnitude * filteredMagnitude;
            rawAngularSquared += square(raw.getHeading().getRad());
            filteredAngularSquared += square(filtered.getHeading().getRad());
            stationarySamples++;
        }
        if (stopping) {
            stopSquared += filteredMagnitude * filteredMagnitude;
            stopAngularSquared += square(filtered.getHeading().getRad());
            stopSamples++;
        }
    }

    double rawStationaryRms() { return rms(rawSquared, stationarySamples); }
    double filteredStationaryRms() { return rms(filteredSquared, stationarySamples); }
    double stopResidualRms() { return rms(stopSquared, stopSamples); }
    double rawAngularRms() { return rms(rawAngularSquared, stationarySamples); }
    double filteredAngularRms() { return rms(filteredAngularSquared, stationarySamples); }
    double stopAngularRms() { return rms(stopAngularSquared, stopSamples); }
    private static double square(double value) { return value * value; }
    int getStationarySamples() { return stationarySamples; }
    int getStopSamples() { return stopSamples; }
    int getInvalidSamples() { return invalidSamples; }

    private static double magnitude(Pose value) {
        double x = value.getX().getIn(), y = value.getY().getIn();
        return Math.hypot(x, y);
    }
    private static double rms(double sum, int count) { return count == 0 ? 0.0 : Math.sqrt(sum / count); }
}
