package tuning.localizer.calibration;

import drivetrains.TranslationAxis;


/** Raw localizer observations captured around one commanded translation pulse. */
public final class DirectionTrial {
    public final TranslationAxis axis;
    public final CalibrationSnapshot start;
    public final CalibrationSnapshot end;

    public DirectionTrial(TranslationAxis axis, CalibrationSnapshot start,
                          CalibrationSnapshot end) {
        this.axis = axis;
        this.start = start;
        this.end = end;
    }
}
