package setup.localization;

import java.util.List;

import localizers.BaseLocalizer;
import localizers.BaseLocalizerConstants;
import drivetrains.TranslationAxis;
import setup.localization.calibration.CalibrationCandidate;
import setup.localization.calibration.CalibrationSnapshot;
import setup.localization.calibration.DirectionTrial;
import setup.localization.calibration.SpinTrial;

/**
 * Supplies the small amount of localizer-specific observation and fitting needed by the tuner.
 * Unknown custom localizers use the monitor-only adapter returned by {@link LocalizerAdapters}.
 */
public interface LocalizerAdapter {
    /** Display name of the configured localizer. */
    String getName();

    /** Returns whether a measured-distance trial can identify this axis. */
    boolean supportsDistance(TranslationAxis axis);

    /** Returns whether rotation can identify offsets or heading scale. */
    boolean supportsSpinCalibration();

    /** Returns a trial-boundary snapshot after the context has updated the localizer. */
    CalibrationSnapshot snapshot(BaseLocalizer<?> localizer);

    /** Selects encoder reversals from commanded positive-axis translation pulses. */
    CalibrationCandidate fitDirections(BaseLocalizerConstants<?> config,
                                       List<DirectionTrial> trials);

    /** Fits geometry from measured translation without mutating the active configuration. */
    CalibrationCandidate fitDistance(BaseLocalizerConstants<?> config, TranslationAxis axis,
                                     CalibrationSnapshot start, CalibrationSnapshot end,
                                     double measuredInches);

    /** Fits geometry from one or more independently measured rotation trials. */
    CalibrationCandidate fitSpin(BaseLocalizerConstants<?> config, List<SpinTrial> trials);
}
