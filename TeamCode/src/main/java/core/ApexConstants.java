package core;

import drivetrains.BaseDrivetrainConstants;
import localizers.BaseLocalizerConstants;

/**
 * Abstract base class for your constants class.
 *
 * @author Dylan B. 18597 RoboClovers - Delta
 * @author Sohum Arora 22985 Paraducks
 */
public abstract class ApexConfig {
    public abstract BaseDrivetrainConstants<?> drivetrainConstants();

    public abstract BaseLocalizerConstants<?> localizerConstants();
}