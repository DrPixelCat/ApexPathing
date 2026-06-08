package core;

import drivetrains.BaseDrivetrainConfig;
import localizers.BaseLocalizerConfig;

/**
 * Abstract base class for your constants
 * Method implemented by {@link org.firstinspires.ftc.teamcode.Constants}
 * @author Dylan B. 18597 RoboClovers - Delta
 * @author Sohum Arora 22985 Paraducks
 */
public abstract class ApexConfig {
    public abstract BaseDrivetrainConfig<?> drivetrainConfig();

    public abstract BaseLocalizerConfig<?> localizerConfig();
    public abstract FollowerConstants followerConfig();

}