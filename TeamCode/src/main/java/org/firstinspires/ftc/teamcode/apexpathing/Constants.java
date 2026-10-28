package org.firstinspires.ftc.teamcode.apexpathing;

import core.ApexConstants;
import drivetrains.BaseDrivetrainConstants;
import drivetrains.Mecanum;
import localizers.BaseLocalizerConstants;
import localizers.Pinpoint;
import geometry.DistUnit;
import drivetrains.Motor;

/**
 * This class extends {@link ApexConstants} and provides the specific drivetrain and localizer
 * configurations for your robot.
 *
 * @author Dylan B. - 18597 RoboClovers - Delta
 * @author Sohum Arora - 22985 Paraducks
 */
public class Constants implements ApexConstants, setup.SavedSetup {
    private final boolean useSavedSetup;
    public Constants() { this(true); }
    /** Pass false for a custom Java configuration that intentionally bypasses Robot Setup. */
    public Constants(boolean useSavedSetup) { this.useSavedSetup = useSavedSetup; }

    private setup.RobotProfile loadSetup() {
        if (!useSavedSetup) { return null; }
        try { return new setup.SetupStore(core.ApexStorage.getDirectory()).load(); }
        catch (java.io.IOException e) { throw new IllegalStateException("Robot setup could not be loaded. Run Robot Setup again.", e); }
    }

    @Override public setup.RobotProfile robotProfile() { return loadSetup(); }
    @Override
    public BaseDrivetrainConstants<?> drivetrainConstants() {
        setup.RobotProfile savedSetup = loadSetup();
        if (savedSetup != null) { return setup.SetupConfigurations.drivetrain(savedSetup); }
        return new Mecanum.Constants()
                .setFrontLeftMotor(new Motor("frontLeftMotor"))
                .setBackLeftMotor(new Motor("backLeftMotor"))
                .setFrontRightMotor(new Motor("frontRightMotor").reverse())
                .setBackRightMotor(new Motor("backRightMotor").reverse())
                .setRobotCentric(true)
                .setMaxPower(1.0);
    }

    @Override
    public BaseLocalizerConstants<?> localizerConstants() {
        setup.RobotProfile savedSetup = loadSetup();
        if (savedSetup != null) { return setup.SetupConfigurations.localizer(savedSetup); }
        return new Pinpoint.Constants()
                .setName("pinpoint")
                .setOffsets(0, 0, DistUnit.IN)
                .setEncoderDirections(Pinpoint.EncoderDirection.FORWARD,
                        Pinpoint.EncoderDirection.FORWARD)
                .setEncoderResolution(Pinpoint.GoBildaPods.goBILDA_4_BAR_POD);
    }
}
