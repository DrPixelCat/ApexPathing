package org.firstinspires.ftc.teamcode.tuning.manual;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import drivetrains.Swerve;
import drivetrains.constants.SwerveConstants;

/**
 * a class to determine swerve offsets
 * @author Xander Haemel 31616 - 404 Not Found
 */
@Configurable
public class SwerveAnglesDebugger extends LinearOpMode {
    public static double rF_angle = 0;
    public static double lF_angle = 0;
    public static double rB_angle = 0;
    public static double lB_angle = 0;
    Swerve swerve = new Swerve(hardwareMap, new SwerveConstants());

    @Override
    public void runOpMode() throws InterruptedException {
        swerve.manuallySetAngles(rF_angle,lF_angle,rB_angle,lB_angle);
        telemetry.update();
        waitForStart();
        while(opModeIsActive()) {
            //these are the true angles
            telemetry.addData("these angles are the actual pod angles", "0 degrees is straight forward.");
            telemetry.addData("fix your offsets until all wheels are pointed forward", "");
            swerve.debug(telemetry);
        }
        telemetry.update();
    }
}
