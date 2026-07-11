package org.firstinspires.ftc.teamcode.apexpathing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import controllers.PDSController.PDSCoefficients;
import drivetrains.BaseDrivetrainConstants;
import drivetrains.CoaxialSwerve;
import geometry.Angle;

/** Manual absolute-offset and steering-PDS tuner for coaxial swerve modules. */
@Configurable
@TeleOp(name = "Apex Swerve Module Tuner", group = "Apex Pathing Tuning")
public class SwerveModuleTuner extends OpMode {
    public static double targetDegrees = 0.0;
    public static double kP = 0.0;
    public static double kD = 0.0;
    public static double kS = 0.0;
    public static double kSDeadzoneRadians = Math.toRadians(1.0);

    private CoaxialSwerve swerve;
    private PDSCoefficients coefficients;
    private String error;

    @Override
    public void init() {
        BaseDrivetrainConstants<?> base = new Constants().drivetrainConstants();
        if (!(base instanceof CoaxialSwerve.Constants)) {
            error = "Configured drivetrain is not CoaxialSwerve.";
            return;
        }
        CoaxialSwerve.Constants constants = (CoaxialSwerve.Constants) base;
        coefficients = constants.steeringCoefficients;
        kP = coefficients.kP; kD = coefficients.kD; kS = coefficients.kS;
        kSDeadzoneRadians = coefficients.kSDeadzone;
        swerve = constants.build(hardwareMap);
    }

    @Override
    public void loop() {
        if (error != null) {
            telemetry.addLine(error);
            return;
        }
        coefficients.setkP(kP);
        coefficients.setkD(kD);
        coefficients.setkS(kS);
        coefficients.setkSDeadzone(kSDeadzoneRadians);

        if (gamepad1.a) swerve.steerModulesForTuning(Angle.fromDeg(targetDegrees));
        else swerve.stopSteering();

        double[] raw = swerve.getRawModuleAnglesRad();
        double[] corrected = swerve.getModuleAnglesRad();
        String[] names = {"FL", "FR", "BL", "BR"};
        telemetry.addLine("Robot on blocks. Manually point all wheels forward; -raw angle is each offset.");
        telemetry.addLine("Hold A to drive modules to targetDegrees using live Configurable gains.");
        for (int i = 0; i < names.length; i++) {
            telemetry.addData(names[i] + " raw deg", Math.toDegrees(raw[i]));
            telemetry.addData(names[i] + " recommended offset deg",
                    Math.toDegrees(Angle.normalize(-raw[i])));
            telemetry.addData(names[i] + " corrected deg", Math.toDegrees(corrected[i]));
        }
    }

    @Override
    public void stop() {
        if (swerve != null) swerve.stopSteering();
    }
}
