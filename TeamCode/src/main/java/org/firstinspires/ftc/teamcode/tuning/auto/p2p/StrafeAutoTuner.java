package org.firstinspires.ftc.teamcode.tuning.auto.p2p;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Strafe PDS auto tuner OpMode
 * @author Joel - 7842 Browncoats Alumni
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
@TeleOp(name = "Strafe Auto Tuner", group = "Apex Pathing Tuning")
public class StrafeAutoTuner extends AutoTuner {
    @Override
    public void runOpMode() {
        // Set tuner type and test target
        angularTuner = false;
        testTarget = 48.0; // Target distance in inches (48 inches = 2 FTC tiles)

        this.initializeTuner();

        waitForStart();

        this.kSTuner();
        this.kPkDTuner();
        this.verification();
    }

    @Override
    public double getCurrentPosition() { return this.localizer.getPose().getY().getIn(); }

    @Override
    public double getCurrentVelocity() { return this.localizer.getVelocity().getY().getIn(); }

    @Override
    public void applyControl(double controlOutput, double headingCorrection) {
        drivetrain.drive(0, controlOutput, headingCorrection);
    }
}