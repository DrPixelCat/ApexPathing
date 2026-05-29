package org.firstinspires.ftc.teamcode.tuning.auto.p2p;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Axial PDS auto tuner OpMode
 * @author Joel - 7842 Browncoats Alumni
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
@TeleOp(name = "Axial Auto Tuner", group = "Apex Pathing Tuning")
public class AxialAutoTuner extends AutoTuner {
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
    public double getCurrentPosition() { return this.localizer.getPose().getX().getIn(); }

    @Override
    public double getCurrentVelocity() { return this.localizer.getVelocity().getX().getIn(); }

    @Override
    public void applyControl(double controlOutput, double headingCorrection) {
        drivetrain.drive(controlOutput, 0, headingCorrection);
    }
}