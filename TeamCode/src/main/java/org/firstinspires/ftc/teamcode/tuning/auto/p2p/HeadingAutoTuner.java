package org.firstinspires.ftc.teamcode.tuning.auto.p2p;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Heading PDS auto tuner OpMode
 * @author Joel - 7842 Browncoats Alumni
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
@TeleOp(name = "Heading Auto Tuner", group = "Apex Pathing Tuning")
public class HeadingAutoTuner extends AutoTuner {
    @Override
    public void runOpMode() {
        // Set tuner type and test target
        angularTuner = true;
        testTarget = Math.PI; // Target distance in radians (180 degrees)

        this.initializeTuner();

        waitForStart();

        this.kSTuner();
        this.kPkDTuner();
        this.verification();
    }

    @Override
    public double getCurrentPosition() { return this.localizer.getPose().getHeading().getRad(); }

    @Override
    public double getCurrentVelocity() { return this.localizer.getVelocity().getHeading().getRad(); }

    @Override
    public void applyControl(double controlOutput, double headingCorrection) {
        // Ignore heading correction
        drivetrain.drive(0, 0, controlOutput);
    }
}