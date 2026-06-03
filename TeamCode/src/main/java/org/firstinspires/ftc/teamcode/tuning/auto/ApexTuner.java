package org.firstinspires.ftc.teamcode.tuning.auto;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.tuning.auto.p2p.AutoTuner;
import controllers.PDSController.PDSCoefficients;

import android.os.Environment;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

/**
 * All-in-one tuner capable of running all 3 tuners in just one OpMode, completely automatically
 * All you have to do is follow the telemetry instructions and press a couple of buttons! Easy, right?
 * Saves the final coefficients directly to the Control Hub storage. No need for copy pasting!
 * TODO: Coming soon - The constants file will automatically read the file containing the saved coefficients!
 * @author Sohum Arora 29285 Paraducks
 */

@TeleOp(name = "Apex Tuner", group = "Apex Pathing Tuning")
public class ApexTuner extends AutoTuner {
    enum TuningState {HEADING, STRAFE, AXIAL, COMPLETE}
    private TuningState currentState = TuningState.HEADING;
    int phase = 1;
    private boolean headingRun = false;
    private boolean strafeRun = false;
    private boolean axialRun = false;
    boolean fileSaved = false;

    private PDSCoefficients headingCoeffs = new PDSCoefficients();
    private PDSCoefficients strafeCoeffs = new PDSCoefficients();
    private PDSCoefficients axialCoeffs = new PDSCoefficients();

    @Override
    public void runOpMode() {
        phase = 1;
        fileSaved = false;

        while (opModeInInit()) {
            telemetry.addLine("Robot Initialized");
            telemetry.addLine("Tuning order: 1) Heading Tuner 2) Strafe Tuner 3) Axial Tuner");
            telemetry.addLine("Run the OpMode to proceed with the Heading Tuner");
            telemetry.addLine("Press 'A' (cross) to run the Strafe Tuner if you have already run the Heading Tuner");
            telemetry.addLine("Press 'B' (circle) to run the Axial Tuner if you have already run the Heading and Strafe tuners");
            telemetry.addLine("IMPORTANT: Do not run the tuners out of order");

            if (gamepad1.a) {
                phase = 2;
                headingRun = true;
            }
            else if (gamepad1.b) {
                phase = 3;
                headingRun = true;
                strafeRun = true;
            }

            if (phase > 1) {
                angularTuner = false;
                testTarget = 48.0;
            } else {
                angularTuner = true;
                testTarget = Math.PI;
            }

            telemetry.addData("Selected Phase", phase);
            telemetry.update();
        }

        initializeTuner();
        waitForStart();

        while (opModeIsActive() && currentState != TuningState.COMPLETE && !isStopRequested()) {

            switch (phase) {
                case 1:
                    currentState = TuningState.HEADING;
                    break;
                case 2:
                    currentState = TuningState.STRAFE;
                    break;
                case 3:
                    currentState = TuningState.AXIAL;
                    break;
            }

            runTuner(currentState);

            if (phase < 3) {
                phase ++;
            } else {
                axialRun = true;
            }

            if (headingRun && axialRun && strafeRun) {
                currentState = TuningState.COMPLETE;
            }
        }

        if (currentState == TuningState.COMPLETE && !fileSaved) {
            saveCoefficientsToFile();
            fileSaved = true;
        }

        while (opModeIsActive()) {
            telemetry.addLine("ALL TUNING CYCLES COMPLETE");
            telemetry.addLine("Values automatically saved to: /sdcard/FIRST/PDSCoefficients.txt");

            if (headingRun) {
                telemetry.addLine("\n--- Heading Coefficients ---");
                telemetry.addData("kP", headingCoeffs.kP);
                telemetry.addData("kD", headingCoeffs.kD);
                telemetry.addData("kS", headingCoeffs.kS);
            }
            if (strafeRun) {
                telemetry.addLine("\n--- Strafe Coefficients ---");
                telemetry.addData("kP", strafeCoeffs.kP);
                telemetry.addData("kD", strafeCoeffs.kD);
                telemetry.addData("kS", strafeCoeffs.kS);
            }
            if (axialRun) {
                telemetry.addLine("\n--- Axial Coefficients ---");
                telemetry.addData("kP", axialCoeffs.kP);
                telemetry.addData("kD", axialCoeffs.kD);
                telemetry.addData("kS", axialCoeffs.kS);
            }

            telemetry.update();
            drivetrain.moveWithVectors(0, 0, 0);
        }
    }

    private void runTuner(TuningState state) {
        coeffs = new PDSCoefficients();
        controller.setCoefficients(coeffs);

        switch (state) {
            case HEADING:
                angularTuner = true;
                testTarget = Math.PI;
                break;
            case AXIAL:
            case STRAFE:
                angularTuner = false;
                testTarget = 48.0;
                break;
        }

        kSTuner();
        kPkDTuner();
        verifyValues();

        switch (state) {
            case HEADING:
                headingCoeffs = new PDSCoefficients(coeffs.kP, coeffs.kD, coeffs.kS, coeffs.kSDeadzone);
                break;
            case STRAFE:
                strafeCoeffs = new PDSCoefficients(coeffs.kP, coeffs.kD, coeffs.kS, coeffs.kSDeadzone);
                break;
            case AXIAL:
                axialCoeffs = new PDSCoefficients(coeffs.kP, coeffs.kD, coeffs.kS, coeffs.kSDeadzone);
                break;
        }

        drivetrain.moveWithVectors(0, 0, 0);
        sleep(1000);
    }

    private void verifyValues() {
        controller.reset();
        double verificationTarget = testTarget;
        controller.setTarget(verificationTarget);

        if (angularTuner) {
            controller.setTolerance(geometry.Angle.fromDeg(3));
        } else {
            controller.setTolerance(geometry.Dist.fromIn(1));
            headingController.reset();
            headingController.setTarget(0);
        }

        timer.reset();
        switch (currentState) {
            case HEADING:
                headingRun = true;
                break;
            case STRAFE:
                strafeRun = true;
                break;
            case AXIAL:
                axialRun = true;
                break;
        }

        while (opModeIsActive()) {
            telemetry.addData("Current Tuner", currentState.name());
            telemetry.addLine("Press 'A' (cross) to ACCEPT values & advance to next tuner.");
            telemetry.addLine();
            telemetry.addData("Calculated kP", coeffs.kP);
            telemetry.addData("Calculated kD", coeffs.kD);
            telemetry.addData("Calculated kS", coeffs.kS);
            telemetry.addLine();
            telemetry.addData("Current Position", getCurrentPosition());
            telemetry.addData("Target", verificationTarget);
            telemetry.update();

            if (gamepad1.aWasPressed()) {
                break;
            }

            localizer.update();
            double currentPosition = getCurrentPosition();
            double output = controller.calculate(currentPosition);

            double headingCorrection = angularTuner ? 0 :
                    headingController.calculate(localizer.getPose().getHeading().getRad());

            applyControl(output, headingCorrection);

            if (controller.isAtTarget()) {
                if (timer.time(java.util.concurrent.TimeUnit.MILLISECONDS) > TARGET_SWITCH_WAIT_TIME_MS) {
                    verificationTarget = (verificationTarget == testTarget) ? 0 : testTarget;
                    controller.reset();
                    controller.setTarget(verificationTarget);
                    timer.reset();
                }
            } else {
                timer.reset();
            }
        }
    }

    private void saveCoefficientsToFile() {
        File directory = new File(Environment.getExternalStorageDirectory(), "FIRST");
        if (!directory.exists()) {
            directory.mkdirs();
        }

        File file = new File(directory, "PDSCoefficients.txt");
        try (FileWriter writer = new FileWriter(file)) {
            writer.write("[Heading]\n");
            writer.write("kP=" + headingCoeffs.kP + "\n");
            writer.write("kD=" + headingCoeffs.kD + "\n");
            writer.write("kS=" + headingCoeffs.kS + "\n\n");

            writer.write("[Strafe]\n");
            writer.write("kP=" + strafeCoeffs.kP + "\n");
            writer.write("kD=" + strafeCoeffs.kD + "\n");
            writer.write("kS=" + strafeCoeffs.kS + "\n\n");

            writer.write("[Axial]\n");
            writer.write("kP=" + axialCoeffs.kP + "\n");
            writer.write("kD=" + axialCoeffs.kD + "\n");
            writer.write("kS=" + axialCoeffs.kS + "\n");

            telemetry.addLine("File Write Success!");
        } catch (IOException e) {
            telemetry.addLine("ERROR: Failed to save coefficients to file: " + e.getMessage());
        }
        telemetry.update();
    }

    @Override
    public double getCurrentPosition() {
        switch (currentState) {
            case HEADING:
                return localizer.getPose().getHeading().getRad();
            case STRAFE:
                return localizer.getPose().getY().getIn();
            case AXIAL:
                return localizer.getPose().getX().getIn();
            default:
                return 0;
        }
    }

    @Override
    public double getCurrentVelocity() {
        switch (currentState) {
            case HEADING:
                return localizer.getVel().getHeading().getRad();
            case STRAFE:
                return localizer.getVel().getY().getIn();
            case AXIAL:
                return localizer.getVel().getX().getIn();
            default:
                return 0;
        }
    }

    @Override
    public void applyControl(double controlOutput, double headingCorrection) {
        switch (currentState) {
            case HEADING:
                drivetrain.drive(0, 0, controlOutput);
                break;
            case STRAFE:
                drivetrain.drive(0, controlOutput, headingCorrection);
                break;
            case AXIAL:
                drivetrain.drive(controlOutput, 0, headingCorrection);
                break;
            default:
                drivetrain.moveWithVectors(0, 0, 0);
                break;
        }
    }
}