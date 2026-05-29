package org.firstinspires.ftc.teamcode.tuning.auto.p2p;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;

import java.util.concurrent.TimeUnit;

import controllers.PDSController.PDSCoefficients;
import controllers.PDSController;
import drivetrains.Drivetrain;
import followers.constants.P2PFollowerConstants;
import localizers.Localizer;
import geometry.Angle;
import geometry.Dist;
import geometry.Pose;

/**
 * Base class for P2P follower PDS controller tuner OpModes
 * @author Joel - 7842 Browncoats Alumni
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public abstract class AutoTuner extends LinearOpMode {
    // Assigned by child classes
    public boolean angularTuner; // Angular or translational control
    public double testTarget; // Thr target distance/angle to travel during the tuning process

    // TODO: Adjust these constants based on testing
    final double HAS_MOVED_THRESHOLD = 0.025;
    final double BINARY_SEARCH_CONVERGENCE_THRESHOLD = 0.01;
    final double INITIAL_MAX_KS_GUESS = 0.3;
    final double TIME_PER_GUESS_MS = 500;
    final long WAIT_TIME_BETWEEN_GUESSES_MS = 500;
    final double PD_TUNER_DURATION = 2000;
    final double TARGET_SWITCH_WAIT_TIME_MS = 1000;

    public JoinedTelemetry fullTelem;
    public Drivetrain drivetrain;
    public Localizer localizer;
    public PDSController controller;
    public PDSController headingController; // For maintaining heading with translational controllers
    public ElapsedTime timer;

    public static double kP;
    public static double kD;
    public static double kS;
    public static double kSDeadzone;

    public abstract double getCurrentPosition();

    public abstract double getCurrentVelocity();
    public abstract void applyControl(double controlOutput, double headingCorrection);

    public void initializeTuner() {
        fullTelem = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
        Constants constants = new Constants();
        P2PFollowerConstants followerConstants = (P2PFollowerConstants) constants.setFollowerConstants();
        drivetrain = constants.buildOnlyDrivetrain(hardwareMap);
        localizer = constants.buildOnlyLocalizer(hardwareMap, Pose.zero());

        controller = new PDSController(new PDSCoefficients(kP, kD, kS, kSDeadzone));
        if (angularTuner) {
            // For angular tuners, set the controller to angular mode
            controller.setAngularController();
        } else {
            // For translational tuners, initialize the heading controller
            headingController = followerConstants.headingController;
            headingController.setTarget(0);
        }

        timer = new ElapsedTime();

        telemetry.addData("Initialized", "Press play to start tuning");
        telemetry.addLine("WARNING: make sure the robot is in a clean area before running");
        telemetry.update();
    }

    public void kSTuner() {
        // Binary search for kS
        double maxGuess = INITIAL_MAX_KS_GUESS;
        double minGuess = 0.0;
        double guess = 0.0;
        double lastGuess = -1.0;
        double maxDetectedDeviation;
        boolean hasMoved;

        while (opModeIsActive() && Math.abs(lastGuess - guess) > BINARY_SEARCH_CONVERGENCE_THRESHOLD) {
            localizer.setPose(Pose.zero()); // Reset pose to isolate the variable being tuned
            localizer.update();

            telemetry.addData("Phase", "1/3: Tuning kS (static friction compensation) with a binary search");
            telemetry.addData("Current Power Guess", guess);
            telemetry.update();

            guess = (maxGuess + minGuess) / 2;

            maxDetectedDeviation = 0.0;
            timer.reset();
            while (opModeIsActive() && timer.time(TimeUnit.MILLISECONDS) < TIME_PER_GUESS_MS) {
                localizer.update();
                maxDetectedDeviation = Math.max(Math.abs(getCurrentPosition()), maxDetectedDeviation);
                double errorSign = Math.signum(0 - getCurrentPosition());
                double controlOutput = (errorSign != 0) ? guess * errorSign : guess; // Ensure output isn't 0
                applyControl(controlOutput, 0); // No heading correction during kS tuning to isolate the variable
            }

            // Adjust binary bounds based on whether the chassis moved
            hasMoved = maxDetectedDeviation > HAS_MOVED_THRESHOLD;
            if (hasMoved) { maxGuess = guess; } else { minGuess = guess; }
            lastGuess = guess;

            drivetrain.moveWithVectors(0,0,0);
            sleep(WAIT_TIME_BETWEEN_GUESSES_MS);
        }

        kS = guess;
    }

    public void kPkDTuner() {
        // Ziegler Nichol's method-based auto PID tuner
        double deltaT;
        double deltaV;
        double accel;
        double maxAccel = 0;
        double maxVel = 0;
        double lastVel = 0;
        double lastTime = System.nanoTime();
        double startTime = System.nanoTime();
        double timeStamp = 0;
        double velAtTimeStamp = 0;

        if (!angularTuner) { headingController.reset(); headingController.setTarget(0); }

        timer.reset();
        while (opModeIsActive() && timer.time(TimeUnit.MILLISECONDS) < PD_TUNER_DURATION) {
            localizer.update();

            telemetry.addData("Phase", "2/3: Tuning kP and kD using the Ziegler-Nichols method");
            telemetry.addData("Time Elapsed (s)", timer.time(TimeUnit.SECONDS));
            telemetry.addData("Max Acceleration Detected", maxAccel);
            telemetry.update();

            double curVel = getCurrentVelocity();
            deltaT = (System.nanoTime() - lastTime) / 1e9;
            deltaV = (curVel - lastVel);

            if (deltaT > 1e-6) { // Avoid division by zero
                accel = deltaV / deltaT;
            } else {
                accel = 0.0;
            }

            // Capture the exact moment of maximum acceleration (the inflection point)
            if (maxAccel < accel) {
                maxAccel = accel;
                timeStamp = (System.nanoTime() - startTime) / 1.0e9;
                velAtTimeStamp = curVel;
            }

            maxVel = Math.max(curVel, maxVel);
            lastVel = curVel;
            lastTime = System.nanoTime();

            double headingCorrection = angularTuner ? 0 :
                    headingController.calculate(localizer.getPose().getHeading().getRad());
            applyControl(1.0, headingCorrection);
        }

        // Calculate Delay Time (L) based on the tangent line of the inflection point
        double L = timeStamp - (velAtTimeStamp / maxAccel);

        // Derive parallel PD gains using Ziegler-Nichols open-loop formulas
        kP = 1.2 / (L * maxAccel);
        kD = 0.6 / maxAccel;

        controller.setCoefficients(new PDSController.PDSCoefficients(kP, kD, kS, kSDeadzone));
    }

    public void verification() {
        // Toggle between targets to verify tuning
        boolean enabledToggle = true;

        localizer.update();
        double currentPosition = getCurrentPosition();
        double distToZero = Math.abs(0 - currentPosition);
        double distToTarget = Math.abs(testTarget - currentPosition);

        // Choose whichever target is further away for the initial verification swing
        double verificationTarget = (distToTarget > distToZero) ? testTarget : 0;
        controller.reset();
        controller.setTarget(verificationTarget);

        // Large tolerances to account for slightly imperfect tuning
        if (angularTuner) {
            controller.setTolerance(Angle.fromDeg(3)); // 2 degree tolerance
        } else {
            controller.setTolerance(Dist.fromIn(1)); // 1 inch tolerance
            headingController.reset(); headingController.setTarget(0);
        }

        timer.reset();
        while (opModeIsActive()) {
            telemetry.addData("Phase", "3/3: Final Verification");
            telemetry.addLine("Press A on your gamepad to stop the test while you copy coefficients");
            telemetry.addLine();
            telemetry.addData("Calculated kP", kP);
            telemetry.addData("Calculated kD", kD);
            telemetry.addData("Calculated kS", kS);
            telemetry.addLine();
            telemetry.addData("Current", currentPosition);
            telemetry.addData("Target", verificationTarget);
            telemetry.addData("Error", controller.getError());
            telemetry.update();

            localizer.update();
            currentPosition = getCurrentPosition();

            if (gamepad1.aWasPressed()) {
                enabledToggle = !enabledToggle;
                if (!enabledToggle) {
                    drivetrain.moveWithVectors(0, 0, 0);
                } else {
                    if (!angularTuner) {
                        headingController.reset(); headingController.setTarget(0);
                    }
                    controller.reset(); controller.setTarget(verificationTarget);
                    timer.reset();
                }
            }

            if (!enabledToggle) {
                continue;
            }

            double output = controller.calculate(currentPosition);
            double headingCorrection = angularTuner ? 0 :
                    headingController.calculate(localizer.getPose().getHeading().getRad());
            applyControl(output, headingCorrection);

            if (controller.isAtTarget()) {
                if (timer.time(TimeUnit.MILLISECONDS) > TARGET_SWITCH_WAIT_TIME_MS) {
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
}