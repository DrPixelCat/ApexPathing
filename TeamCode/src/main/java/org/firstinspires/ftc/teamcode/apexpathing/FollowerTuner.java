package org.firstinspires.ftc.teamcode.apexpathing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import core.ApexConfig;
import core.Follower;
import core.FollowerConstants;
import drivetrains.BaseDrivetrainConfig;
import localizers.BaseLocalizerConfig;
import org.firstinspires.ftc.teamcode.apexpathing.tuning.HeadingPhase;
import org.firstinspires.ftc.teamcode.apexpathing.tuning.TunerContext;
import org.firstinspires.ftc.teamcode.apexpathing.tuning.TuningPhase;
import org.firstinspires.ftc.teamcode.apexpathing.tuning.TranslationPhase;
import org.firstinspires.ftc.teamcode.apexpathing.tuning.VelocityFeedforwardPhase;

/**
 * Single unified automatic tuner capable of completely tuning a robot for Apex in minutes in just a single OpMode!
 * All you have to do is follow the telemetry instructions and press a couple buttons here and there.
 * Once you have run this tuner, your robot is fully tuned and ready to go Path its way to the Peaks.
 * @author Sohum Arora 22985 Paraducks
 */
@Configurable
@TeleOp(name = "Follower Tuner", group = "Apex Pathing Tuning")
public class FollowerTuner extends LinearOpMode {

    public static double kSGuess = 0.0;

    private final Constants baseConstants = new Constants();
    private final FollowerConstants followerConstants = new FollowerConstants();
    private final TunerContext tunerContext = new TunerContext(this, followerConstants);

    private TuningPhase phase = new HeadingPhase(tunerContext);
    private boolean lastA;
    private boolean lastB;
    private boolean lastY;
    private boolean paused;

    @Override
    public void runOpMode() throws InterruptedException {
        FollowerConstants defaults = baseConstants.followerConfig().getConstants();
        tunerContext.loadFrom(defaults);

        boolean headingRun = defaults.headingCoeffs.kP != 0.0 || defaults.headingCoeffs.kD != 0.0 || defaults.headingCoeffs.kS != 0.0;
        boolean translationRun = defaults.driveCoeffs.kP != 0.0 || defaults.driveCoeffs.kD != 0.0 || defaults.driveCoeffs.kS != 0.0;
        boolean velocityFFRun = defaults.lateralKV != 0.0;
        boolean accelRun = defaults.maxLateralAccel > 10.0;

        while (opModeInInit()) {
            telemetry.addLine("Robot Initialized");
            telemetry.addLine("Tuning order:\n 1) Heading PDS \n 2) Translation PDS \n 3) Velocity FF \n 4) Max Lateral Accel");
            telemetry.addLine("Run the OpMode to proceed with the Heading Tuner");

            if (headingRun) telemetry.addLine("Heading tuner has already been run and values have been saved");
            if (translationRun) telemetry.addLine("Translation tuner has already been run and values have been saved");
            if (velocityFFRun) telemetry.addLine("Velocity FF tuner has already been run and values have been saved");
            if (accelRun) telemetry.addLine("Max Lateral Accel tuner has already been run and values have been saved");

            telemetry.addLine("A - Run the Translation Tuner if Heading Tuner has been run. ");
            telemetry.addLine("B - Run the Velocity FF Tuner if Heading & Translation tuners have been run. ");
            telemetry.addLine("Once all 3 complete, press A to run Max Lateral Accel Tuner. ");
            telemetry.addLine("WARNING: Do NOT run the tuners out of order");

            if (gamepad1.a) {
                phase = new TranslationPhase(tunerContext);
            } else if (gamepad1.b) {
                phase = new VelocityFeedforwardPhase(tunerContext);
            }

            telemetry.addData("Selected Phase", phase.phase());
            telemetry.update();
        }

        tunerContext.updateFollowerConfig();
        tunerContext.setFollower(new Follower(customConfig, hardwareMap));

        waitForStart();
        captureCurrentButtons();

        while (opModeIsActive() && phase != null && !isStopRequested()) {
            boolean aPressed = gamepad1.a && !lastA;
            boolean bPressed = gamepad1.b && !lastB;
            boolean yPressed = gamepad1.y && !lastY;

            handlePause(yPressed);

            if (paused && phase.isRunningAutomatic()) {
                tunerContext.stopDrive();
                telemetry.addLine("Tuner Paused, Y to resume.");
                captureCurrentButtons();
                telemetry.update();
                continue;
            }

            phase = phase.update(aPressed, bPressed);

            captureCurrentButtons();
            telemetry.update();
        }

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Status", "All Tuning Cycles Complete! Configuration Saved to JSON.");
            telemetry.update();
            tunerContext.stopDrive();
        }
    }

    private void handlePause(boolean yPressed) {
        if (!yPressed || phase == null || !phase.isRunningAutomatic()) {
            return;
        }

        paused = !paused;
        if (!paused) {
            phase.onResume();
        }
    }

    private void captureCurrentButtons() {
        lastA = gamepad1.a;
        lastB = gamepad1.b;
        lastY = gamepad1.y;
    }

    private final ApexConfig customConfig = new ApexConfig() {
        @Override
        public BaseDrivetrainConfig<?> drivetrainConfig() { return baseConstants.drivetrainConfig(); }
        @Override
        public BaseLocalizerConfig<?> localizerConfig() { return baseConstants.localizerConfig(); }
        @Override
        public FollowerConstants followerConfig() { return followerConstants; }
    };
}
