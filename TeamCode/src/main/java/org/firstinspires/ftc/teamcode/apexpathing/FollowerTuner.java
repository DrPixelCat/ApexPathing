package org.firstinspires.ftc.teamcode.apexpathing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import tuning.TuneContext;
import tuning.TunePhase;
import tuning.CheckPhase;
import tuning.TurnPhase;
import tuning.ForwardPhase;
import tuning.LocalizerPhase;
import tuning.MotorPhase;
import tuning.SetupPhase;
import tuning.StrafePhase;
import tuning.SwervePhase;

import core.Follower;
import drivetrains.CoaxialSwerve;
import geometry.Pose;

/**
 * Unified follower tuning class for Apex Pathing
 *
 * @author Sohum Arora 22985 Paraducks
 * @author Dylan B. 18597 RoboClovers - Delta
 */
@Configurable
@TeleOp(name = "Follower Tuner", group = "Apex Pathing Tuning")
public class FollowerTuner extends LinearOpMode {
    private final TuneContext context = new TuneContext(this);

    private String[] phaseStatus;
    private int selectedPhase;
    private TunePhase phase;

    enum Phase { MOTOR, SETUP, LOCALIZER, FORWARD, STRAFE, TURN, SWERVE, CHECK }
    private static final Phase[] phases = {
            Phase.MOTOR,
            Phase.SETUP,
            Phase.LOCALIZER,
            Phase.FORWARD,
            Phase.STRAFE,
            Phase.TURN,
            Phase.SWERVE,
            Phase.CHECK
    };

    @Override
    public void runOpMode() {
        context.setFollower(new Follower(new Constants(), hardwareMap));
        loadStatus(); // Initialize the phase selector statuses

        while (opModeInInit() && phase == null) phase = selectPhase();

        if (phase != null) {
            context.getFollower().setPose(Pose.zero());
            telemetry.addLine("Press the start button to run the tuner");
            telemetry.addLine("Make sure you have adequate space to run the robot safely!");
            telemetry.update();
        } else {
            requestOpModeStop(); // The OpMode was started without a phase, so we stop it
        }

        waitForStart();

        while (opModeIsActive()) {
            if (phase == null) {
                continue; // OpMode stop was requested already, do nothing
            }
            telemetry.clearAll();
            context.getFollower().update();
            boolean complete = phase.update(gamepad1.aWasPressed(), gamepad1.bWasPressed());
            telemetry.update();
            if (complete) {
                phase = null;
                context.constants.drivetrainType = context.getFollower().getDrivetrain()
                        .getDrivetrainType();
                context.saveConstants();
                requestOpModeStop();
            }
        }
    }

    private void loadStatus() {
        boolean forwardIsTuned = context.constants.translationalKV > 0.0 &&
                context.constants.translationalKA > 0.0 &&
                context.constants.forwardVelLimitIn > 0.0;
        boolean strafeIsTuned = context.constants.lateralCoeffs.kP > 0.0 &&
                context.constants.strafeVelLimitIn > 0.0;
        boolean angularIsTuned = context.constants.angularKV > 0.0 &&
                context.constants.angularKA > 0.0 &&
                context.constants.angularVelLimitRad > 0.0;
        boolean holonomic = context.getFollower().getDrivetrain().isHolonomic();
        boolean swerve = context.getFollower().getDrivetrain() instanceof CoaxialSwerve;

        phaseStatus = new String[]{
                "[ ]",
                "[ ]",
                "[ ]",
                forwardIsTuned ? "[✓]" : "[ ]",
                holonomic ? (strafeIsTuned ? "[✓]" : "[ ]") : "[N/A]",
                angularIsTuned ? "[✓]" : "[ ]",
                swerve ? "[ ]" : "[N/A]",
                "[ ]"
        };

        selectedPhase = 0;
        for (int i = 0; i < phaseStatus.length; i++) {
            if (phaseStatus[i].equals("[ ]")) {
                selectedPhase = i;
                break;
            }
        }
    }

    private TunePhase selectPhase() {
        telemetry.addLine(
                "The Apex Pathing tuners are listed in order of execution below."
        );
        telemetry.addLine(
                "[✓] = Already Tuned (you can still select to retune)," +
                        "[N/A] = Not applicable to this drivetrain," +
                        "[ ] = Next tuner to run. The cursor ('<') is here by default."
        );
        telemetry.addLine("Use the DPad Up and Down buttons to cycle through phases, " +
                "then press B to open the selected phase.");
        telemetry.addLine();

        for (int i = 0; i < phases.length; i++) {
            String cursor = (i == selectedPhase) ? " <" : "";
            telemetry.addLine(phaseStatus[i] + " " +
                    phases[i].toString().replace("_", " ") + cursor);
        }
        telemetry.update();

        if (gamepad1.dpadUpWasPressed()) {
            selectedPhase = (selectedPhase - 1 + phases.length) % phases.length;
            // Don't allow selection of unavailable phases
            while (!isSelectable(selectedPhase)) {
                selectedPhase = (selectedPhase - 1 + phases.length) % phases.length;
            }
        } else if (gamepad1.dpadDownWasPressed()) {
            selectedPhase = (selectedPhase + 1) % phases.length;
            while (!isSelectable(selectedPhase)) {
                selectedPhase = (selectedPhase + 1) % phases.length;
            }
        } else if (gamepad1.bWasPressed()) {
            switch (phases[selectedPhase]) {
                case MOTOR:
                    return new MotorPhase(context);
                case SETUP:
                    return new SetupPhase(context);
                case LOCALIZER:
                    return new LocalizerPhase(context);
                case FORWARD:
                    return new ForwardPhase(context);
                case STRAFE:
                    return new StrafePhase(context);
                case TURN:
                    return new TurnPhase(context);
                case SWERVE:
                    return new SwervePhase(context);
                case CHECK:
                    return new CheckPhase(context);
            }
        }

        return null; // Phase hasn't been selected yet
    }

    private boolean isSelectable(int index) {
        return !phaseStatus[index].equals("[N/A]");
    }
}
