package org.firstinspires.ftc.teamcode.apexpathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import core.Follower;
import geometry.Pose;
import tuning.CentripetalPhase;
import tuning.HeadingPhase;
import tuning.LimitsPhase;
import tuning.TunerContext;
import tuning.TuningPhase;
import tuning.TuningValues;
import tuning.VelocityFeedbackPhase;

/**
 * @author Sohum Arora - 22985 Paraducks
 */
@TeleOp(name = "Follower Tuner", group = "Apex Pathing Tuning")
public class FollowerTuner extends LinearOpMode {
    enum TunerPhase {
        HEADING,
        MOVEMENT_LIMITS,
        CENTRIPETAL,
        VELOCITY_FEEDBACK
    }
    private TunerContext context;
    private TuningPhase[] phases;
    private int selectedPhaseIndex;

    @Override
    public void runOpMode() {
        context = new TunerContext(this);
        context.setFollower(new Follower(new Constants(), hardwareMap, true));
        context.constants.drivetrainType = context.getFollower().getDrivetrain().getDrivetrainType();

        TuningValues values = new TuningValues(context.constants);
        phases = new TuningPhase[]{
                new HeadingPhase(context, values),
                new LimitsPhase(context, values),
                new CentripetalPhase(context, values),
                new VelocityFeedbackPhase(context, values)
        };
        selectFirstIncompletePhase();

        while (opModeInInit()) {
            TuningPhase selectedPhase = phaseSelector();
            if (selectedPhase != null) {
                context.getFollower().setPose(Pose.zero());
                telemetry.addLine("Press Start to run the tuner.");
                telemetry.addLine("Make sure the robot has enough space.");
                telemetry.update();
            }
        }

        waitForStart();

        while (opModeIsActive() && selectedPhaseIndex < phases.length) {
            TuningPhase phase = phases[selectedPhaseIndex];
            phase.run(this);
            if (!opModeIsActive()) {
                break;
            }
            if (phase.isComplete()) {
                context.saveConstants();
                selectedPhaseIndex++;
            }
        }

        context.getFollower().stop();
    }

    private boolean phaseAvailable(int index) {
        for (int i = 0; i < index; i++) {
            if (!phases[i].isComplete()) {
                return false;
            }
        }
        return true;
    }

    private String phaseStatus(int index) {
        if (phases[index].isComplete()) {
            return "[✓]";
        }
        return phaseAvailable(index) ? "[ ]" : "[X]";
    }

    private void selectFirstIncompletePhase() {
        selectedPhaseIndex = 0;
        for (int i = 0; i < phases.length; i++) {
            if (!phases[i].isComplete() && phaseAvailable(i)) {
                selectedPhaseIndex = i;
                return;
            }
        }
    }

    private TuningPhase phaseSelector() {
        telemetry.addLine("Use Dpad Up and Down to choose a phase, then press B to select it.");
        telemetry.addLine();

        for (int i = 0; i < phases.length; i++) {
            String cursor = i == selectedPhaseIndex ? " <" : "";
            telemetry.addLine(phaseStatus(i) + " " + TunerPhase.values()[i].name().replace("_", " ") + cursor);
        }

        telemetry.update();

        if (gamepad1.dpadUpWasPressed()) {
            do {
                selectedPhaseIndex = (selectedPhaseIndex - 1 + phases.length) % phases.length;
            } while (!phaseAvailable(selectedPhaseIndex));
        } else if (gamepad1.dpadDownWasPressed()) {
            do {
                selectedPhaseIndex = (selectedPhaseIndex + 1) % phases.length;
            } while (!phaseAvailable(selectedPhaseIndex));
        } else if (gamepad1.bWasPressed()) {
            return phases[selectedPhaseIndex];
        }

        return null;
    }
}
