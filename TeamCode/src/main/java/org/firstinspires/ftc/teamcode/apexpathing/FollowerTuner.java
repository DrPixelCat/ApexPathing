package org.firstinspires.ftc.teamcode.apexpathing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import tuning.TunerContext;
import tuning.TuningPhase;
import tuning.HeadingPhase;
import tuning.TranslationalPhase;
import tuning.VelocityFeedforwardPhase;
import tuning.MovementLimitsPhase;

import core.Follower;
import geometry.Pose;

/**
 * Unified follower tuning class for Apex Pathing
 *
 * @author Sohum Arora - 22985 Paraducks
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
@Configurable
@TeleOp(name = "Follower Tuner", group = "Apex Pathing Tuning")
public class FollowerTuner extends LinearOpMode {
    private static TunerContext context;

    private String[] phaseSelectorStatuses;
    private int selectedPhaseIndex = 0;
    private static TuningPhase currentPhase = null;

    enum Phase {
        HEADING_PDS,
        TRANSLATIONAL_PDS,
        VELOCITY_FEEDFORWARD,
        MOVEMENT_LIMITS
    }

    private TuningPhase[] phases;

    @Override
    public void runOpMode() {
        context = new TunerContext(this);
        context.setFollower(new Follower(new Constants(), hardwareMap));

        phases = new TuningPhase[]{
                new HeadingPhase(context),
                new TranslationalPhase(context),
                new VelocityFeedforwardPhase(context),
                new MovementLimitsPhase(context)
        };

        resetPhaseSelection(); // Initialize the phase selector statuses

        while (opModeInInit() && currentPhase == null) {
            TuningPhase selectedPhase = phaseSelector(); // Will remain null until a phase is selected
            if (selectedPhase != null) {
                currentPhase = selectedPhase;
            }

            if (currentPhase != null) {
                context.getFollower().setPose(Pose.zero());
                telemetry.addLine("Press the start button to run the tuner");
                telemetry.addLine("Make sure you have adequate space to run the robot safely!");
                telemetry.update();
            }
        }

        waitForStart();

        while (opModeIsActive() && selectedPhaseIndex < phases.length) {
            currentPhase = phases[selectedPhaseIndex];
            currentPhase.run(this);

            if (TuningPhase.complete) {
                context.saveConstants();
                selectedPhaseIndex++;
            } else { // Running the tuning phaes failed so stop safely
                requestOpModeStop();
                return;
            }
        }

        context.getFollower().stop();
        currentPhase = null;
        requestOpModeStop();
    }

    private void resetPhaseSelection() {
        boolean headingIsTuned = context.constants.headingCoeffs.kP != 0.0;
        boolean translationalIsTuned = context.constants.translationalCoeffs.kP != 0.0;
        boolean velocityFFIsTuned = context.constants.translationalKV != 0.0;
        boolean movementLimitsIsTuned = context.constants.strafeAccelLimitIn != 0.0;

        String headingStatus = headingIsTuned ? "[✓]" : "[ ]"; // Heading is always available to tune first.
        String transStatus = translationalIsTuned ? "[✓]" : (headingIsTuned ? "[ ]" : "[X]");
        String velStatus = velocityFFIsTuned ? "[✓]" : (translationalIsTuned ? "[ ]" : "[X]");
        String movementStatus = movementLimitsIsTuned ? "[✓]" : (velocityFFIsTuned ? "[ ]" : "[X]");

        this.phaseSelectorStatuses = new String[]{
                headingStatus,
                transStatus,
                velStatus,
                movementStatus
        };

        this.selectedPhaseIndex = 0;

        for (int i = 0; i < phaseSelectorStatuses.length; i++) {
            if (phaseSelectorStatuses[i].equals("[ ]")) {
                this.selectedPhaseIndex = i;
                break;
            }
        }
    }

    private TuningPhase phaseSelector() {
        telemetry.addLine(
                "The Apex Pathing tuners are listed in order of execution below."
        );
        telemetry.addLine(
                "[✓] = Already Tuned (you can still select to retune)," +
                        "[X] = Not available to tune (incomplete tuners before it)," +
                        "[!] = Next tuner to run. The cursor ('<') is here by default."
        );
        telemetry.addLine(
                "Use the Dpad Up and Down buttons to cycle through phases, " +
                        "then press B to open the selected phase."
        );
        telemetry.addLine();

        for (int i = 0; i < phases.length; i++) {
            String cursor = i == selectedPhaseIndex ? " <" : "";

            telemetry.addLine(
                    phaseSelectorStatuses[i] + " " +
                            Phase.values()[i].toString().replace("_", " ") +
                            cursor
            );
        }

        telemetry.update();

        if (gamepad1.dpadUpWasPressed()) {
            selectedPhaseIndex =
                    (selectedPhaseIndex - 1 + phases.length) % phases.length;

            // Don't allow selection of unavailable phases
            while (phaseSelectorStatuses[selectedPhaseIndex].equals("[X]")) {
                selectedPhaseIndex =
                        (selectedPhaseIndex - 1 + phases.length) % phases.length;
            }
        } else if (gamepad1.dpadDownWasPressed()) {
            selectedPhaseIndex =
                    (selectedPhaseIndex + 1) % phases.length;

            while (phaseSelectorStatuses[selectedPhaseIndex].equals("[X]")) {
                selectedPhaseIndex =
                        (selectedPhaseIndex + 1) % phases.length;
            }
        } else if (gamepad1.bWasPressed()) {
            return phases[selectedPhaseIndex];
        }

        return null; // Phase hasn't been selected yet
    }
}