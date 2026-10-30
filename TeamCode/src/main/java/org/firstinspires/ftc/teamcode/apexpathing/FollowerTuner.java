package org.firstinspires.ftc.teamcode.apexpathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.function.Function;
import java.util.function.Predicate;

import follower.Follower;
import config.ApexConstants;
import follower.FollowerConstants;
import paths.geometry.Pose;
import setup.follower.centripetal.CentripetalPhase;

import setup.follower.feedback.DrivePhase;
import setup.follower.sysid.FeedforwardTuner;
import setup.follower.characterization.MotionCharacterizationPhase;
import setup.follower.feedback.HeadingPhase;
import setup.follower.limits.LimitsPhase;
import setup.follower.tracking.ProfiledTrackingPhase;
import setup.follower.TunerContext;
import setup.follower.TuningPhase;

/**
 * This OpMode is used to tune the Apex Pathing Follower. It allows the user to select a tuning
 * phase at which to begin, then runs each remaining phase in order and saves after every phase.
 *
 * @author Sohum Arora - 22985 Paraducks
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
@TeleOp(name = "Follower Tuner", group = "Apex Pathing")
public class FollowerTuner extends LinearOpMode {
    /** Allows the desktop simulator to exercise phases without saved prerequisite constants. */
    public static final String UNLOCK_PHASES_PROPERTY = "apex.simulation.unlockTunerPhases";

    /**
     * Completion is determined by whether the last saved value of the phase's constants is non-zero
     * Tuners are ran in the order of the enum ordinals
     */
    enum Phase {
        DRIVE_FEEDFORWARD(context -> new FeedforwardTuner(context, FeedforwardTuner.Axis.DRIVE), constants ->
                validFeedforward(constants.translationalFeedforwardKS, constants.translationalKV, constants.translationalKA)),
        HEADING_FEEDFORWARD(context -> new FeedforwardTuner(context, FeedforwardTuner.Axis.HEADING), constants ->
                validFeedforward(constants.angularFeedforwardKS, constants.angularKV, constants.angularKA)),
        MOTION_CHARACTERIZATION(MotionCharacterizationPhase::new, constants ->
                constants.motionCharacterization != null),
        HEADING(HeadingPhase::new, constants ->
                positive(constants.angularCoeffs.kP)),
        DRIVE(DrivePhase::new, constants ->
                positive(constants.translationalCoeffs.kP)),
        LIMITS(LimitsPhase::new, constants ->
                positive(constants.forwardVelLimitIn) &&
                        positive(constants.forwardAccelLimitIn) &&
                        (!constants.requiresStrafeLimits() || (positive(constants.strafeVelLimitIn) &&
                        positive(constants.strafeAccelLimitIn))) &&
                        positive(constants.angularVelLimitRad) &&
                        positive(constants.angularAccelLimitRad)),
        CENTRIPETAL(CentripetalPhase::new, constants ->
                positive(constants.kCentripetal)),
        PROFILED_TRACKING(ProfiledTrackingPhase::new, constants ->
                constants.profiledTrackingTuned);

        final Function<TunerContext, TuningPhase> phaseFactory;
        final Predicate<FollowerConstants> isTunedPredicate;
        /** Compatibility diagnostic; runtime status is computed from this tuner's settings. */
        @Deprecated boolean tuned;

        Phase(Function<TunerContext, TuningPhase> phaseFactory,
              Predicate<FollowerConstants> isTunedPredicate) {
            this.phaseFactory = phaseFactory;
            this.isTunedPredicate = isTunedPredicate;
        }

        TuningPhase create(TunerContext context) {
            return phaseFactory.apply(context);
        }

        void updateTunedStatus(FollowerConstants constants) {
            tuned = isTunedPredicate.test(constants);
        }
    }

    private static final Phase[] phases = Phase.values();
    private static final int phaseAmount = phases.length;

    private TunerContext context;
    private Phase selectedPhaseOrdinal;
    private TuningPhase phase;
    private boolean isPhaseSelected = false;

    @Override
    public void runOpMode() {
        resetPhaseSelection();
        context = new TunerContext(this);
        context.setFollower(Follower.create(createConstants(), hardwareMap, true));
        try {
            context.constants.drivetrainType = context.getFollower().getDrivetrain().getDrivetrainType();


            selectFirstIncompletePhase();

            while (opModeInInit() && !isPhaseSelected) {
                context.updateDebugMode(true);
                isPhaseSelected = phaseSelector();
                sleep(20);
            }

            while (opModeInInit() && isPhaseSelected) {
                context.updateDebugMode(false);
                context.getView().clearAll();
                context.addInterfaceHeader();
        context.getView().addLine(setup.WorkflowText.STOP_HINT);
                context.getView().addLine("Press Start to run the tuner.");
                context.publishFrame(false);
                sleep(20);
            }

            // Starting the OpMode must never silently accept a highlighted option. If Start was
            // pressed before a phase was selected, keep presenting the same menu while RUNNING until
            // the user explicitly confirms a phase.
            while (opModeIsActive() && !isPhaseSelected) {
                context.updateDebugMode(true);
                isPhaseSelected = phaseSelector();
                if (!isPhaseSelected) {
                    context.getFollower().update();
                    context.getFollower().manual(gamepad1);
                }
                sleep(20);
            }
            if (!opModeIsActive()) {
                context.getFollower().stop();
                resetPhaseSelection();
                return;
            }

            while (opModeIsActive()) {
                if (phase.run(this) && opModeIsActive() && !isStopRequested()) {
                    if (selectedPhaseOrdinal != Phase.PROFILED_TRACKING) {
                        context.constants.profiledTrackingTuned = false;
                    }
                    if (!saveCurrentProfile()) { break; }


                    Phase nextPhase = nextPhase(selectedPhaseOrdinal);
                    while (nextPhase != null && !applicable(nextPhase)) { nextPhase = nextPhase(nextPhase); }
                    if (nextPhase == null) {
                        finishTuningWorkflow();
                        break;
                    }

                    context.getFollower().stop();
                    context.getFollower().enableControllers();
                    context.getFollower().setPose(Pose.zero());
                    selectedPhaseOrdinal = nextPhase;
                    selectPhase();
                }
            }

        } finally {
            context.getFollower().stop();
            resetPhaseSelection();
        }
    }

    /** Supplies the robot configuration, allowing simulator-specific tuner OpModes. */
    protected ApexConstants createConstants() { return new Constants(); }

    private boolean saveCurrentProfile() {
        context.beginFrame();
        if (context.saveConstants()) { return true; }
        context.getFollower().stop();
        context.getView().addLine(setup.WorkflowText.STOP_HINT);
        context.publishFrame(false);
        while (opModeIsActive() && !isStopRequested()) { sleep(20); }
        return false;
    }

    private void resetPhaseSelection() {
        selectedPhaseOrdinal = null;
        phase = null;
        isPhaseSelected = false;
    }

    /** Every phase remains selectable; completion state is informational, not a menu lock. */
    static boolean phaseAvailable(Phase phase) {
        return true;
    }

    private boolean applicable(Phase phase) {
        return phase != Phase.CENTRIPETAL || context.getFollower().getDrivetrain().isHolonomic();
    }

    private String phaseStatus(Phase phase) {
        if (!applicable(phase)) { return "[N/A]"; }
        if (phase.isTunedPredicate.test(context.constants)) { return "[DONE]"; }
        return phaseAvailable(phase) ? "[READY]" : "[LOCKED]";
    }

    private void selectFirstIncompletePhase() {
        selectedPhaseOrdinal = phases[0];
        for (int i = 0; i < phaseAmount; i++) {
            if (!phases[i].isTunedPredicate.test(context.constants) && phaseAvailable(phases[i]) && applicable(phases[i])) {
                selectedPhaseOrdinal = phases[i];
                return;
            }
        }
    }

    private boolean phaseSelector() {
        context.getView().clearAll();
        context.addInterfaceHeader();
        context.getView().addLine(setup.WorkflowText.STOP_HINT);
        if (context.getFollower().getDrivetrain() instanceof drivetrains.DualActuated) {
            context.getView().addLine("Dpad Left/Right: choose TANK or HOLONOMIC profile.");
            if (gamepad1.dpadLeftWasPressed() || gamepad1.dpadRightWasPressed()) {
                context.getFollower().stop();
                drivetrains.DualActuated drive = (drivetrains.DualActuated) context.getFollower().getDrivetrain();
                if (drive.isHolonomic()) { drive.activateTractionState(); }
                else { drive.activateHolonomicState(); }
                context.getFollower().update(false);

                selectFirstIncompletePhase();
            }
            if (context.constants.hasUnassignedLegacy()) {
                context.getView().addLine("Y: assign legacy values to " + context.constants.getActiveProfile());
                if (gamepad1.yWasPressed()) {
                    context.constants.assignLegacyToActiveProfile();
                    context.getFollower().reset();
                    context.saveConstants();

                    selectFirstIncompletePhase();
                }
            }
        }
        context.getView().addLine("Select a tuning phase");
        context.getView().addLine("Use Dpad Up and Down to choose a phase, then press A to select it.");

        for (int i = 0; i < phaseAmount; i++) {
            String cursor = i == selectedPhaseOrdinal.ordinal() ? " <" : "";
            context.getView().addLine(phaseStatus(phases[i]) + " " +
                    phaseDisplayName(phases[i]) + cursor);
        }

        context.publishFrame(opModeIsActive()
                && (!(context.getFollower().getDrivetrain() instanceof drivetrains.DualActuated)
                || !((drivetrains.DualActuated) context.getFollower().getDrivetrain()).isTransitioning()));

        if (gamepad1.dpadUpWasPressed()) {
            selectedPhaseOrdinal = phases[
                    (selectedPhaseOrdinal.ordinal() - 1 + phaseAmount) % phaseAmount];
        } else if (gamepad1.dpadDownWasPressed()) {
            selectedPhaseOrdinal = phases[
                    (selectedPhaseOrdinal.ordinal() + 1) % phaseAmount];
        } else if (gamepad1.aWasPressed() && phaseAvailable(selectedPhaseOrdinal) && applicable(selectedPhaseOrdinal)) {
            selectPhase();
            return true;
        }

        return false;
    }

    private void selectPhase() {
        phase = selectedPhaseOrdinal.create(context);
        isPhaseSelected = true;
    }

    static Phase nextPhase(Phase current) {
        int nextOrdinal = current.ordinal() + 1;
        return nextOrdinal < phaseAmount ? phases[nextOrdinal] : null;
    }

    private static boolean positive(double value) { return Double.isFinite(value) && value > 0; }

    private static boolean validFeedforward(double ks, double kv, double ka) {
        return Double.isFinite(ks) && Double.isFinite(kv) && Double.isFinite(ka)
                && ks >= 0 && kv > 0 && ka >= 0;
    }

    private static String phaseDisplayName(Phase phase) {
        if (phase == Phase.PROFILED_TRACKING) { return "TRAJECTORY TRACKING"; }
        if (phase == Phase.DRIVE_FEEDFORWARD) { return "DRIVE FEEDFORWARD SYSID"; }
        if (phase == Phase.HEADING_FEEDFORWARD) { return "HEADING FEEDFORWARD SYSID"; }
        return phase.name().replace('_', ' ');
    }

    private void finishTuningWorkflow() {
        context.getView().clearAll();
        context.addInterfaceHeader();
        context.getView().addLine(setup.WorkflowText.STOP_HINT);
        context.getView().addLine("Follower tuning complete for " + context.constants.getActiveProfile() + ".");
        if (context.getFollower().getDrivetrain() instanceof drivetrains.DualActuated) {
            context.getView().addLine("Run this tuner again and select the other mode to tune it separately.");
        }

        // FTCodeSim does not move its Driver Station out of RUNNING when a LinearOpMode calls
        // requestOpModeStop(). Keep the final lifecycle alive until the red Stop button is used.
        if (Boolean.getBoolean(UNLOCK_PHASES_PROPERTY)) {
            context.getView().addLine("Press the red STOP button to finish this simulation.");
            context.getView().update();
            while (opModeIsActive()) {
                context.updateDebugMode(false);
                sleep(50);
            }
            return;
        }

        context.getView().update();
        requestOpModeStop();
    }
}
