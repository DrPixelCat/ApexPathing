package org.firstinspires.ftc.teamcode.apexpathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.function.Function;
import java.util.function.Predicate;

import core.Follower;
import core.ApexConstants;
import core.FollowerConstants;
import geometry.Pose;
import tuning.follower.phases.CentripetalPhase;

import tuning.follower.phases.DrivePhase;
import tuning.follower.phases.FeedforwardTuner;
import tuning.follower.phases.MotionCharacterizationPhase;
import tuning.follower.phases.HeadingPhase;
import tuning.follower.phases.LimitsPhase;
import tuning.follower.phases.ProfiledTrackingPhase;
import tuning.follower.TunerContext;
import tuning.follower.TuningPhase;

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
                constants.angularCoeffs.kP != 0.0),
        DRIVE(DrivePhase::new, constants ->
                constants.translationalCoeffs.kP != 0.0),
        LIMITS(LimitsPhase::new, constants ->
                constants.forwardVelLimitIn != 0.0 &&
                        constants.forwardAccelLimitIn != 0.0 &&
                        (!constants.requiresStrafeLimits() || (constants.strafeVelLimitIn != 0.0 &&
                        constants.strafeAccelLimitIn != 0.0)) &&
                        constants.angularVelLimitRad != 0.0 &&
                        constants.angularAccelLimitRad != 0.0),
        CENTRIPETAL(CentripetalPhase::new, constants ->
                constants.kCentripetal != 0.0),
        PROFILED_TRACKING(ProfiledTrackingPhase::new, constants ->
                constants.profiledTrackingTuned);

        final Function<TunerContext, TuningPhase> phaseFactory;
        final Predicate<FollowerConstants> isTunedPredicate;
        boolean tuned;

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
        context.setFollower(new Follower(createConstants(), hardwareMap, true));
        context.constants.drivetrainType = context.getFollower().getDrivetrain().getDrivetrainType();

        for (Phase phase : phases) { phase.updateTunedStatus(context.constants); }
        selectFirstIncompletePhase();

        while (opModeInInit() && !isPhaseSelected) {
            context.updateDebugMode(true);
            isPhaseSelected = phaseSelector();
            sleep(20);
        }

        while (opModeInInit() && isPhaseSelected) {
            context.updateDebugMode(false);
            telemetry.clearAll();
            context.addInterfaceHeader();
            telemetry.addLine("Press Start to run the tuner.");
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
                selectedPhaseOrdinal.updateTunedStatus(context.constants);

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

        context.getFollower().stop();
        resetPhaseSelection();
    }

    /** Supplies the robot configuration, allowing simulator-specific tuner OpModes. */
    protected ApexConstants createConstants() { return new Constants(); }

    private boolean saveCurrentProfile() {
        while (opModeIsActive()) {
            context.beginFrame();
            if (context.saveConstants()) { return true; }
            context.getFollower().stop();
            telemetry.addLine("Press A to retry saving. Stop to discard unsaved changes.");
            context.publishFrame(false);
            while (opModeIsActive()) {
                if (gamepad1.aWasPressed()) { break; }
                sleep(20);
            }
        }
        return false;
    }

    /** A reused simulator OpMode instance must always reopen at the phase picker. */
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
        if (phase.tuned) { return "[DONE]"; }
        return phaseAvailable(phase) ? "[READY]" : "[LOCKED]";
    }

    private void selectFirstIncompletePhase() {
        selectedPhaseOrdinal = phases[0];
        for (int i = 0; i < phaseAmount; i++) {
            if (!phases[i].tuned && phaseAvailable(phases[i]) && applicable(phases[i])) {
                selectedPhaseOrdinal = phases[i];
                return;
            }
        }
    }

    private boolean phaseSelector() {
        telemetry.clearAll();
        context.addInterfaceHeader();
        if (context.getFollower().getDrivetrain() instanceof drivetrains.DualActuated) {
            telemetry.addLine("Dpad Left/Right: choose TANK or HOLONOMIC profile.");
            if (gamepad1.dpadLeftWasPressed() || gamepad1.dpadRightWasPressed()) {
                context.getFollower().stop();
                drivetrains.DualActuated drive = (drivetrains.DualActuated) context.getFollower().getDrivetrain();
                if (drive.isHolonomic()) { drive.activateTractionState(); }
                else { drive.activateHolonomicState(); }
                context.getFollower().update(false);
                for (Phase item : phases) { item.updateTunedStatus(context.constants); }
                selectFirstIncompletePhase();
            }
            if (context.constants.hasUnassignedLegacy()) {
                telemetry.addLine("Y: assign legacy values to " + context.constants.getActiveProfile());
                if (gamepad1.yWasPressed()) {
                    context.constants.assignLegacyToActiveProfile();
                    context.getFollower().reset();
                    context.saveConstants();
                    for (Phase item : phases) { item.updateTunedStatus(context.constants); }
                    selectFirstIncompletePhase();
                }
            }
        }
        telemetry.addLine("Select a tuning phase");
        telemetry.addLine("Use Dpad Up and Down to choose a phase, then press A to select it.");

        for (int i = 0; i < phaseAmount; i++) {
            String cursor = i == selectedPhaseOrdinal.ordinal() ? " <" : "";
            telemetry.addLine(phaseStatus(phases[i]) + " " +
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

    private static boolean validFeedforward(double ks, double kv, double ka) {
        return Double.isFinite(ks) && Double.isFinite(kv) && Double.isFinite(ka)
                && ks >= 0 && kv > 0 && ka >= 0;
    }

    private static String phaseDisplayName(Phase phase) {
        if (phase == Phase.DRIVE_FEEDFORWARD) { return "DRIVE FEEDFORWARD SYSID"; }
        if (phase == Phase.HEADING_FEEDFORWARD) { return "HEADING FEEDFORWARD SYSID"; }
        return phase.name().replace('_', ' ');
    }

    private void finishTuningWorkflow() {
        telemetry.clearAll();
        context.addInterfaceHeader();
        telemetry.addLine("Follower tuning complete for " + context.constants.getActiveProfile() + ".");
        if (context.getFollower().getDrivetrain() instanceof drivetrains.DualActuated) {
            telemetry.addLine("Run this tuner again and select the other mode to tune it separately.");
        }

        // FTCodeSim does not move its Driver Station out of RUNNING when a LinearOpMode calls
        // requestOpModeStop(). Keep the final lifecycle alive until the red Stop button is used.
        if (Boolean.getBoolean(UNLOCK_PHASES_PROPERTY)) {
            telemetry.addLine("Press the red STOP button to finish this simulation.");
            telemetry.update();
            while (opModeIsActive()) {
                context.updateDebugMode(false);
                sleep(50);
            }
            return;
        }

        telemetry.update();
        requestOpModeStop();
    }
}
