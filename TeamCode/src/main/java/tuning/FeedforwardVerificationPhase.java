package tuning;

import com.qualcomm.robotcore.util.ElapsedTime;

import geometry.Pose;

/** Operator-started feedforward-only profiles. Records data without fitting or judging results. */
public class FeedforwardVerificationPhase extends TuningPhase {
    private static final String[] HEADER = {"time_s", "run", "state", "axis", "direction", "profile_time_s",
            "target_velocity", "target_acceleration", "raw_velocity", "filtered_velocity", "raw_acceleration",
            "filtered_acceleration", "velocity_error", "interval_target_velocity", "interval_target_acceleration",
            "interval_requested_power", "interval_applied_power", "next_requested_power", "next_applied_power",
            "limited", "battery_v", "kS", "kV", "kA"};
    private final ElapsedTime timer = new ElapsedTime();
    private FeedforwardTuner.TrapezoidProfile angularProfile, driveProfile, activeProfile;
    private boolean angular = true, positive = true, running;
    private int run;
    private double previousRequested, previousApplied, previousTargetV, previousTargetA;

    public FeedforwardVerificationPhase(TunerContext context) { super(context); }
    @Override protected String getPhaseName() { return "Feedforward Verification (CSV only)"; }
    @Override protected boolean manualTuneIsPossible() { return true; }
    @Override protected boolean autoTuneIsPossible() { return false; }
    @Override protected boolean autoTuned() { return false; }

    @Override protected void showPreRunInstructions() {
        context.getTelemetry().addLine("Feedforward only. No PID, fitting, or pass/fail decisions.");
        context.getTelemetry().addLine("Each run requires X. Check clearance and logged profile distance first.");
    }

    @Override protected void init() {
        context.getFollower().stop();
        context.getFollower().disableControllers();
        angularProfile = new FeedforwardTuner.TrapezoidProfile(context.constants.angularVelLimitRad / 2,
                context.constants.angularAccelLimitRad / 2);
        driveProfile = new FeedforwardTuner.TrapezoidProfile(context.constants.forwardVelLimitIn / 2,
                context.constants.forwardAccelLimitIn / 2);
        running = false;
        previousRequested = previousApplied = previousTargetV = previousTargetA = 0;
    }

    @Override protected boolean manualTuned() {
        if (!running) {
            if (opMode.gamepad1.leftBumperWasPressed()) { angular = !angular; }
            if (opMode.gamepad1.rightBumperWasPressed()) { positive = !positive; }
            activeProfile = angular ? angularProfile : driveProfile;
            if (opMode.gamepad1.aWasPressed()) { return true; }
            if (opMode.gamepad1.xWasPressed() && stationary()) {
                run++;
                running = true;
                previousRequested = previousApplied = previousTargetV = previousTargetA = 0;
                timer.reset();
                if (context.getLog() != null) { context.getLog().event(getPhaseName(), "run_start", label()); }
            }
        }

        if (running) {
            double t = timer.seconds();
            boolean abort = opMode.gamepad1.bWasPressed();
            boolean finished = t >= activeProfile.getTotalTime() + 0.75;
            double direction = positive ? 1 : -1;
            double targetV = activeProfile.getVel(t) * direction;
            double targetA = activeProfile.getAccel(t) * direction;
            double kS = angular ? context.constants.angularCoeffs.kS : context.constants.translationalCoeffs.kS;
            double kV = angular ? context.constants.angularKV : context.constants.translationalKV;
            double kA = angular ? context.constants.angularKA : context.constants.translationalKA;
            // At rest the friction term is zero. Acceleration sets the initial motion direction.
            double requested = abort || finished ? 0 : profilePower(kS, kV, kA, targetV, targetA);
            double clipped = FeedforwardTuner.clipManualPower(requested);
            Pose p = context.getFollower().getPose();
            Pose rv = context.getFollower().getLocalizer().getRawVel();
            Pose fv = context.getFollower().getVelocity();
            Pose ra = context.getFollower().getLocalizer().getRawAccel();
            Pose fa = context.getFollower().getAcceleration();
            double rawV = component(rv, p), filteredV = component(fv, p);
            double rawA = component(ra, p), filteredA = component(fa, p);
            context.getFollower().getDrivetrain().moveWithVectors(angular ? 0 : clipped, 0, angular ? clipped : 0);
            double applied = TuningMeasurements.appliedEffort(context.getFollower().getDrivetrain(), angular, requested);
            if (context.getLog() != null) {
                context.getLog().write("feedforward_verification", HEADER, context.getLog().seconds(), run,
                        abort ? "aborted" : finished ? "complete" : t >= activeProfile.getTotalTime() ? "settling" : "profile",
                        angular ? "angular" : "translation", direction, t, targetV, targetA,
                        rawV, filteredV, rawA, filteredA, targetV - filteredV,
                        previousTargetV, previousTargetA, previousRequested, previousApplied, requested, applied,
                        Math.abs(requested - applied) > 0.001, context.batteryVoltage(), kS, kV, kA);
            }
            previousTargetV = targetV;
            previousTargetA = targetA;
            previousRequested = requested;
            previousApplied = applied;
            if (abort || finished) {
                context.getFollower().stop();
                running = false;
                if (context.getLog() != null) { context.getLog().event(getPhaseName(), abort ? "run_aborted" : "run_complete", label()); }
            }
        }
        context.getTelemetry().addData("Profile", label());
        context.getTelemetry().addData("Profile travel", activeProfile.getTotalDistance() + (angular ? " rad" : " in"));
        context.getTelemetry().addLine(running ? "Recording. B: stop run." : "LB: drive/turn; RB: direction; X: start when stationary; A: finish.");
        context.getTelemetry().addLine("Repeat both directions for drive and turn. Review CSV response yourself.");
        context.getTelemetry().update();
        return false;
    }

    private boolean stationary() {
        Pose v = context.getFollower().getLocalizer().getRawVel();
        return Math.hypot(v.getX().getIn(), v.getY().getIn()) < 1
                && Math.abs(v.getHeading().getRad()) < 0.1;
    }

    static double profilePower(double kS, double kV, double kA, double velocity, double acceleration) {
        double sign = Math.abs(velocity) > 1e-9 ? Math.signum(velocity) : Math.signum(acceleration);
        return kS * sign + kV * velocity + kA * acceleration;
    }

    private double component(Pose motion, Pose pose) {
        return angular ? motion.getHeading().getRad() : TuningMeasurements.forward(motion, pose);
    }

    private String label() { return (angular ? "turn " : "drive ") + (positive ? "+" : "-"); }
    @Override protected void reportResults() { context.getTelemetry().addLine("Verification CSV recorded; no automatic assessment or coefficient changes."); }
}
