package tuning;

import com.qualcomm.robotcore.util.ElapsedTime;

import geometry.Angle;
import geometry.Dist;
import geometry.Pose;
import geometry.Vector;
import paths.builders.Builder;
import paths.movements.FollowerMovement;

/** Runs repeatable follower checks without changing constants. */
public class CheckPhase extends TunePhase {
    private enum Trial { FORWARD, RETURN, STRAFE, STRAFE_RETURN, TURN_90, TURN_0, DONE }

    private static final double TIMEOUT_SECONDS = 8.0;
    private final ElapsedTime timer = new ElapsedTime();

    private Trial trial;
    private Pose target;
    private double worstPositionError;
    private double worstHeadingError;
    private int timeouts;
    private boolean holonomic;
    private boolean trialRunning;

    public CheckPhase(TuneContext context) { super(context); }

    @Override protected String name() { return "Follower Check"; }
    @Override protected boolean hasManual() { return true; }
    @Override protected boolean hasAuto() { return true; }

    @Override
    protected void start() {
        holonomic = context.getFollower().getDrivetrain().isHolonomic();
        trial = Trial.FORWARD;
        target = context.getFollower().getPose();
        worstPositionError = 0.0;
        worstHeadingError = 0.0;
        timeouts = 0;
        trialRunning = false;
        if (!isManual()) startTrial(trial);
    }

    @Override
    protected boolean runManual(boolean aPressed, boolean bPressed) {
        if (context.getFollower().isBusy()) {
            checkTimeout();
            if (bPressed) {
                context.getFollower().stop();
                trialRunning = false;
            }
        } else {
            if (trialRunning) {
                recordError();
                trialRunning = false;
            }
            if (bPressed) return true;
            if (context.getGamepad().dpadLeftWasPressed()) trial = previous(trial);
            if (context.getGamepad().dpadRightWasPressed()) trial = next(trial);
            if (aPressed && trial != Trial.DONE) startTrial(trial);
        }
        context.getTelemetry().addLine("D-pad Left/Right - choose check");
        context.getTelemetry().addLine("A - run, B - stop/finish");
        showStatus();
        return false;
    }

    @Override
    protected boolean runAuto(boolean aPressed, boolean bPressed) {
        checkTimeout();
        if (!context.getFollower().isBusy()) {
            recordError();
            trial = next(trial);
            if (!holonomic && trial == Trial.STRAFE) trial = Trial.TURN_90;
            if (trial == Trial.DONE) return true;
            startTrial(trial);
        }
        showStatus();
        return false;
    }

    private void startTrial(Trial selected) {
        FollowerMovement movement = makeTrial(selected);
        target = movement.getEndPose();
        context.getFollower().follow(movement);
        timer.reset();
        trialRunning = true;
    }

    private FollowerMovement makeTrial(Trial selected) {
        Pose current = context.getFollower().getPose();
        switch (selected) {
            case FORWARD: return makePath(current, pose(36, 0, 0));
            case RETURN: return makePath(current, pose(0, 0, 0));
            case STRAFE: return makePath(current, pose(0, 24, 0));
            case STRAFE_RETURN: return makePath(current, pose(0, 0, 0));
            case TURN_90: return Builder.turn(current).turnTo(Angle.fromDeg(90)).quickBuild();
            case TURN_0: return Builder.turn(current).turnTo(Angle.zero()).quickBuild();
            default: throw new IllegalStateException("No movement for " + selected);
        }
    }

    private FollowerMovement makePath(Pose current, Pose end) {
        return holonomic ? Builder.holonomicPath(current, end).quickBuild() :
                Builder.tankPath(current, end).quickBuild();
    }

    private void checkTimeout() {
        if (context.getFollower().isBusy() && timer.seconds() > TIMEOUT_SECONDS) {
            context.getFollower().stop();
            timeouts++;
        }
    }

    private void recordError() {
        if (target == null) return;
        Pose error = target.minus(context.getFollower().getPose());
        worstPositionError = Math.max(worstPositionError, error.getVec().getMag().getIn());
        worstHeadingError = Math.max(worstHeadingError, Math.abs(error.getHeading().getRad()));
    }

    private Trial next(Trial value) {
        return value == Trial.DONE ? Trial.FORWARD : Trial.values()[value.ordinal() + 1];
    }

    private Trial previous(Trial value) {
        return value == Trial.FORWARD ? Trial.TURN_0 : Trial.values()[value.ordinal() - 1];
    }

    private Pose pose(double x, double y, double heading) {
        return new Pose(new Vector(Dist.fromIn(x), Dist.fromIn(y)), Angle.fromDeg(heading));
    }

    private void showStatus() {
        context.getTelemetry().addData("Check", trial);
        context.getTelemetry().addData("Pose", context.getFollower().getPose());
        context.getTelemetry().addData("Target", target);
        context.getTelemetry().addData("Worst position error", "%.3f in", worstPositionError);
        context.getTelemetry().addData("Worst heading error", "%.3f deg",
                Math.toDegrees(worstHeadingError));
        context.getTelemetry().addData("Timeouts", timeouts);
    }

    @Override
    protected void showResults() {
        showStatus();
        context.getTelemetry().addLine(timeouts == 0 ? "All checks completed" :
                "At least one check timed out");
    }
}
