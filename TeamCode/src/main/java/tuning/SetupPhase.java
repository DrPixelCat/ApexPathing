package tuning;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Gamepad;

import geometry.Pose;

/**
 * Localizer verification before commencing tuning
 * @author Sohum Arora 22985 Paraducks
 */
public class SetupPhase extends TunePhase {
    private static final double TEST_POWER = 0.22;
    private static final double TEST_SECONDS = 0.45;
    private static final double MINIMUM_DELTA = 0.05;

    private final ElapsedTime timer = new ElapsedTime();
    private TuneAxis[] axes;
    private int index;
    private Pose start;
    private String failure;
    private boolean manualCheck;

    public SetupPhase(TuneContext context) { super(context); }

    @Override protected String name() { return "Setup Check"; }
    @Override protected boolean hasManual() { return true; }
    @Override protected boolean hasAuto() { return true; }
    @Override
    protected void start() {
        axes = context.getFollower().getDrivetrain().isHolonomic()
                ? new TuneAxis[]{TuneAxis.FORWARD,
                TuneAxis.STRAFE, TuneAxis.ANGULAR}
                : new TuneAxis[]{TuneAxis.FORWARD,
                TuneAxis.ANGULAR};
        index = 0;
        failure = null;
        manualCheck = isManual();
        context.getFollower().disableControllers();
        context.getFollower().stop();
        context.getFollower().setPose(Pose.zero());
        start = context.getFollower().getPose();
        timer.reset();
    }

    @Override
    protected boolean runManual(boolean aWasPressed, boolean bWasPressed) {
        Gamepad gamepad = context.getGamepad();
        if (bWasPressed) {
            context.getFollower().getDrivetrain().stop();
            return true;
        }
        if (aWasPressed) context.getFollower().setPose(Pose.zero());

        double forward = gamepad.dpad_up ? TEST_POWER :
                (gamepad.dpad_down ? -TEST_POWER : 0.0);
        double strafe = gamepad.dpad_left ? TEST_POWER :
                (gamepad.dpad_right ? -TEST_POWER : 0.0);
        double turn = gamepad.left_bumper ? TEST_POWER :
                (gamepad.right_bumper ? -TEST_POWER : 0.0);
        if (!context.getFollower().getDrivetrain().isHolonomic()) strafe = 0.0;
        context.getFollower().getDrivetrain().moveWithVectors(forward, strafe, turn);

        context.getTelemetry().addLine("MANUAL HARDWARE / LOCALIZER PREFLIGHT");
        context.getTelemetry().addLine("D-pad Up/Down - forward/backward");
        context.getTelemetry().addLine("D-pad Left/Right - left/right strafe");
        context.getTelemetry().addLine("Bumpers - counterclockwise/clockwise turn");
        context.getTelemetry().addLine("A - reset pose, B - finish");
        context.getTelemetry().addData("Pose", context.getFollower().getPose());
        context.getTelemetry().addData("Velocity", context.getFollower().getVelocity());
        return false;
    }

    @Override
    protected boolean runAuto(boolean aWasPressed, boolean bWasPressed) {
        TuneAxis axis = axes[index];
        context.driveAxis(axis, TEST_POWER);
        context.getTelemetry().addData("Testing", axis.label());

        if (timer.seconds() < TEST_SECONDS) return false;

        context.getFollower().getDrivetrain().stop();
        Pose delta = context.getFollower().getPose().minus(start);
        double expected = axis.position(delta);
        double largestOther = otherMovement(axis, delta);
        if (expected <= MINIMUM_DELTA) {
            failure = axis.label() + " command did not produce a positive " +
                    axis.label().toLowerCase() + " localizer delta. Check motor/encoder " +
                    "directions and the X-forward/Y-left convention.";
            return true;
        }
        if (largestOther > Math.abs(expected) * 0.75) {
            failure = axis.label() + " command was reported primarily on another axis. " +
                    "Fix localizer axis assignment before tuning.";
            return true;
        }

        index++;
        if (index >= axes.length) return true;
        context.getFollower().setPose(Pose.zero());
        start = context.getFollower().getPose();
        timer.reset();
        return false;
    }

    private double otherMovement(TuneAxis expected, Pose delta) {
        double largest = 0.0;
        for (TuneAxis candidate : TuneAxis.values()) {
            if (candidate == expected) continue;
            largest = Math.max(largest, Math.abs(candidate.position(delta)));
        }
        return largest;
    }

    @Override
    protected void showResults() {
        if (manualCheck) {
            context.getTelemetry().addLine("Manual preflight finished. Confirm every commanded " +
                    "axis moved in the matching positive and negative pose direction.");
            return;
        }
        if (failure == null) {
            context.getTelemetry().addLine("PASS: drivetrain commands and localizer axes agree.");
        } else {
            context.getTelemetry().addLine("FAIL: " + failure);
            context.getTelemetry().addLine("Do not run characterization until this is fixed.");
        }
    }
}
