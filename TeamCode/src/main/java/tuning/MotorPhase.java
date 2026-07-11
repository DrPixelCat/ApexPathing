package tuning;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

/** Checks each drive motor from inside the single follower tuner. */
public class MotorPhase extends TunePhase {
    private static final double POWER = 0.20;
    private static final String[] NAMES = {"Front Left", "Front Right", "Back Left", "Back Right"};

    private final ElapsedTime timer = new ElapsedTime();
    private int motor;
    private boolean driving;

    public MotorPhase(TuneContext context) { super(context); }

    @Override protected String name() { return "Motor Check"; }
    @Override protected boolean hasManual() { return true; }
    @Override protected boolean hasAuto() { return true; }

    @Override
    protected void start() {
        motor = 0;
        driving = true;
        stopMotors();
        timer.reset();
    }

    @Override
    protected boolean runManual(boolean aPressed, boolean bPressed) {
        Gamepad gamepad = context.getGamepad();
        if (bPressed) {
            stopMotors();
            return true;
        }
        if (gamepad.dpadLeftWasPressed()) motor = (motor + 3) % 4;
        if (gamepad.dpadRightWasPressed()) motor = (motor + 1) % 4;

        double power = gamepad.x ? POWER : (gamepad.y ? -POWER : 0.0);
        runMotor(motor, power);

        context.getTelemetry().addLine("Robot must be on blocks");
        context.getTelemetry().addLine("D-pad Left/Right - choose motor");
        context.getTelemetry().addLine("Hold X/Y - forward/reverse");
        context.getTelemetry().addLine("B - finish");
        context.getTelemetry().addData("Motor", NAMES[motor]);
        context.getTelemetry().addData("Power", power);
        return false;
    }

    @Override
    protected boolean runAuto(boolean aPressed, boolean bPressed) {
        if (motor >= 4) {
            stopMotors();
            return true;
        }

        if (driving) {
            runMotor(motor, POWER);
            context.getTelemetry().addData("Testing", NAMES[motor]);
            if (timer.seconds() >= 0.55) {
                stopMotors();
                driving = false;
                timer.reset();
            }
        } else if (timer.seconds() >= 0.35) {
            motor++;
            driving = true;
            timer.reset();
        }
        return false;
    }

    private void runMotor(int index, double power) {
        double[] powers = new double[4];
        powers[index] = power;
        context.getFollower().getDrivetrain().setPowers(
                powers[0], powers[1], powers[2], powers[3]);
    }

    private void stopMotors() {
        context.getFollower().getDrivetrain().stop();
    }

    @Override
    protected void showResults() {
        context.getTelemetry().addLine("Motor check complete. Confirm every installed motor " +
                "matched its label and direction.");
    }
}
