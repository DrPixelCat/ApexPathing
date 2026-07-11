package tuning;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import controllers.PDSController.PDSCoefficients;
import drivetrains.CoaxialSwerve;
import geometry.Angle;

/** Coaxial swerve offset and steering phase for FollowerTuner. */
public class SwervePhase extends TunePhase {
    private static final String[] MODULES = {"FL", "FR", "BL", "BR"};
    private static final double[] AUTO_TARGETS = {0.0, 90.0, -90.0, 0.0};

    private final ElapsedTime timer = new ElapsedTime();
    private CoaxialSwerve swerve;
    private PDSCoefficients gains;
    private TuneValue[] values;
    private int selected;
    private int autoTarget;
    private double targetDegrees;
    private double maxErrorDegrees;

    public SwervePhase(TuneContext context) { super(context); }

    @Override protected String name() { return "Swerve"; }
    @Override protected boolean hasManual() { return true; }
    @Override protected boolean hasAuto() { return true; }

    @Override
    protected void start() {
        if (!(context.getFollower().getDrivetrain() instanceof CoaxialSwerve)) {
            throw new IllegalStateException("Swerve phase requires CoaxialSwerve");
        }
        swerve = (CoaxialSwerve) context.getFollower().getDrivetrain();
        gains = swerve.getSteeringGains();
        targetDegrees = 0.0;
        maxErrorDegrees = 0.0;
        selected = 0;
        autoTarget = 0;
        values = new TuneValue[]{
                new TuneValue("Steering kP", () -> gains.kP, value -> gains.kP = value,
                        0.01, 0.0, 10.0),
                new TuneValue("Steering kD", () -> gains.kD, value -> gains.kD = value,
                        0.002, 0.0, 10.0),
                new TuneValue("Steering kS", () -> gains.kS, value -> gains.kS = value,
                        0.002, 0.0, 0.5),
                new TuneValue("kS deadzone", () -> gains.kSDeadzone,
                        value -> gains.kSDeadzone = value, 0.002, 0.0, Math.PI),
                new TuneValue("Target degrees", () -> targetDegrees,
                        value -> targetDegrees = value, 5.0, -180.0, 180.0)
        };
        timer.reset();
    }

    @Override
    protected boolean runManual(boolean aPressed, boolean bPressed) {
        Gamepad gamepad = context.getGamepad();
        if (bPressed) {
            swerve.stopSteering();
            return true;
        }
        if (gamepad.dpadLeftWasPressed()) selected = (selected + values.length - 1) % values.length;
        if (gamepad.dpadRightWasPressed()) selected = (selected + 1) % values.length;
        double scale = gamepad.right_bumper ? 10.0 : (gamepad.left_bumper ? 0.1 : 1.0);
        if (gamepad.dpadUpWasPressed()) values[selected].adjust(1, scale);
        if (gamepad.dpadDownWasPressed()) values[selected].adjust(-1, scale);

        if (gamepad.x) swerve.aimModules(Angle.fromDeg(targetDegrees));
        else swerve.stopSteering();

        showHelp();
        showAngles();
        return false;
    }

    @Override
    protected boolean runAuto(boolean aPressed, boolean bPressed) {
        if (autoTarget >= AUTO_TARGETS.length) {
            swerve.stopSteering();
            return true;
        }
        targetDegrees = AUTO_TARGETS[autoTarget];
        swerve.aimModules(Angle.fromDeg(targetDegrees));
        double[] angles = swerve.moduleAngles();
        for (double angle : angles) {
            double error = Angle.normalize(Math.toRadians(targetDegrees) - angle);
            maxErrorDegrees = Math.max(maxErrorDegrees, Math.abs(Math.toDegrees(error)));
        }
        context.getTelemetry().addData("Target", targetDegrees);
        showAngles();
        if (timer.seconds() >= 1.5) {
            autoTarget++;
            timer.reset();
        }
        return false;
    }

    private void showHelp() {
        context.getTelemetry().addLine("D-pad Left/Right - choose value");
        context.getTelemetry().addLine("D-pad Up/Down - adjust");
        context.getTelemetry().addLine("Hold X - aim modules, B - finish");
        for (int i = 0; i < values.length; i++) {
            context.getTelemetry().addData(i == selected ? "> " + values[i].getName() :
                    "  " + values[i].getName(), "%.5f", values[i].get());
        }
    }

    private void showAngles() {
        double[] raw = swerve.rawAngles();
        double[] corrected = swerve.moduleAngles();
        for (int i = 0; i < MODULES.length; i++) {
            context.getTelemetry().addData(MODULES[i] + " raw", "%.2f deg", Math.toDegrees(raw[i]));
            context.getTelemetry().addData(MODULES[i] + " offset", "%.2f deg",
                    Math.toDegrees(Angle.normalize(-raw[i])));
            context.getTelemetry().addData(MODULES[i] + " angle", "%.2f deg",
                    Math.toDegrees(corrected[i]));
        }
    }

    @Override
    protected void showResults() {
        context.getTelemetry().addData("Steering kP", gains.kP);
        context.getTelemetry().addData("Steering kD", gains.kD);
        context.getTelemetry().addData("Steering kS", gains.kS);
        context.getTelemetry().addData("Largest test error", "%.2f deg", maxErrorDegrees);
        context.getTelemetry().addLine("Copy the four displayed offsets into Constants");
    }
}
