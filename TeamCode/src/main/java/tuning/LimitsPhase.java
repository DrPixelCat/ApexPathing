package tuning;

import com.qualcomm.robotcore.util.ElapsedTime;

import controllers.PDSController;
import geometry.Angle;
import geometry.Pose;

/**
 * @author Sohum Arora - 22985 Paraducks
 */
public class LimitsPhase extends TuningPhase {
    enum LimitStage {
        TRANSLATION,
        SETTLING,
        RUNNING
    }

    enum LimitTrial {
        FORWARD,
        BACKWARD,
        LEFT,
        RIGHT,
        COUNTERCLOCKWISE,
        CLOCKWISE
    }
    private static final double RUN_TIME = 2000.0;
    private static final double SETTLE_TIME = 800.0;

    private final TuningValues values;
    private final ElapsedTime timer = new ElapsedTime();
    private final double[][] maxima = new double[6][2];
    private PDSRoutine routine;
    private PDSController headingHold;
    private LimitStage stage;
    private int trial;
    private int selected;
    private double heldHeading;
    private boolean measured;
    private boolean complete;

    public LimitsPhase(TunerContext context, TuningValues values) {
        super(context);
        this.values = values;
        double[] limits = {values.forwardVelocity, values.forwardAcceleration, values.strafeVelocity,
                values.strafeAcceleration, values.angularVelocity, values.angularAcceleration,
                values.translationKV, values.translationKA, values.angularKV, values.angularKA};
        complete = values.translation.kP != 0.0 && values.allPositive(limits);
    }

    @Override
    protected String getPhaseName() {
        return "Movement Limits";
    }

    @Override
    protected boolean manualTuneIsPossible() {
        return true;
    }

    @Override
    protected boolean autoTuneIsPossible() {
        return true;
    }

    @Override
    public boolean isComplete() {
        return complete;
    }

    @Override
    protected void init() {
        complete = false;
        for (double[] maximum : maxima) {
            maximum[0] = 0.0;
            maximum[1] = 0.0;
        }
        trial = 0;
        selected = 0;
        measured = false;
        stage = LimitStage.TRANSLATION;
        headingHold = new PDSController(values.heading);
        headingHold.setAngularController();
        routine = new PDSRoutine(context, PDSRoutine.TuningAxis.DRIVE);
        routine.start();
    }

    private boolean updateMeasurements() {
        if (measured) {
            return true;
        }

        switch (stage) {
            case TRANSLATION:
                if (routine.update(context)) {
                    values.translation = values.copy(routine.getCoefficients());
                    context.getFollower().enableControllers();
                    context.getFollower().stop();
                    timer.reset();
                    stage = LimitStage.SETTLING;
                }
                break;
            case SETTLING:
                context.getFollower().stop();
                if (timer.milliseconds() >= SETTLE_TIME) {
                    if (trial >= LimitTrial.values().length) {
                        deriveValues();
                        measured = true;
                        return true;
                    }
                    heldHeading = context.getFollower().getPose().getHeading().getRad();
                    headingHold.reset();
                    timer.reset();
                    stage = LimitStage.RUNNING;
                }
                break;
            case RUNNING:
                runTrial();
                recordMaximums();
                if (timer.milliseconds() >= RUN_TIME) {
                    context.getFollower().stop();
                    trial++;
                    timer.reset();
                    stage = LimitStage.SETTLING;
                }
                break;
        }

        String step = stage == LimitStage.TRANSLATION ? "Translational PDS" :
                trial >= LimitTrial.values().length ? "Calculating" : LimitTrial.values()[trial].name();
        context.getTelemetry().addData("Step", step);
        context.getTelemetry().update();
        return false;
    }

    private void runTrial() {
        LimitTrial current = LimitTrial.values()[trial];
        double x = 0.0;
        double y = 0.0;
        double turn = 0.0;

        switch (current) {
            case FORWARD:
                x = 1.0;
                break;
            case BACKWARD:
                x = -1.0;
                break;
            case LEFT:
                y = 1.0;
                break;
            case RIGHT:
                y = -1.0;
                break;
            case COUNTERCLOCKWISE:
                turn = 1.0;
                break;
            case CLOCKWISE:
                turn = -1.0;
                break;
        }

        if (current != LimitTrial.COUNTERCLOCKWISE && current != LimitTrial.CLOCKWISE) {
            Angle currentHeading = context.getFollower().getPose().getHeading();
            double headingError = currentHeading.getShortestAngleTo(Angle.fromRad(heldHeading)).getRad();
            turn = Math.max(-1.0, Math.min(1.0, headingHold.calculate(headingError)));
        }

        context.getFollower().getDrivetrain().moveWithVectors(x, y, turn);
    }

    private void recordMaximums() {
        Pose velocity = context.getFollower().getVelocity();
        Pose acceleration = context.getFollower().getAcceleration();
        LimitTrial current = LimitTrial.values()[trial];
        double measuredVelocity;
        double measuredAcceleration;

        if (current == LimitTrial.FORWARD || current == LimitTrial.BACKWARD) {
            measuredVelocity = Math.abs(velocity.getX().getIn());
            measuredAcceleration = Math.abs(acceleration.getX().getIn());
        } else if (current == LimitTrial.LEFT || current == LimitTrial.RIGHT) {
            measuredVelocity = Math.abs(velocity.getY().getIn());
            measuredAcceleration = Math.abs(acceleration.getY().getIn());
        } else {
            measuredVelocity = Math.abs(velocity.getHeading().getRad());
            measuredAcceleration = Math.abs(acceleration.getHeading().getRad());
        }

        if (Double.isFinite(measuredVelocity)) {
            maxima[trial][0] = Math.max(maxima[trial][0], measuredVelocity);
        }
        if (Double.isFinite(measuredAcceleration)) {
            maxima[trial][1] = Math.max(maxima[trial][1], measuredAcceleration);
        }
    }

    private double weaker(LimitTrial first, LimitTrial second, int measurement) {
        return Math.min(maxima[first.ordinal()][measurement], maxima[second.ordinal()][measurement]);
    }

    private void deriveValues() {
        double fullForwardVelocity = weaker(LimitTrial.FORWARD, LimitTrial.BACKWARD, 0);
        double fullForwardAcceleration = weaker(LimitTrial.FORWARD, LimitTrial.BACKWARD, 1);
        double fullStrafeVelocity = weaker(LimitTrial.LEFT, LimitTrial.RIGHT, 0);
        double fullStrafeAcceleration = weaker(LimitTrial.LEFT, LimitTrial.RIGHT, 1);
        double fullAngularVelocity = weaker(LimitTrial.COUNTERCLOCKWISE, LimitTrial.CLOCKWISE, 0);
        double fullAngularAcceleration = weaker(LimitTrial.COUNTERCLOCKWISE, LimitTrial.CLOCKWISE, 1);
        double[] measurements = {fullForwardVelocity, fullForwardAcceleration, fullStrafeVelocity,
                fullStrafeAcceleration, fullAngularVelocity, fullAngularAcceleration};

        if (!values.allPositive(measurements)) {
            throw new IllegalStateException("A movement limit measurement was zero.");
        }

        values.forwardVelocity = fullForwardVelocity * 0.95;
        values.forwardAcceleration = fullForwardAcceleration * 0.95;
        values.strafeVelocity = fullStrafeVelocity * 0.95;
        values.strafeAcceleration = fullStrafeAcceleration * 0.95;
        values.angularVelocity = fullAngularVelocity * 0.95;
        values.angularAcceleration = fullAngularAcceleration * 0.95;
        values.translationKV = 1.0 / fullForwardVelocity;
        values.translationKA = 1.0 / fullForwardAcceleration;
        values.angularKV = 1.0 / fullAngularVelocity;
        values.angularKA = 1.0 / fullAngularAcceleration;
        context.getFollower().setMovementTuning(values.translation, values.translationKV, values.translationKA,
                values.angularKV, values.angularKA, values.forwardVelocity, values.strafeVelocity);
    }

    @Override
    protected void autoTuned() {
        if (updateMeasurements()) {
            values.saveMovement(context);
            complete = true;
        }
    }

    @Override
    protected void manualTuned() {
        if (!updateMeasurements()) {
            return;
        }

        if (opMode.gamepad1.leftBumperWasPressed()) {
            selected = (selected + 12) % 13;
        }
        if (opMode.gamepad1.rightBumperWasPressed()) {
            selected = (selected + 1) % 13;
        }

        double change = manualChange();
        if (change != 0.0) {
            adjustSelected(change);
            context.getFollower().setMovementTuning(values.translation, values.translationKV, values.translationKA,
                    values.angularKV, values.angularKA, values.forwardVelocity, values.strafeVelocity);
        }

        context.getTelemetry().addData("Selected", selectedName());
        context.getTelemetry().addData("Value", selectedValue());
        context.getTelemetry().addData("Increment", increment);
        context.getTelemetry().addData("Translation P / D / S", "%.6f / %.6f / %.6f", values.translation.kP,
                values.translation.kD, values.translation.kS);
        context.getTelemetry().addData("Forward vel / accel", "%.6f / %.6f", values.forwardVelocity,
                values.forwardAcceleration);
        context.getTelemetry().addData("Strafe vel / accel", "%.6f / %.6f", values.strafeVelocity,
                values.strafeAcceleration);
        context.getTelemetry().addData("Angular vel / accel", "%.6f / %.6f", values.angularVelocity,
                values.angularAcceleration);
        context.getTelemetry().addData("Translation kV / kA", "%.6f / %.6f", values.translationKV,
                values.translationKA);
        context.getTelemetry().addData("Angular kV / kA", "%.6f / %.6f", values.angularKV, values.angularKA);
        context.getTelemetry().addLine("Up/Down: change value");
        context.getTelemetry().addLine("Left/Right: change increment");
        context.getTelemetry().addLine("LB/RB: select value");
        context.getTelemetry().addLine("A: save");
        context.getTelemetry().update();

        if (opMode.gamepad1.aWasPressed()) {
            values.saveMovement(context);
            complete = true;
        }
    }

    private String selectedName() {
        String[] names = {"Translation P", "Translation D", "Translation S", "Forward Velocity",
                "Forward Acceleration", "Strafe Velocity", "Strafe Acceleration", "Angular Velocity",
                "Angular Acceleration", "Translation kV", "Translation kA", "Angular kV", "Angular kA"};
        return names[selected];
    }

    private double selectedValue() {
        double[] currentValues = {values.translation.kP, values.translation.kD, values.translation.kS,
                values.forwardVelocity, values.forwardAcceleration, values.strafeVelocity, values.strafeAcceleration,
                values.angularVelocity, values.angularAcceleration, values.translationKV, values.translationKA,
                values.angularKV, values.angularKA};
        return currentValues[selected];
    }

    private void adjustSelected(double change) {
        switch (selected) {
            case 0:
                values.translation.kP = Math.max(0.0, values.translation.kP + change);
                break;
            case 1:
                values.translation.kD = Math.max(0.0, values.translation.kD + change);
                break;
            case 2:
                values.translation.kS = Math.max(0.0, values.translation.kS + change);
                break;
            case 3:
                values.forwardVelocity = Math.max(0.0, values.forwardVelocity + change);
                break;
            case 4:
                values.forwardAcceleration = Math.max(0.0, values.forwardAcceleration + change);
                break;
            case 5:
                values.strafeVelocity = Math.max(0.0, values.strafeVelocity + change);
                break;
            case 6:
                values.strafeAcceleration = Math.max(0.0, values.strafeAcceleration + change);
                break;
            case 7:
                values.angularVelocity = Math.max(0.0, values.angularVelocity + change);
                break;
            case 8:
                values.angularAcceleration = Math.max(0.0, values.angularAcceleration + change);
                break;
            case 9:
                values.translationKV = Math.max(0.0, values.translationKV + change);
                break;
            case 10:
                values.translationKA = Math.max(0.0, values.translationKA + change);
                break;
            case 11:
                values.angularKV = Math.max(0.0, values.angularKV + change);
                break;
            case 12:
                values.angularKA = Math.max(0.0, values.angularKA + change);
                break;
        }
    }

    @Override
    protected void reportResults() {
        context.getTelemetry().addData("Forward Velocity", values.forwardVelocity);
        context.getTelemetry().addData("Forward Acceleration", values.forwardAcceleration);
        context.getTelemetry().addData("Strafe Velocity", values.strafeVelocity);
        context.getTelemetry().addData("Strafe Acceleration", values.strafeAcceleration);
        context.getTelemetry().addData("Angular Velocity", values.angularVelocity);
        context.getTelemetry().addData("Angular Acceleration", values.angularAcceleration);
        context.getTelemetry().addData("Translation kV", values.translationKV);
        context.getTelemetry().addData("Translation kA", values.translationKA);
        context.getTelemetry().addData("Angular kV", values.angularKV);
        context.getTelemetry().addData("Angular kA", values.angularKA);
    }
}
