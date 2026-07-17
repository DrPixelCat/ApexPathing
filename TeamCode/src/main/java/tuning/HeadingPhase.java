package tuning;

import geometry.Angle;
import geometry.GeometryFactory;
import paths.movements.Turn;

/**
 * @author Sohum Arora - 22985 Paraducks
 */
public class HeadingPhase extends TuningPhase {
    private final TuningValues values;
    private PDSRoutine routine;
    private int selected;
    private double target = 90.0;
    private boolean complete;

    public HeadingPhase(TunerContext context, TuningValues values) {
        super(context);
        this.values = values;
        complete = values.heading.kP != 0.0;
    }

    @Override
    protected String getPhaseName() {
        return "Heading";
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
        selected = 0;
        if (manualMode) {
            context.getFollower().setHeadingTuning(values.heading);
            return;
        }
        routine = new PDSRoutine(context, PDSRoutine.TuningAxis.HEADING);
        routine.start();
    }

    @Override
    protected void autoTuned() {
        if (!routine.update(context)) {
            return;
        }
        values.heading = values.copy(routine.getCoefficients());
        context.getFollower().enableControllers();
        values.saveHeading(context);
        complete = true;
    }

    @Override
    protected void manualTuned() {
        if (opMode.gamepad1.leftBumperWasPressed()) {
            selected = (selected + 2) % 3;
        }
        if (opMode.gamepad1.rightBumperWasPressed()) {
            selected = (selected + 1) % 3;
        }

        double change = manualChange();
        if (change != 0.0) {
            if (selected == 0) {
                values.heading.kP = Math.max(0.0, values.heading.kP + change);
            } else if (selected == 1) {
                values.heading.kD = Math.max(0.0, values.heading.kD + change);
            } else {
                values.heading.kS = Math.max(0.0, values.heading.kS + change);
            }
            context.getFollower().setHeadingTuning(values.heading);
        }

        if (opMode.gamepad1.xWasPressed() && !context.getFollower().isBusy()) {
            GeometryFactory factory = new GeometryFactory(context.getFollower());
            Turn turn = factory.turn(context.getFollower().getPose()).turnTo(Angle.fromDeg(target)).quickBuild();
            context.getFollower().follow(turn);
            target = -target;
        }

        context.getTelemetry().addData("Selected", selected == 0 ? "P" : selected == 1 ? "D" : "S");
        context.getTelemetry().addData("P / D / S", "%.6f / %.6f / %.6f", values.heading.kP,
                values.heading.kD, values.heading.kS);
        context.getTelemetry().addData("Increment", increment);
        context.getTelemetry().addLine("Up/Down: change value");
        context.getTelemetry().addLine("Left/Right: change increment");
        context.getTelemetry().addLine("LB/RB: select value");
        context.getTelemetry().addLine("X: test turn");
        context.getTelemetry().addLine("A: save");
        context.getTelemetry().update();

        if (opMode.gamepad1.aWasPressed()) {
            context.getFollower().stop();
            values.saveHeading(context);
            complete = true;
        }
    }

    @Override
    protected void reportResults() {
        context.getTelemetry().addData("Heading P", values.heading.kP);
        context.getTelemetry().addData("Heading D", values.heading.kD);
        context.getTelemetry().addData("Heading S", values.heading.kS);
    }
}
