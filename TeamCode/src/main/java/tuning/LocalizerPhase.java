package tuning;

import android.os.Environment;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.json.JSONObject;

import java.io.File;
import java.io.FileWriter;

import geometry.Pose;

/** Guided localizer scale and direction check inside FollowerTuner. */
public class LocalizerPhase extends TunePhase {
    private enum Step { FORWARD_START, FORWARD_END, STRAFE_START, STRAFE_END,
        TURN_START, TURN_END, DONE }

    private Step step;
    private Pose startPose;
    private double distance;
    private double turns;
    private double forwardScale;
    private double strafeScale;
    private double turnScale;
    private double forwardSign;
    private double strafeSign;
    private double turnSign;
    private double turnDriftX;
    private double turnDriftY;
    private double unwrappedHeading;
    private double lastHeading;
    private int selectedValue;

    public LocalizerPhase(TuneContext context) { super(context); }

    @Override protected String name() { return "Localizer"; }
    @Override protected boolean hasManual() { return true; }
    @Override protected boolean hasAuto() { return true; }

    @Override
    protected void start() {
        step = Step.FORWARD_START;
        startPose = context.getFollower().getPose();
        distance = 48.0;
        turns = 5.0;
        forwardScale = Double.NaN;
        strafeScale = Double.NaN;
        turnScale = Double.NaN;
        forwardSign = 0.0;
        strafeSign = 0.0;
        turnSign = 0.0;
        turnDriftX = 0.0;
        turnDriftY = 0.0;
        unwrappedHeading = 0.0;
        lastHeading = context.getFollower().getPose().getHeading().getRad();
        selectedValue = 0;
        context.getFollower().getDrivetrain().stop();
    }

    @Override
    protected boolean runManual(boolean aPressed, boolean bPressed) {
        if (context.getGamepad().dpadLeftWasPressed() ||
                context.getGamepad().dpadRightWasPressed()) {
            selectedValue = 1 - selectedValue;
        }
        if (context.getGamepad().dpadUpWasPressed()) adjustSetting(1);
        if (context.getGamepad().dpadDownWasPressed()) adjustSetting(-1);

        context.getTelemetry().addLine("D-pad Left/Right - choose distance or turns");
        context.getTelemetry().addLine("D-pad Up/Down - adjust selected value");
        context.getTelemetry().addData(selectedValue == 0 ? "> Distance" : "  Distance", distance);
        context.getTelemetry().addData(selectedValue == 1 ? "> Turns" : "  Turns", turns);
        return runStep(aPressed, bPressed);
    }

    @Override
    protected boolean runAuto(boolean aPressed, boolean bPressed) {
        return runStep(aPressed, bPressed);
    }

    private boolean runStep(boolean aPressed, boolean bPressed) {
        Pose pose = context.getFollower().getPose();
        if (step == Step.TURN_END) {
            double heading = pose.getHeading().getRad();
            unwrappedHeading += AngleUnit.normalizeRadians(heading - lastHeading);
            lastHeading = heading;
        }

        switch (step) {
            case FORWARD_START:
                context.getTelemetry().addLine("Place the robot at the start facing +X, then press A");
                if (aPressed) { startPose = pose; step = Step.FORWARD_END; }
                break;
            case FORWARD_END:
                context.getTelemetry().addLine("Push forward exactly " + distance + " in, then press A");
                if (aPressed) {
                    double measured = pose.getX().minus(startPose.getX()).getIn();
                    forwardScale = scale(distance, measured);
                    forwardSign = Math.signum(measured);
                    step = Step.STRAFE_START;
                }
                break;
            case STRAFE_START:
                context.getTelemetry().addLine("Press A for strafe test or B to skip for tank");
                if (aPressed) { startPose = pose; step = Step.STRAFE_END; }
                else if (bPressed) step = Step.TURN_START;
                break;
            case STRAFE_END:
                context.getTelemetry().addLine("Push left exactly " + distance + " in, then press A");
                if (aPressed) {
                    double measured = pose.getY().minus(startPose.getY()).getIn();
                    strafeScale = scale(distance, measured);
                    strafeSign = Math.signum(measured);
                    step = Step.TURN_START;
                }
                break;
            case TURN_START:
                context.getTelemetry().addLine("Press A, rotate CCW exactly " + turns + " turns, then press A again");
                if (aPressed) {
                    startPose = pose;
                    unwrappedHeading = 0.0;
                    lastHeading = pose.getHeading().getRad();
                    step = Step.TURN_END;
                }
                break;
            case TURN_END:
                context.getTelemetry().addLine("Finish " + turns + " CCW turns, then press A");
                context.getTelemetry().addData("Tracked turns", unwrappedHeading / (2.0 * Math.PI));
                if (aPressed) {
                    turnScale = scale(turns * 2.0 * Math.PI, unwrappedHeading);
                    turnSign = Math.signum(unwrappedHeading);
                    turnDriftX = pose.getX().minus(startPose.getX()).getIn();
                    turnDriftY = pose.getY().minus(startPose.getY()).getIn();
                    save();
                    step = Step.DONE;
                }
                break;
            case DONE:
                return true;
        }
        context.getTelemetry().addData("Step", step);
        context.getTelemetry().addData("Pose", pose);
        return false;
    }

    private void adjustSetting(int direction) {
        if (selectedValue == 0) distance = Math.max(12.0, distance + direction);
        else turns = Math.max(1.0, turns + (direction * 0.25));
    }

    private static double scale(double known, double measured) {
        return Math.abs(measured) < 1e-6 ? Double.NaN : known / Math.abs(measured);
    }

    private void save() {
        try {
            JSONObject json = new JSONObject();
            put(json, "forwardScaleMultiplier", forwardScale);
            put(json, "strafeScaleMultiplier", strafeScale);
            put(json, "angularScaleMultiplier", turnScale);
            json.put("forwardAxisSign", forwardSign);
            json.put("strafeAxisSign", strafeSign);
            json.put("angularAxisSign", turnSign);
            json.put("rotationDriftXIn", turnDriftX);
            json.put("rotationDriftYIn", turnDriftY);
            File folder = new File(Environment.getExternalStorageDirectory(), "FIRST/ApexPathing");
            if (!folder.exists() && !folder.mkdirs()) {
                throw new IllegalStateException("Cannot create output folder");
            }
            FileWriter writer = new FileWriter(new File(folder, "localizer-calibration.json"));
            writer.write(json.toString(4));
            writer.close();
        } catch (Exception e) {
            context.getTelemetry().addLine("Could not save calibration: " + e.getMessage());
        }
    }

    private static void put(JSONObject json, String key, double value) throws Exception {
        json.put(key, Double.isFinite(value) ? value : JSONObject.NULL);
    }

    @Override
    protected void showResults() {
        context.getTelemetry().addData("Forward scale", forwardScale);
        context.getTelemetry().addData("Strafe scale", strafeScale);
        context.getTelemetry().addData("Turn scale", turnScale);
        context.getTelemetry().addData("Turn drift X", turnDriftX);
        context.getTelemetry().addData("Turn drift Y", turnDriftY);
        context.getTelemetry().addLine("Saved to FIRST/ApexPathing/localizer-calibration.json");
    }
}
