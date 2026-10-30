package setup.localization.phases;

import setup.AdjustmentStep;

import drivetrains.TranslationAxis;
import paths.geometry.Vector;
import setup.localization.calibration.CalibrationCandidate;
import setup.localization.calibration.CalibrationSnapshot;

import setup.localization.LocalizationTunerContext;
import setup.localization.TuningPhase;

import org.json.JSONArray;
import org.json.JSONObject;

import java.util.ArrayList;
import java.util.List;

/** Calibrates translation scale using equal powered trials in both directions. */
public final class DistancePhase extends TuningPhase {
    private static final double POWER = 0.50;

    private final TranslationAxis axis;
    private final List<CalibrationCandidate> trials = new ArrayList<>();
    private CalibrationSnapshot start;
    private CalibrationSnapshot pendingEnd;
    private int direction;
    private Stage stage;
    private long stageEnteredAt;
    private double measuredInches;
    private final AdjustmentStep increment = new AdjustmentStep(.25, .01, 10, 10);
    private CalibrationCandidate candidate;

    private enum Stage { DRIVE, SETTLE, ENTER_DISTANCE, COMPLETE }

    public DistancePhase(LocalizationTunerContext context, TranslationAxis axis) {
        super(context);
        this.axis = axis;
    }

    @Override protected String getName() {
        return (axis == TranslationAxis.FORWARD ? "Forward" : "Strafe") + " distance";
    }

    @Override protected void reset() {
        trials.clear();
        candidate = null;
        direction = 1;
        increment.reset();
    }

    @Override protected void showPrepare() {
        context.getView().addLine("Mark the robot's start position on the floor.");
        context.getView().addLine("The robot will drive at 50% power for one second twice.");
        context.getView().addLine("Measure actual travel after each direction; signs are automatic.");
        context.getView().addLine("A: begin");
    }

    @Override protected void beginRecording() {
        start = context.getAdapter().snapshot(context.getLocalizer());
        startDrive();
    }

    @Override protected boolean record() {
        switch (stage) {
            case DRIVE:
                drive();
                if (elapsedSeconds() >= 1.0) {
                    context.stop();
                    stage = Stage.SETTLE;
                    stageEnteredAt = System.nanoTime();
                }
                break;
            case SETTLE:
                context.getView().addLine("Settling...");
                if (elapsedSeconds() >= 0.4) {
                    captureDistance();
                    stage = Stage.ENTER_DISTANCE;
                }
                break;
            case ENTER_DISTANCE:
                editMeasuredDistance();
                showDistanceEntry();
                if (confirmPressed()) { confirmDistance(); }
                break;
            case COMPLETE: break;
        }
        return stage == Stage.COMPLETE;
    }

    private void startDrive() {
        stage = Stage.DRIVE;
        stageEnteredAt = System.nanoTime();
    }

    private double elapsedSeconds() { return (System.nanoTime() - stageEnteredAt) * 1e-9; }

    private void drive() {
        double forward = axis == TranslationAxis.FORWARD ? POWER * direction : 0.0;
        double strafe = axis == TranslationAxis.STRAFE ? POWER * direction : 0.0;
        context.getDrivetrain().moveWithVectors(forward, strafe, 0.0);
        context.getView().addData("Trial direction", direction > 0 ? "positive" : "negative");
    }

    private void captureDistance() {
        pendingEnd = context.getAdapter().snapshot(context.getLocalizer());
        measuredInches = Math.max(0.25, Math.abs(reportedDistance(start, pendingEnd)));
    }

    private void showDistanceEntry() {
        context.getView().addData("Measured distance", context.formatNumber(measuredInches) + " in");
        context.getView().addData("Increment", context.formatNumber(increment.get()) + " in");
        context.getView().addLine("Dpad Up/Down: edit   Left/Right: change increment   A: confirm");
    }

    private void recordDirection() {
        trials.add(context.getAdapter().fitDistance(
                context.getConfig(), axis, start, pendingEnd, measuredInches));
    }

    private void confirmDistance() {
        if (direction > 0) {
            recordFirstDirection();
            startDrive();
        } else {
            recordFinalDirection();
            stage = Stage.COMPLETE;
        }
    }

    private void recordFirstDirection() {
        recordDirection();
        direction = -1;
        start = pendingEnd;
    }

    private void recordFinalDirection() {
        recordDirection();
        candidate = average(trials);
    }

    @Override protected void showReview() {
        for (String metric : candidate.getMetrics()) { context.getView().addLine(metric); }
    }

    @Override protected boolean accept() { return context.acceptGeometry(axis.name(), candidate); }

    private void editMeasuredDistance() {
        if (opMode.gamepad1.dpadLeftWasPressed()) { increment.finer(); }
        if (opMode.gamepad1.dpadRightWasPressed()) { increment.coarser(); }
        if (opMode.gamepad1.dpadUpWasPressed()) { measuredInches += increment.delta(1); }
        if (opMode.gamepad1.dpadDownWasPressed()) {
            measuredInches = Math.max(0.01, measuredInches + increment.delta(-1));
        }
    }

    private double reportedDistance(CalibrationSnapshot from, CalibrationSnapshot to) {
        Vector delta = to.pose.getVec().minus(from.pose.getVec())
                .rotate(from.pose.getHeading().times(-1.0));
        return axis == TranslationAxis.FORWARD
                ? delta.getX().getIn() : delta.getY().getIn();
    }

    private static CalibrationCandidate average(List<CalibrationCandidate> values) {
        if (values.size() != 2) { throw new IllegalStateException("Two distance trials are required"); }
        JSONObject averaged = averageObject(values.get(0).getValues(), values.get(1).getValues());
        List<String> metrics = new ArrayList<>();
        metrics.add("Both powered directions were measured.");
        metrics.addAll(values.get(0).getMetrics());
        metrics.addAll(values.get(1).getMetrics());
        return new CalibrationCandidate(averaged, metrics);
    }

    private static JSONObject averageObject(JSONObject first, JSONObject second) {
        JSONObject output = new JSONObject();
        JSONArray names = first.names();
        if (names == null) { return output; }
        try {
            for (int i = 0; i < names.length(); i++) {
                String key = names.getString(i);
                Object a = first.get(key), b = second.opt(key);
                if (a instanceof Number && b instanceof Number) {
                    output.put(key, (((Number) a).doubleValue() + ((Number) b).doubleValue()) / 2.0);
                } else if (a instanceof JSONObject && b instanceof JSONObject) {
                    output.put(key, averageObject((JSONObject) a, (JSONObject) b));
                } else if (a instanceof JSONArray && b instanceof JSONArray) {
                    output.put(key, averageArray((JSONArray) a, (JSONArray) b));
                } else { output.put(key, a); }
            }
            return output;
        } catch (Exception e) { throw new IllegalStateException("Could not combine distance trials", e); }
    }

    private static JSONArray averageArray(JSONArray first, JSONArray second) throws Exception {
        JSONArray output = new JSONArray();
        for (int i = 0; i < first.length(); i++) {
            Object a = first.get(i), b = i < second.length() ? second.get(i) : null;
            output.put(a instanceof JSONObject && b instanceof JSONObject
                    ? averageObject((JSONObject) a, (JSONObject) b) : a);
        }
        return output;
    }
}
