package org.firstinspires.ftc.teamcode.apexpathing.tuning;

import geometry.Angle;
import geometry.Dist;
import geometry.Pose;
import geometry.Vector;
import paths.builders.Builder;
import paths.movements.Path;
import util.DistUnit;

public class LateralAccelerationPhase extends TuningPhase {
    private boolean driftDetected;
    private boolean testingPath;
    private double accelMaxError;

    public LateralAccelerationPhase() {
        super("LATERAL_ACCEL");
    }

    @Override
    protected void beginAutomatic(TunerContext context) {
        driftDetected = false;
        testingPath = false;
        accelMaxError = 0;
    }

    @Override
    protected boolean updateAutomatic(TunerContext context) throws InterruptedException {
        if (testingPath) {
            return updatePathTest(context);
        }

        if (driftDetected || context.maxLateralAccel > 300) {
            if (!driftDetected) {
                context.maxLateralAccel -= 20.0;
            }
            context.updateFollowerConfig();
            return true;
        }

        context.updateFollowerConfig();
        context.resetPose();

        Pose start = context.follower().getPose();
        Path testCurve = Builder.path(
                start,
                new Pose(start.getPos().plus(new Vector(Dist.of(30, DistUnit.IN), Dist.of(0, DistUnit.IN))), start.getHeading()),
                new Pose(start.getPos().plus(new Vector(Dist.of(30, DistUnit.IN), Dist.of(30, DistUnit.IN))), start.getHeading().plus(Angle.fromDeg(90))),
                new Pose(start.getPos().plus(new Vector(Dist.of(0, DistUnit.IN), Dist.of(30, DistUnit.IN))), start.getHeading().plus(Angle.fromDeg(180)))
        ).build();

        context.follower().follow(testCurve);
        accelMaxError = 0;
        testingPath = true;
        return false;
    }

    private boolean updatePathTest(TunerContext context) throws InterruptedException {
        context.follower().update();
        double err = context.follower().getPose().getPos().getMag().getIn();
        if (err > accelMaxError) {
            accelMaxError = err;
        }

        if (context.follower().isBusy()) {
            return false;
        }

        if (accelMaxError > 4.0) {
            driftDetected = true;
            context.maxLateralAccel -= 20.0;
        } else {
            context.maxLateralAccel += 20.0;
            context.sleep(1000);
        }

        testingPath = false;
        return false;
    }

    @Override
    protected double currentManualValue(TunerContext context) {
        return context.maxLateralAccel;
    }

    @Override
    protected void applyManualValue(TunerContext context, double value) {
        context.maxLateralAccel = value;
    }

    @Override
    protected String savePrompt() {
        return "Press 'A' (cross) to SAVE and finish.";
    }

    @Override
    protected String rerunPrompt() {
        return "Press 'B' (circle) to RERUN or ADJUST.";
    }

    @Override
    protected String rerunExecutionPrompt() {
        return "B - Execute Rerun";
    }

    @Override
    protected String manualInstructions() {
        return "Tune kSGuess (Max Lateral Accel) via Config Panels. Drive to test.";
    }

    @Override
    protected String manualTelemetryLabel() {
        return "Current Max Lateral Accel";
    }

    @Override
    protected void reportAutomaticResult(TunerContext context) {
        context.telemetry().addData("Max Lateral Accel", context.maxLateralAccel);
    }

    @Override
    protected void onAccepted(TunerContext context) {
        context.saveConstantsToJson();
    }

    @Override
    protected TuningPhase nextPhase(TunerContext context) {
        return null;
    }
}
