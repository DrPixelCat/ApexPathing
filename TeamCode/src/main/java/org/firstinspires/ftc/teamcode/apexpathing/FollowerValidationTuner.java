package org.firstinspires.ftc.teamcode.apexpathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import core.Follower;
import geometry.Angle;
import geometry.Dist;
import geometry.Pose;
import geometry.Vector;
import paths.builders.Builder;
import paths.movements.FollowerMovement;

/** Repeatable closed-loop acceptance test; it measures performance but never alters constants. */
@TeleOp(name = "Apex Follower Validation", group = "Apex Pathing Tuning")
public class FollowerValidationTuner extends LinearOpMode {
    private enum Trial { FORWARD, RETURN_FORWARD, STRAFE, RETURN_STRAFE, TURN_90, TURN_0, COMPLETE }
    private static final double TRIAL_TIMEOUT_SECONDS = 8.0;

    @Override
    public void runOpMode() {
        Follower follower = new Follower(new Constants(), hardwareMap);
        follower.setPose(Pose.zero());
        boolean holonomic = follower.getDrivetrain().isHolonomic();
        ElapsedTime timer = new ElapsedTime();
        Trial trial = Trial.FORWARD;
        Pose target = Pose.zero();
        double worstPositionError = 0.0;
        double worstHeadingError = 0.0;
        int timeouts = 0;

        telemetry.addLine("Clear at least 48 in forward and 36 in left of the robot.");
        telemetry.addLine("This validation does not modify constants.");
        telemetry.update();
        waitForStart();

        FollowerMovement movement = buildTrial(follower, trial, holonomic);
        target = movement.getEndPose();
        follower.follow(movement);
        timer.reset();

        while (opModeIsActive() && !isStopRequested()) {
            follower.update();

            if (follower.isBusy() && timer.seconds() > TRIAL_TIMEOUT_SECONDS) {
                follower.stop();
                timeouts++;
            }

            if (!follower.isBusy() && trial != Trial.COMPLETE) {
                Pose error = target.minus(follower.getPose());
                worstPositionError = Math.max(worstPositionError, error.getVec().getMag().getIn());
                worstHeadingError = Math.max(worstHeadingError,
                        Math.abs(error.getHeading().getRad()));

                trial = nextTrial(trial, holonomic);
                if (trial != Trial.COMPLETE) {
                    sleep(400);
                    movement = buildTrial(follower, trial, holonomic);
                    target = movement.getEndPose();
                    follower.follow(movement);
                    timer.reset();
                }
            }

            telemetry.addData("Trial", trial);
            telemetry.addData("Pose", follower.getPose());
            telemetry.addData("Target", target);
            telemetry.addData("Worst position error (in)", "%.3f", worstPositionError);
            telemetry.addData("Worst heading error (deg)", "%.3f",
                    Math.toDegrees(worstHeadingError));
            telemetry.addData("Timeouts", timeouts);
            if (trial == Trial.COMPLETE) {
                telemetry.addLine(timeouts == 0 ? "PASS: all trials completed." :
                        "FAIL: at least one trial timed out.");
            }
            telemetry.update();
        }
        follower.stop();
    }

    private FollowerMovement buildTrial(Follower follower, Trial trial, boolean holonomic) {
        Pose current = follower.getPose();
        Pose target;
        switch (trial) {
            case FORWARD:
                target = pose(36, 0, 0);
                return buildPath(current, target, holonomic);
            case RETURN_FORWARD:
                target = pose(0, 0, 0);
                return buildPath(current, target, holonomic);
            case STRAFE:
                target = pose(0, 24, 0);
                return buildPath(current, target, true);
            case RETURN_STRAFE:
                target = pose(0, 0, 0);
                return buildPath(current, target, true);
            case TURN_90:
                return Builder.turn(current).turnTo(Angle.fromDeg(90)).quickBuild();
            case TURN_0:
                return Builder.turn(current).turnTo(Angle.zero()).quickBuild();
            default:
                throw new IllegalStateException("No movement exists for " + trial);
        }
    }

    private FollowerMovement buildPath(Pose current, Pose target, boolean holonomic) {
        return holonomic ? Builder.holonomicPath(current, target).quickBuild() :
                Builder.tankPath(current, target).quickBuild();
    }

    private Trial nextTrial(Trial trial, boolean holonomic) {
        if (!holonomic && trial == Trial.RETURN_FORWARD) return Trial.TURN_90;
        return Trial.values()[trial.ordinal() + 1];
    }

    private Pose pose(double x, double y, double headingDegrees) {
        return new Pose(new Vector(Dist.fromIn(x), Dist.fromIn(y)), Angle.fromDeg(headingDegrees));
    }
}
