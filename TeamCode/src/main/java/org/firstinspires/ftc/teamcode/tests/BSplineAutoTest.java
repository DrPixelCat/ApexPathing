package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Constants;

import followers.MovementFollower;
import followers.constants.BSplineFollowerConstants;
import paths.ExamplePathAPIV3;
import paths.movements.Path;
import geometry.Pose;
import util.PoseFactory;

/**
 * Test Autonomous opMode utilizing {@link paths.ExamplePathAPIV3}
 * IMPORTANT: Make sure that you have your {@link BSplineFollowerConstants} set up after running {@link org.firstinspires.ftc.teamcode.tuning.manual.BSplineTuner}
 * @author Sohum Arora - 22985 Paraducks
 */
@Autonomous(name = "Apex BSpline Auto Test", group = "Apex Pathing Tests")
public class BSplineAutoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MovementFollower follower = (MovementFollower) new Constants().build(hardwareMap, Pose.zero());
        Path autoPath = (Path) new ExamplePathAPIV3(PoseFactory.Mirror.NONE).getAutoRoutine()[0];
        while (opModeInInit()){
            telemetry.addLine("Robot initialized");
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        follower.follow(autoPath);

        while (opModeIsActive()) {
            follower.update();

            Pose currentPose = follower.getPose();
            Pose targetPose = follower.getTargetPose();

            telemetry.addLine(follower.isBusy() ? "Follower IS busy" : "Follower is NOT busy");
            if (targetPose != null) {
                telemetry.addData("Target X", targetPose.getX());
                telemetry.addData("Target Y", targetPose.getY());
            }
            telemetry.addData("Current X", currentPose.getX());
            telemetry.addData("Current Y", currentPose.getY());
            telemetry.addData("Heading", currentPose.getHeading());
            telemetry.update();
        }

        follower.stop();
    }
}