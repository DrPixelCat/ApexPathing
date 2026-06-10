package org.firstinspires.ftc.teamcode.apexpathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import core.Follower;
import paths.ExampleAutoPath;
import util.PoseFactory;

/**
 * Test Auto utilizing {@link ExampleAutoPath}
 * IMPORTANT: Make sure your {@link core.FollowerConstants} have been tuned by running {@link FollowerTuner} before running this OpMode
 * @author Sohum Arora 22985 Paraducks
 */
@Autonomous(name = "Apex BSpline Auto Test", group = "Apex Pathing Tests")
public class AutoTest extends LinearOpMode {
    Constants constants = new Constants();
    ExampleAutoPath path = new ExampleAutoPath(PoseFactory.Mirror.NONE);
    enum Paths {TEST_PATH, TEST_TURN, COMPLETE}
    Paths currentPath = Paths.TEST_PATH;
    boolean pathStarted = false;

    @Override
    public void runOpMode() throws InterruptedException {
        Follower follower = new Follower(constants, hardwareMap);

        while (opModeInInit()){
            telemetry.addLine("Robot initialized");
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            follower.update();

            switch (currentPath) {
                case TEST_PATH:
                    if (!pathStarted) {
                        follower.follow(path.testPath);
                        pathStarted = true;
                    }
                    if (!follower.isBusy()) {
                        currentPath = Paths.TEST_TURN;
                        pathStarted = false;
                    }
                    break;

                case TEST_TURN:
                    if (!pathStarted) {
                        follower.follow(path.testTurn);
                        pathStarted = true;
                    }
                    if (!follower.isBusy()) {
                        currentPath = Paths.COMPLETE;
                        pathStarted = false;
                    }
                    break;

                case COMPLETE:
                    follower.stop();
                    telemetry.addLine("Auto Test complete!");
                    break;
            }

            telemetry.addLine(follower.isBusy() ? "Follower IS busy" : "Follower is NOT busy");
            telemetry.addData("Current Path", currentPath);
            telemetry.addData("Current X", follower.getPose().getX());
            telemetry.addData("Current Y", follower.getPose().getY());
            telemetry.addData("Heading", follower.getPose().getHeading());
            telemetry.update();
        }
    }
}