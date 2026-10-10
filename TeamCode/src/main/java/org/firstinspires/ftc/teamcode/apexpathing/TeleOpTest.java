package org.firstinspires.ftc.teamcode.apexpathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import follower.Follower;
import geometry.Pose;
import util.DeltaTime;

/**
 * Test OpMode for using Apex Pathing in TeleOp mode.
 *
 * @author Sohum Arora - 22985 Paraducks
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
@TeleOp(name = "Apex TeleOp Test", group = "Apex Pathing")
public class TeleOpTest extends LinearOpMode {
    public double loops = 0, loopTime = 0;
    private final DeltaTime loopTiming = new DeltaTime();
    Constants constants = new Constants();

    @Override
    public void runOpMode() {
        Follower follower = new Follower(constants, hardwareMap);

        try {
            telemetry.addLine("Press Start to begin");
            telemetry.update();
            waitForStart();
            loops = loopTime = 0;
            loopTiming.reset(System.nanoTime());

            while (opModeIsActive()) {
                follower.update();
                Pose currentPose = follower.getPose();

                follower.manual(gamepad1);

                loops++;

                if (loops > 15) {

                    loopTime = loopTiming.update(System.nanoTime()) * 1_000 / loops;

                    loops = 0;
                }

                telemetry.addData("Loop time (ms)", loopTime);
                telemetry.addData("X", currentPose.getX());
                telemetry.addData("Y ", currentPose.getY());
                telemetry.addData("Heading", currentPose.getHeading());
                telemetry.update();
            }
        } finally {
            follower.stop();
        }
    }
}
