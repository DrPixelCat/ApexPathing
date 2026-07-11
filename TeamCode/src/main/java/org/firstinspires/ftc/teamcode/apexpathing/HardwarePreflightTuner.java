package org.firstinspires.ftc.teamcode.apexpathing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import drivetrains.BaseDrivetrainConstants;
import util.MotorFactory;

/** Safely verifies motor assignment and configured direction one wheel at a time. */
@TeleOp(name = "Apex Hardware Preflight", group = "Apex Pathing Tuning")
public class HardwarePreflightTuner extends OpMode {
    private static final double TEST_POWER = 0.20;
    private final DcMotorEx[] motors = new DcMotorEx[4];
    private final String[] labels = {"front-left", "front-right", "back-left", "back-right"};

    @Override
    public void init() {
        BaseDrivetrainConstants<?> constants = new Constants().drivetrainConstants();
        MotorFactory[] factories = {
                constants.getFlMotorConfig(), constants.getFrMotorConfig(),
                constants.getBlMotorConfig(), constants.getBrMotorConfig()
        };
        for (int i = 0; i < factories.length; i++) {
            if (factories[i] != null) motors[i] = factories[i].build(hardwareMap);
        }
    }

    @Override
    public void loop() {
        boolean[] pressed = {gamepad1.a, gamepad1.b, gamepad1.x, gamepad1.y};
        telemetry.addLine("Put the robot on blocks. Each button must spin only the named wheel forward.");
        telemetry.addLine("A=front-left, B=front-right, X=back-left, Y=back-right");
        for (int i = 0; i < motors.length; i++) {
            if (motors[i] == null) continue;
            motors[i].setPower(pressed[i] ? TEST_POWER : 0.0);
            telemetry.addData(labels[i], pressed[i] ? "RUNNING" : "stopped");
        }
    }

    @Override
    public void stop() {
        for (DcMotorEx motor : motors) if (motor != null) motor.setPower(0.0);
    }
}
