package org.firstinspires.ftc.teamcode.apexpathing.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class TunerInput {
    private final LinearOpMode opMode;
    private boolean lastA;
    private boolean lastB;
    private boolean lastY;

    public TunerInput(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public boolean aPressed() {
        return opMode.gamepad1.a && !lastA;
    }

    public boolean bPressed() {
        return opMode.gamepad1.b && !lastB;
    }

    public boolean yPressed() {
        return opMode.gamepad1.y && !lastY;
    }

    public void captureCurrentAsPrevious() {
        lastA = opMode.gamepad1.a;
        lastB = opMode.gamepad1.b;
        lastY = opMode.gamepad1.y;
    }
}
