package org.firstinspires.ftc.teamcode.apexpathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import storage.ApexStorage;
import setup.RobotSetupWizard;
import setup.SdkSetupHardware;
import setup.SetupHardware;
import setup.SetupLog;
import setup.SetupStore;

/** First-time setup: contextual configuration, guided hand-motion checks and localization capture. */
@TeleOp(name = "Robot Setup", group = "Apex Pathing")
public class RobotSetup extends LinearOpMode {
    /** Override only for a simulator that supplies its own hardware boundary. */
    protected SetupHardware createSetupHardware() { return new SdkSetupHardware(hardwareMap); }

    @Override public void runOpMode() {
        SetupHardware hardware = createSetupHardware();
        SetupLog log = null;
        RobotSetupWizard wizard = null;
        try {
            log = new SetupLog(ApexStorage.getLogDirectory());
            wizard = new RobotSetupWizard(hardware, new SetupStore(ApexStorage.getDirectory()), log);
            telemetry.addLine("Robot Setup");
            telemetry.addLine("Press Start to begin. Motor tests run only while X is held on a motor screen.");
            telemetry.update();
            waitForStart();
            long origin = System.nanoTime();
            while (opModeIsActive()) {
                wizard.tick((System.nanoTime() - origin) * 1e-9);
                // One action per frame prevents a held/menu button from accepting the next screen.
                if (gamepad1.aWasPressed()) { wizard.act(RobotSetupWizard.Action.CONFIRM); }
                else if (gamepad1.bWasPressed()) { wizard.act(RobotSetupWizard.Action.BACK); }
                else if (gamepad1.dpadUpWasPressed()) { wizard.act(RobotSetupWizard.Action.UP); }
                else if (gamepad1.dpadDownWasPressed()) { wizard.act(RobotSetupWizard.Action.DOWN); }
                else if (gamepad1.dpadLeftWasPressed()) { wizard.act(RobotSetupWizard.Action.LEFT); }
                else if (gamepad1.dpadRightWasPressed()) { wizard.act(RobotSetupWizard.Action.RIGHT); }
                else if (gamepad1.leftBumperWasPressed()) { wizard.act(RobotSetupWizard.Action.FINER); }
                else if (gamepad1.rightBumperWasPressed()) { wizard.act(RobotSetupWizard.Action.COARSER); }
                wizard.updateMotorTest(gamepad1.x && opModeIsActive() && !isStopRequested());
                telemetry.clearAll();
                for (String line : wizard.lines()) { telemetry.addLine(line); }
                telemetry.update();
                sleep(20);
            }
        } catch (Exception failure) {
            hardware.stop();
            if (log != null) { log.event(0, "session", "failure", failure.toString()); }
            telemetry.clearAll();
            telemetry.addLine("Setup stopped. Check the configured devices and available storage, then initialize again.");
            telemetry.addLine("Your previously saved setup is retained.");
            telemetry.update();
        } finally {
            try { hardware.learn(false); hardware.stop(); }
            finally {
                if (wizard != null) { wizard.close(); }
                else if (log != null) { log.close(); }
            }
        }
    }
}
