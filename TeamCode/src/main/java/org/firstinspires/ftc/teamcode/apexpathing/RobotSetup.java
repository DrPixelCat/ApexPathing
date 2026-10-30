package org.firstinspires.ftc.teamcode.apexpathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import storage.ApexStorage;
import setup.hardware.RobotSetupWizard;
import setup.hardware.SdkSetupHardware;
import setup.hardware.SetupHardware;
import setup.hardware.SetupLog;
import setup.hardware.SetupStore;

/** Hardware and encoder configuration; localization calibration belongs to Localization Tuner. */
@TeleOp(name = "Robot Setup", group = "Apex Pathing")
public class RobotSetup extends LinearOpMode {
    /** Override only for a simulator that supplies its own hardware boundary. */
    protected SetupHardware createSetupHardware() { return new SdkSetupHardware(hardwareMap); }

    /** The complete setup progression; field editing never accepts the next screen. */
    public static void handleSetupAction(RobotSetupWizard setup, RobotSetupWizard.Action action) {
        if (!setup.beginAction(action)) { return; }
        switch (setup.getScreen()) {
            case WELCOME:
                setup.chooseWelcome(action);
                if (action == RobotSetupWizard.Action.CONFIRM) {
                    if (setup.useSavedSettings()) {
                        if (setup.validateConfiguration()) { setup.showSave(); }
                    } else { setup.beginConfiguration(); }
                }
                break;
            case CONFIGURE:
                // A advances each field. Review the complete draft after the final field.
                if (setup.editConfiguration(action) && setup.validateConfiguration()) {
                    setup.showSave();
                }
                break;
            case SAVE:
                // Only a new confirmation on this screen may write the profile.
                if (action == RobotSetupWizard.Action.CONFIRM && setup.saveSettings()) {
                    setup.complete();
                }
                break;
            case COMPLETE:
            case ERROR:
                // Wait for STOP; finally releases hardware and closes the session log.
                break;
        }
    }

    @Override public void runOpMode() {
        SetupHardware hardware = createSetupHardware();
        SetupLog log = null;
        RobotSetupWizard wizard = null;
        try {
            log = new SetupLog(ApexStorage.getLogDirectory());
            wizard = new RobotSetupWizard(hardware, new SetupStore(ApexStorage.getDirectory()), log);
            telemetry.addLine("Robot Setup");
            telemetry.addLine("Press Start to begin. To test a selected motor, hold X.");
            telemetry.addLine(setup.WorkflowText.STOP_HINT);
            telemetry.update();
            waitForStart();
            long origin = System.nanoTime();
            while (opModeIsActive()) {
                wizard.tick((System.nanoTime() - origin) * 1e-9);
                // One action per frame prevents a held/menu button from accepting the next screen.
                if (gamepad1.aWasPressed()) { handleSetupAction(wizard, RobotSetupWizard.Action.CONFIRM); }
                else if (gamepad1.dpadUpWasPressed()) { handleSetupAction(wizard, RobotSetupWizard.Action.UP); }
                else if (gamepad1.dpadDownWasPressed()) { handleSetupAction(wizard, RobotSetupWizard.Action.DOWN); }
                else if (gamepad1.dpadLeftWasPressed()) { handleSetupAction(wizard, RobotSetupWizard.Action.LEFT); }
                else if (gamepad1.dpadRightWasPressed()) { handleSetupAction(wizard, RobotSetupWizard.Action.RIGHT); }
                else if (gamepad1.leftBumperWasPressed()) { handleSetupAction(wizard, RobotSetupWizard.Action.FINER); }
                else if (gamepad1.rightBumperWasPressed()) { handleSetupAction(wizard, RobotSetupWizard.Action.COARSER); }
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
            telemetry.addLine(setup.WorkflowText.STOP_HINT);
            telemetry.update();
        } finally {
            try {
                try { hardware.stop(); }
                finally { hardware.learn(false); }
            }
            finally {
                if (wizard != null) { wizard.close(); }
                else if (log != null) { log.close(); }
            }
        }
    }
}
