package setup;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;
import org.json.JSONObject;
import support.FakeHardware;
import java.io.File;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import static org.junit.Assert.*;
import static setup.RobotSetupWizard.Action.*;

public class MotorSelectionTest {
    @Rule public TemporaryFolder temp = new TemporaryFolder();

    static final class Hardware extends RobotSetupWorkflowTest.Hardware {
        String tested;
        boolean reversed, fail;
        double power;
        @Override public MotorTestReading testMotor(String name, boolean reversed) {
            tested = name; this.reversed = reversed; power = .5;
            if (fail) { throw new IllegalStateException("disconnected"); }
            return new MotorTestReading(power, 120, 300, 12.5);
        }
        @Override public void stop() { super.stop(); power = 0; }
    }

    @Test public void heldControlTestsOnlyTheSelectedDriveMotorAndStopsOnReleaseNavigationAndClose() throws Exception {
        Hardware hardware = new Hardware();
        File root = temp.newFolder(); SetupLog log = new SetupLog(new File(root, "logs"));
        try (RobotSetupWizard wizard = new RobotSetupWizard(hardware, new SetupStore(root), log)) {
            wizard.act(CONFIRM); wizard.updateMotorTest(false);
            wizard.act(CONFIRM); wizard.updateMotorTest(false); // localizer
            wizard.act(CONFIRM); wizard.updateMotorTest(false); // front-left motor
            assertTrue(String.join("\n", wizard.lines()).contains("Hold X"));
            wizard.updateMotorTest(true); assertEquals(.5, hardware.power, 0); assertEquals("frontLeftMotor", hardware.tested);
            wizard.updateMotorTest(true); assertEquals(.5, hardware.power, 0);
            wizard.updateMotorTest(false); assertEquals(0, hardware.power, 0);
            wizard.updateMotorTest(true); wizard.act(CONFIRM); // direction screen
            assertEquals(0, hardware.power, 0);
            wizard.updateMotorTest(true); assertEquals("Holding across navigation must not restart", 0, hardware.power, 0);
            wizard.updateMotorTest(false); wizard.act(UP); wizard.updateMotorTest(false);
            wizard.updateMotorTest(true); assertEquals(.5, hardware.power, 0); assertTrue(hardware.reversed);
            wizard.act(UP); wizard.updateMotorTest(true); assertEquals("Editing direction stops the motor until release", 0, hardware.power, 0);
            wizard.updateMotorTest(false); wizard.updateMotorTest(true); assertFalse(hardware.reversed);
            assertFalse(new File(root, "robot-setup.json").exists());
        }
        assertEquals(0, hardware.power, 0);
        String csv = new String(Files.readAllBytes(new File(log.getDirectory(), "motor_tests.csv").toPath()), StandardCharsets.UTF_8);
        assertTrue(csv.contains("held,0.5,0.5,120,300.0,12.5")); assertTrue(csv.contains("stopped,0"));
    }

    @Test public void encoderPickerNeverEnergizesAMotorAndTestFailureStopsOutput() throws Exception {
        Hardware hardware = new Hardware(); hardware.fail = true;
        File root = temp.newFolder();
        try (RobotSetupWizard wizard = new RobotSetupWizard(hardware, new SetupStore(root), new SetupLog(new File(root, "logs")))) {
            wizard.act(CONFIRM); wizard.updateMotorTest(false);
            wizard.act(CONFIRM); wizard.act(UP); wizard.updateMotorTest(false); // TwoWheel
            wizard.act(CONFIRM); wizard.updateMotorTest(false); // first drive motor
            wizard.updateMotorTest(true); assertEquals(0, hardware.power, 0);
            assertTrue(String.join("\n", wizard.lines()).contains("Motor test stopped"));
            hardware.fail = false;
            wizard.updateMotorTest(true); assertEquals(0, hardware.power, 0);
            wizard.updateMotorTest(false);
            // Advance past all eight drive-motor fields into the forward encoder picker.
            for (int i = 0; i < 8; i++) { wizard.act(CONFIRM); wizard.updateMotorTest(false); }
            assertTrue(String.join("\n", wizard.lines()).contains("Forward pod"));
            wizard.updateMotorTest(true); assertEquals(0, hardware.power, 0);
            assertFalse(String.join("\n", wizard.lines()).contains("Hold X"));
        }
    }

    @Test public void sdkTestCommandsHalfPowerAndRestoresOriginalMotorSettingsWithoutResettingEncoders() {
        FakeHardware map = new FakeHardware();
        FakeHardware.MotorState left = map.motor("left"), right = map.motor("right");
        left.mode = DcMotor.RunMode.RUN_USING_ENCODER;
        left.direction = DcMotorSimple.Direction.FORWARD;
        left.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT;
        left.ticks = 123; left.velocity = 345;
        SdkSetupHardware adapter = new SdkSetupHardware(map);
        MotorTestReading reading = adapter.testMotor("left", true);
        assertEquals(.5, left.power, 0); assertEquals(DcMotorSimple.Direction.REVERSE, left.direction);
        assertEquals(DcMotor.RunMode.RUN_WITHOUT_ENCODER, left.mode);
        assertEquals(0, right.power, 0); assertEquals(123, reading.ticks); assertEquals(345, reading.velocityTicksPerSecond, 0);
        adapter.testMotor("right", false);
        assertEquals(0, left.power, 0); assertEquals(.5, right.power, 0);
        assertEquals(DcMotor.RunMode.RUN_USING_ENCODER, left.mode);
        assertEquals(DcMotorSimple.Direction.FORWARD, left.direction);
        assertEquals(DcMotor.ZeroPowerBehavior.FLOAT, left.zeroPowerBehavior);
        adapter.stop(); assertEquals(0, right.power, 0);
        assertEquals(0, left.resets + right.resets);
    }

    @Test public void testDoesNotReenterEncoderResetModeAfterRelease() {
        FakeHardware map = new FakeHardware(); FakeHardware.MotorState state = map.motor("motor");
        state.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER;
        SdkSetupHardware adapter = new SdkSetupHardware(map);
        adapter.testMotor("motor", false); state.ticks = 100;
        adapter.stop();
        assertEquals(0, state.power, 0); assertEquals(100, state.ticks);
        assertEquals(0, state.resets); assertEquals(DcMotor.RunMode.RUN_WITHOUT_ENCODER, state.mode);
    }
}
