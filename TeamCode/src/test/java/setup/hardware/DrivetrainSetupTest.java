package setup.hardware;

import static org.firstinspires.ftc.teamcode.apexpathing.RobotSetup.handleSetupAction;

import drivetrains.*;
import org.json.JSONObject;
import org.junit.Test;
import support.FakeHardware;
import java.util.List;
import static org.junit.Assert.*;

public class DrivetrainSetupTest {
    @org.junit.Rule public org.junit.rules.TemporaryFolder temp = new org.junit.rules.TemporaryFolder();
    static RobotProfile swerve() {
        RobotProfile p = new RobotProfile(); p.set("drive", "COAXIAL_SWERVE"); p.set("steerP", "0.5");
        for (String corner : p.motorKeys()) {
            p.set(corner + "Servo", corner + "Steer"); p.set(corner + "Encoder", corner + "Angle");
            p.set(corner + "ZeroDeg", "90");
        }
        return p;
    }
    static RobotProfile dual() {
        RobotProfile p = new RobotProfile(); p.set("drive", "DUAL_ACTUATED");
        p.set("actuatorCount", "2"); p.set("actuator1", "deployLeft"); p.set("actuator2", "deployRight");
        p.set("actuator2Tank", "1"); p.set("actuator2Holonomic", "0");
        return p;
    }
    private static SetupField field(RobotProfile p, String key) {
        for (SetupField f : SetupField.forProfile(p, new RobotSetupWorkflowTest.Hardware())) {
            if (f.key.equals(key)) { return f; }
        }
        throw new AssertionError("Missing field " + key);
    }

    @Test public void swerveRoundTripPreservesEachModuleAndBuildsTheCorrectConfiguration() {
        RobotProfile p = swerve(); p.set("flEncoderReverse", "true"); p.set("frSteerReverse", "true");
        p = RobotProfile.fromJson(p.toJson());
        CoaxialSwerve.Constants c = (CoaxialSwerve.Constants) SetupConfigurations.drivetrain(p);
        assertEquals("flSteer", c.flServoName); assertEquals("brAngle", c.brEncoderName);
        assertTrue(c.encoderReversed[0]); assertTrue(c.servoReversed[1]);
        assertEquals(Math.PI / 2, c.moduleOffsets[0], 1e-12);
        assertEquals(-Math.PI / 2, c.moduleOffsets[1], 1e-12);
        assertEquals("crservo", field(p, "flServo").role); assertEquals("analog", field(p, "flEncoder").role);
        assertTrue(field(p, "flZeroDeg").accepts("0")); assertTrue(field(p, "flZeroDeg").accepts("-90"));
        assertTrue(field(p, "steerD").accepts("0")); assertFalse(field(p, "steerP").accepts("0"));
    }

    @Test public void dualRoundTripPreservesActuatorsAndSelectedStartupKinematics() {
        RobotProfile p = dual(); p.set("initialState", "TANK"); p.set("transitionSeconds", "1.2");
        p = RobotProfile.fromJson(p.toJson());
        DualActuated.Constants c = (DualActuated.Constants) SetupConfigurations.drivetrain(p);
        assertEquals(2, c.actuators.size()); assertEquals(DualActuated.DriveState.TANK, c.initialState);
        assertEquals(1.2, c.transitionSeconds, 0); assertFalse(p.holonomic());
        assertEquals("servo", field(p, "actuator2").role);
        assertTrue(field(p, "actuator1Tank").accepts("0")); assertTrue(field(p, "actuator1Tank").accepts("1"));
        assertFalse(field(p, "actuator1Tank").accepts("1.01")); assertFalse(field(p, "actuatorCount").accepts("1.5"));
    }

    @Test public void rejectsMissingDuplicateAndInvalidLayoutSettings() {
        RobotProfile p = swerve(); p.set("frServo", "flSteer"); reject(p);
        p = swerve(); p.set("steerP", "0"); reject(p);
        p = dual(); p.set("actuator2", "deployLeft"); reject(p);
        p = dual(); p.set("actuator1Tank", "-0.1"); reject(p);
        p = dual(); p.set("actuatorCount", "9"); reject(p);
        p = new RobotProfile(); p.set("drive", "CUSTOM"); reject(p);
    }
    private static void reject(RobotProfile p) {
        try { p.validate(); fail("Invalid layout should be rejected"); } catch (IllegalArgumentException expected) { }
    }

    @Test public void oldVersionOneProfilesLoadWithoutNewLayoutFields() throws Exception {
        JSONObject json = new RobotProfile().toJson().put("version", 1); JSONObject settings = json.getJSONObject("settings");
        for (String key : new java.util.ArrayList<>(new RobotProfile().values.keySet())) {
            if (key.startsWith("wheel") || key.startsWith("actuator") || key.endsWith("Servo") || key.endsWith("Encoder")
                    || key.endsWith("SteerReverse") || key.endsWith("EncoderReverse") || key.endsWith("ZeroDeg")
                    || key.startsWith("steer") || key.equals("firstWheelDeg") || key.equals("trackWidthIn")
                    || key.equals("initialState") || key.equals("transitionSeconds")) { settings.remove(key); }
        }
        assertEquals("MECANUM", RobotProfile.fromJson(json).get("drive"));
        settings.remove("fl");
        try { RobotProfile.fromJson(json); fail("Missing original settings must still be rejected"); } catch (IllegalArgumentException expected) { }
    }

    @Test public void localizationPreparationDoesNotInitializeSteeringOrDeploymentHardware() {
        // No servo or steering-encoder hardware exists here: calibration must only initialize pods/IMU.
        for (RobotProfile p : new RobotProfile[]{swerve(), dual()}) {
            p.set("localizer", "THREE_WHEEL"); p.set("offsetX", "12");
            p.set("forward", "forward"); p.set("right", "right"); p.set("strafe", "strafe");
            FakeHardware hardware = new FakeHardware(); hardware.motor("forward"); hardware.motor("right"); hardware.motor("strafe");
            SdkSetupHardware adapter = new SdkSetupHardware(hardware) {
                @Override public List<String> devices(String role) { return new java.util.ArrayList<>(p.values.values()); }
            };
            adapter.prepare(p); adapter.stop(); assertNotNull(adapter.getLocalizer());
        }
    }

    @Test public void missingConfiguredHardwareIsRejectedBeforeCalibrationStarts() {
        SdkSetupHardware adapter = new SdkSetupHardware(new FakeHardware()) {
            @Override public List<String> devices(String role) { return java.util.Collections.emptyList(); }
        };
        try { adapter.prepare(swerve()); fail("Missing configured hardware must block setup"); }
        catch (IllegalArgumentException expected) { assertTrue(expected.getMessage().contains("Robot Controller")); }
    }

    @Test public void savedSwerveZeroAnglesAndDirectionsControlRealModuleCommandsAndStop() {
        RobotProfile p = swerve(); p.set("flEncoderReverse", "true"); p.set("frSteerReverse", "true");
        FakeHardware hardware = new FakeHardware();
        double[] steering = new double[4];
        com.qualcomm.robotcore.hardware.DcMotorSimple.Direction[] directions = new com.qualcomm.robotcore.hardware.DcMotorSimple.Direction[4];
        String[] corners = p.motorKeys();
        for (int i = 0; i < corners.length; i++) {
            final int index = i;
            hardware.motor(p.get(corners[i]));
            hardware.register(p.get(corners[i] + "Servo"), java.lang.reflect.Proxy.newProxyInstance(
                    com.qualcomm.robotcore.hardware.CRServo.class.getClassLoader(),
                    new Class<?>[]{com.qualcomm.robotcore.hardware.CRServo.class}, (proxy, method, args) -> {
                        if (method.getName().equals("setPower")) { steering[index] = (double) args[0]; }
                        if (method.getName().equals("setDirection")) { directions[index] = (com.qualcomm.robotcore.hardware.DcMotorSimple.Direction) args[0]; }
                        return null;
                    }));
            com.qualcomm.robotcore.hardware.AnalogInputController controller = (com.qualcomm.robotcore.hardware.AnalogInputController)
                    java.lang.reflect.Proxy.newProxyInstance(com.qualcomm.robotcore.hardware.AnalogInputController.class.getClassLoader(),
                            new Class<?>[]{com.qualcomm.robotcore.hardware.AnalogInputController.class}, (proxy, method, args) -> {
                                if (method.getName().equals("getAnalogInputVoltage")) { return .825; }
                                if (method.getName().equals("getMaxAnalogInputVoltage")) { return 3.3; }
                                return null;
                            });
            hardware.register(p.get(corners[i] + "Encoder"), new com.qualcomm.robotcore.hardware.AnalogInput(controller, i));
        }
        BaseDrivetrain<?> drive = SetupConfigurations.drivetrain(RobotProfile.fromJson(p.toJson())).build(hardware);
        assertEquals(com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE, directions[1]);
        drive.moveWithVectors(.5, 0, 0);
        for (double power : steering) { assertEquals("Physical forward must be zero error", 0, power, 1e-10); }
        drive.moveWithVectors(0, .5, 0);
        assertTrue(Math.abs(steering[0]) > .1);
        drive.stop();
        for (double power : steering) { assertEquals(0, power, 0); }
    }

    @Test public void operatorCanReviewNewLayoutsCaptureForwardAndTestOnlyDriveMotors() throws Exception {
        for (RobotProfile p : new RobotProfile[]{swerve(), dual()}) {
            java.io.File directory = temp.newFolder(); SetupStore store = new SetupStore(directory); store.save(p);
            class Hardware extends RobotSetupWorkflowTest.Hardware {
                double power; int captures;
                @Override public java.util.List<String> devices(String role) {
                    java.util.List<String> names = new java.util.ArrayList<>(p.values.values());
                    return names;
                }
                @Override public MotorTestReading testMotor(String name, boolean reverse) {
                    power = .5; return new MotorTestReading(power, 0, 0, 12);
                }
                @Override public void stop() { super.stop(); power = 0; }
                @Override public double steeringAngleDegrees(String name) { captures++; return 123; }
            }
            Hardware hardware = new Hardware(); SetupLog log = new SetupLog(new java.io.File(directory, "logs"));
            try (RobotSetupWizard wizard = new RobotSetupWizard(hardware, store, log)) {
                handleSetupAction(wizard, RobotSetupWizard.Action.DOWN); handleSetupAction(wizard, RobotSetupWizard.Action.CONFIRM); wizard.updateMotorTest(false);
                for (SetupField field : SetupField.forProfile(p, hardware)) {
                    wizard.updateMotorTest(true);
                    boolean driveMotor = java.util.Arrays.asList(p.motorKeys()).contains(field.key)
                            || java.util.Arrays.stream(p.motorKeys()).anyMatch(key -> field.key.equals(key + "Reverse"));
                    assertEquals(driveMotor ? .5 : 0, hardware.power, 0);
                    if (field.key.endsWith("ZeroDeg")) { assertEquals(123, wizard.getDraft().number(field.key), 0); }
                    wizard.updateMotorTest(false); assertEquals(0, hardware.power, 0);
                    handleSetupAction(wizard, RobotSetupWizard.Action.CONFIRM); wizard.updateMotorTest(false);
                }
                assertEquals(RobotSetupWizard.Screen.SAVE, wizard.getScreen());
                assertEquals(p.get("drive").equals("COAXIAL_SWERVE") ? 4 : 0, hardware.captures);
                assertEquals("Capture edits stay in the draft", p.number("flZeroDeg"), store.load().number("flZeroDeg"), 0);
            }
        }
    }

    @Test public void omniSupportsVariableWheelCountAndCounterclockwiseOrder() {
        RobotProfile p = new RobotProfile(); p.set("drive", "OMNI"); p.set("wheelCount", "6");
        for (String key : p.motorKeys()) { p.set(key, key); }
        p.set("wheel6Reverse", "true"); p.set("firstWheelDeg", "-30");
        p = RobotProfile.fromJson(p.toJson());
        Omni.Constants c = (Omni.Constants) SetupConfigurations.drivetrain(p);
        assertEquals(6, c.getWheelCount()); assertTrue(p.holonomic());
        assertEquals("motor", field(p, "wheel6").role); assertFalse(field(p, "wheelCount").accepts("2"));
        p.set("wheel6", "wheel1"); reject(p);
    }
}
