package org.firstinspires.ftc.teamcode.sim;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.SimLinearOpModeBridge;
import com.qualcomm.robotcore.hardware.Gamepad;

import follower.Follower;
import org.firstinspires.ftc.teamcode.apexpathing.Constants;
import org.junit.Test;
import storage.ApexStorage;
import setup.logging.TuningCsvWriter;
import setup.follower.TunerContext;
import setup.follower.TuningPhase;
import setup.WorkflowText;

import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

import static org.junit.Assert.*;

public class TunerScreenWorkflowTest {
    @Test(timeout = 10_000) public void localizationConfirmationCannotBeginAndSaveTheSameMeasurement() throws Exception {
        String previousStorage = System.getProperty(ApexStorage.DIRECTORY_PROPERTY);
        System.setProperty(ApexStorage.DIRECTORY_PROPERTY,
                Files.createTempDirectory("apex-localization-confirmation-").toString());
        ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
        AtomicInteger starts = new AtomicInteger();
        AtomicInteger saves = new AtomicInteger();
        AtomicInteger cleanups = new AtomicInteger();
        AtomicReference<Boolean> accepted = new AtomicReference<>();
        AtomicReference<String> frame = new AtomicReference<>("");
        ApexSimTelemetry telemetry = new ApexSimTelemetry(frame::set);
        telemetry.setMsTransmissionInterval(0);
        LinearOpMode opMode = new LinearOpMode() {
            @Override public void runOpMode() {
                setup.localization.LocalizationTunerContext context =
                        new setup.localization.LocalizationTunerContext(this, new Constants());
                waitForStart();
                accepted.set(new setup.localization.TuningPhase(context) {
                    @Override protected String getName() { return "Confirmation test"; }
                    @Override protected void reset() { }
                    @Override protected void showPrepare() { context.getView().addLine("Ready to record"); }
                    @Override protected void beginRecording() { starts.incrementAndGet(); }
                    @Override protected boolean record() {
                        assertFalse("The begin edge must not enter collection", confirmPressed());
                        return true;
                    }
                    @Override protected void showReview() { context.getView().addLine("Ready to save calibration"); }
                    @Override protected boolean accept() { saves.incrementAndGet(); return true; }
                    @Override protected void cleanup() { cleanups.incrementAndGet(); }
                }.run(this));
            }
        };
        opMode.hardwareMap = hardware.hardwareMap;
        opMode.telemetry = telemetry;
        opMode.gamepad1 = new Gamepad();
        opMode.gamepad2 = new Gamepad();
        SimLinearOpModeBridge.Session session = SimLinearOpModeBridge.initialize(opMode, () -> { });
        try {
            SimLinearOpModeBridge.start(session);
            pump(session, opMode, telemetry, 100);
            opMode.gamepad1.a = true;
            pump(session, opMode, telemetry, 150);
            assertEquals(1, starts.get());
            assertTrue(frame.get(), frame.get().contains("Ready to save calibration"));
            assertEquals("Holding begin must not save calibration", 0, saves.get());
            assertNull(accepted.get());
            opMode.gamepad1.a = false;
            pump(session, opMode, telemetry, 100);
            opMode.gamepad1.a = true;
            pump(session, opMode, telemetry, 150);
            assertEquals(1, saves.get());
            assertEquals(Boolean.TRUE, accepted.get());
            assertEquals(1, cleanups.get());
        } finally {
            try { SimLinearOpModeBridge.stop(session); }
            finally {
                if (previousStorage == null) { System.clearProperty(ApexStorage.DIRECTORY_PROPERTY); }
                else { System.setProperty(ApexStorage.DIRECTORY_PROPERTY, previousStorage); }
            }
        }
    }

    @Test(timeout = 10_000) public void heldConfirmationCannotStartAndAcceptTheRoutineOrResults() throws Exception {
        try (PhaseSession session = new PhaseSession(true)) {
            session.holdConfirm();
            assertEquals(1, session.starts.get());
            assertFalse("The mode-selection edge must not accept the routine", session.resultsVisible());
            assertNull(session.accepted.get());
            session.releaseConfirm();
            session.holdConfirm();
            assertTrue(session.frame.get(), session.resultsVisible());
            assertNull("The routine-acceptance edge must not accept results", session.accepted.get());
            session.releaseConfirm();
            session.holdConfirm();
            assertEquals(Boolean.TRUE, session.accepted.get());
            assertEquals(1, session.exits.get());
        }
    }

    @Test(timeout = 10_000) public void stopDuringMeasurementClosesEvidenceAndRestoresControllerPolicy() throws Exception {
        PhaseSession session = new PhaseSession(false);
        try {
            session.holdConfirm();
            assertEquals(1, session.starts.get());
            assertNotNull(session.csv.get());
            assertFalse(session.follower.get().isHeadingControllerEnabled());
        } finally { session.close(); }
        assertEquals(Boolean.FALSE, session.accepted.get());
        assertEquals(1, session.exits.get());
        assertFalse(session.follower.get().isDriveControllerEnabled());
        assertTrue(session.follower.get().isHeadingControllerEnabled());
        assertEquals(0, session.follower.get().getDrivetrain().getLastFlPower(), 0);
        assertEquals(0, session.follower.get().getDrivetrain().getLastFrPower(), 0);
        Path evidence = Paths.get(session.csv.get().getPath());
        String text = new String(Files.readAllBytes(evidence), StandardCharsets.UTF_8);
        assertTrue("STOP must flush the buffered measurement log", text.contains("measurement"));
        session.csv.get().writeRow("after close");
        assertEquals(text, new String(Files.readAllBytes(evidence), StandardCharsets.UTF_8));
    }

    /** Exercises the public phase loop with real gamepad edges and OpMode STOP. */
    private static final class PhaseSession implements AutoCloseable {
        final AtomicInteger starts = new AtomicInteger();
        final AtomicInteger exits = new AtomicInteger();
        final AtomicReference<String> frame = new AtomicReference<>("");
        final AtomicReference<Boolean> accepted = new AtomicReference<>();
        final AtomicReference<Follower> follower = new AtomicReference<>();
        final AtomicReference<TuningCsvWriter> csv = new AtomicReference<>();
        final ApexSimTelemetry telemetry = new ApexSimTelemetry(frame::set);
        final LinearOpMode opMode;
        final SimLinearOpModeBridge.Session bridge;
        final String previousStorage;

        PhaseSession(boolean finishOnConfirm) throws Exception {
            previousStorage = System.getProperty(ApexStorage.DIRECTORY_PROPERTY);
            System.setProperty(ApexStorage.DIRECTORY_PROPERTY,
                    Files.createTempDirectory("apex-workflow-test-").toString());
            ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
            telemetry.setMsTransmissionInterval(0);
            opMode = new LinearOpMode() {
                @Override public void runOpMode() {
                    TunerContext context = new TunerContext(this);
                    Follower robot = Follower.create(new Constants(), hardwareMap, true);
                    follower.set(robot);
                    context.setFollower(robot);
                    robot.disableDriveController();
                    robot.enableHeadingController();
                    waitForStart();
                    accepted.set(new TuningPhase(context) {
                        @Override protected String getPhaseName() { return "Confirmation test"; }
                        @Override protected boolean manualTuneIsPossible() { return false; }
                        @Override protected boolean autoTuneIsPossible() { return true; }
                        @Override protected void init() {
                            starts.incrementAndGet();
                            robot.disableControllers();
                            csv.set(TuningCsvWriter.openBuffered("workflow_stop", "stage"));
                            csv.get().writeRow("measurement");
                        }
                        @Override protected boolean manualTuned() { return false; }
                        @Override protected boolean autoTuned() {
                            robot.getDrivetrain().moveWithVectors(.2, 0, 0);
                            return finishOnConfirm && confirmPressed();
                        }
                        @Override protected boolean routineMotionActive() { return true; }
                        @Override protected void reportResults() { context.getView().addLine("Results ready"); }
                        @Override protected void onExit() {
                            if (csv.get() != null) { csv.get().close(); }
                            exits.incrementAndGet();
                        }
                    }.run(this));
                }
            };
            opMode.hardwareMap = hardware.hardwareMap;
            opMode.telemetry = telemetry;
            opMode.gamepad1 = new Gamepad();
            opMode.gamepad2 = new Gamepad();
            bridge = SimLinearOpModeBridge.initialize(opMode, () -> { });
            SimLinearOpModeBridge.start(bridge);
            pump(bridge, opMode, telemetry, 100);
        }

        boolean resultsVisible() { return frame.get().contains("Results ready"); }
        void holdConfirm() throws Exception { opMode.gamepad1.a = true; pump(bridge, opMode, telemetry, 150); }
        void releaseConfirm() throws Exception { opMode.gamepad1.a = false; pump(bridge, opMode, telemetry, 100); }
        @Override public void close() {
            try { SimLinearOpModeBridge.stop(bridge); }
            finally {
                if (previousStorage == null) { System.clearProperty(ApexStorage.DIRECTORY_PROPERTY); }
                else { System.setProperty(ApexStorage.DIRECTORY_PROPERTY, previousStorage); }
            }
        }
    }

    @Test(timeout = 10_000) public void failedRoutineStopsMotionAndIgnoresAttemptsToRestart() throws Exception {
        ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
        AtomicInteger starts = new AtomicInteger();
        AtomicReference<String> frame = new AtomicReference<>("");
        AtomicReference<Follower> follower = new AtomicReference<>();
        ApexSimTelemetry telemetry = new ApexSimTelemetry(frame::set);
        telemetry.setMsTransmissionInterval(0);
        LinearOpMode opMode = new LinearOpMode() {
            @Override public void runOpMode() {
                TunerContext context = new TunerContext(this);
                follower.set(Follower.create(new Constants(), hardwareMap, true));
                context.setFollower(follower.get());
                waitForStart();
                new TuningPhase(context) {
                    @Override protected String getPhaseName() { return "Workflow test"; }
                    @Override protected boolean manualTuneIsPossible() { return false; }
                    @Override protected boolean autoTuneIsPossible() { return true; }
                    @Override protected void init() {
                        starts.incrementAndGet();
                        context.getFollower().getDrivetrain().moveWithVectors(.2, 0, 0);
                    }
                    @Override protected boolean manualTuned() { return false; }
                    @Override protected boolean autoTuned() { throw new IllegalStateException("Trial failed"); }
                    @Override protected void reportResults() { }
                }.run(this);
            }
        };
        opMode.hardwareMap = hardware.hardwareMap;
        opMode.telemetry = telemetry;
        opMode.gamepad1 = new Gamepad();
        opMode.gamepad2 = new Gamepad();
        SimLinearOpModeBridge.Session session = SimLinearOpModeBridge.initialize(opMode, () -> { });
        try {
            SimLinearOpModeBridge.start(session);
            pump(session, opMode, telemetry, 100);
            pressConfirm(session, opMode, telemetry);
            assertTrue(frame.get(), frame.get().contains("Tuning stopped Trial failed"));
            assertTrue(frame.get().contains(WorkflowText.STOP_HINT));
            assertEquals(0, follower.get().getDrivetrain().getLastFlPower(), 0);
            assertEquals(0, follower.get().getDrivetrain().getLastFrPower(), 0);
            pressConfirm(session, opMode, telemetry);
            opMode.gamepad1.x = true;
            pump(session, opMode, telemetry, 60);
            assertEquals(1, starts.get());
            assertTrue(frame.get().contains("Tuning stopped Trial failed"));
            assertFalse(frame.get().contains("Robot can be driven with sticks"));
        } finally {
            SimLinearOpModeBridge.stop(session);
        }
        assertEquals(0, follower.get().getDrivetrain().getLastFlPower(), 0);
    }

    private static void pressConfirm(SimLinearOpModeBridge.Session session,
            LinearOpMode opMode, ApexSimTelemetry telemetry) throws Exception {
        opMode.gamepad1.a = true;
        pump(session, opMode, telemetry, 60);
        opMode.gamepad1.a = false;
        pump(session, opMode, telemetry, 60);
    }

    private static void pump(SimLinearOpModeBridge.Session session, LinearOpMode opMode,
            ApexSimTelemetry telemetry, long milliseconds) throws Exception {
        long deadline = System.nanoTime() + milliseconds * 1_000_000;
        while (System.nanoTime() < deadline) {
            SimLinearOpModeBridge.eventLoopIteration(session, opMode.gamepad1, opMode.gamepad2);
            telemetry.update();
            Thread.sleep(5);
        }
    }
}
