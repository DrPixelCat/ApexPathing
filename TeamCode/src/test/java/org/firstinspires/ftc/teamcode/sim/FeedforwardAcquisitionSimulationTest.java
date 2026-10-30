package org.firstinspires.ftc.teamcode.sim;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.SimLinearOpModeBridge;
import com.qualcomm.robotcore.hardware.Gamepad;
import follower.Follower;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import java.util.concurrent.atomic.AtomicInteger;
import localizers.LocalizationConstants;
import org.firstinspires.ftc.teamcode.apexpathing.Constants;
import org.junit.Test;
import setup.follower.TunerContext;
import setup.follower.sysid.FeedforwardTuner;
import storage.ApexStorage;
import static org.junit.Assert.*;

/** Checks acquisition and honest fit acceptance on the unmodified, nonlinear stock plant. */
public class FeedforwardAcquisitionSimulationTest {
    @Test(timeout = 45_000) public void driveRampUsesFreshMeasurementsAndReportsFitQuality() throws Exception {
        runAcquisition(FeedforwardTuner.Axis.DRIVE);
    }

    @Test(timeout = 45_000) public void headingRampUsesFreshMeasurementsAndReportsFitQuality() throws Exception {
        runAcquisition(FeedforwardTuner.Axis.HEADING);
    }

    private void runAcquisition(FeedforwardTuner.Axis axis) throws Exception {
        String previousStorage = System.getProperty(ApexStorage.DIRECTORY_PROPERTY);
        String previousUnlock = System.getProperty("apex.simulation.unlockTunerPhases");
        Path output = java.nio.file.Paths.get("build", "ks-kv-acquisition", axis.name());
        Files.createDirectories(output);
        Path storage = Files.createTempDirectory(output.toAbsolutePath(), "storage-");
        System.setProperty(ApexStorage.DIRECTORY_PROPERTY, storage.toString());
        AtomicReference<String> latest = new AtomicReference<>("");
        AtomicReference<Boolean> completed = new AtomicReference<>();
        AtomicReference<RuntimeException> workerFailure = new AtomicReference<>();
        AtomicReference<double[]> accelerationGains = new AtomicReference<>();
        AtomicReference<double[]> originalGains = new AtomicReference<>(), finalGains = new AtomicReference<>();
        AtomicInteger placements = new AtomicInteger();
        List<String> trace = new ArrayList<>();
        trace.add("time_s,x_from_center_in,y_from_center_in,vx_in_s,omega_rad_s,front_left_power,raw_body_velocity,raw_omega,measurement_time_ns");
        ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
        LinearOpMode opMode = new LinearOpMode() {
            @Override public void runOpMode() {
                try {
                    Constants robot = new Constants();
                    Follower seed = Follower.create(robot, hardwareMap, true);
                    LocalizationConstants localization = LocalizationConstants.empty();
                    localization.capture("DEFAULT", robot.localizerConstants(), seed.getLocalizer(),
                            LocalizationConstants.Status.ACCEPTED, LocalizationConstants.Status.ACCEPTED);
                    try { localization.save(); }
                    catch (java.io.IOException failure) { throw new IllegalStateException(failure); }
                    TunerContext context = new TunerContext(this) {
                        private long previous = System.nanoTime();
                        private final long start = previous;

                        @Override public void positionRobotForSimulation(paths.geometry.Pose pose) {
                            placements.incrementAndGet();
                            super.positionRobotForSimulation(pose);
                        }

                        @Override public void beginFrame() {
                            long now = System.nanoTime();
                            // Advance all elapsed time under the preceding command, including
                            // scheduler pauses. Dropping time would alter the apparent plant.
                            double remaining = (now - previous) * 1e-9;
                            previous = now;
                            while (remaining > 1e-9) {
                                double dt = Math.min(.005, remaining);
                                ApexSimulation.stepPhysics(hardware, dt);
                                remaining -= dt;
                            }
                            paths.geometry.Pose raw = getFollower().getRawVelocity();
                            double bodyVelocity = raw.getVec().rotate(getFollower().getLocalizer()
                                    .getRawPose().getHeading().times(-1)).getX().getIn();
                            trace.add(String.format(java.util.Locale.US, "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d",
                                    (now-start)*1e-9, hardware.drivetrain.position.x-ApexSimulation.FIELD_CENTER_INCHES,
                                    hardware.drivetrain.position.y-ApexSimulation.FIELD_CENTER_INCHES,
                                    hardware.drivetrain.velocity.x, hardware.drivetrain.velocity.theta,
                                    getFollower().getDrivetrain().getLastFlPower(), bodyVelocity,
                                    raw.getHeading(paths.geometry.AngleUnit.RAD),
                                    getFollower().getLocalizer().getLastMeasurementTimeNanos()));
                            super.beginFrame();
                        }
                    };
                    context.setFollower(Follower.create(robot, hardwareMap, true));
                    // Nonzero sentinels make preservation observable on both axes.
                    context.constants.translationalKA = .013;
                    context.constants.angularKA = .027;
                    originalGains.set(new double[]{context.constants.translationalFeedforwardKS,
                            context.constants.translationalKV, context.constants.angularFeedforwardKS,
                            context.constants.angularKV});
                    waitForStart();
                    completed.set(new FeedforwardTuner(context, axis).run(this));
                    accelerationGains.set(new double[]{context.constants.translationalKA, context.constants.angularKA});
                    finalGains.set(new double[]{context.constants.translationalFeedforwardKS,
                            context.constants.translationalKV, context.constants.angularFeedforwardKS,
                            context.constants.angularKV});
                } catch (RuntimeException failure) {
                    workerFailure.set(failure);
                }
            }
        };
        opMode.hardwareMap = hardware.hardwareMap;
        opMode.telemetry = new ApexSimTelemetry(latest::set);
        opMode.gamepad1 = new Gamepad();
        opMode.gamepad2 = new Gamepad();
        SimLinearOpModeBridge.Session session = null;
        long start = System.nanoTime();
        try {
            session = SimLinearOpModeBridge.initialize(opMode, () -> { });
            SimLinearOpModeBridge.start(session);
            long lastPress = 0;
            while ((System.nanoTime()-start)*1e-9 < 40 && completed.get() == null && workerFailure.get() == null) {
                long now = System.nanoTime();
                String frame = latest.get();
                if (frame.contains("acquisition failed") || frame.contains("Tuning stopped")) break;
                if (frame.contains("Press A") && now-lastPress > 300_000_000L) lastPress = now;
                opMode.gamepad1.a = now-lastPress < 60_000_000L;
                SimLinearOpModeBridge.eventLoopIteration(session, opMode.gamepad1, opMode.gamepad2);
                Thread.sleep(5);
            }
        } finally {
            if (session != null) SimLinearOpModeBridge.stop(session);
            double seconds = (System.nanoTime()-start)*1e-9;
            String result = "Axis: " + axis + "\nStorage: " + storage + "\nSeconds: " + seconds + "\nCompleted: " + completed.get()
                    + "\nWorker failure: " + workerFailure.get() + "\nLatest telemetry:\n" + latest.get();
            Files.write(output.resolve("result.txt"), result.getBytes(StandardCharsets.UTF_8));
            Files.write(output.resolve("trace.csv"), trace, StandardCharsets.UTF_8);
            System.out.println(result);
            restore(ApexStorage.DIRECTORY_PROPERTY, previousStorage);
            restore("apex.simulation.unlockTunerPhases", previousUnlock);
        }
        assertNull("Worker exception", workerFailure.get());
        Path fitCsv;
        try (java.util.stream.Stream<Path> files = Files.list(storage)) {
            fitCsv = files.filter(path -> path.getFileName().toString().startsWith("sysid_fit_")
                    && path.toString().endsWith(".csv")).findFirst().orElse(null);
        }
        assertNotNull("Acquire the full ramp sequence and report its fit, rather than timing out", fitCsv);
        List<String> fitRows = Files.readAllLines(fitCsv, StandardCharsets.UTF_8);
        assertTrue("Report measured fit samples", fitRows.size() > 1);
        String[] fitColumns = fitRows.get(1).split(",");
        boolean accepted = Boolean.parseBoolean(fitColumns[11]);
        assertEquals("The UI must honor the measured fit quality", accepted, completed.get());
        if (accepted) {
            assertTrue("Preserve the fit-quality gate", Double.parseDouble(fitColumns[10]) >= .98);
            int offset = axis == FeedforwardTuner.Axis.DRIVE ? 0 : 2;
            assertEquals(Double.parseDouble(fitColumns[7]), finalGains.get()[offset], 1e-12);
            assertEquals(Double.parseDouble(fitColumns[8]), finalGains.get()[offset + 1], 1e-12);
        } else {
            assertTrue("Explain rejection in the UI", latest.get().contains("acquisition failed"));
            assertArrayEquals("A rejected fit must leave gains intact", originalGains.get(), finalGains.get(), 0);
        }
        assertEquals("Only the initial pose may be staged", 1, placements.get());
        assertArrayEquals("A kS/kV ramp must retain kA on both axes", new double[]{.013, .027},
                accelerationGains.get(), 0);
        double maximumRawSpeed = 0;
        for (int i = 1; i < trace.size(); i++) {
            String[] columns = trace.get(i).split(",");
            assertTrue("Keep the whole run clear of the walls", Math.abs(Double.parseDouble(columns[1])) < 48);
            assertTrue("Keep the whole run clear of the walls", Math.abs(Double.parseDouble(columns[2])) < 48);
            maximumRawSpeed = Math.max(maximumRawSpeed,
                    Math.abs(Double.parseDouble(columns[axis == FeedforwardTuner.Axis.DRIVE ? 6 : 7])));
        }
        assertTrue("Raw velocity must refresh after movement begins", maximumRawSpeed > 2);
    }

    private static void restore(String key, String value) {
        if (value == null) System.clearProperty(key); else System.setProperty(key, value);
    }
}
