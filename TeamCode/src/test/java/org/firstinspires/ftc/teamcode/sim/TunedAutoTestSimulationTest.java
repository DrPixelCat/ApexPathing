package org.firstinspires.ftc.teamcode.sim;

import core.ApexStorage;
import org.junit.Test;
import java.io.File;
import java.nio.file.Files;
import java.util.Arrays;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import static org.junit.Assert.assertTrue;

/** Reproduces the user workflow: every automatic tuning phase, then AutoTest. */
public class TunedAutoTestSimulationTest {
    @Test(timeout = 1_320_000L)
    public void freshlyTunedConstantsCompleteAutoTest() throws Exception {
        runWorkflow(false);
    }

    @Test(timeout = 1_320_000L)
    public void characterizedAfterAcceptedFeedforwardCompletesAutoTest() throws Exception {
        runWorkflow(true);
    }

    private void runWorkflow(boolean afterAcceptedFeedforward) throws Exception {
        String previous = System.getProperty(ApexStorage.DIRECTORY_PROPERTY);
        String outputDirectory = System.getenv("APEX_AUTOTEST_DATA_DIR");
        File directory = outputDirectory == null ? Files.createTempDirectory("apex-tuner-e2e-").toFile()
                : new File(outputDirectory);
        assertTrue(directory.isDirectory() || directory.mkdirs());
        System.setProperty(ApexStorage.DIRECTORY_PROPERTY, directory.getAbsolutePath());
        System.out.println("TUNER E2E DATA: " + directory.getAbsolutePath());
        try {
            new FollowerTunerTelemetryTest().automaticWorkflow(afterAcceptedFeedforward);
            assertProfileTrialCount(directory);
            new AutoTestSimulationTest().runAutoTest(true);
            assertVelocityTracking(directory);
        } finally {
            if (previous == null) { System.clearProperty(ApexStorage.DIRECTORY_PROPERTY); }
            else { System.setProperty(ApexStorage.DIRECTORY_PROPERTY, previous); }
        }
    }

    private static void assertProfileTrialCount(File directory) throws Exception {
        File[] traces = directory.listFiles((dir, name) -> name.startsWith("profiled_tracking_")
                && name.endsWith(".csv"));
        assertTrue("Profiled tuner must produce a trace", traces != null && traces.length > 0);
        File trace = Arrays.stream(traces).max(Comparator.comparingLong(File::lastModified)).get();
        Set<String> trials = new HashSet<>();
        List<String> lines = Files.readAllLines(trace.toPath());
        for (String line : lines.subList(1, lines.size())) {
            trials.add(line.substring(0, line.indexOf(',')));
        }
        assertTrue("Profiled tuner repeatedly retried the baseline: " + trials.size(),
                trials.size() <= 14);
    }

    private static void assertVelocityTracking(File directory) throws Exception {
        File[] traces = directory.listFiles((dir, name) -> name.startsWith("auto-test-outbound-velocity_")
                && name.endsWith(".csv"));
        assertTrue("AutoTest must produce a velocity trace", traces != null && traces.length > 0);
        File trace = Arrays.stream(traces).max(Comparator.comparingLong(File::lastModified)).get();
        List<String> lines = Files.readAllLines(trace.toPath());
        String[] headers = lines.get(0).split(",");
        Map<String, Integer> columns = new HashMap<>();
        for (int i = 0; i < headers.length; i++) { columns.put(headers[i], i); }
        double accelerationPeak = 0, accelerationSquared = 0, brakingSquared = 0;
        double curveHeadingSquared = 0, curveCrossSquared = 0;
        double curveHeadingPeak = 0, curveCrossPeak = 0;
        int accelerationSamples = 0, brakingSamples = 0, curveSamples = 0;
        for (int i = 1; i < lines.size(); i++) {
            String[] row = lines.get(i).split(",");
            double target = Double.parseDouble(row[columns.get("controller_target_in_s")]);
            if (target <= 0) { continue; }
            double error = Double.parseDouble(row[columns.get("controller_measured_in_s")]) - target;
            double acceleration = Double.parseDouble(row[columns.get("target_accel_in_s2")]);
            if (acceleration > 5) {
                accelerationPeak = Math.max(accelerationPeak, error);
                accelerationSquared += error * error;
                accelerationSamples++;
            }
            if (acceleration < -5) { brakingSquared += error * error; brakingSamples++; }
            if (Math.abs(Double.parseDouble(row[columns.get("curvature_in_inv")])) > 0.01
                    && Double.parseDouble(row[columns.get("endpoint_blend")]) < 0.05
                    && Double.parseDouble(row[columns.get("controller_measured_in_s")]) > 3.0) {
                double heading = Math.abs(Math.toDegrees(Double.parseDouble(
                        row[columns.get("heading_error_rad")])));
                double cross = Math.abs(Double.parseDouble(row[columns.get("cross_track_error_in")]));
                curveHeadingSquared += heading * heading;
                curveCrossSquared += cross * cross;
                curveHeadingPeak = Math.max(curveHeadingPeak, heading);
                curveCrossPeak = Math.max(curveCrossPeak, cross);
                curveSamples++;
            }
        }
        assertTrue("Freshly tuned acceleration overspeed: " + accelerationPeak, accelerationPeak < 10);
        assertTrue("Expected acceleration measurements", accelerationSamples > 10);
        double accelerationRms = Math.sqrt(accelerationSquared / accelerationSamples);
        assertTrue("Freshly tuned acceleration RMS: " + accelerationRms, accelerationRms < 8);
        assertTrue("Expected braking measurements", brakingSamples > 10);
        double brakingRms = Math.sqrt(brakingSquared / brakingSamples);
        assertTrue("Freshly tuned braking RMS: " + brakingRms, brakingRms < 8);
        assertTrue("Expected moving curve measurements", curveSamples > 10);
        double headingRms = Math.sqrt(curveHeadingSquared / curveSamples);
        double crossRms = Math.sqrt(curveCrossSquared / curveSamples);
        assertTrue("Freshly tuned curve heading RMS/peak: " + headingRms + "/" + curveHeadingPeak,
                headingRms < 12 && curveHeadingPeak < 20);
        assertTrue("Freshly tuned curve cross-track RMS/peak: " + crossRms + "/" + curveCrossPeak,
                crossRms < 1 && curveCrossPeak < 2);
    }
}
