package org.firstinspires.ftc.teamcode.sim;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import org.junit.Test;
import static org.junit.Assert.*;

/** Independent quality assertions over recorded motion, outside the tuning state machine. */
public class ProfiledTrackingValidationTest {
    static void assertLastTrial(Path csv) throws Exception {
        List<String> lines = Files.readAllLines(csv);
        assertTrue("Missing tracking samples", lines.size() > 1);
        String trial = lines.get(lines.size() - 1).split(",", -1)[0];
        double accelerationSquared = 0, brakingSquared = 0, peak = 0, crossTrack = 0, heading = 0;
        int accelerationCount = 0, brakingCount = 0, cornerCount = 0;
        for (int i = 1; i < lines.size(); i++) {
            String[] row = lines.get(i).split(",", -1);
            if (!trial.equals(row[0]) || Double.parseDouble(row[10]) >= .05) { continue; }
            double target = Double.parseDouble(row[4]);
            double measured = Double.parseDouble(row[6]);
            double acceleration = Double.parseDouble(row[8]);
            double error = measured - Math.abs(target);
            peak = Math.max(peak, Math.abs(target));
            crossTrack = Math.max(crossTrack, Math.abs(Double.parseDouble(row[15])));
            heading = Math.max(heading, Math.abs(Math.toDegrees(Double.parseDouble(row[16]))));
            if (acceleration > 5) { accelerationSquared += error * error; accelerationCount++; }
            if (acceleration < -5) {
                brakingSquared += error * error; brakingCount++;
                if (Math.abs(Double.parseDouble(row[9])) > .005) { cornerCount++; }
            }
        }
        assertTrue("Independent trial needs acceleration, braking, and corner evidence",
                accelerationCount >= 10 && brakingCount >= 10 && cornerCount >= 5);
        assertQuality(Math.sqrt(accelerationSquared / accelerationCount),
                Math.sqrt(brakingSquared / brakingCount), peak, crossTrack, heading);
    }

    private static void assertQuality(double accelerationRms, double brakingRms,
                                      double peak, double crossTrack, double heading) {
        assertTrue("Acceleration RMS", Double.isFinite(accelerationRms)
                && accelerationRms <= Math.max(4, .18 * peak));
        assertTrue("Braking RMS", Double.isFinite(brakingRms)
                && brakingRms <= Math.max(4, .16 * peak));
        assertTrue("Cross-track peak", Double.isFinite(crossTrack) && crossTrack <= 2);
        assertTrue("Heading peak", Double.isFinite(heading) && heading <= 12);
    }

    @Test public void detectsEachTrackingRegressionIndependently() {
        assertQuality(5, 4, 40, 1, 6);
        for (double[] bad : new double[][] {
                {8, 4, 40, 1, 6}, {5, 7, 40, 1, 6},
                {5, 4, 40, 2.1, 6}, {5, 4, 40, 1, 12.1},
                {Double.NaN, 4, 40, 1, 6}}) {
            assertThrows(AssertionError.class, () -> assertQuality(bad[0], bad[1], bad[2], bad[3], bad[4]));
        }
    }
}
