package org.firstinspires.ftc.teamcode.sim;

import org.junit.Test;

/** The single-path Apex AutoTest with explicitly calibrated centered braking and normal feedback. */
public class CenteredBrakingAutoTestSimulationTest {
    @Test(timeout = 130_000L) public void autoTestWithCenteredBraking() throws Exception {
        new AutoTestSimulationTest().runAutoTest(false, false, true);
    }
}
