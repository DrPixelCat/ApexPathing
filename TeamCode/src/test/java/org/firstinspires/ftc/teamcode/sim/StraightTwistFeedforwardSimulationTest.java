package org.firstinspires.ftc.teamcode.sim;

import org.junit.Test;

/** An 80-inch stored straight profile rotating smoothly from 0 to 90 degrees, with feedback off. */
public class StraightTwistFeedforwardSimulationTest {
    @Test(timeout = 35_000L) public void straightLineWithTwistWithoutFeedback() throws Exception {
        LinePowerSimulationTest.run(true, true);
    }
    @Test(timeout = 35_000L) public void oppositeTwistWithoutFeedback() throws Exception {
        LinePowerSimulationTest.run(true,-90,1);
    }
    @Test(timeout = 35_000L) public void reverseTranslationWithTwistWithoutFeedback() throws Exception {
        LinePowerSimulationTest.run(true,90,-1);
    }
    @Test(timeout = 35_000L) public void onlineBuildUsesAnIndependentReferenceWithoutFeedback() throws Exception {
        LinePowerSimulationTest.run(false,true);
    }
    @Test(timeout = 35_000L) public void wallTimedStraightTwistWithoutFeedback() throws Exception {
        LinePowerSimulationTest.run(true,90,1,false);
    }
}
