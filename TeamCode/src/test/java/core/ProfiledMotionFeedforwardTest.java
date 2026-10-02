package core;

import com.acmerobotics.roadrunner.MotorFeedforward;
import org.junit.Test;

import static org.junit.Assert.assertEquals;

/** Verify Apex's power-unit adaptation against the actual Road Runner feedforward. */
public class ProfiledMotionFeedforwardTest {
    @Test public void feedbackBeforeFeedforwardMatchesRoadRunnerInBothDirections() {
        double kV = .94 / 75, kA = .94 / 150, kS = .06, gain = .05;
        MotorFeedforward rr = new MotorFeedforward(kS, kV, kA);
        for (double velocity : new double[]{-30, -2, 2, 30}) {
            for (double acceleration : new double[]{-60, -30, 0, 30, 60}) {
                for (double measured : new double[]{-50, -15, 0, 15, 50}) {
                    double feedback = gain * (velocity - measured);
                    double sign = MotionFeedforward.profiledStaticSign(
                            velocity, acceleration, feedback, kV);
                    double power = sign * kS + kV * velocity + kA * acceleration + feedback;
                    assertEquals(rr.compute(velocity + feedback / kV, acceleration), power, 1e-12);
                }
            }
        }
    }

    @Test public void breakawayMatchesApexProfileModelAndZeroVelocityGainIsSupported() {
        assertEquals(1, MotionFeedforward.profiledStaticSign(0, 30, 0, .01), 0);
        assertEquals(-1, MotionFeedforward.profiledStaticSign(0, -30, 0, .01), 0);
        assertEquals(0, MotionFeedforward.profiledStaticSign(0, 0, 0, .01), 0);
        assertEquals(-1, MotionFeedforward.profiledStaticSign(-2, 30, .2, 0), 0);
    }
}
