package tuning;

import org.junit.Test;
import static org.junit.Assert.*;

public class FeedforwardVerificationTest {
    @Test public void profileTracksAnIndependentPlantInBothDirectionsAndBrakesToRest() {
        // Simulate power=kS*sign(v)+kV*v+kA*a at 1 kHz, with no feedback controller.
        double kS = .08, kV = .02, kA = .005, dt = .001;
        FeedforwardTuner.TrapezoidProfile profile = new FeedforwardTuner.TrapezoidProfile(15, 30);
        for (double direction : new double[]{1, -1}) {
            double velocity = 0, position = 0;
            for (int step = 0; step < Math.round(profile.getTotalTime() / dt); step++) {
                double time = step * dt;
                double targetV = profile.getVel(time) * direction, targetA = profile.getAccel(time) * direction;
                double power = FeedforwardVerificationPhase.profilePower(kS, kV, kA, targetV, targetA);
                double frictionDirection = Math.abs(velocity) < 1e-8 ? direction : Math.signum(velocity);
                double acceleration = (power - kS * frictionDirection - kV * velocity) / kA;
                velocity += acceleration * dt;
                position += velocity * dt;
                assertEquals("Velocity should follow the profile without PID", profile.getVel(time + dt) * direction, velocity, .05);
            }
            assertEquals(0, velocity, .05);
            assertEquals(profile.getTotalDistance() * direction, position, .05);
        }
        assertEquals(0, FeedforwardVerificationPhase.profilePower(kS, kV, kA, 0, 0), 0);
    }

    @Test public void angularProfilesPreserveSignsWhileDecelerating() {
        FeedforwardTuner.TrapezoidProfile turn = new FeedforwardTuner.TrapezoidProfile(1, 2);
        double time = turn.getCruiseEnd() + .25;
        double positive = FeedforwardVerificationPhase.profilePower(.05, .2, .1, turn.getVel(time), turn.getAccel(time));
        double negative = FeedforwardVerificationPhase.profilePower(.05, .2, .1, -turn.getVel(time), -turn.getAccel(time));
        assertTrue("Deceleration can require power opposite the current motion", positive < 0);
        assertEquals(-positive, negative, 0);
    }
}
