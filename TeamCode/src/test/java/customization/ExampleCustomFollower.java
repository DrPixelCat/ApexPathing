package customization;

import com.qualcomm.robotcore.hardware.HardwareMap;
import config.ApexConstants;
import follower.drivetrains.HolonomicFollower;
import geometry.Vector;
import java.util.function.LongSupplier;
import paths.movements.Path;
import trajectory.TrajectorySample;

/** A different feedback law using public/protected APIs only; no built-in path controller. */
public final class ExampleCustomFollower extends HolonomicFollower {
    public boolean explicitFeedforward;

    public ExampleCustomFollower(ApexConstants config, HardwareMap hardwareMap, LongSupplier clock) {
        super(config, hardwareMap, true, clock);
    }

    @Override protected MovementOutcome updatePath(Path path) {
        TrajectorySample target = samplePathTarget(path);
        Vector correction = target.toPath.times(.04)
                .plus(target.velocity.minus(getVelocity().getVec()).times(.01));
        double headingCorrection = .8 * getPose().getHeading().getShortestAngleTo(target.heading).getRad();
        return explicitFeedforward
                ? executeHolonomicPath(path, target, correction, headingCorrection, Vector.of(.2, 0, geometry.DistUnit.IN), .1)
                : executeHolonomicPath(path, target, correction, headingCorrection);
    }
}
