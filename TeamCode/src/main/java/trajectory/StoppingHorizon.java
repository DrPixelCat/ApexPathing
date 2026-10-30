package trajectory;

import follower.FollowerConstants;
import feedforward.MotionCharacterization;
import drivetrains.models.DriveModel;
import paths.geometry.Vector;
import trajectory.profile.DynamicMotionProfile;
import paths.movements.Path;

/**
 * Fixed lookahead selected once from power-limited braking, with distance and reaction headroom.
 *
 * @author Joel H
 */
public final class StoppingHorizon {
    private StoppingHorizon() { }

    public static double distance(FollowerConstants c, Path path, double power) {
        return distance(c,path,power,path.getDriveModel(c));
    }
    public static double distance(FollowerConstants c, Path path, double power, DriveModel driveModel) {
        double result = axis(c, MotionCharacterization.Axis.FORWARD, c.forwardVelLimitIn, c.forwardAccelLimitIn,
                c.translationalKV, c.translationalKA, c.translationalFeedforwardKS, power);
        if (path.getPathType() == Path.PathType.HOLONOMIC && c.strafeVelLimitIn > 0 && c.strafeAccelLimitIn > 0) {
            result = Math.max(result, axis(c, MotionCharacterization.Axis.STRAFE, c.strafeVelLimitIn, c.strafeAccelLimitIn,
                    c.translationalKV, c.translationalKA, c.translationalFeedforwardKS, power));
            // Include custom directional acceleration limits and every circular wheel layout.
            for (int degrees=0;degrees<360;degrees++) {
                double x=Math.cos(Math.toRadians(degrees)), y=Math.sin(Math.toRadians(degrees));
                Vector body=TrajectorySample.vector(x,y);
                double speed=driveModel.maximumVelocity(body,0,0,c.angularVelLimitRad);
                double acceleration=driveModel.maximumTangentialAcceleration(body);
                double factor=driveModel.translationDemand(x,y);
                result=Math.max(result,stoppingDistance(speed,acceleration,c.translationalKV*factor,
                        c.translationalKA*factor,c.translationalFeedforwardKS*factor,power));
            }
        }

        return Math.max(2, result);
    }

    private static double axis(FollowerConstants c, MotionCharacterization.Axis axis, double velocity,
                               double acceleration, double kV, double kA, double kS, double power) {
        return stoppingDistance(measuredLimit(c, axis, velocity), acceleration, kV, kA, kS, power);
    }

    private static double measuredLimit(FollowerConstants c, MotionCharacterization.Axis axis, double configured) {
        return c.motionCharacterization == null ? configured
                : Math.min(configured, c.motionCharacterization.maximumMeasuredVelocity(axis));
    }

    private static double stoppingDistance(double velocity, double acceleration, double kV, double kA,
                                           double kS, double power) {
        DynamicMotionProfile.PowerModel model = new DynamicMotionProfile.PowerModel(velocity, acceleration, kV, kA, kS, power);
        // Size for the calibrated/configured maximum, including a moving start above
        // sustainable cruising speed. Braking reachability accepts that measured boundary.
        double speed = velocity;
        // Invert the same braking reachability used by scalar profiles. Both direction models are
        // included in maximumMeasuredVelocity; signed braking retains the identified inertia.
        double low = 0, high = Math.max(1, speed * speed / (2 * acceleration));
        DynamicMotionProfile.Reachability braking = model.backwardReachability();
        for (int i = 0; i < 32 && braking.maxReachableVelocity(0, speed, high, 0, 0) < speed - 1e-9; i++) { high *= 2; }
        for (int i = 0; i < 40; i++) {
            double mid = (low + high) / 2;
            if (braking.maxReachableVelocity(0, speed, mid, 0, 0) >= speed - 1e-9) { high = mid; }
            else { low = mid; }
        }
        return 1.5 * high + .05 * speed;
    }
}
