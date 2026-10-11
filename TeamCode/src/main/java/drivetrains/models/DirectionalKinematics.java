package drivetrains.models;

import drivetrains.Mecanum;
import follower.FollowerConstants;
import geometry.Angle;
import geometry.DistUnit;
import geometry.Vector;

/** Direction is robot-relative: zero is forward, positive pi/2 is left.
 * Providers must be hardware-independent. Recreate them for each settings snapshot. */
@FunctionalInterface
public interface DirectionalKinematics {
    DirectionalLimits limitsAt(Angle bodyDirection);
    /** Custom/live providers default to uncached solver evaluation. */
    default boolean cacheSolverSamples() { return false; }

    static DirectionalKinematics isotropic(FollowerConstants settings) {
        return new DirectionalKinematics() {
            @Override public DirectionalLimits limitsAt(Angle direction) {
                return new DirectionalLimits(settings.forwardVelLimitIn, settings.forwardAccelLimitIn);
            }
            @Override public boolean cacheSolverSamples() { return true; }
        };
    }
    /** Conservative fallback when a standalone builder has no physical wheel geometry. */
    static DirectionalKinematics omni(FollowerConstants settings) {
        return new DirectionalKinematics() {
            @Override public DirectionalLimits limitsAt(Angle direction) {
                return new DirectionalLimits(Math.min(settings.forwardVelLimitIn,settings.strafeVelLimitIn),
                        Math.min(settings.forwardAccelLimitIn,settings.strafeAccelLimitIn));
            }
            @Override public boolean cacheSolverSamples() { return true; }
        };
    }
    static DirectionalKinematics mecanum(FollowerConstants settings) {
        return new DirectionalKinematics() {
            private Mecanum.DirectionalLut lut;
            private double forwardVelocity, forwardAcceleration, strafeVelocity, strafeAcceleration;
            @Override public DirectionalLimits limitsAt(Angle direction) {
                if (lut == null || forwardVelocity != settings.forwardVelLimitIn || forwardAcceleration != settings.forwardAccelLimitIn
                        || strafeVelocity != settings.strafeVelLimitIn || strafeAcceleration != settings.strafeAccelLimitIn) {
                    forwardVelocity = settings.forwardVelLimitIn; forwardAcceleration = settings.forwardAccelLimitIn;
                    strafeVelocity = settings.strafeVelLimitIn; strafeAcceleration = settings.strafeAccelLimitIn;
                    lut = new Mecanum.DirectionalLut(forwardVelocity, forwardAcceleration, strafeVelocity, strafeAcceleration);
                }
                Mecanum.DirectionalLut.DirectionalKinematics limits = lut.getKinematics(
                        Vector.of(Math.cos(direction.getRad()), Math.sin(direction.getRad()), DistUnit.IN), Angle.zero());
                return new DirectionalLimits(limits.maxVel, limits.maxAccel);
            }
            @Override public boolean cacheSolverSamples() { return true; }
        };
    }
    /** Identical-wheel envelope fitted conservatively to the configured forward and strafe limits. */
    static DirectionalKinematics projected(FollowerConstants settings, WheelProjectionModel wheels) {
        return new DirectionalKinematics() {
            @Override public DirectionalLimits limitsAt(Angle direction) {
                double forward = wheels.demand(1, 0, 0), strafe = wheels.demand(0, 1, 0);
                double wheelVelocity = Math.min(settings.forwardVelLimitIn * forward, settings.strafeVelLimitIn * strafe);
                double wheelAcceleration = Math.min(settings.forwardAccelLimitIn * forward, settings.strafeAccelLimitIn * strafe);
                double factor = wheels.demand(Math.cos(direction.getRad()), Math.sin(direction.getRad()), 0);
                return new DirectionalLimits(wheelVelocity / factor, wheelAcceleration / factor);
            }
            @Override public boolean cacheSolverSamples() { return true; }
        };
    }
}
