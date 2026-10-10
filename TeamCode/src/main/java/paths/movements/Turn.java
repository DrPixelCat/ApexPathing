package paths.movements;

import follower.FollowerConstants;
import trajectory.profile.FFLut;
import trajectory.generators.TurnProfileGenerator;
import geometry.Angle;
import geometry.Pose;
import paths.Callback;

/**
 * Represents a stationary point-turn movement. The robot will remain at its starting (x, y)
 * coordinates and rotate to the specified target heading.
 *
 * @author Joel H
 * @author Sohum Arora - 22985 Paraducks
 */
public class Turn extends FollowerMovement {
    private FollowerConstants.Profile driveProfile;
    private FollowerConstants planningConstants;
    public FollowerConstants.Profile getDriveProfile() { return driveProfile; }
    public void setDriveProfile(FollowerConstants.Profile profile) {
        if (profile != driveProfile) {
            driveProfile = profile;
            if (planningConstants != null && profile != null) {
                planningConstants = planningConstants.forProfile(profile);
            }
            FFLut = null;
        }
    }
    private final Pose startPose;
    private FFLut FFLut;
    private Path.BuildMode buildMode = Path.BuildMode.ONLINE;
    public Path.BuildMode getBuildMode() { return buildMode; }
    public void setBuildMode(Path.BuildMode mode) { buildMode = mode; }
    public FollowerConstants getPlanningConstants() {
        if (planningConstants == null) {
            FollowerConstants constants = FollowerConstants.getInstance();
            planningConstants = driveProfile == null ? constants.copy() : constants.forProfile(driveProfile);
        }
        return planningConstants.copy();
    }
    public void setPlanningConstants(FollowerConstants constants) {
        planningConstants = driveProfile == null ? constants.copy() : constants.forProfile(driveProfile);
        FFLut = null;
    }
    public double getAngularVelocityLimit(FollowerConstants c) {
        return Double.isFinite(angularVelLimitRad) ? angularVelLimitRad : c.angularVelLimitRad;
    }
    public double getAngularAccelerationLimit(FollowerConstants c) {
        return Double.isFinite(angularAccelLimitRad) ? angularAccelLimitRad : c.angularAccelLimitRad;
    }
    /** Builder initialization: computes the angular LUT once and never replaces it. */
    public void precomputeTrajectory() {
        if (buildMode == Path.BuildMode.MEMORY && FFLut == null) {
            FollowerConstants c = getPlanningConstants();
            FFLut = new TurnProfileGenerator(getAngularVelocityLimit(c), getAngularAccelerationLimit(c), c).generate(this);
        }
    }
    private final double angularVelLimitRad;
    private final double angularAccelLimitRad;

    /**
     * Constructs a Turn movement.
     *
     * @param startPose The robot's state at the beginning of the turn.
     * @param targetHeading The final angle the robot should face.
     */
    public Turn(Pose startPose, Angle targetHeading) {
        this(startPose, targetHeading, Double.NaN, Double.NaN);
    }

    public Turn(Pose startPose, Angle targetHeading, double angularVelLimitRad,
                double angularAccelLimitRad) {
        this.startPose = startPose;
        this.endPose = new Pose(startPose.getVec(), targetHeading);
        this.angularVelLimitRad = angularVelLimitRad;
        this.angularAccelLimitRad = angularAccelLimitRad;
    }

    /** Attaches a heading callback to this traversal and returns the turn for chaining. */
    public Turn addAngularCallback(Angle heading, Runnable action) {
        addCallback(new Callback(heading, action));
        return this;
    }

    public Pose getStartPose() { return startPose; }

    public FFLut getFeedforwardLut() { return FFLut; }

    public void setFeedforwardLut(FFLut FFLut) { this.FFLut = FFLut; }

    @Override
    public Turn reversed() {
        Turn result = new Turn(endPose, startPose.getHeading(), angularVelLimitRad,
                angularAccelLimitRad);
        result.setDriveProfile(driveProfile);
        result.planningConstants = planningConstants == null ? null : planningConstants.copy();
        result.buildMode = buildMode;
        result.precomputeTrajectory();
        return result;
    }
}
