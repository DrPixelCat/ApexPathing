package setup.follower;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import storage.ApexStorage;
import follower.Follower;
import follower.FollowerConstants;
import paths.geometry.Pose;

/**
 * Provides a context for the tuner phases to operate in, including access th the OpMode, telemetry,
 * and the follower instance.
 *
 * @author Sohum Arora - 22985 Paraducks
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public class TunerContext extends setup.TunerContext {
    private Follower follower;
    public FollowerConstants constants;
    public TunerContext(LinearOpMode opMode) { super(opMode); }

    public void setFollower(Follower follower) {
        this.follower = java.util.Objects.requireNonNull(follower, "Follower");
        this.constants = follower.getConstants();
    }

    public Follower getFollower() { return follower; }

    /** Adds controls which must remain visible independently of the current phase. */
    public void addInterfaceHeader() {
        addDebugHeader();
        if (follower != null && follower.getDrivetrain() instanceof drivetrains.DualActuated) {
            getView().addData("Tuning profile", constants.getActiveProfile());
        }
    }

    public void publishFrame(boolean sticksEnabled) {
        if (sticksEnabled) { getView().addLine("Robot can be driven with sticks"); }
        getView().update();
    }

    /** Teleports only FTCodeSim; real hardware must still be positioned by its operator. */
    public void positionRobotForSimulation(Pose pose) {
        if (!Boolean.getBoolean("apex.simulation.unlockTunerPhases")) { return; }
        follower.stop();
        follower.setPose(pose);
    }

    public boolean saveConstants() {
        try {
            ApexStorage.saveConstants(constants.toJson().toString(4));
            return true;
        } catch (Exception e) {
            getView().addLine("Could not save constants: " + e.getMessage());
            return false;
        }
    }
}
