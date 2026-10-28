package setup;

import java.util.List;
import org.json.JSONObject;

/** Hardware boundary shared by the robot wizard and developer workflow tests. */
public interface SetupHardware {
    List<String> devices(String role);
    void prepare(RobotProfile profile);
    SetupSnapshot update();
    void learn(boolean enabled);
    JSONObject captureFilter();
    /** Run only this motor at positive 0.5 duty cycle, with the selected logical direction. */
    default MotorTestReading testMotor(String name, boolean reversed) {
        throw new UnsupportedOperationException("Motor testing is unavailable in this hardware adapter");
    }
    void stop();
}
