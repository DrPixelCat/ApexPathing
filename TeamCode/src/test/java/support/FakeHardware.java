package support;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.lang.reflect.Proxy;
import java.util.HashMap;
import java.util.Map;

/** Minimal SDK devices for exercising production kinematics and motor writes. */
public final class FakeHardware extends HardwareMap {
    private final Map<String, Object> devices = new HashMap<>();
    public double yaw;

    public FakeHardware() { super(null, null); }

    @Override public <T> T get(Class<? extends T> type, String name) {
        return type.cast(devices.get(name));
    }

    public static final class MotorState {
        public int ticks, resets, writes;
        public double power;
        public double velocity;
        public com.qualcomm.robotcore.hardware.DcMotorSimple.Direction direction = com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
        public DcMotor.ZeroPowerBehavior zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE;
        public DcMotor.RunMode mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
    }

    public MotorState motor(String name) {
        MotorState state = new MotorState();
        devices.put(name, Proxy.newProxyInstance(DcMotorEx.class.getClassLoader(),
                new Class<?>[]{DcMotorEx.class}, (proxy, method, args) -> {
                    switch (method.getName()) {
                        case "getCurrentPosition": return state.ticks;
                        case "getPower": return state.power;
                        case "getVelocity": return state.velocity;
                        case "getDirection": return state.direction;
                        case "setDirection": state.direction = (com.qualcomm.robotcore.hardware.DcMotorSimple.Direction) args[0]; return null;
                        case "getZeroPowerBehavior": return state.zeroPowerBehavior;
                        case "setZeroPowerBehavior": state.zeroPowerBehavior = (DcMotor.ZeroPowerBehavior) args[0]; return null;
                        case "setPower": state.power = (double) args[0]; state.writes++; return null;
                        case "getMode": return state.mode;
                        case "setMode":
                            state.mode = (DcMotor.RunMode) args[0];
                            if (state.mode == DcMotor.RunMode.STOP_AND_RESET_ENCODER) {
                                state.resets++; state.ticks = 0;
                            }
                            return null;
                        case "getDeviceName": return name;
                        case "hashCode": return System.identityHashCode(proxy);
                        case "equals": return proxy == args[0];
                        default: return null;
                    }
                }));
        return state;
    }

    public void imu(String name) {
        devices.put(name, Proxy.newProxyInstance(IMU.class.getClassLoader(),
                new Class<?>[]{IMU.class}, (proxy, method, args) -> {
                    switch (method.getName()) {
                        case "initialize": return true;
                        case "getRobotYawPitchRollAngles":
                            return new YawPitchRollAngles(AngleUnit.RADIANS, yaw, 0, 0, 0);
                        default: return null;
                    }
                }));
    }
}
