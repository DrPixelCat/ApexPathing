package org.firstinspires.ftc.teamcode.sim;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.codeblooded.fit.MotorModel;
import org.codeblooded.ftcodesim.hardware.devices.SimMotor;

import java.util.IdentityHashMap;
import java.util.Map;
import java.util.concurrent.atomic.AtomicBoolean;

/** Desktop-only, scoped powered back-EMF override for automatic and manual SysId. */
public final class SimulatorSysIdMotorOverride {
    private SimulatorSysIdMotorOverride() { }

    public static Runnable open(HardwareMap hardwareMap) {
        Map<MotorModel, MotorModel.Term> originals = new IdentityHashMap<>();
        Runnable restore = () -> originals.forEach((model, term) -> model.terms[1] = term);
        try {
            for (DcMotorEx device : hardwareMap.getAll(DcMotorEx.class)) {
                if (!(device instanceof SimMotor)) continue;
                MotorModel model = ((SimMotor) device).config.motorModel;
                if (originals.containsKey(model)) continue;
                originals.put(model, model.terms[1]);
                // TODO: Verify the PWM/duty-dependent back-EMF model with the FTCodeSim
                // developer and real REV Hub measurements before making this permanent.
                // Temporary SysId-only workaround: remove the powered same-direction duty
                // multiplier. Leave coefficients, zero-power braking and reverse branches intact.
                model.terms[1] = (v, d, b) -> Math.signum(v) == Math.signum(d) ? -v : 0;
            }
        } catch (RuntimeException failure) {
            restore.run();
            throw failure;
        }
        AtomicBoolean closed = new AtomicBoolean();
        return () -> { if (closed.compareAndSet(false, true)) restore.run(); };
    }
}
