package org.firstinspires.ftc.teamcode.sim;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import follower.Follower;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import org.codeblooded.fit.MotorModel;
import org.codeblooded.ftcodesim.hardware.devices.SimMotor;
import org.firstinspires.ftc.teamcode.apexpathing.Constants;
import org.junit.Test;
import setup.follower.TunerContext;
import setup.follower.TuningPhase;
import setup.follower.sysid.FeedforwardTuner;
import static org.junit.Assert.*;

/** Tuning initialization and cleanup must never replace the simulator's motor equations. */
public class SimulatorSysIdPlantTest {
    @Test public void manualAndAutomaticTuningPreserveEveryMotorTerm() throws Exception {
        for (boolean manual : new boolean[]{false, true}) {
            ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
            LinearOpMode opMode = new LinearOpMode() {
                @Override public void runOpMode() { }
            };
            opMode.hardwareMap = hardware.hardwareMap;
            opMode.telemetry = new ApexSimTelemetry(frame -> { });
            Follower follower = Follower.create(new Constants(), hardware.hardwareMap, true);
            TunerContext context = new TunerContext(opMode);
            context.setFollower(follower);
            SimMotor[] motors = StockSimTestSupport.motors(hardware);
            MotorModel.Term[][] originals = new MotorModel.Term[motors.length][];
            double[][] coefficients = new double[motors.length][];
            for (int i = 0; i < motors.length; i++) {
                originals[i] = motors[i].config.motorModel.terms.clone();
                coefficients[i] = motors[i].config.modelCoefficients.clone();
            }
            FeedforwardTuner tuner = new FeedforwardTuner(context);
            Field mode = TuningPhase.class.getDeclaredField("manualMode");
            Field op = TuningPhase.class.getDeclaredField("opMode");
            mode.setAccessible(true);
            op.setAccessible(true);
            mode.setBoolean(tuner, manual);
            op.set(tuner, opMode);
            Method init = FeedforwardTuner.class.getDeclaredMethod("init");
            Method exit = FeedforwardTuner.class.getDeclaredMethod("onExit");
            init.setAccessible(true);
            exit.setAccessible(true);
            try {
                init.invoke(tuner);
                for (int i = 0; i < motors.length; i++) {
                    assertArrayEquals(originals[i], motors[i].config.motorModel.terms);
                    assertArrayEquals(coefficients[i], motors[i].config.modelCoefficients, 0);
                    assertEquals(-12 * .3, motors[i].config.motorModel.terms[1].calculate(12, .3, 13), 1e-12);
                }
            } finally {
                exit.invoke(tuner);
                follower.stop();
            }
            for (int i = 0; i < motors.length; i++) {
                assertArrayEquals(originals[i], motors[i].config.motorModel.terms);
            }
        }
    }
}
