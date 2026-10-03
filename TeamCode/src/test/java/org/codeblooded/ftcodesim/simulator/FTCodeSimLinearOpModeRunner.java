package org.codeblooded.ftcodesim.simulator;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.SimLinearOpModeBridge;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.psilynx.psikit.core.Logger;

import java.util.concurrent.atomic.AtomicBoolean;

import follower.Follower;

/** Runs FTCodeSim with the FTC SDK's real LinearOpMode lifecycle. */
public final class FTCodeSimLinearOpModeRunner {
    private FTCodeSimLinearOpModeRunner() { }

    public static void run(FTCodeSim simulator) throws InterruptedException {
        try {
            while (simulator.windowIsRunning()) {
                OpModeLifecycle lifecycle = simulator.opModeLifecycle;
                if (lifecycle == null) {
                    Thread.sleep(simulator.config.loopTimeMs);
                    continue;
                }

                if (lifecycle.opMode instanceof LinearOpMode) {
                    runLinearOpMode(lifecycle);
                } else {
                    lifecycle.runOpMode();
                }

                if (simulator.opModeLifecycle == lifecycle) {
                    simulator.opModeLifecycle = null;
                }
            }
        } finally {
            simulator.close();
        }
    }

    static void runLinearOpMode(OpModeLifecycle lifecycle) throws InterruptedException {
        // OpModeLifecycle.wrap() unconditionally sleeps loopTimeMs after all the work in the
        // iteration. Budget that work inside the period instead: motion makes physics and
        // logging slower, and the unconditional sleep turns that extra work into timing jitter.
        long originalLoopTimeMs = lifecycle.loopTimeMs;
        long periodNanos = originalLoopTimeMs * 1_000_000L;
        LinearOpMode opMode = (LinearOpMode) lifecycle.opMode;
        opMode.telemetry = lifecycle.telemetry;
        opMode.hardwareMap = lifecycle.simHardwareMap;
        opMode.gamepad1 = new Gamepad();
        opMode.gamepad2 = new Gamepad();

        SimFtcLogger ftcLog = new SimFtcLogger();
        ftcLog.start(opMode, 5800, "", true, "sim-logs");
        Logger.setSimulation(true);
        long start = System.nanoTime();
        Logger.setTimeSource(() -> (System.nanoTime() - start) * 1e-9);
        ApexAdvantageScopeLogger apexLogger = new ApexAdvantageScopeLogger();
        Follower.setDiagnostics(apexLogger);

        AtomicBoolean userRequestedStop = new AtomicBoolean(false);
        SimLinearOpModeBridge.Session session = null;

        try {
            lifecycle.loopTimeMs = 0;
            session = SimLinearOpModeBridge.initialize(
                    opMode,
                    () -> userRequestedStop.set(true)
            );
            while (!lifecycle.isStarted
                    && !lifecycle.isStopped
                    && !userRequestedStop.get()) {
                runEventLoopIteration(lifecycle, session, apexLogger, periodNanos);
            }

            if (lifecycle.isStarted
                    && !lifecycle.isStopped
                    && !userRequestedStop.get()) {
                SimLinearOpModeBridge.start(session);
            }

            while (!lifecycle.isStopped && !userRequestedStop.get()) {
                runEventLoopIteration(lifecycle, session, apexLogger, periodNanos);
            }
        } finally {
            lifecycle.loopTimeMs = originalLoopTimeMs;
            if (session != null) {
                SimLinearOpModeBridge.stop(session);
            }
            Follower.setDiagnostics(null);
            Logger.end();
        }
    }

    private static void runEventLoopIteration(
            OpModeLifecycle lifecycle,
            SimLinearOpModeBridge.Session session,
            ApexAdvantageScopeLogger apexLogger,
            long periodNanos
    ) throws InterruptedException {
        long iterationStartedNanos = System.nanoTime();
        Gamepad gamepad1 = new Gamepad();
        Gamepad gamepad2 = new Gamepad();
        gamepad1.fromByteArray(lifecycle.latestGamepad1Data);
        gamepad2.fromByteArray(lifecycle.latestGamepad2Data);

        lifecycle.wrap(() -> {
            lifecycle.simHardwareMap.update();
            SimLinearOpModeBridge.eventLoopIteration(session, gamepad1, gamepad2);
            apexLogger.flush();
        });
        long deadlineNanos = iterationStartedNanos + periodNanos;
        // Windows timer granularity and scheduler stalls make even 1 ms sleeps miss many
        // deadlines. This simulator runs on a desktop JVM; spin for a stable 50 Hz physics
        // cadence instead of allowing the wait itself to introduce large motion jumps.
        while (System.nanoTime() < deadlineNanos) {
            Thread.onSpinWait();
        }
        Logger.recordOutput("OpMode/totalLoopTimeMs",
                (System.nanoTime() - iterationStartedNanos) * 1e-6);
    }
}
