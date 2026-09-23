package org.firstinspires.ftc.teamcode.sim;

import org.codeblooded.ftcodesim.hardware.SimHardwareMap;
import org.codeblooded.ftcodesim.hardware.drivetrain.SimMecanumConfig;
import org.codeblooded.ftcodesim.hardware.drivetrain.SimTankConfig;
import org.codeblooded.ftcodesim.hardware.drivetrain.SimulatedDrivetrain;
import org.codeblooded.ftcodesim.hardware.drivetrain.SimulatedMecanum;
import org.codeblooded.ftcodesim.hardware.drivetrain.SimulatedTank;
import org.codeblooded.ftcodesim.input.DefaultKeybinds;
import org.codeblooded.ftcodesim.ascope.boundaries.MotionVector;
import org.codeblooded.ftcodesim.ascope.boundaries.RobotGeometry;
import org.codeblooded.ftcodesim.ascope.SourceType;
import org.codeblooded.ftcodesim.simulator.FTCodeSim;
import org.codeblooded.ftcodesim.simulator.FTCodeSimTelemetryInstaller;
import org.codeblooded.ftcodesim.ascope.ApexAdvantageScopeLayout;
import org.codeblooded.ftcodesim.simulator.SimConfig;
import org.firstinspires.ftc.teamcode.apexpathing.FollowerTuner;

import java.io.File;
import java.io.IOException;

import core.ApexStorage;

/** Shared FTCodeSim configuration used by every Apex Pathing OpMode. */
public final class ApexSimulation {
    static final String TANK_SIMULATION_PROPERTY = "apex.simulation.tank";
    public static final String FRONT_LEFT_MOTOR = "frontLeftMotor";
    public static final String FRONT_RIGHT_MOTOR = "frontRightMotor";
    public static final String BACK_LEFT_MOTOR = "backLeftMotor";
    public static final String BACK_RIGHT_MOTOR = "backRightMotor";
    public static final String PINPOINT = "pinpoint";
    public static final String IMU = "imu";

    // FTCodeSim's DECODE field is represented in corner-origin coordinates.
    static final double FIELD_CENTER_INCHES = 141.5 / 2.0;

    private ApexSimulation() { }

    public static SimConfig createConfig() {
        System.setProperty(TANK_SIMULATION_PROPERTY, "false");
        return createConfig(createHardware());
    }

    /** Creates an interactive Code Sim configuration backed by differential-drive physics. */
    public static SimConfig createTankConfig() {
        System.setProperty(TANK_SIMULATION_PROPERTY, "true");
        return createConfig(createTankHardware());
    }

    static boolean isTankSimulation() {
        return Boolean.parseBoolean(System.getProperty(TANK_SIMULATION_PROPERTY, "false"));
    }

    private static SimConfig createConfig(Hardware hardware) {
        SimConfig config = new SimConfig();
        config.gamepad1Keybinds = new DefaultKeybinds();
        config.gamepad2Keybinds = new DefaultKeybinds();
        hardware.hardwareMap.register(IMU, SimApexImu.create(hardware.drivetrain));
        config.simHardwareMap = hardware.hardwareMap;
        config.loopTimeMs = 20;
        return config;
    }

    public static FTCodeSim createSimulator() throws IOException {
        FTCodeSim simulator = new FTCodeSim(createConfig());
        ApexAdvantageScopeLayout.install(isTankSimulation());
        installTelemetryAdapter(simulator);
        return simulator;
    }

    /** Creates Code Sim with the tank hardware used by the tank-only test OpModes. */
    public static FTCodeSim createTankSimulator() throws IOException {
        FTCodeSim simulator = new FTCodeSim(createTankConfig());
        ApexAdvantageScopeLayout.install(isTankSimulation());
        installTelemetryAdapter(simulator);
        return simulator;
    }

    static Hardware createHardware() {
        configureDesktopStorage();

        SimMecanumConfig config = new SimMecanumConfig();
        // The alpha release consumes logical powers with Apex's forward/left/CCW convention.
        // SimMotor ignores SDK direction; the old snapshot's mirrored slot workaround is wrong here.
        config.frontLeftMotorName = FRONT_LEFT_MOTOR;
        config.frontRightMotorName = FRONT_RIGHT_MOTOR;
        config.backLeftMotorName = BACK_LEFT_MOTOR;
        config.backRightMotorName = BACK_RIGHT_MOTOR;

        // Geometry/speed parameters from the DECODE example.
        config.wheelbase = 14;
        config.trackWidth = 14;
        config.wheelRadius = 1.889765;
        config.maxAcceleration = 150;
        config.maxVelocity = 75;
        // Preserve the configured natural-deceleration input. In alpha.1 it contributes
        // to the motor scale and regenerative-braking fit, not a 6% Coulomb friction term.
        // The release motor has duty-dependent drag and no static breakaway gate.
        config.naturalDeceleration = config.maxAcceleration * 0.06 / (1.0 - 0.06);
        // alpha.1 moved directional friction to the chassis. Zero translation radii
        // make its directional interpolation divide by zero (even during a pure turn).
        config.forwardNaturalDeceleration = config.naturalDeceleration;
        config.strafeNaturalDeceleration = config.naturalDeceleration;
        config.turnNaturalDeceleration = config.naturalDeceleration
                / ((config.wheelbase + config.trackWidth) / 2.0);
        config.staticVelocityRegion = 0.05;
        config.staticFriction = (config.maxAcceleration + config.naturalDeceleration) * 0.08;
        // v0.1.0-alpha.1 removed strafeEfficiency; use the release's mecanum kinematics.
        config.robotGeometry = new RobotGeometry(12, 18, 2, 0);
        config.robotModel = SourceType.ROBOT_CODE_BLOODED_DECODE;

        SimulatedDrivetrain drivetrain = new SimulatedMecanum(config);
        SimHardwareMap hardwareMap = new StableSimHardwareMap();
        hardwareMap.register(drivetrain);

        // Put Apex's centered origin at the center of FTCodeSim's corner-origin field.
        drivetrain.setPosition(new MotionVector(
                FIELD_CENTER_INCHES,
                FIELD_CENTER_INCHES,
                0.0
        ));
        hardwareMap.register(PINPOINT, new SimApexPinpoint(drivetrain, FIELD_CENTER_INCHES));
        return new Hardware(hardwareMap, drivetrain);
    }

    static Hardware createTankHardware() {
        configureDesktopStorage();

        SimTankConfig config = new SimTankConfig();
        // Apex uses positive turn for left/CCW. SimulatedTank's wheel-slot convention is the
        // opposite, and SimMotor ignores SDK motor direction, so mirror the physical slots while
        // retaining the production tank motor mix.
        config.frontLeftMotorName = FRONT_RIGHT_MOTOR;
        config.frontRightMotorName = FRONT_LEFT_MOTOR;
        config.backLeftMotorName = BACK_RIGHT_MOTOR;
        config.backRightMotorName = BACK_LEFT_MOTOR;
        config.trackWidth = 14.0;
        config.wheelRadius = 2.0;
        config.staticVelocityRegion = 0.05;
        config.staticFriction = 0.1;
        config.maxAcceleration = 150;
        config.maxVelocity = 75;
        config.naturalDeceleration = 40;
        config.robotGeometry = new RobotGeometry(12, 18, 2, 0);
        config.robotModel = SourceType.ROBOT_TANK;

        SimulatedDrivetrain drivetrain = new SimulatedTank(config);
        SimHardwareMap hardwareMap = new StableSimHardwareMap();
        hardwareMap.register(drivetrain);
        // Use duty-independent back EMF: power = kS*sign(v) + kV*v + kA*a.
        // The library default multiplies back EMF by duty, so its steady-state response
        // is nonlinear and cannot be represented by Apex's constant kS/kV coefficients.
        double kS = 0.06;
        double kV = (1.0 - kS) / config.maxVelocity;
        double kA = (1.0 - kS) / config.maxAcceleration;
        for (String name : drivetrain.motorNames) {
            org.codeblooded.ftcodesim.hardware.devices.SimMotor motor =
                    (org.codeblooded.ftcodesim.hardware.devices.SimMotor) hardwareMap.get(
                            com.qualcomm.robotcore.hardware.DcMotorEx.class, name);
            double[] coefficients = {1.0 / (kA * config.wheelRadius * config.nominalVoltage),
                    0.0, kV / kA, kS / (kA * config.wheelRadius)};
            motor.config.modelCoefficients = coefficients;
            motor.config.zeroPowerBrakeCoefficients = coefficients.clone();
        }
        drivetrain.setPosition(new MotionVector(FIELD_CENTER_INCHES, FIELD_CENTER_INCHES, 0.0));
        hardwareMap.register(PINPOINT, new SimApexPinpoint(drivetrain, FIELD_CENTER_INCHES));
        return new Hardware(hardwareMap, drivetrain);
    }

    private static void configureDesktopStorage() {
        System.setProperty(FollowerTuner.UNLOCK_PHASES_PROPERTY, "true");
        if (System.getProperty(ApexStorage.DIRECTORY_PROPERTY) == null) {
            File directory = new File(System.getProperty("user.dir"), "build/ftcodesim-data");
            System.setProperty(ApexStorage.DIRECTORY_PROPERTY, directory.getAbsolutePath());
        }
    }

    private static void installTelemetryAdapter(FTCodeSim simulator) {
        FTCodeSimTelemetryInstaller.install(simulator, new ApexSimTelemetry(simulator));
    }

    static final class Hardware {
        final SimHardwareMap hardwareMap;
        final SimulatedDrivetrain drivetrain;
        long lastPhysicsUpdateNanos = System.nanoTime();

        Hardware(SimHardwareMap hardwareMap, SimulatedDrivetrain drivetrain) {
            this.hardwareMap = hardwareMap;
            this.drivetrain = drivetrain;
        }
    }

    /** Advance the same complete drivetrain mechanism used by the desktop simulator. */
    static void stepPhysics(Hardware hardware, double seconds) {
        for (String name : hardware.drivetrain.motorNames) {
            ((org.codeblooded.ftcodesim.hardware.devices.SimMotor) hardware.hardwareMap.get(
                    com.qualcomm.robotcore.hardware.DcMotorEx.class, name)).update(seconds);
        }
        try {
            hardware.drivetrain.update(seconds);
        } catch (NullPointerException missingViewer) {
            // The alpha release registers a visualization source after completing physics.
            // Headless tests have no viewer. Suppress only that specific post-physics error.
            StackTraceElement[] trace = missingViewer.getStackTrace();
            if (trace.length == 0) throw missingViewer;
            StackTraceElement top = trace[0];
            if (org.codeblooded.ftcodesim.ascope.AdvantageScopeRunner.INSTANCE != null
                    || !top.getClassName().equals("org.codeblooded.ftcodesim.ascope.boundaries.MotionVector")
                    || !top.getMethodName().equals("log")) throw missingViewer;
        }
        if (!Double.isFinite(hardware.drivetrain.position.x)
                || !Double.isFinite(hardware.drivetrain.position.y)
                || !Double.isFinite(hardware.drivetrain.position.theta)) {
            throw new IllegalStateException("Simulator produced a nonfinite drivetrain pose");
        }
    }
}
