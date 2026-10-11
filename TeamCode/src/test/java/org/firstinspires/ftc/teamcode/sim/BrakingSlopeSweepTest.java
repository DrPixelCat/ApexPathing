package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.*;

import com.acmerobotics.roadrunner.DisplacementProfile;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.Profiles;
import com.acmerobotics.roadrunner.Rotation2dDual;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeProfile;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import config.ApexConstants;
import storage.ApexStorage;
import follower.Follower;
import follower.FollowerConstants;
import drivetrains.BaseDrivetrainConstants;
import drivetrains.Mecanum;
import drivetrains.Motor;
import trajectory.profile.FFLut;
import trajectory.profile.MotionParameters;
import geometry.Angle;
import geometry.GeometryFactory;
import geometry.Pose;
import geometry.Vector;
import localizers.BaseLocalizer;
import localizers.BaseLocalizerConstants;
import org.codeblooded.ftcodesim.hardware.devices.SimMotor;
import org.firstinspires.ftc.teamcode.apexpathing.FollowerTuner;
import org.json.JSONObject;
import org.junit.Test;
import org.knowm.xchart.BitmapEncoder;
import org.knowm.xchart.XYChart;
import org.knowm.xchart.XYChartBuilder;
import org.knowm.xchart.style.Styler;
import org.knowm.xchart.style.markers.SeriesMarkers;
import paths.movements.Path;
import paths.builders.PathBuilder;
import paths.constraint.PathConstraint;
import paths.constraint.TranslationalConstraint;
import geometry.Dist;
import geometry.PathPoint;

import java.io.File;
import java.lang.reflect.Field;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/** Calls the published Road Runner 1.0.1 profile, controller, kinematics, and feedforward code.
 * All policies share one profile per slope. Only the Road Runner holonomic TIME case
 * advances the reference by time; the other policies preserve Apex's displacement sampling.
 * Tank RR_DISPLACEMENT is a straight-line adaptation, not a native Ramsete comparison. */
public class BrakingSlopeSweepTest {
    private static final double LENGTH = 480, STOP = 420, POSITIVE_ACCEL = 60;
    private static final double[] CHANGES = {0, 90, 120, 210, 240, 330, 360};
    private static final double[] SPEEDS = {42, 18, 40, 12, 36, 20, 42};
    private static final double[] BOUNDARIES = {90, 210, 330};
    private static final double[] BRAKES = {120, 90, 60, 30, 15};
    enum Policy { APEX_PROFILED, PROPOSED_SCALED, RR_DISPLACEMENT, RR_TIME }

    @Test public void followerKeepsSlowTargetsAndSignedAcceleration() throws Exception {
        Field singleton = FollowerConstants.class.getDeclaredField("instance");
        singleton.setAccessible(true);
        Object previous = singleton.get(null);
        try {
            for (boolean tank : new boolean[]{false,true}) for (double a : new double[]{-20,20}) {
                for (double measured : new double[]{0,2,8}) {
                    FollowerConstants c = constants(tank); singleton.set(null,c);
                    Bench bench = new Bench(tank);
                    bench.h.drivetrain.position.x = ApexSimulation.FIELD_CENTER_INCHES+30;
                    StockSimTestSupport.setSpeed(bench.h,measured);
                    Sensor sensor = new Sensor(bench,0); sensor.sample(.02,0);
                    GeometryFactory g = new GeometryFactory();
                    Path path = tank ? g.tankPath(g.pose(0,0,0),g.pose(LENGTH,0,0)).buildToMemory()
                            : g.holonomicPath(g.pose(0,0,0),g.pose(LENGTH,0,0)).buildToMemory();
                    FFLut lut = new FFLut(new MotionParameters[]{new MotionParameters(2,a,0,0,0),
                            new MotionParameters(2,a,0,0,LENGTH)});
                    path.setFeedforwardLut(lut);
                    Follower follower = Follower.create(robot(tank,sensor),bench.h.hardwareMap);
                    try {
                        follower.follow(path); follower.update();
                        assertEquals("No minimum-speed override",2,follower.getTrackingVelocityTarget(),1e-10);
                        double feedback = c.velocityFeedbackGain*(2-measured);
                        MotorFeedforward rr = new MotorFeedforward(c.translationalFeedforwardKS,
                                c.translationalKV,c.translationalKA);
                        assertEquals("Continuous signed profile command",rr.compute(2+feedback/c.translationalKV,a),
                                bench.power(),1e-10);
                        assertSame(lut,path.getFeedforwardLut());
                        assertEquals(a,lut.getFFParams(30).getTangentialAccel(),0);
                    } finally { follower.stop(); }
                }
            }
        } finally { singleton.set(null,previous); }
    }

    @Test public void apexGeneratedProfilesTrackWithoutChangingTheirConstraints() throws Exception {
        File output = new File("build/apex-generated-braking-validation").getAbsoluteFile();
        assertTrue(output.isDirectory() || output.mkdirs());
        Field singleton = FollowerConstants.class.getDeclaredField("instance");
        singleton.setAccessible(true);
        Object previous = singleton.get(null);
        String storage = System.getProperty(ApexStorage.DIRECTORY_PROPERTY);
        List<Run> runs = new ArrayList<>();
        try {
            System.setProperty(ApexStorage.DIRECTORY_PROPERTY, new File(output, "storage").toString());
            for (boolean tank : new boolean[]{false, true}) {
                FollowerConstants c = constants(tank);
                c.forwardAccelLimitIn = c.strafeAccelLimitIn = 30;
                singleton.set(null, c);
                GeometryFactory g = new GeometryFactory();
                PathBuilder<?> builder = tank
                        ? g.tankPath(g.pose(0,0,0), g.pose(LENGTH,0,0))
                        : g.holonomicPath(g.pose(0,0,0), g.pose(LENGTH,0,0));
                for (int i = 0; i < CHANGES.length; i++) {
                    builder.addConstraint(new TranslationalConstraint(CHANGES[i]/LENGTH,
                            PathConstraint.Type.VELOCITY, Dist.fromIn(SPEEDS[i])));
                }
                Path path = builder.buildToMemory(); // Actual Apex generator, no injected RR profile.
                FFLut original = path.getFeedforwardLut();
                List<MotionParameters> before = new ArrayList<>();
                for (PathPoint point : path.getGeneratedPoints()) {
                    double length = path.getParametricPath().getLengthIn();
                    double s = length - point.getDistanceToEndIn();
                    MotionParameters p = original.getFFParams(s);
                    int phase = 0;
                    double progress = path.constraintProgress(1-point.getDistanceToEndIn()/length);
                    while (phase+1 < CHANGES.length && progress >= CHANGES[phase+1]/LENGTH) phase++;
                    assertTrue("Generated velocity ceiling at " + s + ": " + p.getTangentialVel(),
                            p.getTangentialVel() <= SPEEDS[phase]+1e-6);
                    assertTrue("Generated acceleration ceiling", Math.abs(p.getTangentialAccel()) <= 30+1e-6);
                    before.add(p);
                }
                for (double dt : new double[]{.01,.02,.04}) for (int sensor=0;sensor<3;sensor++) {
                    singleton.set(null,c);
                    Run run = execute(tank,c,path,null,Policy.APEX_PROFILED,30,dt,sensor);
                    runs.add(run); writeTrace(output,run); graphRun(output,run);
                }
                assertSame("Execution must preserve the generated LUT", original,path.getFeedforwardLut());
                for (MotionParameters p : before) {
                    MotionParameters after = original.getFFParams(p.getProgression());
                    assertEquals(p.getTangentialVel(),after.getTangentialVel(),0);
                    assertEquals(p.getTangentialAccel(),after.getTangentialAccel(),0);
                    assertEquals(p.getAngularVel(),after.getAngularVel(),0);
                    assertEquals(p.getAngularAccel(),after.getAngularAccel(),0);
                }
            }
            writeSummary(output,runs,new ArrayList<>());
            gallery(output,runs);
            for (Run run : runs) assertTrue("Acceleration prerequisite: "+run.name(),run.accelPass());
            for (Run run : runs) assertTrue("Generated-profile braking: "+run.name(),run.brakePass());
        } finally { singleton.set(null,previous); restore(ApexStorage.DIRECTORY_PROPERTY,storage); }
    }

    @Test public void gentlerBrakingIsComparedOnlyAfterAccelerationPasses() throws Exception {
        // Preserve the earlier current-Apex baseline gallery as historical evidence.
        File output = new File("build/profiled-braking-validation").getAbsoluteFile();
        assertTrue(output.isDirectory() || output.mkdirs());
        Field singleton = FollowerConstants.class.getDeclaredField("instance");
        singleton.setAccessible(true);
        Object previousConstants = singleton.get(null);
        String previousStorage = System.getProperty(ApexStorage.DIRECTORY_PROPERTY);
        String previousUnlock = System.getProperty(FollowerTuner.UNLOCK_PHASES_PROPERTY);
        List<Run> runs = new ArrayList<>();
        List<String> prerequisites = new ArrayList<>();
        try {
            System.setProperty(ApexStorage.DIRECTORY_PROPERTY, new File(output, "isolated-storage").toString());
            for (boolean tank : new boolean[]{false, true}) {
                FollowerConstants c = constants(tank);
                double ramp = accelerationRamp(tank, c, output);
                prerequisites.add(String.format(Locale.US, "%s feedforward-only acceleration ramp RMS %.6f in/s",
                        tank ? "TANK" : "MECANUM", ramp));
                assertTrue("Acceleration model must pass before the sweep", ramp < .5);
                Files.write(new File(output, (tank ? "TANK" : "MECANUM") + "-constants.json").toPath(),
                        c.toJson().toString(2).getBytes(StandardCharsets.UTF_8));
                for (double braking : BRAKES) {
                    DisplacementProfile profile = profile(braking);
                    singleton.set(null, c);
                    Path path = path(tank, profile);
                    verifyProfile(profile, braking);
                    for (double dt : new double[]{.01, .02, .04}) for (int sensor = 0; sensor < 3; sensor++) {
                        for (Policy policy : Policy.values()) {
                            if (tank && policy == Policy.RR_TIME) continue;
                            singleton.set(null, c);
                            Run run = execute(tank, c, path, profile, policy, braking, dt, sensor);
                            runs.add(run);
                            writeTrace(output, run);
                        }
                    }
                    Files.write(new File(output, "progress.txt").toPath(),
                            (runs.size() + " runs completed").getBytes(StandardCharsets.UTF_8));
                }
            }
            // All raw evidence is preserved even when a policy's acceleration prerequisite fails.
            writeSummary(output, runs, prerequisites);
            boolean accelerationPassed = true;
            for (Run run : runs) accelerationPassed &= run.accelPass();
            for (Run run : runs) graphRun(output, run);
            graphComparisons(output, runs);
            gallery(output, runs);
            assertEquals(315, runs.size());
            assertTrue("Acceleration failed; do not interpret the affected policy's braking", accelerationPassed);
            for (Run run : runs) {
                if (run.policy == Policy.APEX_PROFILED && run.braking <= 60) {
                    assertTrue("Profiled braking target failed: " + run.name(), run.brakePass());
                }
            }
        } finally {
            singleton.set(null, previousConstants);
            restore(ApexStorage.DIRECTORY_PROPERTY, previousStorage);
            restore(FollowerTuner.UNLOCK_PHASES_PROPERTY, previousUnlock);
        }
    }

    private static FollowerConstants constants(boolean tank) throws Exception {
        FollowerConstants c = FollowerConstants.fromJson(new JSONObject().put("drivetrainType", tank ? "TANK" : "MECANUM"));
        c.translationalFeedforwardKS = .06;
        c.translationalKV = .94/75;
        c.translationalKA = .94/150;
        c.deriveVelocityFeedbackGains();
        c.forwardVelLimitIn = c.strafeVelLimitIn = 65;
        c.forwardAccelLimitIn = c.strafeAccelLimitIn = POSITIVE_ACCEL;
        c.angularVelLimitRad = 5; c.angularAccelLimitRad = 10;
        return c;
    }

    private static DisplacementProfile profile(double braking) {
        List<Double> disps = new ArrayList<>(), speeds = new ArrayList<>();
        List<Double> negative = new ArrayList<>(), positive = new ArrayList<>();
        for (int i = 0; i <= LENGTH*2; i++) disps.add(i*.5);
        for (int i = 0; i < disps.size()-1; i++) {
            double center = (disps.get(i)+disps.get(i+1))*.5;
            int phase = 0;
            while (phase+1 < CHANGES.length && center >= CHANGES[phase+1]) phase++;
            speeds.add(SPEEDS[phase]); negative.add(-braking); positive.add(POSITIVE_ACCEL);
        }
        return Profiles.profile(disps, 0, speeds, negative, positive).baseProfile;
    }

    private static Path path(boolean tank, DisplacementProfile profile) {
        GeometryFactory g = new GeometryFactory();
        Path path = tank ? g.tankPath(g.pose(0,0,0), g.pose(LENGTH,0,0)).buildToMemory()
                : g.holonomicPath(g.pose(0,0,0), g.pose(LENGTH,0,0)).buildToMemory();
        MotionParameters[] samples = new MotionParameters[profile.disps.size()];
        for (int i = 0; i < samples.length; i++) samples[i] = new MotionParameters(profile.vels.get(i),
                i < profile.accels.size() ? profile.accels.get(i) : 0, 0, 0, profile.disps.get(i));
        // Shared RR-generated profile isolates controller response and permits independent braking limits.
        path.setFeedforwardLut(new FFLut(samples));
        return path;
    }

    private static void verifyProfile(DisplacementProfile profile, double braking) {
        int count = 0; boolean previous = false;
        for (int i = 0; i < profile.accels.size() && profile.disps.get(i) < STOP; i++) {
            double a = profile.accels.get(i);
            assertTrue(a <= POSITIVE_ACCEL + 1e-8 && a >= -braking - 1e-8);
            if (a < -5 && !previous) count++;
            previous = a < -5;
        }
        assertEquals("Three interior slowdowns, endpoint excluded", 3, count);
        for (int i = 0; i < 3; i++) assertEquals(SPEEDS[2*i+1], profile.get(BOUNDARIES[i]).get(1), 1e-6);
    }

    private static Run execute(boolean tank, FollowerConstants c, Path path, DisplacementProfile profile,
                               Policy policy, double braking, double dt, int sensor) throws Exception {
        Bench bench = new Bench(tank);
        Sensor localizer = new Sensor(bench, sensor);
        Follower follower = Follower.create(robot(tank,localizer), bench.h.hardwareMap);
        follower.follow(path);
        TimeProfile clock = profile == null ? null : new TimeProfile(profile);
        // Convert the same power-error gain into RR's velocity correction units; kV*KvRR = Kpower.
        HolonomicController rr = new HolonomicController(policy == Policy.RR_TIME ? 2 : 0, 0, 0,
                c.velocityFeedbackGain/c.translationalKV, 0, 0);
        double nominalVoltage = bench.h.drivetrain.config.nominalVoltage;
        MotorFeedforward ff = new MotorFeedforward(c.translationalFeedforwardKS*nominalVoltage,
                c.translationalKV*nominalVoltage, c.translationalKA*nominalVoltage);
        MecanumKinematics kinematics = new MecanumKinematics(14);
        Run run = new Run(tank, policy, braking, dt, sensor);
        double time = 0, previousPower = 0; int frame = 0, nextBoundary = 0;
        boolean wasBraking = false;
        try {
            while (bench.x() < STOP && time < 45) {
                double step = sensor == 2 ? dt * new double[]{.7,1.3,1.1,.9}[frame%4] : dt;
                localizer.sample(step, frame);
                MotionParameters apexReference = path.getFeedforwardLut().getFFParams(bench.x());
                DualNum<Time> common = profile == null ? new DualNum<>(new double[]{bench.x(),
                        apexReference.getTangentialVel(),apexReference.getTangentialAccel()}) : profile.get(bench.x());
                DualNum<Time> reference = policy == Policy.RR_TIME ? clock.get(time) : common;
                double target = reference.get(1), a = reference.get(2), commandVelocity = target;
                double measured = localizer.getVel().getX().getIn();
                double feedforward, feedback;
                if (policy == Policy.APEX_PROFILED) {
                    follower.update(); target = follower.getTrackingVelocityTarget();
                    a = path.getFeedforwardLut().getFFParams(bench.x()).getTangentialAccel();
                    feedforward = follower.getTrackingFeedforward(); feedback = follower.getTrackingVelocityFeedback();
                    commandVelocity = target;
                } else if (policy == Policy.PROPOSED_SCALED) {
                    double applied = a;
                    if (a < 0 && target > 1e-6) {
                        double ratio = Math.max(0, Math.min(1, measured/target)); applied *= ratio*ratio;
                    }
                    feedforward = c.translationalFeedforwardKS + c.translationalKV*target + c.translationalKA*applied;
                    feedback = c.velocityFeedbackGain*(target-measured);
                    bench.power(feedforward+feedback);
                } else {
                    Pose2dDual<Time> targetPose = new Pose2dDual<>(new Vector2dDual<>(reference,
                            new DualNum<>(new double[]{0,0,0})), Rotation2dDual.exp(new DualNum<>(new double[]{0,0,0})));
                    PoseVelocity2dDual<Time> command = rr.compute(targetPose, new Pose2d(bench.x(),0,0),
                            new PoseVelocity2d(new Vector2d(measured,0),0));
                    commandVelocity = command.linearVel.x.get(0);
                    MecanumKinematics.WheelVelocities<Time> wheels = kinematics.inverse(command);
                    double voltage = bench.motors[0].config.voltageSensor.getVoltage();
                    double power = ff.compute(wheels.leftFront)/voltage;
                    // Straight-line symmetry is verified instead of bypassing wheel kinematics.
                    assertEquals(power, ff.compute(wheels.leftBack)/voltage, 1e-9);
                    assertEquals(power, ff.compute(wheels.rightFront)/voltage, 1e-9);
                    assertEquals(power, ff.compute(wheels.rightBack)/voltage, 1e-9);
                    feedforward = ff.compute(target,a)/voltage;
                    feedback = power-feedforward;
                    bench.power(power);
                }
                double truth = bench.speed(), power = bench.power();
                double error = truth-common.get(1), nativeError = truth-target;
                boolean accelerating = common.get(2) > 5 && bench.x() > 3;
                boolean brakingNow = common.get(2) < -5;
                if (accelerating) {
                    run.accelSquared += error*error*step; run.accelTime += step;
                    run.accelPeak = Math.max(run.accelPeak, Math.abs(error));
                }
                if (brakingNow) {
                    if (!wasBraking) run.intervals++;
                    run.brakeSquared += error*error*step; run.brakeTime += step;
                    run.brakeSigned += error*step;
                    run.brakePeakOver = Math.max(run.brakePeakOver,error);
                    run.brakePeakUnder = Math.max(run.brakePeakUnder,-error);
                    run.nativeSquared += nativeError*nativeError*step;
                    if (wasBraking) {
                        run.variation += Math.abs(power-previousPower);
                        run.maxPowerStep = Math.max(run.maxPowerStep, Math.abs(power-previousPower));
                    }
                }
                run.rows.add(new double[]{time,bench.x(),common.get(1),target,truth,measured,power,
                        common.get(2),a,commandVelocity,feedforward,feedback,error,nativeError,reference.get(0)-bench.x(),step});
                previousPower = power; wasBraking = brakingNow;
                double beforeX = bench.x(), beforeSpeed = truth;
                bench.step(step);
                while (nextBoundary < 3 && bench.x() >= BOUNDARIES[nextBoundary]) {
                    double fraction = (BOUNDARIES[nextBoundary]-beforeX)/(bench.x()-beforeX);
                    double speed = beforeSpeed+fraction*(bench.speed()-beforeSpeed);
                    run.boundaryErrors[nextBoundary] = speed-SPEEDS[2*nextBoundary+1]; nextBoundary++;
                }
                time += step; frame++;
            }
            run.completed = bench.x() >= STOP && nextBoundary == 3;
            assertTrue("Acceleration samples required", run.accelTime > .2);
            assertEquals("Three braking intervals per run", 3, run.intervals);
            return run;
        } finally { follower.stop(); }
    }

    private static ApexConstants robot(boolean tank, Sensor localizer) {
        return new ApexConstants() {
            @Override public BaseDrivetrainConstants<?> drivetrainConstants() {
                if (tank) return new TankSimulationConstants().drivetrainConstants();
                return new Mecanum.Constants().setFrontLeftMotor(new Motor(ApexSimulation.FRONT_LEFT_MOTOR))
                        .setFrontRightMotor(new Motor(ApexSimulation.FRONT_RIGHT_MOTOR))
                        .setBackLeftMotor(new Motor(ApexSimulation.BACK_LEFT_MOTOR))
                        .setBackRightMotor(new Motor(ApexSimulation.BACK_RIGHT_MOTOR)).setMaxPower(1);
            }
            @Override public BaseLocalizerConstants<?> localizerConstants() { return localizer.configuration; }
        };
    }

    private static double accelerationRamp(boolean tank, FollowerConstants c, File output) throws Exception {
        Bench bench = new Bench(tank);
        StockSimTestSupport.setSpeed(bench.h, 6);
        List<String> csv = new ArrayList<>(); csv.add("time_s,target_in_s,true_speed_in_s,power");
        double sum = 0;
        for (int i = 0; i < 50; i++) {
            double target = 6+i*.6, truth = bench.speed();
            sum += (truth-target)*(truth-target);
            double power = c.translationalFeedforwardKS+c.translationalKV*target+c.translationalKA*30;
            csv.add(String.format(Locale.US,"%.3f,%.6f,%.6f,%.6f",i*.02,target,truth,power));
            bench.power(power); bench.step(.02);
        }
        Files.write(new File(output,(tank?"TANK":"MECANUM")+"-acceleration-ramp.csv").toPath(),csv,StandardCharsets.UTF_8);
        return Math.sqrt(sum/50);
    }

    private static final class Bench {
        final ApexSimulation.Hardware h; final SimMotor[] motors;
        Bench(boolean tank) {
            h = tank ? ApexSimulation.createTankHardware() : ApexSimulation.createHardware();
            motors = StockSimTestSupport.motors(h);
            for (SimMotor m : motors) m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        double x() { return h.drivetrain.position.x-ApexSimulation.FIELD_CENTER_INCHES; }
        double speed() { return h.drivetrain.velocity.x; }
        void power(double u) { for (SimMotor m : motors) m.setPower(u); }
        double power() { double u=0; for (SimMotor m:motors) u+=m.getPower(); return u/4; }
        void step(double dt) {
            int steps=(int)Math.ceil(dt/.001); double step=dt/steps;
            for (int i=0;i<steps;i++) {
                // Stock interactive chassis/motor stepping, including upstream collision handling.
                ApexSimulation.stepPhysics(h,step);
            }
        }
    }

    private static final class SensorConfig implements BaseLocalizerConstants<SensorConfig> {
        final Sensor sensor; SensorConfig(Sensor sensor) { this.sensor=sensor; }
        @Override public BaseLocalizer<?> build(HardwareMap map) { return sensor; }
    }
    private static final class Sensor extends BaseLocalizer<SensorConfig> {
        final Bench bench; final int mode; final SensorConfig configuration; double filtered;
        Sensor(Bench bench,int mode) { super(null); this.bench=bench; this.mode=mode; configuration=new SensorConfig(this); }
        void sample(double dt,int frame) {
            double measured=bench.speed()+(mode==2?.2*Math.sin(frame*1.71):0);
            filtered+=(mode==0?1:1-Math.exp(-dt/.04))*(measured-filtered);
            pose=new Pose(Vector.of(bench.x(),0,geometry.DistUnit.IN),Angle.zero());
            velocity=new Pose(Vector.of(filtered,0,geometry.DistUnit.IN),Angle.zero());
            rawVelocity=new Pose(Vector.of(measured,0,geometry.DistUnit.IN),Angle.zero());
        }
        @Override public void update() { }
        @Override public void setPose(Pose pose) {
            bench.h.drivetrain.position.x=pose.getX().getIn()+ApexSimulation.FIELD_CENTER_INCHES; this.pose=pose;
        }
    }

    private static final class Run {
        final boolean tank; final Policy policy; final double braking,dt; final int sensor;
        final List<double[]> rows=new ArrayList<>(); final double[] boundaryErrors=new double[3];
        double accelSquared,accelTime,accelPeak,brakeSquared,brakeTime,brakeSigned,brakePeakOver,brakePeakUnder;
        double nativeSquared,variation,maxPowerStep; int intervals; boolean completed;
        Run(boolean tank,Policy policy,double braking,double dt,int sensor) {
            this.tank=tank;this.policy=policy;this.braking=braking;this.dt=dt;this.sensor=sensor;
        }
        String plant() { return tank?"TANK":"MECANUM"; }
        String name() { return plant()+"-b"+(int)braking+"-"+policy+"-"+(int)(dt*1000)+"ms-s"+sensor; }
        double accelRms() { return Math.sqrt(accelSquared/accelTime); }
        double brakeRms() { return Math.sqrt(brakeSquared/brakeTime); }
        double boundaryPeak() { double max=0;for(double e:boundaryErrors)max=Math.max(max,Math.abs(e));return max; }
        boolean accelPass() { return completed&&accelRms()<=4.2&&accelPeak<=8.4; }
        boolean brakePass() { return accelPass()&&brakeRms()<=3&&brakePeakOver<=6&&brakePeakUnder<=6&&boundaryPeak()<=3; }
    }

    private static String sensorName(int sensor) { return new String[]{"exact","40ms lag","lag+noise+jitter"}[sensor]; }
    private static void restore(String key,String old) { if(old==null)System.clearProperty(key);else System.setProperty(key,old); }
    private static void writeTrace(File output,Run run) throws Exception {
        List<String> csv=new ArrayList<>();
        csv.add("time_s,distance_in,shared_target_in_s,controller_target_in_s,true_speed_in_s,measured_speed_in_s,power,"
                +"shared_accel_in_s2,controller_accel_in_s2,corrected_velocity_in_s,feedforward,feedback,"
                +"shared_error_in_s,native_error_in_s,reference_position_error_in,dt_s");
        for(double[] row:run.rows) {
            StringBuilder line=new StringBuilder();for(double x:row){if(line.length()>0)line.append(',');line.append(String.format(Locale.US,"%.8f",x));}
            csv.add(line.toString());
        }
        Files.write(new File(output,run.name()+".csv").toPath(),csv,StandardCharsets.UTF_8);
    }

    private static void writeSummary(File output,List<Run> runs,List<String> prerequisites) throws Exception {
        boolean apexGenerated = output.getName().startsWith("apex-generated");
        List<String> report=new ArrayList<>(), csv=new ArrayList<>();
        report.add("# Braking slope sweep — actual Road Runner 1.0.1\n");
        report.add(apexGenerated ? "Updated project motor/chassis plant; actual Apex-generated constrained profiles. "
                +"Acceleration and braking limits 30 in/s²; three interior slowdowns; endpoint capture excluded.\n"
                : "Updated project motor/chassis plant; common RR-generated profile. Positive acceleration stays at 60 in/s². "
                +"Braking limits: 120, 90, 60, 30, 15 in/s². Interior boundaries: 90, 210, 330 in; stop at 420 in of a 480 in course.\n");
        if (!apexGenerated) report.add("RR_TIME calls the holonomic controller with time sampling and axial position gain 2/s. "
                +"RR_DISPLACEMENT calls the same controller with displacement sampling and zero axial position error. "
                +"Both use the same velocity correction strength as Apex, in RR velocity units. "
                +"Neither uses the earlier prototype's acceleration scaling. Tank RR_DISPLACEMENT is an adaptation, not native Ramsete.\n");
        report.addAll(prerequisites);
        report.add("\nAll speed scores use the shared profile evaluated at actual distance. Native RMS additionally scores the policy's own reference. "
                +"Acceleration gate: RMS ≤4.2 in/s, peak ≤8.4 in/s, complete course; first 3 in excluded. "
                +"Braking target: RMS ≤3, peak absolute error ≤6, all boundary errors ≤3 in/s. "
                +"Interpret braking only for policies with passing acceleration.\n");
        report.add("| Plant | Brake limit | Policy | Loop ms | Sensor | Accel RMS | Accel peak | Brake RMS | Native RMS | Bias | Peak over | Peak under | Boundary error | Power variation/s | Power step | Accel | Brake target |\n"
                +"|---|---:|---|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---|");
        csv.add("plant,braking_limit,policy,loop_ms,sensor,accel_rms,accel_peak,brake_rms,native_brake_rms,brake_bias,"
                +"peak_over,peak_under,boundary_error,power_variation_per_s,max_power_step,accel_pass,brake_pass,graph");
        for(Run r:runs) {
            String values=String.format(Locale.US,"%s,%.0f,%s,%.0f,%s,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%s,%s",
                    r.plant(),r.braking,r.policy,r.dt*1000,sensorName(r.sensor),r.accelRms(),r.accelPeak,r.brakeRms(),
                    Math.sqrt(r.nativeSquared/r.brakeTime),r.brakeSigned/r.brakeTime,r.brakePeakOver,r.brakePeakUnder,
                    r.boundaryPeak(),r.variation/r.brakeTime,r.maxPowerStep,r.accelPass()?"PASS":"FAIL",r.brakePass()?"MET":"MISSED");
            csv.add(values+","+r.name()+"-speed.png"); report.add("| "+values.replace(","," | ")+" |");
        }
        Files.write(new File(output,"results.md").toPath(),report,StandardCharsets.UTF_8);
        Files.write(new File(output,"summary.csv").toPath(),csv,StandardCharsets.UTF_8);
    }

    private static XYChart chart(String title,String x,String y,int width,int height) {
        XYChart chart=new XYChartBuilder().width(width).height(height).title(title).xAxisTitle(x).yAxisTitle(y).build();
        chart.getStyler().setLegendPosition(Styler.LegendPosition.InsideNW);
        chart.getStyler().setDefaultSeriesRenderStyle(org.knowm.xchart.XYSeries.XYSeriesRenderStyle.Line);
        return chart;
    }
    private static void series(XYChart chart,String name,Run run,int xIndex,int yIndex) {
        double[] x=new double[run.rows.size()],y=new double[x.length];
        for(int i=0;i<x.length;i++){x[i]=run.rows.get(i)[xIndex];y[i]=run.rows.get(i)[yIndex];}
        chart.addSeries(name,x,y).setMarker(SeriesMarkers.NONE);
    }
    private static void graphRun(File output,Run run) throws Exception {
        String label=run.plant()+" / "+run.policy+" / braking "+(int)run.braking+" in/s² / "+(int)(run.dt*1000)+" ms / "+sensorName(run.sensor);
        XYChart velocity=chart(label,"Distance (in)","Speed (in/s)",1200,400);
        series(velocity,"Shared displacement target",run,1,2);series(velocity,"True speed",run,1,4);
        series(velocity,"Controller target",run,1,3);
        velocity.getStyler().setYAxisMin(0.0);velocity.getStyler().setYAxisMax(50.0);
        XYChart power=chart("Motor command","Distance (in)","Power",600,340);
        series(power,"Applied power",run,1,6);power.getStyler().setYAxisMin(-1.0);power.getStyler().setYAxisMax(1.0);
        XYChart errors=chart("Speed tracking errors","Time (s)","Speed error (in/s)",600,340);
        series(errors,"Shared displacement error",run,0,12);series(errors,"Own reference error",run,0,13);
        XYChart acceleration=chart("Requested acceleration — positive acceleration held fixed","Distance (in)","Acceleration (in/s²)",1200,280);
        series(acceleration,"Shared profile",run,1,7);series(acceleration,"Controller reference",run,1,8);
        BitmapEncoder.saveBitmap(velocity,new File(output,run.name()+"-speed").toString(),BitmapEncoder.BitmapFormat.PNG);
        BitmapEncoder.saveBitmap(power,new File(output,run.name()+"-power").toString(),BitmapEncoder.BitmapFormat.PNG);
        BitmapEncoder.saveBitmap(errors,new File(output,run.name()+"-error").toString(),BitmapEncoder.BitmapFormat.PNG);
        BitmapEncoder.saveBitmap(acceleration,new File(output,run.name()+"-accel").toString(),BitmapEncoder.BitmapFormat.PNG);
    }

    private static void graphComparisons(File output,List<Run> runs) throws Exception {
        for(boolean tank:new boolean[]{false,true}) for(int sensor:new int[]{0,1,2}) {
            for(double braking:BRAKES) {
                XYChart plot=chart((tank?"TANK":"MECANUM")+" — braking "+(int)braking+", 20 ms, "+sensorName(sensor),
                        "Distance (in)","Speed (in/s)",1200,500);
                for(Run r:runs)if(r.tank==tank&&r.sensor==sensor&&r.braking==braking&&r.dt==.02) {
                    series(plot,r.policy.toString(),r,1,4);
                    if(r.policy==Policy.APEX_PROFILED)series(plot,"Shared profile",r,1,2);
                }
                BitmapEncoder.saveBitmap(plot,new File(output,(tank?"TANK":"MECANUM")+"-b"+(int)braking+"-overlay-s"+sensor).toString(),BitmapEncoder.BitmapFormat.PNG);
            }
            for(boolean power:new boolean[]{false,true}) {
                XYChart plot=chart((tank?"TANK":"MECANUM")+" — 20 ms, "+sensorName(sensor),"Braking limit magnitude (in/s²)",
                        power?"Power variation / second":"Braking RMS speed error (in/s)",1100,500);
                for(Policy policy:Policy.values()) {
                    List<Double>x=new ArrayList<>(),y=new ArrayList<>();
                    for(int i=BRAKES.length-1;i>=0;i--)for(Run r:runs)
                        if(r.tank==tank&&r.sensor==sensor&&r.policy==policy&&r.dt==.02&&r.braking==BRAKES[i]) {
                            x.add(r.braking);y.add(power?r.variation/r.brakeTime:r.brakeRms());
                        }
                    if(!x.isEmpty())plot.addSeries(policy.toString(),x,y);
                }
                BitmapEncoder.saveBitmap(plot,new File(output,(tank?"TANK":"MECANUM")+"-"+(power?"power":"rms")+"-slope-s"+sensor).toString(),BitmapEncoder.BitmapFormat.PNG);
            }
        }
    }

    private static void gallery(File output,List<Run> runs) throws Exception {
        boolean apexGenerated = output.getName().startsWith("apex-generated");
        StringBuilder page=new StringBuilder("<!doctype html><html><head><meta charset='utf-8'><title>Braking slope experiments</title>"
                +"<style>body{font:16px system-ui;margin:24px;background:#f5f7fa;color:#17212d}select{padding:8px;margin:5px}"
                +"article{background:white;padding:16px;margin:18px 0;border-radius:10px}img{max-width:100%;height:auto}"
                +"header{position:sticky;top:0;background:#f5f7fa;padding:8px;z-index:1}a{color:#155bbb}</style></head><body>"
                +"<h1>Braking validation</h1><p>"+runs.size()+" complete runs. Every graph includes speed, power, tracking error and acceleration. "
                +(apexGenerated ? "Actual Apex-generated profiles; acceleration and braking limits 30 in/s².</p>"
                    : "Acceleration stays at +60 in/s². All controllers share one Road Runner-generated profile per braking limit.</p>")
                +"<p><a href='results.md'>Full results</a> · <a href='summary.csv'>Summary CSV</a></p><header>");
        String[][] filters={{"plant","MECANUM","TANK"},{"brake","120","90","60","30","15"},
                {"policy","APEX_PROFILED","PROPOSED_SCALED","RR_DISPLACEMENT","RR_TIME"},
                {"loop","10","20","40"},{"sensor","0","1","2"}};
        for(String[] filter:filters) {
            page.append("<label>").append(filter[0]).append(" <select id='").append(filter[0]).append("'><option value=''>All</option>");
            for(int i=1;i<filter.length;i++)page.append("<option>").append(filter[i]).append("</option>");
            page.append("</select></label>");
        }
        page.append("<span id='count'></span></header><p>Sensor 0: exact; 1: 40 ms first-order lag; 2: lag, ±0.2 in/s noise and loop jitter. "
                +"RR_TIME is native holonomic controller timing; RR_DISPLACEMENT is the adaptation for Apex. Tank native Ramsete is outside this sweep.</p>");
        if (!apexGenerated) page.append("<h2>Braking limit overview — 20 ms loop with 40 ms velocity lag</h2>"
                +"<a href='MECANUM-rms-slope-s1.png'><img src='MECANUM-rms-slope-s1.png' alt='Mecanum braking RMS versus limit'></a>"
                +"<a href='TANK-rms-slope-s1.png'><img src='TANK-rms-slope-s1.png' alt='Tank braking RMS versus limit'></a>");
        for(Run r:runs) {
            page.append("<article data-plant='").append(r.plant()).append("' data-brake='").append((int)r.braking)
                    .append("' data-policy='").append(r.policy).append("' data-loop='").append((int)(r.dt*1000)).append("' data-sensor='").append(r.sensor)
                    .append("'><h2>").append(r.name()).append("</h2><p>").append(String.format(Locale.US,"Acceleration %s (RMS %.3f), braking RMS %.3f in/s, boundary error %.3f in/s. ",
                            r.accelPass()?"PASS":"FAIL",r.accelRms(),r.brakeRms(),r.boundaryPeak()))
                    .append("<a href='").append(r.name()).append(".csv'>CSV</a></p>");
            for(String kind:new String[]{"speed","power","error","accel"}) page.append("<a href='").append(r.name()).append('-').append(kind)
                    .append(".png'><img loading='lazy' src='").append(r.name()).append('-').append(kind)
                    .append(".png' alt='").append(kind).append(" for ").append(r.name()).append("'></a>");
            page.append("</article>");
        }
        page.append("<script>const ids=['plant','brake','policy','loop','sensor'];function filter(){let count=0;document.querySelectorAll('article').forEach(a=>{"
                +"a.hidden=ids.some(id=>document.getElementById(id).value&&a.dataset[id]!==document.getElementById(id).value);if(!a.hidden)count++;});"
                +"document.getElementById('count').textContent=count+' of 315 runs';}ids.forEach(id=>document.getElementById(id).addEventListener('change',filter));"
                +"document.getElementById('plant').value='MECANUM';document.getElementById('loop').value='20';document.getElementById('sensor').value='1';filter();</script></body></html>");
        Files.write(new File(output,"index.html").toPath(),page.toString().getBytes(StandardCharsets.UTF_8));
    }
}
