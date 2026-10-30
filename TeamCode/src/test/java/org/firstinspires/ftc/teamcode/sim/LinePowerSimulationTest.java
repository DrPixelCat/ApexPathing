package org.firstinspires.ftc.teamcode.sim;

import util.DeltaTime;

import follower.Follower;
import follower.FollowerConstants;
import paths.geometry.Angle;
import paths.geometry.AngleUnit;
import paths.geometry.DistUnit;
import paths.geometry.GeometryFactory;
import org.firstinspires.ftc.teamcode.apexpathing.Constants;
import org.junit.Test;
import org.knowm.xchart.BitmapEncoder;
import org.knowm.xchart.XYChart;
import org.knowm.xchart.XYChartBuilder;
import org.knowm.xchart.style.Styler;
import paths.heading.InterpolationStyle;
import paths.movements.Path;
import storage.ApexStorage;
import trajectory.TrajectorySample;

import java.io.File;
import java.io.PrintWriter;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import static org.junit.Assert.*;

/** Real follower and stock simulator on an 80-inch line, with all motion feedback disabled. */
public class LinePowerSimulationTest {
    private static final double LENGTH_IN = 80;
    @Test(timeout = 30_000L) public void onlineLineWithoutFeedback() throws Exception { run(false); }
    @Test(timeout = 30_000L) public void storedLineWithoutFeedback() throws Exception { run(true); }

    private static void run(boolean stored) throws Exception {
        run(stored, false);
    }

    static void run(boolean stored, boolean twisting) throws Exception {
        run(stored,twisting?90:0,1);
    }
    static void run(boolean stored,double twistDegrees,double travelSign) throws Exception {
        run(stored,twistDegrees,travelSign,true);
    }
    static void run(boolean stored,double twistDegrees,double travelSign,boolean deterministic) throws Exception {
        boolean twisting=twistDegrees!=0;
        final long[] simulatedNanos={1_000_000_000L};
        java.util.function.LongSupplier clock=deterministic?()->simulatedNanos[0]:System::nanoTime;
        File directory = new File("build/path-power-graphs").getAbsoluteFile();
        String configured = System.getenv("APEX_AUTOTEST_GRAPH_DIR");
        if (configured != null) { directory = new File(configured); }
        Files.createDirectories(directory.toPath());
        String previousStorage = System.getProperty(ApexStorage.DIRECTORY_PROPERTY);
        System.setProperty(ApexStorage.DIRECTORY_PROPERTY, new File(directory, "line-data").getAbsolutePath());
        String name = (twisting ? "Straight_Twist_" : "Line_")
                + (stored ? "Stored" : "Online") + (travelSign<0?"_Reverse":"")
                + (twistDegrees<0?"_NegativeTwist":"") + (deterministic?"":"_WallTimed") + "_Feedforward_Only";
        String title = "80-inch " + (twisting ? "Line + " + twistDegrees + " deg Twist" : "Line")
                + ", Feedback Off (" + (stored ? "stored" : "online") + ")";
        Follower follower = null;
        try {
            ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
            FollowerConstants constants = FollowerConstants.getInstance();
            SimulatorTuning.apply(constants);
            Constants robotConstants = new Constants();
            if(deterministic) robotConstants=new Constants() {
                @Override public localizers.BaseLocalizerConstants<?> localizerConstants() {
                    return new localizers.Pinpoint.Constants() {
                        { setName("pinpoint"); }
                        @Override public localizers.Pinpoint build(com.qualcomm.robotcore.hardware.HardwareMap hardwareMap) {
                            final localizers.Pinpoint.Driver sensor=hardwareMap.get(localizers.Pinpoint.Driver.class,"pinpoint");
                            return new localizers.Pinpoint(this,hardwareMap) {
                                @Override public void update() {
                                    sensor.update();pose=sensor.getPosition();
                                    calculate(sensor.getVelocity(),clock.getAsLong());
                                }
                            };
                        }
                    };
                }
            };
            follower = Follower.create(robotConstants, hardware.hardwareMap,false,clock);
            // Apply after construction: configure() can reload the active profile.
            SimulatorTuning.apply(constants);
            follower.reset();
            follower.disableControllers();
            // Keep nonzero configured velocity gains: zero correction assertions below
            // verify the disable switch itself, including endpoint capture.
            GeometryFactory geometry = new GeometryFactory().setDistUnit(DistUnit.IN).setAngleUnit(AngleUnit.DEG);
            // Leave room beyond the endpoint for feedforward-only overshoot and braking.
            Path path = geometry.holonomicPath(geometry.pose(-travelSign*LENGTH_IN / 2, 0, 0),
                            geometry.pose(travelSign*LENGTH_IN / 2, 0, twistDegrees))
                    .interpolateWith(twisting ? InterpolationStyle.SMOOTH_START_TO_END
                            : InterpolationStyle.CONSTANT_START_HEADING).quickBuild();
            if (stored) { path.setBuildMode(Path.BuildMode.MEMORY); path.precomputeTrajectory(); }
            follower.setPose(geometry.pose(-travelSign*LENGTH_IN / 2, 0, 0));
            follower.follow(path);
            File csv = new File(directory, name + ".csv");
            List<Double> distances = new ArrayList<>(), targets = new ArrayList<>(), measured = new ArrayList<>();
            List<Double> plantVelocities = new ArrayList<>();
            List<Double> headingTargets = new ArrayList<>(), measuredHeadings = new ArrayList<>();
            List<Double> plantHeadings = new ArrayList<>(), angularTargets = new ArrayList<>();
            List<Double> measuredAngular = new ArrayList<>(), plantAngular = new ArrayList<>();
            List<Double> positionsX = new ArrayList<>(), positionsY = new ArrayList<>();
            List<Double> actualDistances = new ArrayList<>();
            List<Double> requestedTurns = new ArrayList<>(), deliveredTurns = new ArrayList<>();
            double maxFeedback = 0, maxBlend = 0, speedSquaredError = 0;
            double angularSquaredError = 0, headingSquaredError = 0, maxCrossTrack = 0;
            double maxTargetAngular = 0, maxPlantAngular = 0;
            double maxControlInterval=0;
            int activeBrakingSamples = 0, limitedSamples = 0;
            boolean braking = false;
            long start = clock.getAsLong();
            double maxProgress = 0;
            double terminalTargetVelocity = Double.NaN;
            long previousPhysicsNanos = clock.getAsLong();
            try (PrintWriter writer = new PrintWriter(Files.newBufferedWriter(csv.toPath()))) {
                writer.println("elapsed_s,path_distance_in,expected_motor_power,command_power,target_velocity_in_s," +
                        "measured_velocity_in_s,plant_velocity_in_s,target_accel_in_s2,feedback_power,endpoint_blend," +
                        "signed_translation_ff,left_front_command,right_front_command," +
                        "target_heading_deg,measured_heading_deg,plant_heading_deg,target_omega_rad_s," +
                        "measured_omega_rad_s,plant_omega_rad_s,target_alpha_rad_s2," +
                        "actual_x_in,actual_y_in,cross_track_error_in,heading_feedback_power," +
                        "heading_velocity_feedback_power,feedforward_scale,centered_braking_active," +
                        "left_back_command,right_back_command,requested_turn_ff,allocated_turn_power," +
                        "allocated_body_x_power,allocated_body_y_power,raw_feedforward_demand");
                while (follower.isBusy() && clock.getAsLong() - start < 15_000_000_000L) {
                    long tick = clock.getAsLong();
                    // Localization differentiates pose using wall time. Advance the plant
                    // by that same elapsed interval, including host scheduling delays.
                    double physicsSeconds = DeltaTime.seconds(tick - previousPhysicsNanos);
                    maxControlInterval=Math.max(maxControlInterval,physicsSeconds);
                    previousPhysicsNanos = tick;
                    while (physicsSeconds > 1e-9) {
                        double step = Math.min(.005, physicsSeconds);
                        ApexSimulation.stepPhysics(hardware, step);
                        physicsSeconds -= step;
                    }
                    follower.update();
                    TrajectorySample target = follower.getTrajectoryTarget();
                    // The completing update retains its final target even though isBusy is false.
                    // Check schedule completion independently of the active-command power trace.
                    if (target != null) {
                        maxProgress = Math.max(maxProgress, target.progressIn);
                        terminalTargetVelocity = target.tangentialVelocity();
                    }
                    // Completion stops the motors; do not compare that stop to a stale target.
                    if (target != null && follower.isBusy()) {
                        Follower.CommandDemand demand = follower.getLastCommandDemand();
                        double feedback = demand.crossTrack + demand.tangentCorrection + demand.headingCorrection
                                + demand.forwardVelocity + demand.headingVelocity;
                        maxFeedback = Math.max(maxFeedback, Math.abs(feedback));
                        maxBlend = Math.max(maxBlend, follower.getTrackingEndpointBlend());
                        double actual = Math.max(Math.max(Math.abs(follower.getDrivetrain().getLastFlPower()),
                                Math.abs(follower.getDrivetrain().getLastFrPower())),
                                Math.max(Math.abs(follower.getDrivetrain().getLastBlPower()),
                                        Math.abs(follower.getDrivetrain().getLastBrPower())));
                        double velocity = follower.getVelocity().getVec().dot(target.tangent).getIn();
                        double plantVelocity = hardware.drivetrain.velocity.x*travelSign;
                        distances.add(target.progressIn); targets.add(target.tangentialVelocity()); measured.add(velocity);
                        plantVelocities.add(plantVelocity);
                        speedSquaredError += Math.pow(plantVelocity - target.tangentialVelocity(), 2);
                        braking |= target.tangentialAcceleration() < -1;
                        double measuredHeading = follower.getPose().getHeading().getRad();
                        double plantHeading = hardware.drivetrain.position.theta;
                        double measuredOmega = follower.getVelocity().getHeading().getRad();
                        double plantOmega = hardware.drivetrain.velocity.theta;
                        double crossTrack = follower.getCrossTrackErrorIn();
                        headingTargets.add(Math.toDegrees(target.heading.getRad()));
                        measuredHeadings.add(Math.toDegrees(measuredHeading)); plantHeadings.add(Math.toDegrees(plantHeading));
                        angularTargets.add(target.angularVelocity); measuredAngular.add(measuredOmega); plantAngular.add(plantOmega);
                        positionsX.add(hardware.drivetrain.position.x - ApexSimulation.FIELD_CENTER_INCHES);
                        positionsY.add(hardware.drivetrain.position.y - ApexSimulation.FIELD_CENTER_INCHES);
                        actualDistances.add(LENGTH_IN/2+travelSign*positionsX.get(positionsX.size()-1));
                        angularSquaredError += Math.pow(plantOmega - target.angularVelocity, 2);
                        headingSquaredError += Math.pow(Math.toDegrees(Angle.wrap(plantHeading - target.heading.getRad())), 2);
                        maxCrossTrack = Math.max(maxCrossTrack, Math.abs(crossTrack));
                        maxTargetAngular = Math.max(maxTargetAngular, Math.abs(target.angularVelocity));
                        maxPlantAngular = Math.max(maxPlantAngular, Math.abs(plantOmega));
                        if (follower.isCenteredBrakingActive()) { activeBrakingSamples++; }
                        if (demand.feedforwardScale < 1 - 1e-6) { limitedSamples++; }
                        double fl = follower.getDrivetrain().getLastFlPower(), fr = follower.getDrivetrain().getLastFrPower();
                        double bl = follower.getDrivetrain().getLastBlPower(), br = follower.getDrivetrain().getLastBrPower();
                        double requestedTurn = follower.getTrackingHeadingFeedforward();
                        double allocatedTurn = (-fl + fr - bl + br) / 4;
                        requestedTurns.add(requestedTurn); deliveredTurns.add(allocatedTurn);
                        if (twisting) {
                            // Independently re-sample the stored spatial reference to ensure
                            // measured progress speed does not alter the angular target.
                            if (stored && !follower.isWheelFeedforwardActive()) {
                                TrajectorySample planned = path.getTrajectoryField().sample(follower.getPose().getVec());
                                assertEquals("No measured-speed angular compensation", planned.angularVelocity, target.angularVelocity, 1e-9);
                                assertEquals("No measured-speed angular acceleration compensation", planned.angularAcceleration, target.angularAcceleration, 1e-9);
                            }
                            assertEquals("No heading position feedback", 0, demand.headingCorrection, 0);
                            assertEquals("No heading rate feedback", 0, demand.headingVelocity, 0);
                            assertEquals("Turn feedforward must reach wheel mixing after allocation",
                                    requestedTurn * demand.feedforwardScale, allocatedTurn, .001);
                        }
                        writer.printf(Locale.US, "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f," +
                                        "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%s," +
                                        "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f%n",
                                DeltaTime.seconds(clock.getAsLong() - start), target.progressIn, target.expectedMotorPower, actual,
                                target.tangentialVelocity(), velocity, plantVelocity, target.tangentialAcceleration(),
                                feedback, follower.getTrackingEndpointBlend(), follower.getTrackingFeedforward(),
                                follower.getDrivetrain().getLastFlPower(), follower.getDrivetrain().getLastFrPower(),
                                Math.toDegrees(target.heading.getRad()), Math.toDegrees(measuredHeading), Math.toDegrees(plantHeading),
                                target.angularVelocity, measuredOmega, plantOmega, target.angularAcceleration,
                                positionsX.get(positionsX.size() - 1), positionsY.get(positionsY.size() - 1), crossTrack,
                                demand.headingCorrection, demand.headingVelocity, demand.feedforwardScale, follower.isCenteredBrakingActive(),
                                bl, br, requestedTurn, allocatedTurn, (fl + fr + bl + br) / 4,
                                (-fl + fr + bl - br) / 4, demand.feedforwardTotal);
                    }
                    if(deterministic) simulatedNanos[0]+=20_000_000L;
                    else {
                        long remaining = 20_000_000L - (System.nanoTime() - tick);
                        if (remaining > 0) { Thread.sleep(remaining / 1_000_000L, (int) (remaining % 1_000_000L)); }
                    }
                }
            }
            double finalPosition = follower.getPose().getVec().getX().getIn();
            boolean complete = !follower.isBusy();
            follower.stop();
            AutoTestPowerGraphs.write(csv, directory, name, title);
            XYChart velocity = new XYChartBuilder().width(1000).height(460).title(title + ": Velocity")
                    .xAxisTitle("Scheduled distance along path (in)").yAxisTitle("Velocity (in/s)").build();
            velocity.getStyler().setLegendPosition(Styler.LegendPosition.OutsideE);
            velocity.getStyler().setMarkerSize(0);
            velocity.addSeries("Target", distances, targets);
            velocity.addSeries("Measured (localizer)", distances, measured);
            velocity.addSeries("Actual (simulator plant)", distances, plantVelocities);
            BitmapEncoder.saveBitmap(velocity, new File(directory, name + "_Velocity").getAbsolutePath(), BitmapEncoder.BitmapFormat.PNG);
            XYChart spatialVelocity = new XYChartBuilder().width(1000).height(460).title(title + ": Velocity Along Actual Path")
                    .xAxisTitle("Distance along path (in)").yAxisTitle("Velocity (in/s)").build();
            spatialVelocity.getStyler().setLegendPosition(Styler.LegendPosition.OutsideE);spatialVelocity.getStyler().setMarkerSize(0);
            spatialVelocity.addSeries("Target at planned distance",distances,targets);
            spatialVelocity.addSeries("Measured at actual distance",actualDistances,measured);
            spatialVelocity.addSeries("Plant at actual distance",actualDistances,plantVelocities);
            BitmapEncoder.saveBitmap(spatialVelocity,new File(directory,name+"_Velocity_Distance").getAbsolutePath(),BitmapEncoder.BitmapFormat.PNG);
            if (twisting) {
                writeMotionChart(directory, name + "_Heading", title + ": Heading", "Heading (deg)",
                        distances, headingTargets, measuredHeadings, plantHeadings);
                writeMotionChart(directory, name + "_Angular_Velocity", title + ": Angular Velocity", "Angular velocity (rad/s)",
                        distances, angularTargets, measuredAngular, plantAngular);
                XYChart turnPower = new XYChartBuilder().width(1000).height(460).title(title + ": Turn Power Delivery")
                        .xAxisTitle("Distance along path (in)").yAxisTitle("Signed turn power").build();
                turnPower.getStyler().setLegendPosition(Styler.LegendPosition.OutsideE); turnPower.getStyler().setMarkerSize(0);
                turnPower.addSeries("Requested feedforward", distances, requestedTurns);
                turnPower.addSeries("Delivered (four-wheel mix)", distances, deliveredTurns);
                BitmapEncoder.saveBitmap(turnPower, new File(directory, name + "_Turn_Power").getAbsolutePath(), BitmapEncoder.BitmapFormat.PNG);
                XYChart track = new XYChartBuilder().width(1000).height(460).title(title + ": Actual Path")
                        .xAxisTitle("Field X (in)").yAxisTitle("Field Y (in)").build();
                track.getStyler().setLegendPosition(Styler.LegendPosition.OutsideE); track.getStyler().setMarkerSize(0);
                track.addSeries("Target straight line", new double[]{-LENGTH_IN / 2, LENGTH_IN / 2}, new double[]{0, 0});
                track.addSeries("Actual (simulator plant)", positionsX, positionsY);
                BitmapEncoder.saveBitmap(track, new File(directory, name + "_Path").getAbsolutePath(), BitmapEncoder.BitmapFormat.PNG);
                System.out.printf(Locale.US,
                        "STRAIGHT TWIST FEEDFORWARD: angularRMSE=%.6f headingRMSEDeg=%.6f " +
                                "finalHeadingDeg=%.3f maxCrossTrackIn=%.6f maxTargetOmega=%.6f maxPlantOmega=%.6f " +
                                "limitedSamples=%d centeredBrakingSamples=%d%n",
                        Math.sqrt(angularSquaredError / distances.size()), Math.sqrt(headingSquaredError / distances.size()),
                        Math.toDegrees(follower.getPose().getHeading().getRad()), maxCrossTrack,
                        maxTargetAngular, maxPlantAngular, limitedSamples, activeBrakingSamples);
            }
            System.out.printf(Locale.US,
                    "LINE FEEDFORWARD: mode=%s samples=%d maxProgress=%.3f finalPosition=%.3f followerStopped=%s " +
                            "endpointError=%.3f plantSpeedRMSE=%.6f maxFeedback=%.9f maxEndpointBlend=%.9f terminalTargetVelocity=%.9f%n",
                    stored ? "stored" : "online", distances.size(), maxProgress, finalPosition, complete,
                    finalPosition - travelSign*LENGTH_IN / 2,
                    Math.sqrt(speedSquaredError / distances.size()), maxFeedback, maxBlend, terminalTargetVelocity);
            assertEquals("Every feedback contribution must remain disabled", 0, maxFeedback, 1e-12);
            assertEquals("Endpoint capture must not replace feedforward", 0, maxBlend, 0);
            System.out.printf(Locale.US,"CONTROL TIMING: deterministic=%s maximumIntervalSeconds=%.6f%n",deterministic,maxControlInterval);
            if(!deterministic) org.junit.Assume.assumeTrue(
                    "Host missed the 20 ms command schedule (max interval="+maxControlInterval+"); motion accuracy is inconclusive",
                    maxControlInterval<=.05);
            assertTrue("Expected a substantial trace", distances.size() > 50);
            assertTrue("Expected to observe braking", braking);
            if (twisting) {
                assertTrue("The path must demand translation and rotation together", maxTargetAngular > .1);
                assertTrue("The plant must rotate during the straight test", maxPlantAngular > .01);
                assertTrue("Feedforward-only velocity accuracy",Math.sqrt(speedSquaredError/distances.size())<1.5);
                assertTrue("Feedforward-only angular velocity accuracy",Math.sqrt(angularSquaredError/distances.size())<.15);
                assertTrue("Feedforward-only lateral drift",maxCrossTrack<1);
                assertEquals("Feedforward-only terminal heading",twistDegrees,Math.toDegrees(follower.getPose().getHeading().getRad()),3);
                assertEquals("Feedforward-only terminal position",travelSign*40,finalPosition,2);
                assertTrue("Execute through the planned zero-velocity stop",maxProgress>LENGTH_IN-.01);
                assertTrue("Feedforward-only path must complete",complete);
                assertEquals("Terminal target must be at rest",0,terminalTargetVelocity,1e-9);
            } else {
                assertTrue("Expected to traverse the line; see trace", maxProgress > LENGTH_IN - 2);
            }
        } finally {
            if (follower != null) { follower.stop(); follower.enableControllers(); }
            if (previousStorage == null) { System.clearProperty(ApexStorage.DIRECTORY_PROPERTY); }
            else { System.setProperty(ApexStorage.DIRECTORY_PROPERTY, previousStorage); }
        }
    }

    private static void writeMotionChart(File directory, String name, String title, String yTitle,
                                         List<Double> distance, List<Double> target, List<Double> measured,
                                         List<Double> plant) throws Exception {
        XYChart chart = new XYChartBuilder().width(1000).height(460).title(title)
                .xAxisTitle("Distance along path (in)").yAxisTitle(yTitle).build();
        chart.getStyler().setLegendPosition(Styler.LegendPosition.OutsideE); chart.getStyler().setMarkerSize(0);
        chart.addSeries("Target", distance, target);
        chart.addSeries("Measured (localizer)", distance, measured);
        chart.addSeries("Actual (simulator plant)", distance, plant);
        BitmapEncoder.saveBitmap(chart, new File(directory, name).getAbsolutePath(), BitmapEncoder.BitmapFormat.PNG);
    }
}
