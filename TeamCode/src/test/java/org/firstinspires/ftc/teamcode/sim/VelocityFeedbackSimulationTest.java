package org.firstinspires.ftc.teamcode.sim;

import util.DeltaTime;

import follower.*;
import geometry.*;
import org.firstinspires.ftc.teamcode.apexpathing.Constants;
import org.junit.Test;
import org.knowm.xchart.*;
import org.knowm.xchart.style.Styler;
import paths.movements.Path;
import storage.ApexStorage;
import trajectory.TrajectorySample;
import java.io.*;
import java.nio.file.Files;
import java.util.*;
import java.util.function.LongSupplier;
import static org.junit.Assert.*;

/** Real follower, native sensor and unmodified plant on the single-path Apex twisting geometry. */
public class VelocityFeedbackSimulationTest {
    @Test(timeout=120_000L) public void onlineTwistTracksWithAndWithoutDisturbances() throws Exception {
        for(int scenario=0;scenario<3;scenario++) run(false,scenario);
    }
    @Test(timeout=120_000L) public void storedTwistRemainsAccurate() throws Exception {
        run(true, 0);
    }
    @Test(timeout=120_000L) public void experimentalStoredPursuitTwistRemainsAccurate() throws Exception {
        for (int scenario = 0; scenario < 4; scenario++) run(true, scenario, 0, trajectory.VelocityPursuit.DEFAULT_TIME_SECONDS);
    }
    @Test(timeout=120_000L) public void comparePlannerPreviewLengths() throws Exception {
        for(int scenario=0;scenario<3;scenario++) {
            for(double preview:new double[]{0,.02,.05}) {
                double[] metrics=run(false,scenario,preview);
                System.out.printf(Locale.US,"APEX PREVIEW COMPARISON: scenario=%d previewMs=%.0f vRMSE=%.6f headingRMSEdeg=%.6f endpoint=%.6f duration=%.6f%n",
                        scenario,preview*1000,metrics[0],metrics[1],metrics[2],metrics[3]);
            }
        }
    }
    @Test(timeout=120_000L) public void compareSpatialVelocityConvergence() throws Exception {
        for(int scenario=0;scenario<3;scenario++) {
            for(double pursuit:new double[]{0,.1,.2,.35,.5}) {
                double[] metrics=run(false,scenario,0,pursuit);
                System.out.printf(Locale.US,"PURSUIT COMPARISON: scenario=%d convergenceMs=%.0f nominalEnvelopeRMSE=%.6f headingRMSEdeg=%.6f endpoint=%.6f duration=%.6f accelerationVariation=%.6f powerVariation=%.6f%n",
                        scenario,pursuit*1000,metrics[4],metrics[1],metrics[2],metrics[3],metrics[5],metrics[6]);
            }
        }
    }
    @Test(timeout=120_000L) public void compareConvergenceWithDelayedNoisySensors() throws Exception {
        for(double pursuit:new double[]{0,.2,.35,.5}) {
            double[] metrics;
            try { metrics=run(false,3,0,pursuit); }
            catch(IllegalStateException failure) {
                if(pursuit>0) throw failure;
                System.out.println("DELAYED LEGACY BASELINE FAILED: "+failure.getMessage());continue;
            }
            System.out.printf(Locale.US,"DELAYED PURSUIT COMPARISON: convergenceMs=%.0f nominalEnvelopeRMSE=%.6f headingRMSEdeg=%.6f endpoint=%.6f duration=%.6f accelerationVariation=%.6f powerVariation=%.6f%n",
                    pursuit*1000,metrics[4],metrics[1],metrics[2],metrics[3],metrics[5],metrics[6]);
        }
    }

    private static double[] run(boolean stored,int scenario) throws Exception {
        return run(stored, scenario, 0, 0);
    }
    private static double[] run(boolean stored,int scenario,double previewSeconds) throws Exception {
        return run(stored,scenario,previewSeconds,0);
    }
    private static double[] run(boolean stored,int scenario,double previewSeconds,double pursuitSeconds) throws Exception {
        String configured=System.getenv("APEX_AUTOTEST_GRAPH_DIR");
        File directory=new File(configured==null?"build/path-power-graphs/velocity-feedback-current":configured);
        Files.createDirectories(directory.toPath());
        String priorStorage=System.getProperty(ApexStorage.DIRECTORY_PROPERTY);
        System.setProperty(ApexStorage.DIRECTORY_PROPERTY,new File(directory,"deterministic-data").getAbsolutePath());
        Follower follower=null;
        try {
            ApexSimulation.Hardware hardware=ApexSimulation.createHardware();
            final long[] nanos={1_000_000_000L};LongSupplier clock=()->nanos[0];
            SimulatorTuning.apply(FollowerConstants.getInstance());
            Constants robot=new Constants() {
                @Override public localizers.BaseLocalizerConstants<?> localizerConstants() {
                    return new localizers.Pinpoint.Constants() {
                        {setName("pinpoint");}
                        @Override public localizers.Pinpoint build(com.qualcomm.robotcore.hardware.HardwareMap map) {
                            final localizers.Pinpoint.Driver sensor=map.get(localizers.Pinpoint.Driver.class,"pinpoint");
                            return new localizers.Pinpoint(this,map) {
                                final List<SensorPacket> history=new ArrayList<>();
                                @Override public void update() {
                                    sensor.update();
                                    long stamp=clock.getAsLong();
                                    if(scenario==3) {
                                        Pose sensed=sensor.getVelocity();double seconds=DeltaTime.seconds(stamp-1_000_000_000L);
                                        sensed=new Pose(sensed.getVec().plus(TrajectorySample.vector(.3*Math.sin(2*Math.PI*7*seconds),
                                                .3*Math.cos(2*Math.PI*9*seconds))),sensed.getHeading());
                                        history.add(new SensorPacket(sensor.getPosition(),sensed,stamp));
                                        while(history.size()>1 && history.get(1).stamp<=stamp-60_000_000L) history.remove(0);
                                        SensorPacket packet=history.get(0);pose=packet.pose;calculate(packet.velocity,packet.stamp);
                                    } else {pose=sensor.getPosition();calculate(sensor.getVelocity(),stamp);}
                                }
                            };
                        }
                    };
                }
            };
            follower=Follower.create(robot,hardware.hardwareMap,false,clock);
            SimulatorTuning.apply(FollowerConstants.getInstance());follower.reset();
            GeometryFactory g=new GeometryFactory().setDistUnit(DistUnit.IN).setAngleUnit(AngleUnit.DEG);
            final boolean[] callback={false};
            Path path=g.holonomicPath(Pose.zero(),g.arcPose(30,0,7),g.arcPose(30,-30,7),
                            g.arcPose(-30,-30,7),g.arcPose(-30,30,7),g.pose(30,30,146))
                    .addHeadingNode(.30,g.angle(83)).addHeadingNode(.65,g.angle(-71))
                    .addDistanceCallback(.5,()->callback[0]=true).quickBuild();
            if(stored) {path.setBuildMode(Path.BuildMode.MEMORY);path.precomputeTrajectory();}
            follower.setPose(Pose.zero());follower.follow(path);
            trajectory.profile.FFLut nominal=new trajectory.profile.FFLut(path.createGenerator(path.getPlanningConstants()).generateRestToRest());
            if(previewSeconds>0 || pursuitSeconds>0) {
                // Future controllers are explicit historical experiments, outside normal following.
                java.lang.reflect.Field runtimeField=Follower.class.getDeclaredField("runtime");
                runtimeField.setAccessible(true);Object runtime=runtimeField.get(follower);
                java.lang.reflect.Field stateField=runtime.getClass().getDeclaredField("pathState");
                stateField.setAccessible(true);Object state=stateField.get(runtime);
                java.lang.reflect.Field plannerField=state.getClass().getDeclaredField("onlinePlanner");
                plannerField.setAccessible(true);
                plannerField.set(state,new trajectory.OnlineTrajectoryPlanner(path,path.getPlanningConstants(),previewSeconds,pursuitSeconds));
            }
            String name="Apex_Twist_"+(stored?"Stored":"Online")+"_"+new String[]{"Clean","ExtraDrag","ImpulseJitter","SensorDelayNoise"}[scenario];
            if(previewSeconds>0) name+="_Preview"+Math.round(previewSeconds*1000)+"ms";
            if(pursuitSeconds>0) name+="_Pursuit"+Math.round(pursuitSeconds*1000)+"ms";
            List<Double> distances=new ArrayList<>(),targets=new ArrayList<>(),actual=new ArrayList<>();
            List<Double> nominalVelocities=new ArrayList<>();
            List<Double> updateMilliseconds = new ArrayList<>();
            List<Double> headingTargets=new ArrayList<>(),headings=new ArrayList<>();
            double time=0,vSquared=0,wSquared=0,hSquared=0,feedbackSquared=0,maxCross=0;
            int count=0;boolean impulse=false;
            double nominalSquared=0,accelerationVariation=0,powerVariation=0,previousAcceleration=Double.NaN,previousPower=Double.NaN;
            File csvFile=new File(directory,name+".csv");
            try(PrintWriter csv=new PrintWriter(csvFile)) {
                csv.println("elapsed_s,path_distance_in,expected_motor_power,command_power,reference_velocity_in_s,"+
                        "plant_velocity_in_s,measured_velocity_in_s,planner_velocity_in_s,target_accel_in_s2,"+
                        "velocity_feedback_power,target_heading_deg,plant_heading_deg,heading_error_deg,"+
                        "nominal_velocity_in_s,goal_distance_in,goal_velocity_in_s,endpoint_blend");
                for(int tick=0;tick<1400 && follower.isBusy();tick++) {
                    if(scenario==2 && !impulse && time>=2) {
                        double speed=Math.hypot(hardware.drivetrain.velocity.x,hardware.drivetrain.velocity.y);
                        double scale=Math.max(0,(speed-5)/speed);
                        hardware.drivetrain.velocity.x*=scale;hardware.drivetrain.velocity.y*=scale;
                        hardware.drivetrain.updateWheelRollVelocities();impulse=true;
                    }
                    long updateStarted = System.nanoTime();
                    follower.update();
                    if (tick > 20) updateMilliseconds.add((System.nanoTime()-updateStarted)*1e-6);
                    TrajectorySample target=follower.getTrajectoryTarget();
                    if(target==null) break;
                    double v=hardware.drivetrain.velocity.x*target.tangent.getX().getIn()
                            +hardware.drivetrain.velocity.y*target.tangent.getY().getIn();
                    double reference=follower.getTrackingVelocityTarget();
                    double nominalVelocity=nominal.getKinematicParams(target.progressIn).getTangentialVel();
                    nominalVelocities.add(nominalVelocity);
                    double headingError=Math.toDegrees(Angle.wrap(hardware.drivetrain.position.theta-target.heading.getRad()));
                    double feedback=follower.getTrackingVelocityFeedback();
                    if (pursuitSeconds == 0 && previewSeconds == 0) {
                        assertTrue("Normal following must not publish a future speed goal", Double.isNaN(follower.getVelocityGoal()));
                        if (scenario < 3) assertEquals("Current velocity error drives proportional feedback",
                                FollowerConstants.getInstance().velocityFeedbackGain * (reference - follower.getTrackingMeasuredVelocity()),
                                feedback, 1e-8);
                    }
                    double power=Math.max(Math.max(Math.abs(follower.getDrivetrain().getLastFlPower()),Math.abs(follower.getDrivetrain().getLastFrPower())),
                            Math.max(Math.abs(follower.getDrivetrain().getLastBlPower()),Math.abs(follower.getDrivetrain().getLastBrPower())));
                    distances.add(target.progressIn);targets.add(reference);actual.add(v);
                    headingTargets.add(Math.toDegrees(target.heading.getRad()));headings.add(Math.toDegrees(target.heading.getRad())+headingError);
                    if(follower.getTrackingEndpointBlend()<.05) {
                        vSquared+=Math.pow(v-reference,2);wSquared+=Math.pow(hardware.drivetrain.velocity.theta-target.angularVelocity,2);
                        hSquared+=headingError*headingError;feedbackSquared+=feedback*feedback;count++;
                        nominalSquared+=Math.pow(v-nominalVelocity,2);
                        if(Double.isFinite(previousAcceleration)) {
                            accelerationVariation+=Math.abs(target.tangentialAcceleration()-previousAcceleration);
                            powerVariation+=Math.abs(power-previousPower);
                        }
                    }
                    previousAcceleration=target.tangentialAcceleration();previousPower=power;
                    maxCross=Math.max(maxCross,Math.abs(follower.getCrossTrackErrorIn()));
                    csv.printf(Locale.US,"%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f%n",
                            time,target.progressIn,target.expectedMotorPower,power,reference,v,follower.getTrackingMeasuredVelocity(),
                            target.tangentialVelocity(),target.tangentialAcceleration(),feedback,
                            Math.toDegrees(target.heading.getRad()),Math.toDegrees(target.heading.getRad())+headingError,headingError,
                            nominalVelocity,follower.getVelocityGoalDistanceIn(),follower.getVelocityGoal(),follower.getTrackingEndpointBlend());
                    double dt=scenario==2?new double[]{.01,.03,.02}[tick%3]:.02,remaining=dt;
                    while(remaining>1e-9) {
                        double sub=Math.min(.005,remaining);ApexSimulation.stepPhysics(hardware,sub);
                        if(scenario==1 && time>.8 && time<4) {
                            double speed=Math.hypot(hardware.drivetrain.velocity.x,hardware.drivetrain.velocity.y);
                            double scale=speed>1?Math.max(0,(speed-4*sub)/speed):1;
                            hardware.drivetrain.velocity.x*=scale;hardware.drivetrain.velocity.y*=scale;
                            hardware.drivetrain.updateWheelRollVelocities();
                        }
                        remaining-=sub;
                    }
                    nanos[0]+=Math.round(dt*1e9);time+=dt;
                }
            }
            AutoTestPowerGraphs.write(csvFile,directory,name+"_Power",name+", Feedback Enabled");
            chart(directory,name+"_Velocity","Velocity (in/s)",pursuitSeconds>0?"Future speed goal":"Current-time reference",distances,targets,actual);
            chart(directory,name+"_Nominal_Envelope","Velocity (in/s)","Nominal speed at current progress",distances,nominalVelocities,actual);
            chart(directory,name+"_Heading","Unwrapped heading (deg)","Heading at current progress",distances,headingTargets,headings);
            double endpoint=follower.getPose().distanceTo(path.getEndPose()).getIn();
            double finalHeading=Math.abs(follower.getPose().getHeading().getShortestAngleTo(path.getEndPose().getHeading()).getDeg());
            Collections.sort(updateMilliseconds);
            System.out.printf(Locale.US,"FOLLOWER CPU: case=%s medianMs=%.6f p95Ms=%.6f maxMs=%.6f%n",name,
                    updateMilliseconds.get(updateMilliseconds.size()/2),updateMilliseconds.get(updateMilliseconds.size()*95/100),
                    updateMilliseconds.get(updateMilliseconds.size()-1));
            System.out.printf(Locale.US,"VELOCITY FOLLOWER: case=%s complete=%s duration=%.6f goalVelocityDifferenceRMS=%.6f omegaRMSE=%.6f headingRMSEdeg=%.6f feedbackRMS=%.6f maxCross=%.6f endpoint=%.6f endHeadingErrorDeg=%.6f%n",
                    name,!follower.isBusy(),time,Math.sqrt(vSquared/count),Math.sqrt(wSquared/count),Math.sqrt(hSquared/count),
                    Math.sqrt(feedbackSquared/count),maxCross,endpoint,finalHeading);
            assertFalse("Twisting path must complete",follower.isBusy());assertTrue(callback[0]);
            if(pursuitSeconds==0 && scenario<3) assertTrue("Current-time velocity accuracy",Math.sqrt(vSquared/count)<1.5);
            if (pursuitSeconds == 0 && previewSeconds == 0 && scenario < 3)
                assertTrue("Proportional velocity feedback must respond to real tracking error", feedbackSquared > 1e-8);
            if (pursuitSeconds > 0) {
                assertTrue("Independent spatial speed envelope accuracy", Math.sqrt(nominalSquared/count) < 3.5);
                assertTrue("Pursuit must complete without excessive slowdown", time < 12);
                assertEquals("Tangential speed correction is owned by pursuit", 0, Math.sqrt(feedbackSquared/count), 1e-9);
            }
            assertTrue("Twist heading accuracy",Math.sqrt(hSquared/count)<(scenario==3?5:3));
            assertTrue("Lateral accuracy",maxCross<1.5);assertTrue("Endpoint accuracy",endpoint<1.5);
            assertTrue("Endpoint heading accuracy",finalHeading<3);
            return new double[]{Math.sqrt(vSquared/count),Math.sqrt(hSquared/count),endpoint,time,
                    Math.sqrt(nominalSquared/count),accelerationVariation/time,powerVariation/time};
        } finally {
            if(follower!=null) follower.stop();
            if(priorStorage==null) System.clearProperty(ApexStorage.DIRECTORY_PROPERTY);
            else System.setProperty(ApexStorage.DIRECTORY_PROPERTY,priorStorage);
        }
    }
    private static final class SensorPacket {
        final Pose pose,velocity;final long stamp;
        SensorPacket(Pose pose,Pose velocity,long stamp) {this.pose=pose;this.velocity=velocity;this.stamp=stamp;}
    }
    private static void chart(File directory,String name,String axis,String targetLabel,List<Double> distance,List<Double> target,List<Double> actual) throws Exception {
        XYChart chart=new XYChartBuilder().width(1000).height(460).title(name).xAxisTitle("Distance along path (in)").yAxisTitle(axis).build();
        chart.getStyler().setLegendPosition(Styler.LegendPosition.OutsideE);chart.getStyler().setMarkerSize(0);
        chart.addSeries(targetLabel,distance,target);chart.addSeries("Independent plant",distance,actual);
        BitmapEncoder.saveBitmap(chart,new File(directory,name).getAbsolutePath(),BitmapEncoder.BitmapFormat.PNG);
    }
}
