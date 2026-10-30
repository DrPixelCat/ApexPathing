package org.firstinspires.ftc.teamcode.sim;

import feedforward.MecanumWheelModel;
import paths.geometry.Angle;

import org.codeblooded.ftcodesim.ascope.boundaries.MotionVector;
import org.junit.Test;
import java.io.*;
import java.nio.file.Files;
import java.util.Locale;
import static org.junit.Assert.*;

/** Compare velocity-reference hypotheses against an unchanged plant before follower integration. */
public class VelocityFeedbackExperimentTest {
    private static final double RATE = 183 * .04464838061276286;

    @Test public void futureConvergenceRejectsDelayedCruiseNoise() throws Exception {
        double[] legacy = cruiseNoise(false), pursuit = cruiseNoise(true);
        System.out.printf(Locale.US, "CRUISE NOISE: legacy velocityRMS=%.6f accelerationRMS=%.6f powerStd=%.6f; pursuit velocityRMS=%.6f accelerationRMS=%.6f powerStd=%.6f%n",
                legacy[0], legacy[1], legacy[2], pursuit[0], pursuit[1], pursuit[2]);
        assertTrue("Future convergence reduces physical speed oscillation", pursuit[0] < .85 * legacy[0]);
        assertTrue("Damping reduces noisy acceleration commands", pursuit[1] < .6 * legacy[1]);
        assertTrue("Future convergence reduces wheel power oscillation", pursuit[2] < .85 * legacy[2]);
    }

    /** Equal actual-state feedforward, unchanged stock plant; only speed feedback differs. */
    private static double[] cruiseNoise(boolean future) throws Exception {
        MeasuredMecanumExperimentTest.Rig rig = new MeasuredMecanumExperimentTest.Rig();
        rig.hardware.drivetrain.setPosition(new MotionVector(ApexSimulation.FIELD_CENTER_INCHES - 40,
                ApexSimulation.FIELD_CENTER_INCHES, 0));
        rig.hardware.drivetrain.velocity.x = 16; rig.hardware.drivetrain.updateWheelRollVelocities();
        MecanumWheelModel model = SimulatorTuning.snapshot().motionCharacterization.mecanumWheelModel;
        MecanumWheelModel.Demand demand = new MecanumWheelModel.Demand();
        trajectory.VelocityPursuit pursuit = new trajectory.VelocityPursuit();
        double[] debt = {.5,.5,.5,.5}, powers = new double[4], sensed = new double[300];
        double velocitySquared = 0, accelerationSquared = 0, powerSum = 0, powerSquared = 0;
        int count = 0;
        // Five seconds keeps the straight cruise clear of the stock field obstacles.
        for (int tick = 0; tick < 250; tick++) {
            double v = rig.hardware.drivetrain.velocity.x;
            sensed[tick] = v + .3 * Math.sin(2 * Math.PI * 7 * tick * .02);
            double measured = sensed[Math.max(0, tick - 3)]; // Identical 60 ms sensor delay.
            double acceleration;
            if (future) {
                trajectory.VelocityPursuit.Goal goal = pursuit.goal(0, 100, measured, .35,
                        new double[]{0,100}, new double[]{256,256}, 2);
                acceleration = pursuit.damp(goal, 0, measured, .35, .02);
                pursuit.applied(acceleration);
            } else acceleration = RATE * (16 - measured);
            model.solveBounded(v + acceleration * .0075, rig.hardware.drivetrain.velocity.y,
                    rig.hardware.drivetrain.velocity.theta, acceleration,
                    -8 * rig.hardware.drivetrain.velocity.y, -8 * rig.hardware.drivetrain.velocity.theta, 1, demand);
            demand.pulse(debt, powers); rig.command(powers);
            for (int sub = 0; sub < 4; sub++) ApexSimulation.stepPhysics(rig.hardware, .005);
            if (tick >= 75) {
                double actual = rig.hardware.drivetrain.velocity.x;
                double power = (powers[0] + powers[1] + powers[2] + powers[3]) / 4;
                velocitySquared += Math.pow(actual - 16, 2); accelerationSquared += acceleration * acceleration;
                powerSum += power; powerSquared += power * power; count++;
            }
        }
        return new double[]{Math.sqrt(velocitySquared/count), Math.sqrt(accelerationSquared/count),
                Math.sqrt(Math.max(0, powerSquared/count - Math.pow(powerSum/count, 2)))};
    }

    @Test public void compareCurrentTimeReferenceWithPreviewUnderDisturbance() throws Exception {
        File directory = new File("build/path-power-graphs/velocity-reference-experiment");
        Files.createDirectories(directory.toPath());
        try (PrintWriter summary = new PrintWriter(new File(directory,"summary.csv"))) {
            summary.println("scenario,mode,velocity_rmse,feedback_rms,endpoint_x,heading_deg");
            for (int scenario=0;scenario<3;scenario++) {
              double[] errors=new double[4];
              for (int mode=0;mode<4;mode++) {
                double[] result=run(scenario,mode,directory);
                String row=String.format(Locale.US,"%d,%d,%.8f,%.8f,%.8f,%.8f",scenario,mode,
                        result[0],result[1],result[2],result[3]);
                summary.println(row);System.out.println("VELOCITY REFERENCE EXPERIMENT: "+row);
                assertTrue(Double.isFinite(result[0]));
                errors[mode]=result[0];
              }
              assertTrue("Current-time reference rejects disturbances",errors[3]<.6);
              for(int mode=0;mode<3;mode++) assertTrue("Current-time reference improves scenario "+scenario+
                      " versus reference policy "+mode,errors[3]<errors[mode]);
            }
        }
    }

    private static double[] run(int scenario,int mode,File directory) throws Exception {
        MeasuredMecanumExperimentTest.Rig rig=new MeasuredMecanumExperimentTest.Rig();
        MecanumWheelModel model=SimulatorTuning.snapshot().motionCharacterization.mecanumWheelModel;
        rig.hardware.drivetrain.setPosition(new MotionVector(ApexSimulation.FIELD_CENTER_INCHES-40,
                ApexSimulation.FIELD_CENTER_INCHES,0));
        MecanumWheelModel.Demand demand=new MecanumWheelModel.Demand();
        double[] debt={.5,.5,.5,.5},powers=new double[4];
        follower.internal.VelocityReference continuous=new follower.internal.VelocityReference();
        double time=0;
        double squared=0,feedbackSquared=0,duration=0;boolean impulse=false;int tick=0;
        try(PrintWriter csv=new PrintWriter(new File(directory,"scenario-"+scenario+"-mode-"+mode+".csv"))) {
            csv.println("time_s,target_velocity,plant_velocity,feedback_accel,reference_velocity");
            while(time<6) {
                double dt=scenario==2?new double[]{.01,.03,.02}[tick%3]:.02;
                // End commands at known reference transitions, equally for every candidate.
                for(double boundary:new double[]{.5,5,5.5,6}) if(time<boundary-1e-8) {
                    dt=Math.min(dt,boundary-time);break;
                }
                if(scenario==2 && !impulse && time>=2) {
                    rig.hardware.drivetrain.velocity.x-=5;
                    rig.hardware.drivetrain.updateWheelRollVelocities();impulse=true;
                }
                double v=rig.hardware.drivetrain.velocity.x,vy=rig.hardware.drivetrain.velocity.y;
                double omega=rig.hardware.drivetrain.velocity.theta,heading=rig.hardware.drivetrain.position.theta;
                double[] nominal=reference(time);
                long stamp=1_000_000_000L+Math.round(time*1e9);
                double referenceVelocity=continuous.update(stamp,stamp,v,nominal[2]);
                double error=mode==3?referenceVelocity-v:nominal[2]*(mode==0?0:mode==1?.02:.05);
                double correction=(mode==3?model.gain*continuous.powerGain(RATE/model.gain,model.gain):RATE)*error;
                double ax=nominal[2]+correction;
                double ay=-8*vy;
                double alpha=nominal[4]+4* Angle.wrap(nominal[5]-heading)+8*(nominal[3]-omega);
                double lead=Math.max(0,dt/2-.0025);
                double h=heading+omega*lead,c=Math.cos(h),s=Math.sin(h);
                double vxPreview=v+ax*lead,vyPreview=vy+ay*lead;
                double scale=model.solveBounded(vxPreview*c+vyPreview*s,-vxPreview*s+vyPreview*c,
                        omega+alpha*lead,ax*c+ay*s,-ax*s+ay*c,alpha,1,demand);
                demand.pulse(debt,powers);rig.command(powers);
                if(mode==3) continuous.condition(scale);
                double remaining=dt;
                while(remaining>1e-9) {
                    double sub=Math.min(.005,remaining);
                    ApexSimulation.stepPhysics(rig.hardware,sub);
                    if(scenario==1 && time>=.8 && time<4) {
                        rig.hardware.drivetrain.velocity.x-=4*sub;
                        rig.hardware.drivetrain.updateWheelRollVelocities();
                    }
                    remaining-=sub;
                }
                time+=dt;tick++;
                double target=reference(time)[1],actual=rig.hardware.drivetrain.velocity.x;
                squared+=(actual-target)*(actual-target)*dt;feedbackSquared+=correction*correction*dt;duration+=dt;
                csv.printf(Locale.US,"%.8f,%.8f,%.8f,%.8f,%.8f%n",time,target,actual,correction,referenceVelocity);
            }
        }
        return new double[]{Math.sqrt(squared/duration),Math.sqrt(feedbackSquared/duration),
                rig.hardware.drivetrain.position.x-ApexSimulation.FIELD_CENTER_INCHES,
                Math.toDegrees(rig.hardware.drivetrain.position.theta)};
    }

    private static double[] reference(double t) {
        double s,v,a;
        if(t<.5-1e-9) {s=16*t*t;v=32*t;a=32;}
        else if(t<5-1e-9) {s=4+16*(t-.5);v=16;a=0;}
        else if(t<5.5-1e-9) {double u=t-5;s=76+16*u-16*u*u;v=16-32*u;a=-32;}
        else {s=80;v=a=0;}
        double p=s/80,prime=Math.PI/2*(6*p-6*p*p)/80,second=Math.PI/2*(6-12*p)/(80*80);
        return new double[]{s,v,a,prime*v,second*v*v+prime*a,Math.PI/2*(3*p*p-2*p*p*p)};
    }
}
