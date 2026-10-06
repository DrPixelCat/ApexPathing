package org.firstinspires.ftc.teamcode.sim;

import follower.Follower;
import follower.FollowerConstants;
import geometry.AngleUnit;
import geometry.Pose;
import drivetrains.BaseDrivetrain;
import org.codeblooded.ftcodesim.ascope.boundaries.MotionVector;
import org.firstinspires.ftc.teamcode.apexpathing.Constants;
import org.junit.Test;
import storage.ApexStorage;
import java.lang.reflect.*;
import java.nio.file.*;
import java.util.*;
import java.util.concurrent.atomic.AtomicLong;
import static org.junit.Assert.*;

/** Paired regressions on one excitation, with a shared deterministic sensor/physics clock. */
public class SysIdMethodComparisonTest {
    static final double DT = .02;
    static final class Window {
        final int run;
        final double time, power, nativeV, nativeA, filteredV, filteredA;
        Window(int run, double time, double power, double nv, double na, double fv, double fa) {
            this.run=run; this.time=time; this.power=power;
            nativeV=nv; nativeA=na; filteredV=fv; filteredA=fa;
        }
    }
    static final class Gains {
        final double ks,kv,ka;
        Gains(double ks,double kv,double ka) { this.ks=ks; this.kv=kv; this.ka=ka; }
    }
    @Test public void compareExistingFitsAndSensorPathsOnIdenticalRuns() throws Exception {
        Path output=Paths.get("build/sysid-method-comparison").toAbsolutePath();
        Files.createDirectories(output);
        String directory=System.getProperty(ApexStorage.DIRECTORY_PROPERTY);
        String unlock=System.getProperty("apex.simulation.unlockTunerPhases");
        Field singleton=FollowerConstants.class.getDeclaredField("instance"); singleton.setAccessible(true);
        Object old=singleton.get(null);
        System.setProperty(ApexStorage.DIRECTORY_PROPERTY, Files.createTempDirectory("sysid-methods-").toString());
        List<String> windows=new ArrayList<>(), summary=new ArrayList<>(), profiles=new ArrayList<>();
        windows.add("model,axis,run,time_s,power,native_velocity,native_acceleration,filtered_velocity,filtered_acceleration");
        summary.add("model,axis,method,kS,kV,kA,acceleration_mean_error,cruise_mean_error,cruise_rmse,holding_power,holding_acceleration");
        profiles.add("model,axis,method,direction,time_s,target,physical_velocity,power");
        try {
          for(boolean control:new boolean[]{false,true}) {
            for(boolean heading:new boolean[]{false,true}) {
                singleton.set(null,null);
                String model=control?"DC-back-emf-control":"stock";
                List<Window> data=collect(heading,control);
                for(Window w:data) windows.add(model+","+(heading?"HEADING":"DRIVE")+","+w.run+","+w.time+","+w.power+","+w.nativeV+","+w.nativeA+","+w.filteredV+","+w.filteredA);
                for(boolean filtered:new boolean[]{false,true}) {
                    for(boolean sequential:new boolean[]{false,true}) {
                        String method=(sequential?"RR-sequential":"Apex-joint")+(filtered?"-filtered":"-native");
                        Gains gains=sequential?sequential(data,filtered):joint(data,filtered,heading);
                        assertTrue(method,Double.isFinite(gains.ks)&&Double.isFinite(gains.kv)&&Double.isFinite(gains.ka));
                        double[] scores=profile(heading,gains,method,profiles,control);
                        double peak=heading?1.5:24, power=gains.ks+gains.kv*peak;
                        double holdingA=probe(heading,peak,power,control);
                        if(control&&!filtered&&!sequential) {
                            assertEquals("Existing joint fit must work on the compatible positive control",0,scores[1],.03*peak);
                            assertEquals("Independent holding probe",0,holdingA,heading?.02:.2);
                        }
                        String row=model+","+(heading?"HEADING":"DRIVE")+","+method+","+gains.ks+","+gains.kv+","+gains.ka+","+scores[0]+","+scores[1]+","+scores[2]+","+power+","+holdingA;
                        summary.add(row); System.out.println(row);
                    }
                }
            }
          }
          List<String> slopes=new ArrayList<>(); slopes.add("speed,stock_acceleration_per_power,control_acceleration_per_power");
          for(double speed:new double[]{8,24,40}) {
              double stock=(probe(false,speed,.5,false)-probe(false,speed,.35,false))/.15;
              double dc=(probe(false,speed,.5,true)-probe(false,speed,.35,true))/.15;
              slopes.add(speed+","+stock+","+dc);
              assertEquals(183-150.0/65*speed,stock,1e-5); assertEquals(183,dc,1e-5);
          }
          Files.write(output.resolve("authority-probes.csv"),slopes);
        } finally {
            singleton.set(null,old);
            restore(ApexStorage.DIRECTORY_PROPERTY,directory); restore("apex.simulation.unlockTunerPhases",unlock);
            Files.write(output.resolve("windows.csv"),windows);
            Files.write(output.resolve("comparison.csv"),summary);
            Files.write(output.resolve("profiles.csv"),profiles);
        }
    }
    private static List<Window> collect(boolean heading,boolean control) {
        List<Window> data=new ArrayList<>();
        for(int run=0;run<4;run++) {
            ApexSimulation.Hardware h=ApexSimulation.createHardware();
            configureControl(h,control);
            AtomicLong clock=new AtomicLong(1_000_000_000L);
            ((SimApexPinpoint)h.hardwareMap.get(localizers.Pinpoint.Driver.class,ApexSimulation.PINPOINT)).setClock(clock::get);
            Follower f=new Follower(new Constants(),h.hardwareMap,true,clock::get);
            f.setPose(new Pose(new geometry.Vector(geometry.Dist.fromIn(run%2==0?-55:55),geometry.Dist.fromIn(0)),geometry.Angle.zero()));
            f.disableControllers(); f.update(false);
            double startX=h.drivetrain.position.x, travel=0, lastHeading=h.drivetrain.position.theta;
            double prevN=velocity(f.getLocalizer().getRawVel(),f.getLocalizer().getRawPose(),heading);
            double prevF=velocity(f.getVelocity(),f.getPose(),heading), previousPower=0;
            double wt=0,nv=0,fv=0,p=0,startN=prevN,startF=prevF;
            for(int tick=1;tick<=450;tick++) {
                step(h,DT); clock.addAndGet(20_000_000L); f.update(false);
                double t=tick*DT;
                double n=velocity(f.getLocalizer().getRawVel(),f.getLocalizer().getRawPose(),heading);
                double v=velocity(f.getVelocity(),f.getPose(),heading);
                double threshold=heading?.05:.5;
                boolean moving=Math.abs(n)>=threshold&&Math.abs(prevN)>=threshold&&Math.signum(n)==Math.signum(prevN);
                if(moving) {
                    wt+=DT; nv+=.5*(n+prevN)*DT; fv+=.5*(v+prevF)*DT; p+=previousPower*DT;
                    if(wt>=.1-1e-9) {
                        data.add(new Window(run,t,p/wt,nv/wt,(n-startN)/wt,fv/wt,(v-startF)/wt));
                        wt=nv=fv=p=0; startN=n; startF=v;
                    }
                } else { wt=nv=fv=p=0; startN=n; startF=v; }
                prevN=n; prevF=v;
                travel+=Math.abs(h.drivetrain.position.theta-lastHeading); lastHeading=h.drivetrain.position.theta;
                if(t>=(run<2?9:2)||(heading?travel>=4*Math.PI:Math.abs(h.drivetrain.position.x-startX)>=96)) break;
                double command=(run%2==0?1:-1)*(run<2?Math.min(.9,.1*t):.5);
                f.getDrivetrain().moveWithVectors(heading?0:command,0,heading?command:0);
                previousPower=Math.copySign(Math.abs(f.getDrivetrain().getLastFrPower()),command);
            }
            f.stop();
        }
        return data;
    }
    private static double velocity(Pose velocity,Pose pose,boolean heading) {
        return heading?velocity.getHeading(AngleUnit.RAD):velocity.getVec().rotate(pose.getHeading().times(-1)).getX().getIn();
    }
    private static Gains joint(List<Window> data,boolean filtered,boolean heading) throws Exception {
        Class<?> type=Class.forName("tuning.follower.sysid.SysIdRegression");
        Constructor<?> constructor=type.getDeclaredConstructor(); constructor.setAccessible(true);
        Object regression=constructor.newInstance();
        Method add=type.getDeclaredMethod("add",int.class,double.class,double.class,double.class,double.class); add.setAccessible(true);
        for(Window w:data) add.invoke(regression,w.run,w.time,filtered?w.filteredV:w.nativeV,filtered?w.filteredA:w.nativeA,w.power);
        Method fit=type.getDeclaredMethod("fit",double.class); fit.setAccessible(true);
        Object result=fit.invoke(regression,heading?.2:2);
        return new Gains(field(result,"kS"),field(result,"kV"),field(result,"kA"));
    }
    // Road Runner legacy RegressionUtil: ramp power-on-velocity OLS, then acceleration residual OLS.
    private static Gains sequential(List<Window> data,boolean filtered) {
        double count=0,v=0,p=0,vv=0,vp=0;
        for(Window w:data) if(w.run<2) {
            double x=Math.abs(filtered?w.filteredV:w.nativeV), y=w.power*(w.run%2==0?1:-1);
            count++; v+=x; p+=y; vv+=x*x; vp+=x*y;
        }
        double kv=(count*vp-v*p)/(count*vv-v*v), ks=(p-kv*v)/count;
        ks=Math.abs(ks); kv=Math.abs(kv);
        double aa=0,ap=0;
        for(Window w:data) if(w.run>=2) {
            double x=filtered?w.filteredV:w.nativeV, a=filtered?w.filteredA:w.nativeA;
            if(x*a<0)continue;
            aa+=a*a; ap+=a*(w.power-ks*Math.signum(x)-kv*x);
        }
        return new Gains(ks,kv,Math.abs(ap/aa));
    }
    private static double field(Object object,String name) throws Exception {
        Field f=object.getClass().getDeclaredField(name); f.setAccessible(true); return f.getDouble(object);
    }
    private static double[] profile(boolean heading,Gains g,String method,List<String> trace,boolean control) {
        double accel=0,cruise=0,squared=0,peak=heading?1.5:24;
        for(int direction:new int[]{1,-1}) {
            ApexSimulation.Hardware h=ApexSimulation.createHardware();
            configureControl(h,control);
            BaseDrivetrain<?> drive=new Constants().drivetrainConstants().build(h.hardwareMap);
            for(int tick=0;tick<150;tick++) {
                double t=tick*DT, v=direction*peak*(t<1?t:t<2?1:3-t);
                double a=direction*peak*(t<1?1:t<2?0:-1);
                double power=g.ks*(v==0?Math.signum(a):Math.signum(v))+g.kv*v+g.ka*a;
                drive.moveWithVectors(heading?0:power,0,heading?power:0);
                double measured=heading?h.drivetrain.velocity.theta:h.drivetrain.velocity.x;
                double error=direction*(measured-v);
                if(t<1)accel+=error*DT; else if(t<2){cruise+=error*DT;squared+=error*error*DT;}
                trace.add((control?"DC-back-emf-control":"stock")+","+(heading?"HEADING":"DRIVE")+","+method+","+direction+","+t+","+v+","+measured+","+power);
                step(h,DT);
            }
            drive.stop();
        }
        return new double[]{accel/2,cruise/2,Math.sqrt(squared/2)};
    }
    static double probe(boolean heading,double speed,double power,boolean control) {
        ApexSimulation.Hardware h=ApexSimulation.createHardware();
        configureControl(h,control);
        BaseDrivetrain<?> drive=new Constants().drivetrainConstants().build(h.hardwareMap);
        h.drivetrain.velocity=new MotionVector(heading?0:speed,0,heading?speed:0); h.drivetrain.updateWheelRollVelocities();
        drive.moveWithVectors(heading?0:power,0,heading?power:0);
        StockSimTestSupport.step(h,.0001);
        return ((heading?h.drivetrain.velocity.theta:h.drivetrain.velocity.x)-speed)/.0001;
    }
    private static void configureControl(ApexSimulation.Hardware h,boolean control) {
        if(!control)return;
        // Deliberately labeled test-only ablation: remove ONLY the duty factor from the
        // powered, same-direction back-emf term. Preserve gains, friction and braking branches.
        for(org.codeblooded.ftcodesim.hardware.devices.SimMotor motor:StockSimTestSupport.motors(h)) {
            motor.config.motorModel.terms[1]=(v,d,b)->Math.signum(v)==Math.signum(d)?-v:0;
        }
    }
    private static void step(ApexSimulation.Hardware h,double dt) {
        for(int i=0;i<4;i++)StockSimTestSupport.step(h,dt/4);
    }
    private static void restore(String name,String value) {
        if(value==null)System.clearProperty(name);else System.setProperty(name,value);
    }
}
