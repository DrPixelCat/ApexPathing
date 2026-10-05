package org.firstinspires.ftc.teamcode.sim;

import geometry.*;
import follower.FollowerConstants;
import org.codeblooded.ftcodesim.ascope.boundaries.MotionVector;
import org.codeblooded.ftcodesim.hardware.devices.SimMotor;
import org.firstinspires.ftc.teamcode.apexpathing.Constants;
import org.junit.Test;
import paths.heading.InterpolationStyle;
import paths.movements.Path;
import trajectory.generators.MecanumProfileGenerator;
import trajectory.profile.FFLut;
import trajectory.profile.MotionParameters;

import java.io.*;
import java.nio.file.Files;
import java.util.Locale;

import static org.junit.Assert.*;

/** Prove the coupled wheel-response hypothesis before changing the production follower. */
public class MeasuredMecanumExperimentTest {
    static final double DT = .02;

    static final class Rig {
        final ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
        final SimMotor[] motors;
        Rig() { new Constants().drivetrainConstants().build(hardware.hardwareMap); motors = StockSimTestSupport.motors(hardware); }
        double[] probe(double vx, double vy, double omega, double[] powers) {
            hardware.drivetrain.setPosition(new MotionVector(ApexSimulation.FIELD_CENTER_INCHES, ApexSimulation.FIELD_CENTER_INCHES, 0));
            hardware.drivetrain.velocity = new MotionVector(vx, vy, omega); hardware.drivetrain.updateWheelRollVelocities();
            command(powers);
            ApexSimulation.stepPhysics(hardware, .0001);
            return new double[]{(hardware.drivetrain.velocity.x-vx)/.0001, (hardware.drivetrain.velocity.y-vy)/.0001,
                    (hardware.drivetrain.velocity.theta-omega)/.0001};
        }
        void command(double[] powers) {
            for (int i=0;i<4;i++) { assertTrue("No hidden clipping", Math.abs(powers[i]) <= 1.000001); motors[i].setPower(powers[i]); }
        }
    }

    static Model calibrate() {
        Rig rig = new Rig();
        double[][] fits = new double[3][];
        for (int axis=0;axis<3;axis++) {
            double[][] normal = new double[5][6];
            for (double speed : axis == 2 ? new double[]{.2,.8,2,4,6} : new double[]{6,17,33,48,60}) {
                for (double power : new double[]{.03,.21,.6,-.03,-.12,0}) {
                    double[] acceleration = rig.probe(axis==0?speed:0,axis==1?speed:0,axis==2?speed:0,
                            wheels(axis==0?power:0,axis==1?power:0,axis==2?power:0,1));
                    double[] features = {power, -speed*Math.max(power,0), power<0?-speed:0, power==0?-speed:0,-1};
                    for(int i=0;i<5;i++) { for(int j=0;j<5;j++) normal[i][j]+=features[i]*features[j]; normal[i][5]+=features[i]*acceleration[axis]; }
                }
            }
            fits[axis] = solve(normal);
        }
        for(int i=0;i<4;i++) assertEquals("Linear axes share measured wheel response", fits[0][i],fits[1][i],1e-6);
        for(int i=1;i<4;i++) assertEquals("Angular probes share measured wheel response", fits[0][i],fits[2][i],1e-6);
        return new Model(fits[0][0],fits[0][1],fits[0][2],fits[0][3],fits[0][4],fits[1][4],fits[2][4],fits[0][0]/fits[2][0]);
    }

    static final class Model {
        final double gain, speedLoss, regen, neutral, forwardDrag, strafeDrag, angularDrag, radius;
        Model(double gain,double speedLoss,double regen,double neutral,double fd,double sd,double ad,double radius) {
            this.gain=gain;this.speedLoss=speedLoss;this.regen=regen;this.neutral=neutral;
            forwardDrag=fd;strafeDrag=sd;angularDrag=ad;this.radius=radius;
        }
        Demand inverse(double vx,double vy,double omega,double ax,double ay,double alpha) {
            feedforward.MecanumWheelModel measured=new feedforward.MecanumWheelModel(gain,speedLoss,regen,neutral,
                    forwardDrag,strafeDrag,angularDrag,radius,60,0);
            feedforward.MecanumWheelModel.Demand demand=new feedforward.MecanumWheelModel.Demand();
            measured.solve(vx,vy,omega,ax,ay,alpha,demand);
            assertTrue("Held-out states stay inside the measured domain",demand.insideDomain);
            return new Demand(demand);
        }
    }
    static final class Demand {
        final feedforward.MecanumWheelModel.Demand nativeDemand;
        final double[] positive,negative,duty;
        Demand(feedforward.MecanumWheelModel.Demand demand) {
            nativeDemand=demand;positive=demand.aligned;negative=demand.opposing;duty=demand.opposingDuty;
        }
        double[] pulse(double[] debt) {
            double[] powers=new double[4];
            nativeDemand.pulse(debt,powers);
            return powers;
        }
    }

    @Test public void heldOutCombinedStatesValidateBeforeIntegration() throws Exception {
        Model model=calibrate(); Rig rig=new Rig();
        File directory=new File("build/path-power-graphs/measured-mecanum-experiment"); Files.createDirectories(directory.toPath());
        double squared=0,peak=0; int count=0;
        try(PrintWriter csv=new PrintWriter(new File(directory,"held-out-combined.csv")); PrintWriter summary=new PrintWriter(new File(directory,"model.txt"))) {
            summary.printf(Locale.US,"gain=%.12f speedLoss=%.12f regen=%.12f neutral=%.12f forwardDrag=%.12f strafeDrag=%.12f angularDrag=%.12f radius=%.12f%n",
                    model.gain,model.speedLoss,model.regen,model.neutral,model.forwardDrag,model.strafeDrag,model.angularDrag,model.radius);
            csv.println("direction_deg,speed_in_s,omega_rad_s,tangent_accel,alpha_rad_s2,error_ax,error_ay,error_alpha");
            for(double heading:new double[]{0,.001,15,42,79,90,135,210,315}) for(double speed:new double[]{4,13,28})
                for(double omega:new double[]{-.8,0,.65}) for(double a:new double[]{-25,0,35}) for(double alpha:new double[]{-1,0,.8}) {
                    double c=Math.cos(Math.toRadians(heading)),s=Math.sin(Math.toRadians(heading));
                    Demand d=model.inverse(speed*c,speed*s,omega,a*c,a*s,alpha);
                    double[] actual=new double[3]; boolean feasible=true;
                    for(int i=0;i<4;i++) feasible &= Math.abs(d.positive[i])<=1 && Math.abs(d.negative[i])<=1;
                    if(!feasible) continue;
                    for(int mask=0;mask<16;mask++) {
                        double probability=1; double[] powers=new double[4];
                        for(int i=0;i<4;i++) { boolean reverse=(mask&(1<<i))!=0; probability*=reverse?d.duty[i]:1-d.duty[i];powers[i]=reverse?d.negative[i]:d.positive[i]; }
                        if(probability<1e-12) continue;
                        double[] response=rig.probe(speed*c,speed*s,omega,powers);
                        for(int i=0;i<3;i++) actual[i]+=probability*response[i];
                    }
                    double[] error={actual[0]-a*c,actual[1]-a*s,actual[2]-alpha};
                    for(double e:error) { squared+=e*e;peak=Math.max(peak,Math.abs(e));count++; }
                    csv.printf(Locale.US,"%.6f,%.6f,%.6f,%.6f,%.6f,%.9f,%.9f,%.9f%n",heading,speed,omega,a,alpha,error[0],error[1],error[2]);
                }
            summary.printf(Locale.US,"heldOutComponents=%d RMSE=%.12f peak=%.12f%n",count,Math.sqrt(squared/count),peak);
        }
        System.out.printf(Locale.US,"COUPLED HYPOTHESIS: components=%d RMSE=%.12f peak=%.12f%n",count,Math.sqrt(squared/count),peak);
        assertTrue("Coverage across at least 600 feasible combined states",count>=1800);
        assertTrue("Independent combined-state probes must validate",peak<1e-5);
    }

    @Test public void twistTraversalWithoutAnyErrorFeedback() throws Exception {
        Model model=calibrate(); Rig rig=new Rig();
        FollowerConstants constants=SimulatorTuning.snapshot();
        GeometryFactory geometry=new GeometryFactory().setDistUnit(DistUnit.IN).setAngleUnit(AngleUnit.DEG);
        Path path=geometry.holonomicPath(geometry.pose(-40,0,0),geometry.pose(40,0,90))
                .interpolateWith(InterpolationStyle.SMOOTH_START_TO_END).quickBuild();
        rig.hardware.drivetrain.setPosition(new MotionVector(ApexSimulation.FIELD_CENTER_INCHES-40,ApexSimulation.FIELD_CENTER_INCHES,0));
        double[] debt={.5,.5,.5,.5};
        double squaredV=0,squaredW=0,maxY=0;int samples=0;
        File directory=new File("build/path-power-graphs/measured-mecanum-experiment"); Files.createDirectories(directory.toPath());
        try(PrintWriter csv=new PrintWriter(new File(directory,"twist-prototype.csv"))) {
            csv.println("time_s,distance_in,target_v,actual_v,target_omega,actual_omega,target_heading_deg,actual_heading_deg,y_in");
            for(int tick=0;tick<320;tick++) {
                // A fixed time reference avoids trusting the old planner's incorrect
                // mixed-motion power envelope while evaluating the physical hypothesis.
                double[] ref=reference(tick*DT+.0075);
                double v=ref[1],omega=ref[3];
                double heading=rig.hardware.drivetrain.position.theta+omega*.0075;
                double c=Math.cos(heading),s=Math.sin(heading);
                rig.command(model.inverse(v*c,-v*s,omega,ref[2]*c,-ref[2]*s,ref[4]).pulse(debt));
                for(int sub=0;sub<4;sub++) ApexSimulation.stepPhysics(rig.hardware,.005);
                double endDistance=Math.max(0,Math.min(80,rig.hardware.drivetrain.position.x-ApexSimulation.FIELD_CENTER_INCHES+40));
                double[] endRef=reference((tick+1)*DT);
                double actualV=rig.hardware.drivetrain.velocity.x,actualW=rig.hardware.drivetrain.velocity.theta;
                double y=rig.hardware.drivetrain.position.y-ApexSimulation.FIELD_CENTER_INCHES;
                squaredV+=Math.pow(actualV-endRef[1],2); squaredW+=Math.pow(actualW-endRef[3],2);maxY=Math.max(maxY,Math.abs(y));samples++;
                csv.printf(Locale.US,"%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f%n",(tick+1)*DT,endDistance,endRef[1],actualV,
                        endRef[3],actualW,90*(3*Math.pow(endRef[0]/80,2)-2*Math.pow(endRef[0]/80,3)),
                        Math.toDegrees(rig.hardware.drivetrain.position.theta),y);
                if((tick+1)*DT>6 && Math.hypot(actualV,rig.hardware.drivetrain.velocity.y)<.1 && Math.abs(actualW)<.03) break;
            }
        }
        double endX=rig.hardware.drivetrain.position.x-ApexSimulation.FIELD_CENTER_INCHES;
        double heading=Math.toDegrees(rig.hardware.drivetrain.position.theta);
        System.out.printf(Locale.US,"COUPLED PROTOTYPE: vRMSE=%.6f omegaRMSE=%.6f maxY=%.6f endX=%.6f heading=%.6f samples=%d%n",
                Math.sqrt(squaredV/samples),Math.sqrt(squaredW/samples),maxY,endX,heading,samples);
        assertTrue("No-feedback translation acceptance",Math.sqrt(squaredV/samples)<1.5);
        assertTrue("No-feedback rotation-rate acceptance",Math.sqrt(squaredW/samples)<.15);
        assertTrue("No-feedback lateral drift acceptance",maxY<1);
        assertEquals("No-feedback heading acceptance",90,heading,3);
        assertEquals("No-feedback full-path acceptance",40,endX,2);
    }

    @Test public void plannedTwistFitsWheelAuthorityBetweenSolverNodes() {
        FollowerConstants c=SimulatorTuning.snapshot();
        GeometryFactory g=new GeometryFactory().setDistUnit(DistUnit.IN).setAngleUnit(AngleUnit.DEG);
        Path path=g.holonomicPath(g.pose(-40,0,0),g.pose(40,0,90))
                .interpolateWith(InterpolationStyle.SMOOTH_START_TO_END).quickBuild();
        FFLut lut=new FFLut(new MecanumProfileGenerator(c,path).generateRestToRest());
        feedforward.MecanumWheelModel model=c.motionCharacterization.mecanumWheelModel;
        feedforward.MecanumWheelModel.Demand d=new feedforward.MecanumWheelModel.Demand();
        for(int i=0;i<2000;i++) {
            MotionParameters m=lut.getKinematicParamsByTime(lut.getDurationSeconds()*i/2000);
            PathPoint point=path.getParametricPath().getPointAtDistance(m.getProgression());
            geometry.Vector end=path.getParametricPath().getFirstDerivative(1);
            double prime=path.getInterpolator().getHeadingFirstDerivative(point.getDistanceToEndIn(),point.getSignedCurvature(),end);
            double second=path.getInterpolator().getHeadingSecondDerivative(point.getDistanceToEndIn(),point.getCurvatureDerivative(),end);
            double v=m.getTangentialVel(),a=m.getTangentialAccel(),omega=prime*v,alpha=second*v*v+prime*a;
            double heading=path.getInterpolator().getHeadingTarg(point.getDistanceToEndIn(),point.getFirstDerivative(),end).getRad();
            double x=Math.cos(heading),y=-Math.sin(heading);
            model.solve(v*x,v*y,omega,a*x,a*y,alpha,d);
            assertTrue("Power budget between stored nodes",d.peak<=c.profilePowerLimit()+.002);
            heading+=omega*model.commandPreviewSeconds;x=Math.cos(heading);y=-Math.sin(heading);
            v+=a*model.commandPreviewSeconds;omega+=alpha*model.commandPreviewSeconds;
            model.solve(v*x,v*y,omega,a*x,a*y,alpha,d);
            assertTrue("Held-command preview must avoid clipping",d.insideDomain && d.peak<=1);
        }
    }

    private static double[] reference(double time) {
        double speed=16,accel=30,ramp=speed/accel,brake=80/speed,end=brake+ramp;
        double s,v,a;
        if(time<ramp) { s=.5*accel*time*time;v=accel*time;a=accel; }
        else if(time<brake) { s=.5*accel*ramp*ramp+speed*(time-ramp);v=speed;a=0; }
        else if(time<end) { double t=time-brake;s=80-.5*accel*ramp*ramp+speed*t-.5*accel*t*t;v=speed-accel*t;a=-accel; }
        else { s=80;v=a=0; }
        double p=s/80,prime=Math.PI/2*(6*p-6*p*p)/80,second=Math.PI/2*(6-12*p)/(80*80);
        return new double[]{s,v,a,prime*v,second*v*v+prime*a};
    }

    static double[] wheels(double x,double y,double turn,double radius) { return new double[]{x-y-turn*radius,x+y+turn*radius,x+y-turn*radius,x-y+turn*radius}; }
    private static double[] solve(double[][] a) {
        int n=a.length;
        for(int c=0;c<n;c++) {
            int p=c;for(int r=c+1;r<n;r++) if(Math.abs(a[r][c])>Math.abs(a[p][c]))p=r;
            double[] swap=a[c];a[c]=a[p];a[p]=swap;assertTrue(Math.abs(a[c][c])>1e-10);
            double scale=a[c][c];for(int i=c;i<=n;i++)a[c][i]/=scale;
            for(int r=0;r<n;r++)if(r!=c){double factor=a[r][c];for(int i=c;i<=n;i++)a[r][i]-=factor*a[c][i];}
        }
        double[] result=new double[n];for(int i=0;i<n;i++)result[i]=a[i][n];return result;
    }
}
