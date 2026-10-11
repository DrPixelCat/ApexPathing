package drivetrains.models;

import feedforward.MotionPowerCalculator;
import feedforward.MotionCharacterization;
import feedforward.MotorPowerModel;
import follower.FollowerConstants;
import geometry.*;
import org.json.JSONObject;
import org.junit.Test;
import paths.builders.HolonomicPathBuilder;
import paths.heading.InterpolationStyle;
import paths.movements.Path;
import trajectory.generators.PathProfileGenerator;
import trajectory.profile.MotionParameters;
import static org.junit.Assert.*;

public class DriveModelIntegrationTest {
    private FollowerConstants constants(boolean calibrated) throws Exception {
        FollowerConstants c=FollowerConstants.fromJson(new JSONObject().put("drivetrainType","MECANUM"));
        c.translationalKV=.01; c.translationalKA=.004; c.translationalFeedforwardKS=.03;
        c.angularKV=.1; c.angularKA=.02; c.angularFeedforwardKS=.02; c.kCentripetal=.001;
        c.forwardVelLimitIn=c.strafeVelLimitIn=60; c.forwardAccelLimitIn=c.strafeAccelLimitIn=100;
        c.angularVelLimitRad=8; c.angularAccelLimitRad=20;
        if(calibrated) {
            MotorPowerModel[][] axes=new MotorPowerModel[3][2];
            for(MotionCharacterization.Axis axis:MotionCharacterization.Axis.values()) for(int dir=0;dir<2;dir++)
                axes[axis.ordinal()][dir]=new MotorPowerModel(200+dir*30,-1,20,.1,
                        axis==MotionCharacterization.Axis.ANGULAR?10:60,0);
            c.motionCharacterization=new MotionCharacterization(axes,true);
        }
        return c;
    }
    @Test public void holonomicPlanningAndExecutionShareSignedReferencePower() throws Exception {
        java.util.Random random=new java.util.Random(8741);
        for(boolean calibrated:new boolean[]{false,true}) {
            FollowerConstants c=constants(calibrated);
            for(DriveModel model:new DriveModel[]{DriveModels.mecanum(c),DriveModels.swerve(c),DriveModels.omni(c)}) {
                for(int i=0;i<300;i++) {
                    double theta=random.nextDouble()*2*Math.PI;
                    Vector body=Vector.of(Math.cos(theta),Math.sin(theta),DistUnit.IN);
                    double v=random.nextDouble()*100-50,a=random.nextDouble()*160-80;
                    double omega=random.nextDouble()*6-3,alpha=random.nextDouble()*20-10,curvature=random.nextDouble()*.08-.04;
                    MotionPowerCalculator.Output power=model.powerCalculator().calculate(body,v,a,omega,alpha,curvature*v*v);
                    double execution=model.commandDemand(power.translation().plus(power.normal()),power.heading);
                    assertEquals(execution,model.powerEvaluator(body,curvature).utilization(v,a,omega,alpha,v),1e-12);
                }
            }
        }
    }
    @Test public void tankPlanningReservesBothCalibratedTravelDirections() throws Exception {
        for(boolean calibrated:new boolean[]{false,true}) {
            FollowerConstants c=constants(calibrated);
            DriveModel model=DriveModels.tank(c);
            for(double v:new double[]{0,10,40}) for(double a:new double[]{-40,0,40}) {
                Vector body=Vector.of(1,0,DistUnit.IN);
                double positive=model.powerCalculator().calculate(body,v,a,-1,2,0).forward;
                double negative=model.powerCalculator().calculate(body,-v,-a,-1,2,0).forward;
                double turn=model.powerCalculator().calculate(body,v,a,-1,2,0).heading;
                assertEquals(Math.max(Math.abs(positive),Math.abs(negative))+Math.abs(turn),
                        model.powerEvaluator(Vector.of(1,0,DistUnit.IN),.1).utilization(v,a,-1,2,v),1e-12);
            }
        }
    }
    @Test public void pathBindsSettingsAndCircularGeometryForReversalAndStoredProfiles() throws Exception {
        FollowerConstants c=constants(false);
        double[] x={0,-Math.sqrt(3)/2,Math.sqrt(3)/2},y={1,-.5,-.5};
        DriveModel model=new HolonomicDriveModel(c,HolonomicDriveModel.Metric.OMNI,x,y);
        GeometryFactory g=new GeometryFactory();
        Path path=new HolonomicPathBuilder(g.pose(0,0,0),g.pose(80,0,0))
                .setDriveModel(model).interpolateWith(InterpolationStyle.CONSTANT_START_HEADING).buildToMemory();
        double expected=Math.sqrt(3)/2;
        x[1]=999; y[0]=999;
        c.translationalKV=.5; c.forwardVelLimitIn=1;
        assertEquals(.01,path.getPlanningConstants().translationalKV,0);
        assertEquals(expected,path.createGenerator().getDriveModel().translationDemand(1,0),1e-12);
        assertEquals(expected,path.reversed().createGenerator().getDriveModel().translationDemand(1,0),1e-12);
        Object stored=path.getTrajectoryField();
        assertSame(stored,path.getTrajectoryField());
        FollowerConstants detached=path.getPlanningConstants(); detached.translationalKV=.9;
        assertEquals(.01,path.createGenerator().getDriveModel().getConstants().translationalKV,0);
    }
    @Test public void generatedPowerUsesTheSameEvaluatorAsReachability() throws Exception {
        FollowerConstants c=constants(true);
        GeometryFactory g=new GeometryFactory();
        for(HolonomicDriveModel.Metric metric:HolonomicDriveModel.Metric.values()) {
            Path path=new HolonomicPathBuilder(g.pose(0,0,15),g.arcPose(30,0,7),g.pose(30,40,80))
                    .interpolateWith(InterpolationStyle.SMOOTH_START_TO_END)
                    .setDriveModel(new HolonomicDriveModel(c,metric)).quickBuild();
            PathProfileGenerator generator=path.createGenerator();
            MotionParameters[] profile=generator.generateRestToRest();
            for(MotionParameters state:profile) {
                PathPoint point=path.getParametricPath().getPointAtDistance(state.getProgression());
                Vector end=path.getParametricPath().getFirstDerivative(1);
                Angle heading=path.getInterpolator().getHeadingTarg(point.getDistanceToEndIn(),point.getFirstDerivative(),end);
                Vector body=point.getFirstDerivative().normalize().rotate(heading.times(-1));
                double expected=generator.getDriveModel().powerEvaluator(body,point.getSignedCurvature()).utilization(
                        state.getTangentialVel(),state.getTangentialAccel(),state.getAngularVel(),state.getAngularAccel(),state.getTangentialVel());
                assertEquals(expected,state.getMotorPower(),1e-8);
                assertTrue(state.getMotorPower()<=generator.getProfilePowerLimit()+1e-6);
            }
        }
    }
    @Test public void explicitProfileVelocityReserveDoesNotAlterRuntimeFeedforward() throws Exception {
        FollowerConstants c=constants(false); c.profileVelocityPowerCoefficient=.02;
        DriveModel model=DriveModels.mecanum(c);
        Vector body=Vector.of(1,0,DistUnit.IN);
        assertEquals(.43,model.powerEvaluator(body,0).utilization(20,0,0,0,20),1e-12);
        assertEquals(.23,model.powerCalculator().calculate(body,20,0,0,0,0).forward,1e-12);
    }
    @Test public void runtimeAndPlanningKeepDistinctNormalPowerPolicies() throws Exception {
        FollowerConstants c=constants(false); c.kCentripetal=.1;
        MotionPowerCalculator calculator=DriveModels.mecanum(c).powerCalculator();
        Vector body=Vector.of(0,1,DistUnit.IN);
        MotionPowerCalculator.Output runtime=calculator.calculate(body,10,0,0,0,-20);
        assertEquals(.65,runtime.normalForward,1e-12);
        assertEquals(.13,runtime.strafe,1e-12);
        MotionPowerCalculator.Output planning=new MotionPowerCalculator.Output();
        calculator.calculate(0,1,10,0,0,0,-20,true,1,planning);
        assertEquals(2,planning.normalForward,1e-12);
        assertEquals(2.13,planning.demand,1e-12);
    }
    @Test public void reusableOutputsPreserveSignedPowerAndCallerStaticPolicy() throws Exception {
        FollowerConstants c=constants(false);
        MotionPowerCalculator calculator=DriveModels.mecanum(c).powerCalculator();
        MotionPowerCalculator.Output first=new MotionPowerCalculator.Output(), second=new MotionPowerCalculator.Output();
        calculator.calculate(.6,.8,-20,5,-1,2,4,false,-1,first);
        assertEquals(-.126,first.forward,1e-12);
        assertEquals(-.168,first.strafe,1e-12);
        assertEquals(-.08,first.heading,1e-12);
        calculator.calculate(1,0,0,0,0,0,0,false,-1,second);
        assertEquals(-.03,second.forward,1e-12);
        assertEquals(-.126,first.forward,1e-12);
        calculator.calculate(1,0,0,0,0,0,0,false,0,first);
        assertEquals(0,first.demand,0);
        assertEquals(0,first.normalForward,0);
        assertEquals(0,first.normalStrafe,0);
    }
    @Test public void customMutableModelDefaultsToLiveGeometryEvaluation() throws Exception {
        FollowerConstants c=constants(false);
        double[] ceiling={12};
        GeometryFactory g=new GeometryFactory();
        Path path=new HolonomicPathBuilder(g.pose(),g.pose(100,0)).setDriveModel(new MutableModel(c,ceiling)).quickBuild();
        PathProfileGenerator generator=path.createGenerator();
        assertTrue(peakVelocity(generator.generateRestToRest())>10);
        ceiling[0]=3;
        assertEquals(3,peakVelocity(generator.generateRestToRest()),1e-9);
    }
    private static double peakVelocity(MotionParameters[] states) {
        double peak=0; for(MotionParameters state:states) peak=Math.max(peak,state.getTangentialVel()); return peak;
    }
    private static final class MutableModel extends DriveModel {
        private final double[] ceiling;
        private final DriveModel delegate;
        MutableModel(FollowerConstants c,double[] ceiling) { super(c);this.ceiling=ceiling;delegate=DriveModels.swerve(c); }
        @Override public DriveModel withConstants(FollowerConstants c) { return new MutableModel(c,ceiling); }
        @Override public boolean isHolonomic() { return true; }
        @Override public double maximumVelocity(Vector body,double curvature,double prime,double angular) {
            return Math.min(ceiling[0],delegate.maximumVelocity(body,curvature,prime,angular));
        }
        @Override public double maximumTangentialAcceleration(Vector body) { return delegate.maximumTangentialAcceleration(body); }
        @Override public PowerEvaluator powerEvaluator(Vector body,double curvature) { return delegate.powerEvaluator(body,curvature); }
        @Override public double translationDemand(double x,double y) { return delegate.translationDemand(x,y); }
    }
    @Test public void pathRejectsAConflictingModelMode() throws Exception {
        FollowerConstants c=constants(false);
        GeometryFactory g=new GeometryFactory();
        assertThrows(IllegalArgumentException.class,()->g.holonomicPath(g.pose(),g.pose(30,0)).setDriveModel(DriveModels.tank(c)));
        assertThrows(IllegalArgumentException.class,()->g.tankPath(g.pose(),g.pose(30,0)).setDriveModel(DriveModels.omni(c)));
        Path path=g.holonomicPath(g.pose(),g.pose(30,0)).quickBuild();
        assertThrows(IllegalArgumentException.class,()->new PathProfileGenerator(path,DriveModels.tank(c)));
    }
}
