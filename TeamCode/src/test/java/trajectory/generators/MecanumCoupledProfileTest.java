package trajectory.generators;

import trajectory.generators.MecanumProfileGenerator;

import follower.FollowerConstants;
import feedforward.MotionCharacterization;
import feedforward.MotorPowerModel;
import drivetrains.Mecanum;
import trajectory.profile.MotionParameters;
import geometry.*;
import org.json.JSONObject;
import org.junit.Test;
import org.junit.Before;
import org.junit.After;
import storage.ApexStorage;
import paths.builders.HolonomicPathBuilder;
import paths.constraint.*;
import paths.heading.InterpolationStyle;
import paths.movements.Path;
import java.util.Random;
import static org.junit.Assert.*;

/** Independent state and wheel-command checks for the reported mecanum failures. */
public class MecanumCoupledProfileTest {
    private String previousStorage;
    @Before public void desktopStorage() {
        previousStorage = System.getProperty(ApexStorage.DIRECTORY_PROPERTY);
        System.setProperty(ApexStorage.DIRECTORY_PROPERTY, new java.io.File("build/coupled-profile-tests").getAbsolutePath());
    }
    @After public void restoreStorage() {
        if (previousStorage == null) { System.clearProperty(ApexStorage.DIRECTORY_PROPERTY); }
        else { System.setProperty(ApexStorage.DIRECTORY_PROPERTY, previousStorage); }
    }
    private final GeometryFactory g = new GeometryFactory().setDistUnit(DistUnit.IN).setAngleUnit(AngleUnit.DEG);
    private FollowerConstants constants() throws Exception {
        FollowerConstants c = FollowerConstants.fromJson(new JSONObject().put("drivetrainType", "MECANUM"));
        c.forwardVelLimitIn = 72.5; c.forwardAccelLimitIn = 62.5;
        c.strafeVelLimitIn = 58; c.strafeAccelLimitIn = 50;
        c.angularVelLimitRad = Math.toRadians(291); c.angularAccelLimitRad = Math.toRadians(2000);
        c.translationalKV = .0135; c.translationalKA = .0026;
        c.angularKV = .011; c.angularKA = .0021; c.kCentripetal = .001;
        return c;
    }
    @Test public void facingPointRespectsAccelerationAtTheActualSample() throws Exception {
        Path p = new HolonomicPathBuilder(g.pose(0,-50,90), g.pose(0,50,0))
                .interpolateWith(InterpolationStyle.FACING_POINT, g.vector(10,0)).quickBuild();
        verify(constants(), p);
    }
    @Test public void tightHeadingAccelerationAllowsCancellationWithoutViolations() throws Exception {
        FollowerConstants c = constants(); c.angularAccelLimitRad = 1;
        Path p = new HolonomicPathBuilder(g.pose(0,0,0), g.pose(100,0,180))
                .interpolateWith(InterpolationStyle.SMOOTH_START_TO_END).quickBuild();
        verify(c, p);
    }
    @Test public void straightPhysicalAccelerationEnvelopeIsMaximal() throws Exception {
        FollowerConstants c = constants(); c.translationalKV = .001; c.translationalKA = .001;
        c.forwardAccelLimitIn = 10;
        Path p = new HolonomicPathBuilder(g.pose(0,0,0),g.pose(40,0,0))
                .interpolateWith(InterpolationStyle.CONSTANT_START_HEADING).quickBuild();
        MotionParameters[] profile = verify(c, p);
        double length = p.getParametricPath().getLengthIn();
        for (MotionParameters state : profile) {
            double expected = Math.sqrt(20 * Math.min(state.getProgression(), length - state.getProgression()));
            assertEquals("Analytic maximal sampled rest-to-rest envelope", expected, state.getTangentialVel(), 1e-5);
        }
    }
    @Test public void powerSaturationMatchesFourWheelMixerAcrossHeadings() throws Exception {
        FollowerConstants c = constants();
        c.forwardVelLimitIn = c.strafeVelLimitIn = 150;
        c.forwardAccelLimitIn = c.strafeAccelLimitIn = 100;
        for (double heading : new double[]{0,25,45,70,90,140}) {
            Path p = new HolonomicPathBuilder(g.pose(0,0,heading),g.pose(250,0,heading))
                    .interpolateWith(InterpolationStyle.CONSTANT_START_HEADING).setProfilePowerLimit(.7).quickBuild();
            double max = 0;
            for (MotionParameters state : verify(c,p)) { max = Math.max(max,state.getMotorPower()); }
            assertEquals("No extra directional power tax at heading " + heading,.7,max,1e-6);
        }
    }
    @Test public void authoredLimitsUseLatestProgressionEvenWhenAddedOutOfOrder() throws Exception {
        FollowerConstants c = constants();
        Path p = new HolonomicPathBuilder(g.pose(0,0,0),g.pose(150,0,0))
                .interpolateWith(InterpolationStyle.CONSTANT_START_HEADING)
                .addConstraint(new TranslationalConstraint(.5, PathConstraint.Type.ACCELERATION, Dist.fromIn(8)))
                .addConstraint(new TranslationalConstraint(0, PathConstraint.Type.ACCELERATION, Dist.fromIn(20)))
                .addConstraint(new TranslationalConstraint(.6, PathConstraint.Type.VELOCITY, Dist.fromIn(18)))
                .addConstraint(new AngularConstraint(0, PathConstraint.Type.ACCELERATION, Angle.fromRad(.8)))
                .quickBuild();
        verify(c,p); verify(c,p.reversed());
    }
    @Test public void curvedAndChangingHeadingCasesRespectLegacyAndMeasuredBranches() throws Exception {
        Random random = new Random(1902);
        for (int i=0;i<18;i++) {
            FollowerConstants c=constants();
            c.angularAccelLimitRad = .7+random.nextDouble()*5;
            c.translationalFeedforwardKS=.04; c.angularFeedforwardKS=.03;
            if(i%2==0) {
                MotorPowerModel[][] models=new MotorPowerModel[3][2];
                for(MotionCharacterization.Axis axis:MotionCharacterization.Axis.values()) {
                    for(int d=0;d<2;d++) {
                        models[axis.ordinal()][d]=axis==MotionCharacterization.Axis.ANGULAR
                                ?new MotorPowerModel(20+d*2,0,1,2,5,0)
                                :new MotorPowerModel(180+d*10,-.5,7,1.2,65,0);
                    }
                }
                c.motionCharacterization=new MotionCharacterization(models,true);
            }
            Path p=new HolonomicPathBuilder(g.pose(0,0,random.nextDouble()*180),
                    g.arcPose(45,0,15),g.pose(45,55,random.nextDouble()*180))
                    .interpolateWith(InterpolationStyle.SMOOTH_START_TO_END)
                    .setProfilePowerLimit(.75).quickBuild();
            verify(c,p);
        }
    }
    private MotionParameters[] verify(FollowerConstants c, Path p) {
        MecanumProfileGenerator generator=new MecanumProfileGenerator(c,p);
        MotionParameters[] profile=generator.generate();
        PathPoint[] points=p.getGeneratedPoints(); Vector end=p.getParametricPath().getFirstDerivative(1);
        Mecanum.DirectionalLut limits=new Mecanum.DirectionalLut(c.forwardVelLimitIn,c.forwardAccelLimitIn,c.strafeVelLimitIn,c.strafeAccelLimitIn);
        for(int i=0;i<profile.length;i++) {
            MotionParameters state=profile[i]; PathPoint point=points[i];
            Angle heading=p.getInterpolator().getHeadingTarg(point.getDistanceToEndIn(),point.getFirstDerivative(),end);
            Vector body=point.getFirstDerivative().normalize().rotate(heading.times(-1));
            Mecanum.DirectionalLut.DirectionalKinematics direction=limits.getKinematics(point.getFirstDerivative(),heading);
            double v=state.getTangentialVel(),a=state.getTangentialAccel();
            double progress=p.constraintProgress(state.getProgression()/p.getParametricPath().getLengthIn());
            double maxV=direction.maxVel,maxA=direction.maxAccel,maxOmega=c.angularVelLimitRad,maxAlpha=c.angularAccelLimitRad;
            double[] latest={-1,-1,-1,-1};
            for(PathConstraint constraint:p.getConstraints()) {
                int type=constraint instanceof TranslationalConstraint?0:2;
                if(constraint.getType()==PathConstraint.Type.ACCELERATION) type++;
                if(constraint.getS()<=progress+1e-12 && constraint.getS()>=latest[type]) {
                    latest[type]=constraint.getS();
                    if(type==0) maxV=Math.min(direction.maxVel,((TranslationalConstraint)constraint).getValueIn());
                    if(type==1) maxA=Math.min(direction.maxAccel,((TranslationalConstraint)constraint).getValueIn());
                    if(type==2) maxOmega=Math.min(c.angularVelLimitRad,((AngularConstraint)constraint).getValueRad());
                    if(type==3) maxAlpha=Math.min(c.angularAccelLimitRad,((AngularConstraint)constraint).getValueRad());
                }
            }
            assertTrue("velocity at "+i,v<=maxV+1e-6);
            assertTrue("acceleration at "+i,Math.abs(a)<=maxA+1e-6);
            assertTrue("omega at "+i,Math.abs(state.getAngularVel())<=maxOmega+1e-6);
            assertTrue("alpha at "+i,Math.abs(state.getAngularAccel())<=maxAlpha+1e-6);
            if(i<profile.length-1) {
                double ds=profile[i+1].getProgression()-state.getProgression();
                assertEquals((profile[i+1].getTangentialVel()*profile[i+1].getTangentialVel()-v*v)/(2*ds),a,1e-7);
            }
            double prime=p.getInterpolator().getHeadingFirstDerivative(point.getDistanceToEndIn(),point.getSignedCurvature(),end);
            double second=p.getInterpolator().getHeadingSecondDerivative(point.getDistanceToEndIn(),point.getCurvatureDerivative(),end);
            assertEquals(prime*v,state.getAngularVel(),1e-8);
            assertEquals(second*v*v+prime*a,state.getAngularAccel(),1e-8);
            double scalar=v*c.translationalKV+a*c.translationalKA+sign(v,a)*c.translationalFeedforwardKS;
            Vector tangent=c.characterizedTranslation(body,v,a,body.times(scalar));
            double lateral=v*v*point.getSignedCurvature()*c.kCentripetal;
            double x=tangent.getX().getIn()-body.getY().getIn()*lateral;
            double y=tangent.getY().getIn()+body.getX().getIn()*lateral;
            double turn=c.characterizedPower(MotionCharacterization.Axis.ANGULAR,state.getAngularVel(),state.getAngularAccel(),
                    state.getAngularVel()*c.angularKV+state.getAngularAccel()*c.angularKA+sign(state.getAngularVel(),state.getAngularAccel())*c.angularFeedforwardKS);
            double wheelPeak=Math.max(Math.max(Math.abs(x-y-turn),Math.abs(x+y+turn)),Math.max(Math.abs(x+y-turn),Math.abs(x-y+turn)));
            assertEquals("Planner agrees with four independent wheel commands at "+i,wheelPeak,state.getMotorPower(),1e-8);
            assertTrue("power at "+i,wheelPeak<=generator.getProfilePowerLimit()+1e-6);
        }
        assertEquals(0,profile[0].getTangentialVel(),0);
        assertEquals(0,profile[profile.length-1].getTangentialVel(),0);
        assertTrue(generator.getLastDebugReport().powerFeasible);
        assertTrue(generator.getLastDebugReport().kinematicsFeasible);
        return profile;
    }
    private static double sign(double v,double a) {return Math.abs(v)>1e-6?Math.signum(v):Math.abs(a)>1e-6?Math.signum(a):0;}
}
