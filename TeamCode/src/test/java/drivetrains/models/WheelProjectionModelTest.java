package drivetrains.models;

import follower.FollowerConstants;

import org.json.JSONObject;
import org.junit.Test;
import paths.builders.HolonomicPathBuilder;
import paths.geometry.Angle;
import paths.geometry.GeometryFactory;
import paths.geometry.PathPoint;
import paths.geometry.Vector;
import paths.heading.InterpolationStyle;
import paths.movements.Path;
import trajectory.profile.MotionParameters;
import static org.junit.Assert.*;

public class WheelProjectionModelTest {
    private WheelProjectionModel wheels(int n,double offset) {
        double[] x=new double[n],y=new double[n];
        for(int i=0;i<n;i++) { x[i]=-Math.sin(offset+i*2*Math.PI/n); y[i]=Math.cos(offset+i*2*Math.PI/n); }
        return new WheelProjectionModel(x,y,1);
    }
    @Test public void exactDemandIncludesSignedRotationForAllUniformLayouts() {
        java.util.Random random=new java.util.Random(6482);
        for(int n:new int[]{3,4,5,6,8,17,37}) for(double offset:new double[]{0,.4,Math.PI/2}) {
            WheelProjectionModel model=wheels(n,offset);
            for(int j=0;j<200;j++) {
                double x=4*random.nextDouble()-2,y=4*random.nextDouble()-2,turn=2*random.nextDouble()-1,peak=0;
                for(int i=0;i<n;i++) {
                    double theta=offset+i*2*Math.PI/n;
                    peak=Math.max(peak,Math.abs(-Math.sin(theta)*x+Math.cos(theta)*y+turn));
                }
                assertEquals(peak,model.demand(x,y,turn),1e-12);
            }
        }
        assertEquals(.75,wheels(3,Math.PI/2).demand(1,0,.25),1e-12);
    }
    @Test public void projectionGeometryIsImmutableAndRejectsInvalidInputs() {
        double[] x={-1,.5,.5},y={0,-Math.sqrt(3)/2,Math.sqrt(3)/2};
        WheelProjectionModel model=new WheelProjectionModel(x,y,1);
        x[0]=999; y[0]=999; model.getForwardProjections()[0]=999;
        assertEquals(.75,model.demand(1,0,.25),1e-12);
        assertThrows(IllegalArgumentException.class,()->new WheelProjectionModel(new double[2],new double[2],1));
        assertThrows(IllegalArgumentException.class,()->new WheelProjectionModel(new double[]{1,0,Double.NaN},new double[3],1));
    }
    @Test public void generatedCurvedOmniProfilesMatchIndependentWheelPowerReconstruction() throws Exception {
        FollowerConstants c=FollowerConstants.fromJson(new JSONObject().put("drivetrainType","OMNI"));
        c.translationalKV=.01; c.translationalKA=.004; c.translationalFeedforwardKS=.03;
        c.angularKV=.1; c.angularKA=.02; c.angularFeedforwardKS=.02; c.kCentripetal=.001;
        c.forwardVelLimitIn=c.strafeVelLimitIn=60; c.forwardAccelLimitIn=c.strafeAccelLimitIn=100;
        c.angularVelLimitRad=8; c.angularAccelLimitRad=20;
        GeometryFactory g=new GeometryFactory();
        for(int n:new int[]{3,5,6,8}) {
            double offset=.3;
            Path path=new HolonomicPathBuilder(g.pose(0,0,15),g.arcPose(30,0,7),g.pose(30,40,80))
                    .interpolateWith(InterpolationStyle.SMOOTH_START_TO_END)
                    .setDriveModel(new HolonomicDriveModel(c,wheels(n,offset))).quickBuild();
            for(MotionParameters state:path.createGenerator().generateRestToRest()) {
                PathPoint point=path.getParametricPath().getPointAtDistance(state.getProgression());
                Angle heading=path.getInterpolator().getHeadingTarg(point.getDistanceToEndIn(),point.getFirstDerivative(),
                        path.getParametricPath().getFirstDerivative(1));
                Vector body=point.getFirstDerivative().normalize().rotate(heading.times(-1));
                double v=state.getTangentialVel(),a=state.getTangentialAccel(),omega=state.getAngularVel(),alpha=state.getAngularAccel();
                double tangent=.01*v+.004*a+.03*sign(v,a),normal=.001*point.getSignedCurvature()*v*v;
                double x=body.getX().getIn()*tangent-body.getY().getIn()*normal;
                double y=body.getY().getIn()*tangent+body.getX().getIn()*normal;
                double turn=.1*omega+.02*alpha+.02*sign(omega,alpha),peak=0;
                for(int i=0;i<n;i++) {
                    double theta=offset+i*2*Math.PI/n;
                    peak=Math.max(peak,Math.abs(-Math.sin(theta)*x+Math.cos(theta)*y+turn));
                }
                assertEquals(peak,state.getMotorPower(),1e-8);
                assertTrue(peak<=1+1e-6);
            }
        }
    }
    private static double sign(double v,double a) {
        return Math.abs(v)>1e-6?Math.signum(v):Math.abs(a)>1e-6?Math.signum(a):0;
    }
}
