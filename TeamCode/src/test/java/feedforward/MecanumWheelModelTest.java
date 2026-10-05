package feedforward;

import org.junit.Test;
import static org.junit.Assert.*;

public class MecanumWheelModelTest {
    private MecanumWheelModel model() {
        return new MecanumWheelModel(100,1,3,2,8,16,2,3,40,.01);
    }
    @Test public void finiteValidatedCalibrationRoundTripsAndRejectsInvalidDomain() {
        MecanumWheelModel restored=MecanumWheelModel.fromJson(model().toJson());
        assertEquals(3,restored.regenerativeBraking,0);
        assertEquals(2,restored.neutralBraking,0);
        assertEquals(40,restored.maximumWheelSpeed,0);
        assertThrows(IllegalArgumentException.class,()->new MecanumWheelModel(100,3,3,2,8,16,2,3,40,0));
        assertThrows(IllegalArgumentException.class,()->new MecanumWheelModel(100,1,3,2,8,16,2,3,40,Double.NaN));
        assertThrows(IllegalArgumentException.class,()->new MecanumWheelModel(100,1,3,2,8,16,2,3,40,.05));
        MecanumWheelModel.Demand d=new MecanumWheelModel.Demand();
        restored.solve(39,0,1,0,0,0,d);
        assertFalse(d.insideDomain);assertEquals(Double.POSITIVE_INFINITY,d.peak,0);
    }
    @Test public void tinyLateralDirectionDoesNotAddASecondFrictionOffset() {
        MecanumWheelModel model=model();
        MecanumWheelModel.Demand a=new MecanumWheelModel.Demand(),b=new MecanumWheelModel.Demand();
        model.solve(16,0,.47,0,0,0,a);
        model.solve(16,-.000279,.47,0,0,0,b);
        for(int i=0;i<4;i++) assertEquals(a.mean(i),b.mean(i),1e-5);
        model.solve(16,0,.47,0,0,-.001,b);
        for(int i=0;i<4;i++) assertEquals("Braking onset is continuous in chassis force",a.mean(i),b.mean(i),1e-4);
    }
    @Test public void analyticBoundsMatchTheInverseAcrossDirectionsAndBraking() {
        MecanumWheelModel model=model();
        MecanumWheelModel.Demand demand=new MecanumWheelModel.Demand();
        int feasible=0;
        for(int deg=0;deg<360;deg+=15) for(double v:new double[]{4,13,20})
            for(double prime:new double[]{-.04,0,.03}) for(double second:new double[]{-.003,0,.002}) {
                double x=Math.cos(Math.toRadians(deg)),y=Math.sin(Math.toRadians(deg));
                double a=model.maximumAcceleration(x,y,.01,v,prime,second,-60,60,.7);
                if(Double.isNaN(a))continue;
                assertTrue(Double.isFinite(a));feasible++;
                model.solve(x*v,y*v,prime*v,x*a-y*.01*v*v,y*a+x*.01*v*v,second*v*v+prime*a,demand);
                assertTrue("Analytic bound must be feasible",demand.peak<=.700001);
                if(a<59.999) {
                    double next=a+1e-3;
                    model.solve(x*v,y*v,prime*v,x*next-y*.01*v*v,y*next+x*.01*v*v,second*v*v+prime*next,demand);
                    assertTrue("The next acceleration must exceed the power budget",demand.peak>.7);
                }
            }
        assertTrue(feasible>500);
    }
    @Test public void pulseDebtAveragesTheDiscontinuousWheelResponseAndRemainsPerExecutor() {
        MecanumWheelModel.Demand d=new MecanumWheelModel.Demand();
        model().solve(16,0,0,-8,0,0,d);
        assertTrue(d.opposingDuty[0]>0 && d.opposingDuty[0]<1);
        double[] debt={.5,.5,.5,.5},power=new double[4];int opposite=0;
        for(int i=0;i<1000;i++) { d.pulse(debt,power);if(power[0]<0)opposite++; }
        assertEquals(1000*d.opposingDuty[0],opposite,1);
        double[] fresh={.5,.5,.5,.5},first=new double[4],again=new double[4];
        d.pulse(fresh,first);d.pulse(new double[]{.5,.5,.5,.5},again);
        assertArrayEquals(first,again,0);
    }
    @Test public void saturatedCorrectionsPreserveChassisForceRatiosAcrossWheelRegimes() {
        MecanumWheelModel model=model(); MecanumWheelModel.Demand d=new MecanumWheelModel.Demand();
        double vx=16,vy=-5,omega=.5,ax=-35,ay=20,alpha=50;
        double scale=model.solveBounded(vx,vy,omega,ax,ay,alpha,.7,d);
        assertTrue(scale>0 && scale<1);assertTrue(d.peak<=.700000001);
        double fx=0,fy=0,torque=0;
        for(int i=0;i<4;i++) {
            int ys=i==0 || i==3?-1:1,ts=i==0 || i==2?-1:1;
            double w=vx+ys*vy+ts*model.effectiveRadius*omega;
            double force=response(model,w,d.aligned[i])*(1-d.opposingDuty[i])
                    +response(model,w,d.opposing[i])*d.opposingDuty[i];
            fx+=force/4;fy+=ys*force/4;torque+=ts*force/(4*model.effectiveRadius);
        }
        double drag=model.dragMagnitude(vx,vy),mag=Math.hypot(vx,vy);
        assertEquals(scale*(ax+drag*vx/mag),fx,1e-8);
        assertEquals(scale*(ay+drag*vy/mag),fy,1e-8);
        assertEquals(scale*(alpha+model.angularDrag),torque,1e-8);
        assertThrows(IllegalArgumentException.class,()->model.solveBounded(0,0,0,0,0,0,Double.NaN,d));
    }
    private static double response(MecanumWheelModel m,double w,double power) {
        if(power==0)return -m.neutralBraking*w;
        return power*w>=0?m.gain*power-m.speedLoss*w*Math.abs(power):m.gain*power-m.regenerativeBraking*w;
    }
}
