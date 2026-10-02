package feedforward;

import org.junit.Test;
import static org.junit.Assert.*;

public class CoupledMotionProfileTest {
    @Test public void mandatorySignedDecelerationIsPreservedInBothPasses() {
        double[] profile = CoupledMotionProfile.solve(new double[]{0,1,2,3},new double[]{3,3,3,3},false,0,
                (i,q,ceiling) -> {
                    if(i==1) {
                        double next=.6*q;
                        return next <= ceiling+1e-12 ? next : Double.NaN;
                    }
                    double acceleration=i==0?1:.3;
                    return q-2*acceleration>ceiling+1e-12?Double.NaN:Math.min(ceiling,q+2*acceleration);
                });
        assertEquals(0,profile[0],0);
        assertEquals(1,profile[1],1e-6);
        assertEquals(Math.sqrt(.6),profile[2],1e-6);
        assertEquals(0,profile[3],0);
        assertEquals(-.2,(profile[2]*profile[2]-profile[1]*profile[1])/2,1e-6);
    }
    @Test public void rejectsInvalidSampleDomains() {
        CoupledMotionProfile.Transitions transitions=(i,q,ceiling)->ceiling;
        assertThrows(IllegalArgumentException.class,()->CoupledMotionProfile.solve(new double[]{0},new double[]{1},false,0,transitions));
        assertThrows(IllegalArgumentException.class,()->CoupledMotionProfile.solve(new double[]{1,0},new double[]{1,1},false,0,transitions));
        assertThrows(IllegalArgumentException.class,()->CoupledMotionProfile.solve(new double[]{0,1},new double[]{1,Double.NaN},false,0,transitions));
    }
}
