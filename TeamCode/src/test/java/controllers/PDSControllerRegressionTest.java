package controllers;

import org.junit.Test;
import static org.junit.Assert.*;

public class PDSControllerRegressionTest {
    @Test public void angularDerivativeUsesTheShortDifferenceAcrossTheErrorWrap() throws Exception {
        PDSController controller=new PDSController(new PDSController.PDSCoefficients(0,1,0));
        controller.setAngularController();
        assertEquals(0,controller.calculate(Math.PI-.01),0);
        java.lang.reflect.Field time=PDSController.class.getDeclaredField("lastTimestamp");
        time.setAccessible(true); time.setLong(controller,System.nanoTime()-20_000_000L);
        double output=controller.calculate(-Math.PI+.01);
        assertTrue("A small angular change must not create a full-revolution derivative kick: "+output,
                output>0 && output<2);
    }
}
