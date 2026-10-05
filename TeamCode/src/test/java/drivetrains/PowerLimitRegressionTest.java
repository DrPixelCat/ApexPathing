package drivetrains;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.junit.Test;
import static org.junit.Assert.*;

public class PowerLimitRegressionTest {
    private static final class Constants extends BaseDrivetrainConstants<Constants> {
        @Override public BaseDrivetrain<Constants> build(HardwareMap map) { return null; }
    }
    @Test public void preservesValidFractionalLimitsAndClampsBothEnds() {
        Constants constants = new Constants();
        assertEquals(.3, constants.setMaxPower(.3).maxPower, 0);
        assertEquals(0, constants.setMaxPower(-1).maxPower, 0);
        assertEquals(1, constants.setMaxPower(2).maxPower, 0);
    }
    @Test(expected = IllegalArgumentException.class)
    public void rejectsNonfinitePower() { new Constants().setMaxPower(Double.NaN); }
}
