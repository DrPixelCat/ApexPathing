package localizers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 *  This is the helper class for the two and three wheel odometry localizers
 *
 *  @author Topher F. - 23571 alumni
 */
public class OdometryPod {
    private String name;

    /**
     * This will be in the form ticks/inch
     */
    private double conversionToInches;
    private DcMotorEx odometry;

    /**
     * This is used to calculate where we were last loop
     */
    private int tickCountLastLoop;

    private int currentTicks;
    private double deltaTicks;

    public OdometryPod(HardwareMap hardwareMap, String name) {
        this.name = name;
        this.odometry = hardwareMap.get(DcMotorEx.class, this.name);
        this.conversionToInches = 1.0;
    }

    public OdometryPod(HardwareMap hardwareMap, String name, double conversionToInches) {
        this.name = name;
        this.odometry = hardwareMap.get(DcMotorEx.class, this.name);
        this.conversionToInches = conversionToInches;
    }

    public void setConversionToInches(double conversionToInches) {
        this.conversionToInches = conversionToInches;
    }

    public void update() {
        currentTicks = odometry.getCurrentPosition();
        deltaTicks = currentTicks - tickCountLastLoop;
        tickCountLastLoop = currentTicks;
    }

    /**
     * @return Overall ticks travelled
     */
    public double getTicks() {
        return currentTicks;
    }

    /**
     * @return Overall inches travelled
     */
    public double getInches() {
        return currentTicks / conversionToInches;
    }

    public String getName() {
        return this.name;
    }

    public void resetEncoder() {
        tickCountLastLoop = 0;
        odometry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * @return Ticks this loop
     */
    public double getDeltaTicks() {
        return deltaTicks;
    }

    /**
     * @return Ticks this loop
     */
    public double getDeltaInches() {
        return deltaTicks / conversionToInches;
    }
}

