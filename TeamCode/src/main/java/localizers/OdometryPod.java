package localizers;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Helper class for 2/3 wheel odometry and drive encoder localizers.
 *
 * @author Topher F. - 23571 alum
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public class OdometryPod {
    private final String name;

    private final double ticksPerInch;
    private final DcMotorEx odometry;
    private final double direction;

    private int lastTicks;
    private int currentTicks;
    private int resetTicks;
    private double deltaTicks;

    public OdometryPod(HardwareMap hardwareMap, String name, double ticksPerInch) {
        this(hardwareMap, name, ticksPerInch, false);
    }

    public OdometryPod(HardwareMap hardwareMap, String name, double ticksPerInch, boolean reversed) {
        if (!Double.isFinite(ticksPerInch) || ticksPerInch <= 0) {
            throw new IllegalArgumentException("Encoder ticks per inch must be finite and positive");
        }
        this.name = name;
        this.odometry = hardwareMap.get(DcMotorEx.class, this.name);
        this.ticksPerInch = ticksPerInch;
        this.direction = reversed ? -1 : 1;
        reset();
    }

    public String getName() { return this.name; }

    public void update() {
        currentTicks = odometry.getCurrentPosition();
        deltaTicks = currentTicks - lastTicks;
        lastTicks = currentTicks;
    }

    public void reset() {
        // A drive encoder can share its motor with propulsion. Rebase the software
        // reading without changing that motor's run mode or its hardware counter.
        currentTicks = odometry.getCurrentPosition();
        lastTicks = resetTicks = currentTicks;
        deltaTicks = 0;
    }

    /** @return the amount of inches the encoder has moved since the last reset */
    public double getInches() { return direction * (currentTicks - resetTicks) / ticksPerInch; }

    /** @return the amount of inches the encoder has moved since the last loop. */
    public double getDeltaInches() { return direction * deltaTicks / ticksPerInch; }

    public int getTicks() { return currentTicks; }
}

