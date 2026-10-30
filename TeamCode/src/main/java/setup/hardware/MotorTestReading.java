package setup.hardware;

/** One selected-motor sample while the operator holds the test control. */
public final class MotorTestReading {
    public final double power, velocityTicksPerSecond, batteryVolts;
    public final int ticks;

    public MotorTestReading(double power, int ticks, double velocityTicksPerSecond, double batteryVolts) {
        this.power = power; this.ticks = ticks;
        this.velocityTicksPerSecond = velocityTicksPerSecond; this.batteryVolts = batteryVolts;
    }
}
