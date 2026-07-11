package tuning;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

/**
 * One bounded value exposed by a manual tuning phase.
 * @author Sohum Arora 22985 Paraducks
 */
public class TuneValue {
    private final String name;
    private final DoubleSupplier getter;
    private final DoubleConsumer setter;
    private final double increment;
    private final double minimum;
    private final double maximum;

    public TuneValue(String name, DoubleSupplier getter, DoubleConsumer setter, double increment, double minimum, double maximum) {
        this.name = name;
        this.getter = getter;
        this.setter = setter;
        this.increment = increment;
        this.minimum = minimum;
        this.maximum = maximum;
    }

    public String getName() { return name; }

    public double get() { return getter.getAsDouble(); }

    public void adjust(int direction, double scale) {
        double next = get() + (direction * increment * scale);
        setter.accept(Math.max(minimum, Math.min(maximum, next)));
    }
}
