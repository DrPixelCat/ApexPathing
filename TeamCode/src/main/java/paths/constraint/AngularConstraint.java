package paths.constraint;

import paths.geometry.Angle;

/**
 * A constraint that applies to the robot's angular movement along a path, such as angular velocity
 * or angular acceleration.
 */
public class AngularConstraint implements PathConstraint {
    private double s;
    private final Type type;

    private final double value_rad;
    private final java.util.List<Runnable> changeListeners = new java.util.ArrayList<>();

    public AngularConstraint(double s, Type type, Angle value) {
        this.s = s;
        this.type = type;
        this.value_rad = value.getRad();
    }

    @Override
    public double getS() { return s; }

    @Override
    public void setS(double s) {
        this.s = s;
        for (Runnable listener : changeListeners) { listener.run(); }
    }

    @Override public boolean addChangeListener(Runnable listener) {
        if (getClass() != AngularConstraint.class) { return false; }
        changeListeners.add(listener); return true;
    }

    @Override
    public Type getType() { return type; }

    @SuppressWarnings("PublicMethodNotExposedInInterface")
    public double getValueRad() { return value_rad; }
    @Override public Domain getDomain() { return Domain.ANGULAR; }
    @Override public double getLimit() { return getValueRad(); }
    @Override public PathConstraint copyForTrajectory() {
        return new AngularConstraint(getS(), getType(), Angle.fromRad(getLimit()));
    }
}
