package paths.callbacks;

import util.Angle;

public class AngleCallback implements Callback {
    private final Angle theta;
    private final Runnable action;
    private boolean triggered = false;

    public AngleCallback(Angle theta, Runnable action) {
        this.theta = theta; //TODO: normalize with new angle class
        this.action = action;
    }

    public Angle getTheta() {
        return theta;
    }

    @Override
    public Runnable getAction() {
        return action;
    }

    @Override
    public boolean isTriggered() { // Changed to public
        return triggered;
    }

    @Override
    public void setTriggered(boolean triggered) { // Changed to public
        this.triggered = triggered;
    }
}