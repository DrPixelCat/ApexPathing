package paths.callbacks;

import com.qualcomm.robotcore.util.Range;

public class DistanceCallback implements Callback {
    private final double s;
    private final Runnable action;
    private boolean triggered = false;

    public DistanceCallback(double s, Runnable action) {
        this.s = Range.clip(s, 0.0, 1.0);
        this.action = action;
    }

    public double getS() {
        return s;
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