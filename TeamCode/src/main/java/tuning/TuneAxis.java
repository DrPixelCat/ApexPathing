package tuning;

import geometry.Pose;

/** Robot axes characterized independently by the tuner. */
public enum TuneAxis {
    FORWARD("Forward", 0.50, 60.0),
    STRAFE("Strafe", 0.50, 60.0),
    ANGULAR("Angular", 0.04, Math.PI * 1.5);

    private final String label;
    private final double minSpeed;
    private final double maxTravel;

    TuneAxis(String label, double minSpeed,
                         double maxTravel) {
        this.label = label;
        this.minSpeed = minSpeed;
        this.maxTravel = maxTravel;
    }

    public String label() { return label; }

    public double minSpeed() { return minSpeed; }

    public double maxTravel() { return maxTravel; }

    public double position(Pose pose) {
        switch (this) {
            case FORWARD: return pose.getX().getIn();
            case STRAFE: return pose.getY().getIn();
            case ANGULAR: return pose.getHeading().getRad();
            default: throw new IllegalStateException("Unhandled characterization axis");
        }
    }

    public double velocity(Pose pose) { return position(pose); }
}
