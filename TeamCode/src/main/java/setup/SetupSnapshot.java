package setup;

import java.util.LinkedHashMap;
import java.util.Map;

/** One coherent hardware update. Translation is field-frame inches, heading is radians. */
public final class SetupSnapshot {
    public double[] pose = new double[3], rawPose = new double[3];
    public double[] velocity = new double[3], rawVelocity = new double[3];
    public double[] acceleration = new double[3], rawAcceleration = new double[3];
    /** Directed wheel travel in inches; forward, [right parallel], strafe. */
    public double[] pods = new double[0];
    public int[] ticks = new int[0];
    public double[] noise = new double[6]; // x R/Q, y R/Q, heading R/Q
    public final Map<String, Double> diagnostics = new LinkedHashMap<>();
    public double battery = Double.NaN;
    public boolean valid = true;
}
