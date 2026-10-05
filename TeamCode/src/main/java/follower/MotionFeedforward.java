package follower;

import com.qualcomm.robotcore.util.Range;

/** Pure feedforward calculations shared by holonomic and tank path execution. */
final class MotionFeedforward {
    private MotionFeedforward() { }

    static double motionSign(double targetVelocity, double targetAcceleration) {
        return feedforward.HolonomicFeedforward.motionSign(targetVelocity, targetAcceleration);
    }

    static double timeScale(double targetTangentialVelocity, double measuredTangentialVelocity) {
        if (!Double.isFinite(targetTangentialVelocity)
                || !Double.isFinite(measuredTangentialVelocity)
                || targetTangentialVelocity <= 1e-6) {
            return 0.0;
        }
        return Range.clip(measuredTangentialVelocity / targetTangentialVelocity, 0.0, 1.0);
    }

    /**
     * Road Runner applies velocity feedback before motor feedforward. Apex stores that feedback
     * in power units, so kV * (v + feedback / kV) is exactly kV * v + feedback. Only the static
     * term's direction needs the corrected velocity; keep the split terms for Apex's allocator
     * and diagnostics. Acceleration remains the generated, signed reference acceleration.
     * At rest retain Apex's acceleration-directed breakaway, matching its profile power model.
     */
    static double profiledStaticSign(double targetVelocity, double targetAcceleration,
                                    double feedbackPower, double kV) {
        double commandedVelocity = kV > 1e-9
                ? targetVelocity + feedbackPower / kV : targetVelocity;
        return motionSign(commandedVelocity, targetAcceleration);
    }

    static double scaleBrakingAcceleration(double targetVelocity, double targetAcceleration,
                                           double measuredVelocity) {
        if (targetVelocity * targetAcceleration >= 0.0) {
            return targetAcceleration;
        }
        double scale = timeScale(Math.abs(targetVelocity),
                measuredVelocity * Math.signum(targetVelocity));
        return targetAcceleration * scale * scale;
    }
}
