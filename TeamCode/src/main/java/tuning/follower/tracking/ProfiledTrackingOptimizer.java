package tuning.follower.tracking;

import follower.FollowerConstants;

/**
 * Schedules measured, one-parameter-at-a-time trials on the same single-corner path.
 */
final class ProfiledTrackingOptimizer {
    enum Parameter {
        PROFILE_VELOCITY_POWER, CORNER_SPEED_SCALE, VELOCITY_GAIN
    }
    private enum Stage { BASELINE_FIRST, BASELINE_REPEAT, CANDIDATE, VALIDATION, COMPLETE }

    private final Parameter[] order;
    private final boolean tank;
    private final double velocityLimit;
    private final double translationKV;
    private final double maxPower;
    private final Settings original;
    private Settings best;
    private Settings candidate;
    private Stage stage = Stage.BASELINE_FIRST;
    private int parameterIndex;
    private int candidateNumber;
    private double firstScore;
    private double bestScore;
    private double baselineNoise;
    private double firstBias;
    private double brakingBias;
    private String completionNote = "Automatic calibration complete";

    ProfiledTrackingOptimizer(FollowerConstants constants, boolean tank, double maxPower) {
        this.tank = tank;
        order = constants.motionCharacterization == null ? Parameter.values()
                : new Parameter[]{Parameter.CORNER_SPEED_SCALE, Parameter.VELOCITY_GAIN};
        this.velocityLimit = constants.forwardVelLimitIn;
        this.translationKV = constants.translationalKV;
        this.maxPower = maxPower;
        original = Settings.capture(constants, tank);
        best = original.copy();
        candidate = best.copy();
    }

    Settings candidate() { return candidate; }
    Settings best() { return best; }
    boolean complete() { return stage == Stage.COMPLETE; }
    boolean isValidating() { return stage == Stage.VALIDATION; }
    Settings original() { return original.copy(); }
    boolean canRejectCurrentCandidate() { return stage == Stage.CANDIDATE; }
    int candidateNumber() { return candidateNumber; }
    int candidateLimit() { return order.length; }
    String status() {
        if (stage == Stage.BASELINE_FIRST || stage == Stage.BASELINE_REPEAT) {
            return "Baseline " + (stage == Stage.BASELINE_FIRST ? "1/2" : "2/2");
        }
        if (stage == Stage.CANDIDATE) {
            return "Candidate " + candidateNumber + "/" + candidateLimit()
                    + " " + order[parameterIndex] + " "
                    + (directionFor(order[parameterIndex]) < 0 ? "lower" : "higher");
        }
        return stage == Stage.VALIDATION ? "Validating selected values"
                : completionNote;
    }

    void record(double score, double bias) {
        if (complete() || !Double.isFinite(score) || score < 0.0
                || !Double.isFinite(bias)) {
            throw new IllegalArgumentException("Invalid profiled tracking trial score");
        }
        if (stage == Stage.BASELINE_FIRST) {
            firstScore = score;
            firstBias = bias;
            stage = Stage.BASELINE_REPEAT;
            return;
        }
        if (stage == Stage.BASELINE_REPEAT) {
            bestScore = 0.5 * (firstScore + score);
            baselineNoise = Math.abs(firstScore - score);
            brakingBias = 0.5 * (firstBias + bias);
            stage = Stage.CANDIDATE;
            parameterIndex = candidateNumber = 0;
            advanceCandidate();
            return;
        }
        if (stage == Stage.VALIDATION) {
            if (score > bestScore + improvementMargin(bestScore)) {
                best = original.copy();
                candidate = original.copy();
                completionNote = "Candidate not repeatable; saved measured baseline";
            }
            stage = Stage.COMPLETE;
            return;
        }
        if (score < bestScore - improvementMargin(bestScore)) {
            best = candidate.copy();
            bestScore = score;
            brakingBias = bias;
        }
        parameterIndex++;
        advanceCandidate();
    }

    private double improvementMargin(double total) {
        return Math.max(0.02 * Math.max(total, 1.0), baselineNoise);
    }

    void rejectCurrentCandidate() {
        if (stage != Stage.CANDIDATE) {
            throw new IllegalStateException("Only a candidate trial can be rejected");
        }
        parameterIndex++;
        advanceCandidate();
    }

    void rejectValidation() {
        if (stage != Stage.VALIDATION) {
            throw new IllegalStateException("Only the selected candidate can fail validation");
        }
        best = original.copy();
        candidate = original.copy();
        completionNote = "Validation failed; saved measured baseline";
        stage = Stage.COMPLETE;
    }

    private void advanceCandidate() {
        while (parameterIndex < order.length) {
            Settings proposal = best.copy();
            proposal.perturb(order[parameterIndex], directionFor(order[parameterIndex]),
                    velocityLimit, translationKV, maxPower);
            candidateNumber++;
            if (!proposal.sameAs(best)) {
                candidate = proposal;
                return;
            }
            parameterIndex++;
        }
        if (best.sameAs(original)) {
            // The original settings already have two independent measurements.
            stage = Stage.COMPLETE;
            return;
        }
        stage = Stage.VALIDATION;
        candidate = best.copy();
    }

    private double directionFor(Parameter parameter) {
        // Signed measured-minus-target braking bias picks the direction likely to reduce it.
        // Each proposal still has to improve the fixed-reference single-corner score.
        boolean fast = brakingBias > 0.0;
        switch (parameter) {
            case PROFILE_VELOCITY_POWER:
                return fast ? 1.0 : -1.0;
            case VELOCITY_GAIN:
                return 1.0;
            case CORNER_SPEED_SCALE:
                return fast ? -1.0 : 1.0;
            default: throw new AssertionError(parameter);
        }
    }

    /** Active-profile values only. This avoids touching the other mode of a dual-actuated drive. */
    static final class Settings {
        double velocityGain;
        double overspeedTolerance;
        double cornerTolerance;
        double pulsePower;
        int cooldown;
        double neutralMargin;
        double profileVelocityPower;
        double cornerSpeedScale;

        static Settings capture(FollowerConstants constants, boolean tank) {
            Settings values = new Settings();
            values.velocityGain = constants.velocityFeedbackGain;
            values.overspeedTolerance = constants.brakingOverspeedToleranceIn;
            values.cornerTolerance = tank ? constants.tankBrakingCornerToleranceIn
                    : constants.brakingCornerToleranceIn;
            values.pulsePower = tank ? constants.tankBrakingPulsePower
                    : constants.brakingPulsePower;
            values.cooldown = constants.brakingPulseCooldownCycles;
            values.neutralMargin = constants.brakingNeutralPowerMargin;
            values.profileVelocityPower = constants.profileVelocityPowerCoefficient;
            values.cornerSpeedScale = constants.profileCornerSpeedScale;
            return values;
        }

        void apply(FollowerConstants constants, boolean tank) {
            constants.velocityFeedbackGain = velocityGain;
            constants.brakingOverspeedToleranceIn = overspeedTolerance;
            if (tank) {
                constants.tankBrakingCornerToleranceIn = cornerTolerance;
                constants.tankBrakingPulsePower = pulsePower;
            } else {
                constants.brakingCornerToleranceIn = cornerTolerance;
                constants.brakingPulsePower = pulsePower;
            }
            constants.brakingPulseCooldownCycles = cooldown;
            constants.brakingNeutralPowerMargin = neutralMargin;
            constants.profileVelocityPowerCoefficient = profileVelocityPower;
            constants.profileCornerSpeedScale = cornerSpeedScale;
        }

        Settings copy() {
            Settings copy = new Settings();
            copy.velocityGain = velocityGain;
            copy.overspeedTolerance = overspeedTolerance;
            copy.cornerTolerance = cornerTolerance;
            copy.pulsePower = pulsePower;
            copy.cooldown = cooldown;
            copy.neutralMargin = neutralMargin;
            copy.profileVelocityPower = profileVelocityPower;
            copy.cornerSpeedScale = cornerSpeedScale;
            return copy;
        }

        boolean sameAs(Settings other) {
            return Double.compare(velocityGain, other.velocityGain) == 0
                    && Double.compare(overspeedTolerance, other.overspeedTolerance) == 0
                    && Double.compare(cornerTolerance, other.cornerTolerance) == 0
                    && Double.compare(pulsePower, other.pulsePower) == 0
                    && cooldown == other.cooldown
                    && Double.compare(neutralMargin, other.neutralMargin) == 0
                    && Double.compare(profileVelocityPower, other.profileVelocityPower) == 0
                    && Double.compare(cornerSpeedScale, other.cornerSpeedScale) == 0;
        }

        void perturb(Parameter parameter, double direction, double velocityLimit,
                     double translationKV, double maxPower) {
            switch (parameter) {
                case VELOCITY_GAIN:
                    velocityGain = clip(velocityGain + direction
                            * Math.max(0.25 * velocityGain, translationKV), 0.0,
                            FollowerConstants.MAX_TRANSLATION_VELOCITY_FEEDBACK_GAIN);
                    break;
                case PROFILE_VELOCITY_POWER:
                    double effective = profileVelocityPower > 0.0
                            ? profileVelocityPower : translationKV;
                    profileVelocityPower = Math.max(0.0,
                            effective * (direction > 0.0 ? 2.0 : 0.5));
                    break;
                case CORNER_SPEED_SCALE:
                    cornerSpeedScale = clip(cornerSpeedScale + direction * 0.2, 0.01, 1.0);
                    break;
                default: throw new AssertionError(parameter);
            }
        }

        private static double clip(double value, double low, double high) {
            return Math.max(low, Math.min(high, value));
        }
    }
}
