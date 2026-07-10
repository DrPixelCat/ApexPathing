package tuning;

import geometry.Pose;
import paths.builders.Builder;
import paths.heading.HolonomicInterpolationStyle;
import paths.movements.Path;
import util.AngleUnit;
import util.DistUnit;
import util.PoseFactory;

// TODO: We gotta figure out how to measure lateral error and also add stuff for the other axes
public class MovementLimitsPhase extends TuningPhase {
    private final TuningPhase.BinarySearch binarySearch;
    private boolean forwardPathRunning;
    private double maxLateralError; // Inches
    private double maxLateralAccel; // Inches per second squared

    private Path forwardCurve;
    private Path backwardCurve;

    public MovementLimitsPhase(TunerContext context) {
        super(context);
        binarySearch = new TuningPhase.BinarySearch(10, 200, 5);
    }

    @Override
    protected String getPhaseName() { return "Max Lateral Acceleration"; }

    @Override
    protected boolean manualTuneIsPossible() { return true; }

    @Override
    protected boolean autoTuneIsPossible() { return true; }

    @Override
    protected void init() {
        forwardPathRunning = true;
        maxLateralError = 0;

        PoseFactory poseFactory = new PoseFactory(DistUnit.IN, AngleUnit.DEG);
        Pose startPose = context.getFollower().getPose();
        Pose midPose = startPose.plus(poseFactory.of(40, 20));
        Pose endPose = startPose.plus(poseFactory.of(60, 0));
        // TODO: Don't hardcode holonomicPath once the new path builder is implemented
        forwardCurve = Builder.holonomicPath(startPose, midPose, endPose)
                .interpolateWith(HolonomicInterpolationStyle.TANGENT_FORWARD)
                .quickBuild();
        backwardCurve = Builder.holonomicPath(endPose, midPose, startPose)
                .interpolateWith(HolonomicInterpolationStyle.TANGENT_FORWARD)
                .quickBuild();

        maxLateralAccel = binarySearch.getGuess();
        context.getFollower().getConstants().strafeAccelLimitIn = maxLateralAccel;
        context.getFollower().follow(forwardCurve);
    }

    @Override
    protected boolean manualUpdate() {
        return false; // TODO: Implement manual tuning
    }

    @Override
    protected boolean automaticUpdate() {
        if (!context.getFollower().isBusy()) {
            if (forwardPathRunning) {
                context.getFollower().follow(backwardCurve);
            } else {
                // TODO: Adjust this threshold based on testing
                boolean converged = binarySearch.updateGuess(maxLateralError > 4.0);
                maxLateralAccel = binarySearch.getGuess();
                if (converged) {
                    context.constants.strafeAccelLimitIn = maxLateralAccel;
                    return true;
                }

                context.getFollower().getConstants().strafeAccelLimitIn = maxLateralAccel;
                maxLateralError = 0;
                context.getFollower().follow(forwardCurve);
            }
            forwardPathRunning = !forwardPathRunning;
        }

        // TODO: Add an actual lateral error calculation and replace 0
        maxLateralError = Math.max(maxLateralError, 0);
        return false;
    }

    @Override
    protected void reportResults() {
        context.getTelemetry().addData("Max Lateral Acceleration (in/s^2)", maxLateralAccel);
    }
}
