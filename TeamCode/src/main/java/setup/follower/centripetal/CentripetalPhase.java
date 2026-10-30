package setup.follower.centripetal;

import setup.follower.limits.LimitsPhase;

import setup.follower.TunerContext;
import setup.follower.TuningPhase;

import setup.logging.TuningCsvWriter;

import com.qualcomm.robotcore.util.ElapsedTime;

import paths.geometry.AngleUnit;
import paths.geometry.Dist;
import paths.geometry.DistUnit;
import paths.geometry.GeometryFactory;
import paths.geometry.PathPoint;
import paths.geometry.Pose;
import paths.constraint.PathConstraint;
import paths.constraint.TranslationalConstraint;
import paths.builders.PathBuilder;
import paths.heading.InterpolationStyle;
import paths.movements.Path;

/**
 * Runs the robot on a forward and backward arc to measure the cross-track error. The error is used
 * to tune the centripetal gain with a binary search. The user can also manually tune the gain.
 *
 * @author Sohum Arora - 22985 Paraducks
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public class CentripetalPhase extends TuningPhase {
    private static final double LEG_TIMEOUT_SECONDS = 25.0;
    private static final double TURNAROUND_PROGRESS = 0.985;
    private static final double TURNAROUND_DISTANCE_INCHES = 1.5;
    private static final double TURNAROUND_SPEED_INCHES_PER_SECOND = 6.0;
    private static final double TEST_TOTAL_POWER_FRACTION = 0.75;
    private static final double TEST_CENTRIPETAL_POWER_FRACTION = 0.20;
    private static final double TEST_ARC_RADIUS_INCHES = 32.0;
    private static final int TEST_ARC_POINT_COUNT = 7;

    private enum Leg { IDLE, OUTBOUND, RETURN }

    private BinarySearch search;
    private Path forwardArc;
    private Path backwardArc;

    private Leg leg = Leg.IDLE;
    private double errorSum;
    private int samples;
    private double averageError;
    private int trialNumber;
    private TuningCsvWriter manualCsv;
    private String manualCsvPath = "Not started";
    private final ElapsedTime legTimer = new ElapsedTime();

    public CentripetalPhase(TunerContext context) { super(context); }

    @Override
    protected String getPhaseName() { return "Centripetal"; }

    @Override protected String getTuningConstants() { return "kCentripetal"; }

    @Override
    protected boolean manualTuneIsPossible() { return true; }

    @Override
    protected boolean autoTuneIsPossible() { return true; }

    @Override
    protected void showPreRunInstructions() {
        context.getView().addLine(
                "Place the robot at one corner of a clear 32 by 32 inch test area.");
    }

    @Override
    protected void init() {
        GeometryFactory factory = new GeometryFactory(context.getFollower()).setDistUnit(DistUnit.IN)
                .setAngleUnit(AngleUnit.DEG);

        // Center the complete 32x32 test footprint on the field, not merely its starting point.
        Pose[] forwardPoints = createTestArc(factory);
        Path geometry = factory.holonomicPath(forwardPoints).quickBuild();
        // Sampled-circle splines do not have exactly 0/90-degree endpoint tangents.
        // Keep this experiment purely tangent-facing: a terminal heading blend would
        // introduce characterized strafe feedforward and contaminate the lateral test.
        forwardPoints[0] = new Pose(forwardPoints[0].getVec(),
                geometry.getParametricPath().getFirstDerivative(0).getTheta());
        int last = forwardPoints.length - 1;
        forwardPoints[last] = new Pose(forwardPoints[last].getVec(),
                geometry.getParametricPath().getFirstDerivative(1).getTheta());
        Pose start = forwardPoints[0];
        if (Boolean.getBoolean("apex.simulation.unlockTunerPhases")) {
            positionRobotForSimulation(start);
        } else {
            context.getFollower().setPose(start);
        }
        double fullStrafeAcceleration = context.constants.strafeAccelLimitIn /
                LimitsPhase.MARGIN_MULTIPLIER;
        double nominalGain = 1.0 / fullStrafeAcceleration;
        double seed = context.constants.kCentripetal > 0.0 ?
                context.constants.kCentripetal : nominalGain;
        double lower = 0.25 * nominalGain;
        double upper = 1.25 * nominalGain;

        double testVelocity = safeTestVelocity(geometry, upper,
                context.constants.forwardVelLimitIn, context.constants.translationalKV,
                context.constants.translationalFeedforwardKS);
        double savedGain = context.constants.kCentripetal;
        context.constants.kCentripetal = upper;
        try {
            PathBuilder<?> forwardBuilder = factory.holonomicPath(forwardPoints)
                    .interpolateWith(InterpolationStyle.TANGENT_FORWARD);
            forwardBuilder.addConstraint(new TranslationalConstraint(0.0,
                    PathConstraint.Type.VELOCITY, Dist.fromIn(testVelocity)));
            forwardArc = forwardBuilder.quickBuild();
            backwardArc = forwardArc.reversed();
        } finally {
            // Restore the seed after constructing the test geometry; each online traversal
            // plans against the candidate being evaluated.
            context.constants.kCentripetal = savedGain;
        }

        search = new BinarySearch(lower, upper, (upper - lower) / 64.0);
        context.constants.kCentripetal = manualMode ? seed : search.current();

        if (manualMode) {
            context.getFollower().setCentripetal(context.constants.kCentripetal);
            context.getFollower().stop();
            leg = Leg.IDLE;
            trialNumber = 0;
            manualCsv = TuningCsvWriter.open("manual_centripetal_response",
                    "trial", "time_s", "gain", "direction", "path_t", "signed_error_in");
            manualCsvPath = manualCsv.getPath();
        } else {
            startTrial();
        }
    }

    private void startTrial() {
        context.getFollower().stop();
        errorSum = 0;
        samples = 0;
        averageError = 0;

        context.getFollower().setCentripetal(context.constants.kCentripetal);
        trialNumber++;
        startLeg(Leg.OUTBOUND);
    }

    private void startLeg(Leg nextLeg) {
        leg = nextLeg;
        context.getFollower().follow(nextLeg == Leg.OUTBOUND ? forwardArc : backwardArc);
        legTimer.reset();
    }

    private boolean trialRunning() { return leg != Leg.IDLE; }

    private boolean outbound() { return leg == Leg.OUTBOUND; }

    private void sampleError() {
        double t = context.getFollower().getBestT();
        if (t <= 0.25 || t >= 0.75) {
            return;
        }

        double error = context.getFollower().getCentripetalErrorIn();
        if (!Double.isFinite(error)) {
            return;
        }
        errorSum += error;
        samples++;
        if (manualMode && manualCsv != null) {
            manualCsv.writeRow(trialNumber, legTimer.seconds(),
                    context.constants.kCentripetal,
                    outbound() ? "OUTBOUND" : "RETURN", t, error);
        }
    }

    private boolean updateTrial() {
        if (context.getFollower().isBusy()) {
            sampleError();
            if (readyForTurnaround()) {
                if (context.getFollower().getMovementOutcome() == follower.Follower.MovementOutcome.FAILED) {
            throw new IllegalStateException(context.getFollower().getTrajectoryFailure());
        }
        return finishLeg();
            }
            if (legTimer.seconds() > LEG_TIMEOUT_SECONDS) {
                context.getFollower().stop();
                throw new IllegalStateException(
                        "Centripetal " + (outbound() ? "outbound" : "return") +
                                " arc timed out at pose " + context.getFollower().getPose() +
                                ", t=" + context.getFollower().getBestT() +
                                ", velocity=" + context.getFollower().getVelocity() +
                                ". Verify endpoint tolerances, localization, and feedback constants."
                );
            }
            return false;
        }
        return finishLeg();
    }

    private boolean readyForTurnaround() {
        Pose endpoint = outbound() ? forwardArc.getEndPose() : backwardArc.getEndPose();
        double endpointDistance = context.getFollower().getPose().getVec()
                .distanceTo(endpoint.getVec()).getIn();
        double speed = context.getFollower().getVelocity().getVec().getMag().getIn();
        return readyForTurnaround(context.getFollower().getBestT(), endpointDistance, speed);
    }

    static boolean readyForTurnaround(double progress, double endpointDistance, double speed) {
        return Double.isFinite(progress) && Double.isFinite(endpointDistance) &&
                Double.isFinite(speed) && progress >= TURNAROUND_PROGRESS &&
                endpointDistance <= TURNAROUND_DISTANCE_INCHES &&
                speed <= TURNAROUND_SPEED_INCHES_PER_SECOND;
    }

    private boolean finishLeg() {
        // The general follower completion gate may still be waiting on heading settling. End this
        // phase-local leg explicitly so the matching reverse arc can be queued immediately.
        if (context.getFollower().isBusy()) { context.getFollower().stop(); }
        if (outbound()) {
            startLeg(Leg.RETURN);
            return false;
        }

        if (samples == 0) {
            throw new IllegalStateException(
                    "Centripetal trial completed without usable middle-arc samples. " +
                            "Verify localization and path tracking before tuning kCentripetal."
            );
        }
        averageError = errorSum / samples;
        leg = Leg.IDLE;
        return true;
    }

    @Override
    protected boolean autoTuned() {
        if (!updateTrial()) {
            return false;
        }

        if (search.hasConverged()) { return true; }
        search.advance(searchDirection(averageError));
        context.constants.kCentripetal = search.current();
        context.getFollower().setCentripetal(context.constants.kCentripetal);
        startTrial(); // Measure the final narrowed candidate before accepting it.

        return false;
    }

    /** Builds a nearly constant-curvature, 32-inch-radius quarter circle. */
    static Pose[] createTestArc(GeometryFactory factory) {
        Pose[] points = new Pose[TEST_ARC_POINT_COUNT];
        for (int i = 0; i < points.length; i++) {
            double fraction = i / (double) (points.length - 1);
            double angle = -Math.PI / 2.0 + fraction * Math.PI / 2.0;
            double x = -16.0 + TEST_ARC_RADIUS_INCHES * Math.cos(angle);
            double y = 16.0 + TEST_ARC_RADIUS_INCHES * Math.sin(angle);
            points[i] = factory.pose(x, y, fraction * 90.0);
        }
        return points;
    }

    /** Chooses a speed with explicit budgets for total and centripetal motor power. */
    static double safeTestVelocity(Path path, double maximumGain, double velocityLimit,
                                   double translationalKV, double staticPower) {
        double maximumCurvature = 0.0;
        for (PathPoint point : path.getGeneratedPoints()) {
            if (point.getT() < 0.20 || point.getT() > 0.80) { continue; }
            double curvature = Math.abs(point.getSignedCurvature());
            if (Double.isFinite(curvature)) {
                maximumCurvature = Math.max(maximumCurvature, curvature);
            }
        }
        double lateralCoefficient = maximumCurvature * Math.max(0.0, maximumGain);
        double centripetalLimited = lateralCoefficient > 1e-9
                ? Math.sqrt(TEST_CENTRIPETAL_POWER_FRACTION / lateralCoefficient)
                : Double.POSITIVE_INFINITY;

        double availableDynamicPower = Math.max(0.0,
                TEST_TOTAL_POWER_FRACTION - Math.abs(staticPower));
        double totalPowerLimited;
        double linearCoefficient = Math.abs(translationalKV);
        if (lateralCoefficient > 1e-9) {
            double discriminant = linearCoefficient * linearCoefficient +
                    4.0 * lateralCoefficient * availableDynamicPower;
            totalPowerLimited = (-linearCoefficient +
                    Math.sqrt(Math.max(0.0, discriminant))) /
                    (2.0 * lateralCoefficient);
        } else if (linearCoefficient > 1e-9) {
            totalPowerLimited = availableDynamicPower / linearCoefficient;
        } else {
            totalPowerLimited = velocityLimit;
        }

        double selected = Math.min(0.75 * velocityLimit,
                Math.min(centripetalLimited, totalPowerLimited));
        return Double.isFinite(selected) ? Math.max(0.0, selected) : 0.25 * velocityLimit;
    }

    /** Positive outward error needs more centripetal gain; inward error needs less. */
    static BinarySearch.SearchDirection searchDirection(double signedError) {
        return signedError > 0.0
                ? BinarySearch.SearchDirection.HIGHER
                : BinarySearch.SearchDirection.LOWER;
    }

    @Override
    protected boolean manualTuned() {
        double change = trialRunning() ? 0.0 : manualChange();
        if (change != 0.0) {
            context.constants.kCentripetal = Math.max(0.0, context.constants.kCentripetal + change);
            context.getFollower().setCentripetal(context.constants.kCentripetal);
        }

        if (opMode.gamepad1.xWasPressed() && !trialRunning()) {
            startTrial();
        } else if (trialRunning() && updateTrial()) {
            context.getFollower().stop();
        }

        reportResults();
        context.getView().addData("Increment", number(increment.get()));
        if (context.isDebugMode()) {
            context.getView().addData("Test state", trialRunning()
                    ? (outbound() ? "OUTBOUND" : "RETURN")
                    : "IDLE - press X to run");
            context.getView().addData("Usable samples", samples);
            context.getView().addData("Response CSV", manualCsvPath);
        }
        context.getView().addLine("Dpad Up/Down: change centripetal gain while idle");
        context.getView().addLine("X: run arc test");
        context.getView().addLine("A: save");
        if (confirmPressed()) {
            context.getFollower().stop();
            return true;
        }
        return false;
    }

    @Override
    protected void reportResults() {
        context.getView().addData("Centripetal Gain", number(context.constants.kCentripetal));
        if (manualMode) {
            context.getView().addData("Mean signed error",
                    number(trialRunning() && samples > 0 ? errorSum / samples : averageError));
        }
    }

    @Override
    protected boolean routineMotionActive() { return trialRunning(); }

    @Override protected void onExit() {
        if (manualCsv != null) { manualCsv.close(); }
    }
}
