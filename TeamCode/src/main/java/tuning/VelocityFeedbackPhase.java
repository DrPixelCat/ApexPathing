package tuning;

import feedforward.MotionParameters;
import geometry.AngleUnit;
import geometry.DistUnit;
import geometry.GeometryFactory;
import geometry.PathSegment;
import geometry.Pose;
import geometry.Vector;
import paths.heading.InterpolationStyle;
import paths.movements.Path;
import paths.movements.Turn;

enum FeedbackAxis {
    TRANSLATION,
    ANGULAR
}

/**
 * @author Sohum Arora - 22985 Paraducks
 */
public class VelocityFeedbackPhase extends TuningPhase {
    private static final int SEARCH_ROUNDS = 4;

    private final TuningValues values;
    private final double[] gains = new double[3];
    private final double[] scores = new double[3];
    private Path[] straightTests;
    private Turn[] turnTests;
    private FeedbackAxis axis;
    private int leg;
    private int candidate;
    private int round;
    private double center;
    private double step;
    private double errorSquared;
    private int errorSamples;
    private double lastScore;
    private double translationScore;
    private double angularScore;
    private boolean complete;

    public VelocityFeedbackPhase(TunerContext context, TuningValues values) {
        super(context);
        this.values = values;
        complete = values.allPositive(values.translationFeedback, values.angularFeedback);
    }

    @Override
    protected String getPhaseName() {
        return "Velocity Feedback";
    }

    @Override
    protected boolean manualTuneIsPossible() {
        return true;
    }

    @Override
    protected boolean autoTuneIsPossible() {
        return true;
    }

    @Override
    public boolean isComplete() {
        return complete;
    }

    @Override
    protected void init() {
        complete = false;
        if (values.translation.kD > 0.0) {
            values.translationFeedback = values.translation.kD;
        }
        if (values.heading.kD > 0.0) {
            values.angularFeedback = values.heading.kD;
        }
        context.getFollower().setVelocityFeedbackTuning(values.translationFeedback, values.angularFeedback);
        buildTests();
        axis = FeedbackAxis.TRANSLATION;
        if (manualMode) {
            startTest();
        } else {
            startSearch(FeedbackAxis.TRANSLATION);
        }
    }

    private void buildTests() {
        GeometryFactory factory = new GeometryFactory(context.getFollower()).setDistUnit(DistUnit.IN)
                .setAngleUnit(AngleUnit.DEG);
        Pose start = factory.pose(0, 0, 0);
        Pose end = factory.pose(48, 0, 0);
        straightTests = new Path[]{
                factory.path(start, end).interpolateWith(InterpolationStyle.CONSTANT_START_HEADING)
                        .profiledBuild(),
                factory.path(end, start).interpolateWith(InterpolationStyle.CONSTANT_START_HEADING)
                        .profiledBuild()
        };

        Pose turned = factory.pose(0, 0, 90);
        turnTests = new Turn[]{
                factory.turn(start).turnTo(turned.getHeading()).profiledBuild(),
                factory.turn(turned).turnTo(start.getHeading()).profiledBuild()
        };
    }

    private void startSearch(FeedbackAxis nextAxis) {
        axis = nextAxis;
        center = axis == FeedbackAxis.TRANSLATION ? values.translationFeedback : values.angularFeedback;
        double feedforward = axis == FeedbackAxis.TRANSLATION ? values.translationKV : values.angularKV;
        step = Math.max(center * 0.5, Math.max(feedforward * 0.25, 0.00001));
        if (center <= 0.0) {
            center = step;
        }
        round = 0;
        startRound();
    }

    private void startRound() {
        gains[0] = Math.max(0.0, center - step);
        gains[1] = center;
        gains[2] = center + step;
        candidate = 0;
        startCandidate();
    }

    private void startCandidate() {
        if (axis == FeedbackAxis.TRANSLATION) {
            values.translationFeedback = gains[candidate];
        } else {
            values.angularFeedback = gains[candidate];
        }
        context.getFollower().setVelocityFeedbackTuning(values.translationFeedback, values.angularFeedback);
        startTest();
    }

    private void startTest() {
        context.getFollower().stop();
        context.getFollower().setPose(Pose.zero());
        leg = 0;
        errorSquared = 0.0;
        errorSamples = 0;
        if (axis == FeedbackAxis.TRANSLATION) {
            context.getFollower().follow(straightTests[0]);
        } else {
            context.getFollower().follow(turnTests[0]);
        }
    }

    private void sampleTest() {
        if (!context.getFollower().isBusy()) {
            return;
        }

        if (axis == FeedbackAxis.TRANSLATION) {
            Path path = straightTests[leg];
            PathSegment segment = path.getParametricPath();
            Vector current = context.getFollower().getPose().getVec();
            double t = segment.getBestT(current);
            Vector target = segment.getPosition(t);
            double remaining = segment.getDistanceToEndIn(target, t);
            double traveled = segment.getLengthIn() - remaining;
            MotionParameters desired = path.getFeedforwardLut().getFeedforwardParams(traveled);
            Vector tangent = segment.getFirstDerivative(t).normalize();
            double actual = context.getFollower().getVelocity().getVec().dot(tangent).getIn();
            addError(desired.getTangentialVel(), actual, 1.0);
        } else {
            Turn turn = turnTests[leg];
            double direction = Math.signum(turn.getStartPose().getHeading()
                    .getShortestAngleTo(turn.getEndPose().getHeading()).getRad());
            double traveled = turn.getStartPose().getHeading()
                    .getShortestAngleTo(context.getFollower().getPose().getHeading()).getRad() * direction;
            MotionParameters desired = turn.getFeedforwardLut().getFeedforwardParams(Math.max(0.0, traveled));
            double actual = context.getFollower().getVelocity().getHeading().getRad();
            addError(desired.getAngularVel(), actual, 0.05);
        }
    }

    private void addError(double target, double actual, double minimumTarget) {
        if (Math.abs(target) > minimumTarget) {
            double error = target - actual;
            errorSquared += error * error;
            errorSamples++;
        }
    }

    private boolean updateTest() {
        sampleTest();
        if (context.getFollower().isBusy()) {
            return false;
        }
        if (leg == 0) {
            leg = 1;
            if (axis == FeedbackAxis.TRANSLATION) {
                context.getFollower().follow(straightTests[1]);
            } else {
                context.getFollower().follow(turnTests[1]);
            }
            return false;
        }
        if (errorSamples == 0) {
            throw new IllegalStateException("No velocity feedback samples were recorded.");
        }
        lastScore = Math.sqrt(errorSquared / errorSamples);
        return true;
    }

    @Override
    protected void autoTuned() {
        if (!updateTest()) {
            return;
        }

        scores[candidate] = lastScore;
        candidate++;
        if (candidate < gains.length) {
            startCandidate();
            return;
        }

        int best = 0;
        for (int i = 1; i < scores.length; i++) {
            if (scores[i] < scores[best]) {
                best = i;
            }
        }

        center = gains[best];
        if (axis == FeedbackAxis.TRANSLATION) {
            translationScore = scores[best];
        } else {
            angularScore = scores[best];
        }

        round++;
        if (round < SEARCH_ROUNDS) {
            step *= 0.5;
            startRound();
            return;
        }

        if (axis == FeedbackAxis.TRANSLATION) {
            values.translationFeedback = center;
            context.getFollower().setVelocityFeedbackTuning(values.translationFeedback, values.angularFeedback);
            startSearch(FeedbackAxis.ANGULAR);
        } else {
            values.angularFeedback = center;
            values.saveFeedback(context);
            complete = true;
        }
    }

    @Override
    protected void manualTuned() {
        if (opMode.gamepad1.leftBumperWasPressed() || opMode.gamepad1.rightBumperWasPressed()) {
            axis = axis == FeedbackAxis.TRANSLATION ? FeedbackAxis.ANGULAR : FeedbackAxis.TRANSLATION;
            startTest();
        }

        double change = manualChange();
        if (change != 0.0) {
            if (axis == FeedbackAxis.TRANSLATION) {
                values.translationFeedback = Math.max(0.0, values.translationFeedback + change);
            } else {
                values.angularFeedback = Math.max(0.0, values.angularFeedback + change);
            }
            context.getFollower().setVelocityFeedbackTuning(values.translationFeedback, values.angularFeedback);
            startTest();
        } else if (opMode.gamepad1.xWasPressed()) {
            startTest();
        } else if (updateTest()) {
            if (axis == FeedbackAxis.TRANSLATION) {
                translationScore = lastScore;
            } else {
                angularScore = lastScore;
            }
            startTest();
        }

        context.getTelemetry().addData("Selected", axis.name());
        context.getTelemetry().addData("Translation feedback", values.translationFeedback);
        context.getTelemetry().addData("Angular feedback", values.angularFeedback);
        context.getTelemetry().addData("Translation RMS error", translationScore);
        context.getTelemetry().addData("Angular RMS error", angularScore);
        context.getTelemetry().addData("Increment", increment);
        context.getTelemetry().addLine("Up/Down: change value");
        context.getTelemetry().addLine("Left/Right: change increment");
        context.getTelemetry().addLine("LB/RB: select value");
        context.getTelemetry().addLine("X: restart test");
        context.getTelemetry().addLine("A: save");
        context.getTelemetry().update();

        if (opMode.gamepad1.aWasPressed()) {
            context.getFollower().stop();
            values.saveFeedback(context);
            complete = true;
        }
    }

    @Override
    protected void reportResults() {
        context.getTelemetry().addData("Translation feedback", values.translationFeedback);
        context.getTelemetry().addData("Translation RMS error", translationScore);
        context.getTelemetry().addData("Angular feedback", values.angularFeedback);
        context.getTelemetry().addData("Angular RMS error", angularScore);
    }
}
