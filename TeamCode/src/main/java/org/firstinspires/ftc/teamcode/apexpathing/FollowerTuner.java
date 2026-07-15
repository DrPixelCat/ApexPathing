package org.firstinspires.ftc.teamcode.apexpathing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import controllers.PDSController;
import controllers.PDSController.PDSCoefficients;
import core.Follower;
import feedforward.MotionParameters;
import geometry.Angle;
import geometry.AngleUnit;
import geometry.DistUnit;
import geometry.GeometryFactory;
import geometry.PathSegment;
import geometry.Pose;
import geometry.Vector;
import paths.heading.InterpolationStyle;
import paths.movements.Path;
import paths.movements.Turn;
import tuning.TunerContext;
import tuning.TuningPhase;

@Configurable
@TeleOp(name = "Follower Tuner", group = "Apex Pathing Tuning")
public class FollowerTuner extends LinearOpMode {
    private TunerContext context;
    private TuningPhase[] phases;
    private int selectedPhaseIndex;

    private PDSCoefficients heading;
    private PDSCoefficients translation;
    private double forwardVelocity;
    private double forwardAcceleration;
    private double strafeVelocity;
    private double strafeAcceleration;
    private double angularVelocity;
    private double angularAcceleration;
    private double translationKV;
    private double translationKA;
    private double angularKV;
    private double angularKA;
    private double centripetal;
    private double translationFeedback;
    private double angularFeedback;

    private enum Phase {
        HEADING,
        MOVEMENT_LIMITS,
        CENTRIPETAL,
        VELOCITY_FEEDBACK
    }

    private enum LimitStage {
        TRANSLATION,
        SETTLING,
        RUNNING
    }

    private enum LimitTrial {
        FORWARD,
        BACKWARD,
        LEFT,
        RIGHT,
        COUNTERCLOCKWISE,
        CLOCKWISE
    }

    private enum FeedbackAxis {
        TRANSLATION,
        ANGULAR
    }

    @Override
    public void runOpMode() {
        context = new TunerContext(this);
        context.setFollower(new Follower(new Constants(), hardwareMap, true));
        context.constants.drivetrainType = context.getFollower().getDrivetrain().getDrivetrainType();
        loadValues();

        phases = new TuningPhase[]{new Heading(), new Limits(), new Centripetal(), new VelocityFeedback()};
        selectFirstIncompletePhase();

        while (opModeInInit()) {
            TuningPhase selectedPhase = phaseSelector();
            if (selectedPhase != null) {
                context.getFollower().setPose(Pose.zero());
                telemetry.addLine("Press Start to run the tuner.");
                telemetry.addLine("Make sure the robot has enough space.");
                telemetry.update();
            }
        }

        waitForStart();

        while (opModeIsActive() && selectedPhaseIndex < phases.length) {
            TuningPhase phase = phases[selectedPhaseIndex];
            phase.run(this);
            if (!opModeIsActive()) {
                break;
            }
            if (phase.isComplete()) {
                context.saveConstants();
                selectedPhaseIndex++;
            }
        }

        context.getFollower().stop();
    }

    private void loadValues() {
        heading = copy(context.constants.headingCoeffs);
        translation = copy(context.constants.translationalCoeffs);
        forwardVelocity = context.constants.forwardVelLimitIn;
        forwardAcceleration = context.constants.forwardAccelLimitIn;
        strafeVelocity = context.constants.strafeVelLimitIn;
        strafeAcceleration = context.constants.strafeAccelLimitIn;
        angularVelocity = context.constants.angularVelLimitRad;
        angularAcceleration = context.constants.angularAccelLimitRad;
        translationKV = context.constants.translationalKV;
        translationKA = context.constants.translationalKA;
        angularKV = context.constants.angularKV;
        angularKA = context.constants.angularKA;
        centripetal = context.constants.Kcentripetal;
        translationFeedback = context.constants.velocityFeedbackGain;
        angularFeedback = context.constants.angularVelocityFeedbackGain;
    }

    private PDSCoefficients copy(PDSCoefficients coefficients) {
        return new PDSCoefficients(coefficients.kP, coefficients.kD, coefficients.kS);
    }

    private boolean allPositive(double... values) {
        for (double value : values) {
            if (!Double.isFinite(value) || value <= 0.0) {
                return false;
            }
        }
        return true;
    }

    private void saveHeading() {
        context.constants.headingCoeffs.setkP(heading.kP);
        context.constants.headingCoeffs.setkD(heading.kD);
        context.constants.headingCoeffs.setkS(heading.kS);
        context.getFollower().setHeadingTuning(heading);
    }

    private void saveMovement() {
        context.constants.translationalCoeffs.setkP(translation.kP);
        context.constants.translationalCoeffs.setkD(translation.kD);
        context.constants.translationalCoeffs.setkS(translation.kS);
        context.constants.forwardVelLimitIn = forwardVelocity;
        context.constants.forwardAccelLimitIn = forwardAcceleration;
        context.constants.strafeVelLimitIn = strafeVelocity;
        context.constants.strafeAccelLimitIn = strafeAcceleration;
        context.constants.angularVelLimitRad = angularVelocity;
        context.constants.angularAccelLimitRad = angularAcceleration;
        context.constants.translationalKV = translationKV;
        context.constants.translationalKA = translationKA;
        context.constants.angularKV = angularKV;
        context.constants.angularKA = angularKA;
        context.getFollower().setMovementTuning(translation, translationKV, translationKA, angularKV,
                angularKA, forwardVelocity, strafeVelocity);
    }

    private void saveCentripetal() {
        context.constants.Kcentripetal = centripetal;
        context.getFollower().setCentripetalTuning(centripetal);
    }

    private void saveFeedback() {
        context.constants.velocityFeedbackGain = translationFeedback;
        context.constants.angularVelocityFeedbackGain = angularFeedback;
        context.getFollower().setVelocityFeedbackTuning(translationFeedback, angularFeedback);
    }

    private boolean phaseAvailable(int index) {
        for (int i = 0; i < index; i++) {
            if (!phases[i].isComplete()) {
                return false;
            }
        }
        return true;
    }

    private String phaseStatus(int index) {
        if (phases[index].isComplete()) {
            return "[✓]";
        }
        return phaseAvailable(index) ? "[ ]" : "[X]";
    }

    private void selectFirstIncompletePhase() {
        selectedPhaseIndex = 0;
        for (int i = 0; i < phases.length; i++) {
            if (!phases[i].isComplete() && phaseAvailable(i)) {
                selectedPhaseIndex = i;
                return;
            }
        }
    }

    private TuningPhase phaseSelector() {
        telemetry.addLine("Use Dpad Up and Down to choose a phase, then press B to select it.");
        telemetry.addLine();

        for (int i = 0; i < phases.length; i++) {
            String cursor = i == selectedPhaseIndex ? " <" : "";
            telemetry.addLine(phaseStatus(i) + " " + Phase.values()[i].name().replace("_", " ") + cursor);
        }

        telemetry.update();

        if (gamepad1.dpadUpWasPressed()) {
            do {
                selectedPhaseIndex = (selectedPhaseIndex - 1 + phases.length) % phases.length;
            } while (!phaseAvailable(selectedPhaseIndex));
        } else if (gamepad1.dpadDownWasPressed()) {
            do {
                selectedPhaseIndex = (selectedPhaseIndex + 1) % phases.length;
            } while (!phaseAvailable(selectedPhaseIndex));
        } else if (gamepad1.bWasPressed()) {
            return phases[selectedPhaseIndex];
        }

        return null;
    }

    private class Heading extends TuningPhase {
        private PDSRoutine routine;
        private int selected;
        private double target = 90.0;
        private boolean complete;

        Heading() {
            super(FollowerTuner.this.context);
            complete = heading.kP != 0.0;
        }

        @Override
        protected String getPhaseName() {
            return "Heading";
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
            selected = 0;
            if (manualMode) {
                context.getFollower().setHeadingTuning(heading);
                return;
            }
            routine = new PDSRoutine(context, PDSRoutine.Axis.HEADING);
            routine.start();
        }

        @Override
        protected void autoTune() {
            if (!routine.update(context)) {
                return;
            }
            heading = copy(routine.getCoefficients());
            context.getFollower().enableControllers();
            saveHeading();
            complete = true;
        }

        @Override
        protected void manualTune() {
            if (gamepad1.dpadLeftWasPressed()) {
                selected = (selected + 2) % 3;
            }
            if (gamepad1.dpadRightWasPressed()) {
                selected = (selected + 1) % 3;
            }

            double direction = gamepad1.dpadUpWasPressed() ? 1.0 : gamepad1.dpadDownWasPressed() ? -1.0 : 0.0;
            if (direction != 0.0) {
                if (selected == 0) {
                    heading.kP = Math.max(0.0, heading.kP + direction * 0.01);
                } else if (selected == 1) {
                    heading.kD = Math.max(0.0, heading.kD + direction * 0.001);
                } else {
                    heading.kS = Math.max(0.0, heading.kS + direction * 0.005);
                }
                context.getFollower().setHeadingTuning(heading);
            }

            if (gamepad1.xWasPressed() && !context.getFollower().isBusy()) {
                GeometryFactory factory = new GeometryFactory(context.getFollower());
                Turn turn = factory.turn(context.getFollower().getPose()).turnTo(Angle.fromDeg(target)).quickBuild();
                context.getFollower().follow(turn);
                target = -target;
            }

            telemetry.addData("Selected", selected == 0 ? "P" : selected == 1 ? "D" : "S");
            telemetry.addData("P", heading.kP);
            telemetry.addData("D", heading.kD);
            telemetry.addData("S", heading.kS);
            telemetry.addLine("Left/Right selects a value.");
            telemetry.addLine("Up/Down changes the value.");
            telemetry.addLine("X runs a test turn.");
            telemetry.addLine("A accepts the values.");
            telemetry.update();

            if (gamepad1.aWasPressed()) {
                context.getFollower().stop();
                saveHeading();
                complete = true;
            }
        }

        @Override
        protected void reportResults() {
            telemetry.addData("Heading P", heading.kP);
            telemetry.addData("Heading D", heading.kD);
            telemetry.addData("Heading S", heading.kS);
        }
    }

    private class Limits extends TuningPhase {
        private static final double RUN_TIME = 2000.0;
        private static final double SETTLE_TIME = 800.0;

        private final ElapsedTime timer = new ElapsedTime();
        private final double[][] maxima = new double[6][2];
        private PDSRoutine routine;
        private PDSController headingHold;
        private LimitStage stage;
        private int trial;
        private int selected;
        private double heldHeading;
        private boolean measured;
        private boolean complete;

        Limits() {
            super(FollowerTuner.this.context);
            double[] values = {forwardVelocity, forwardAcceleration, strafeVelocity, strafeAcceleration,
                    angularVelocity, angularAcceleration, translationKV, translationKA, angularKV, angularKA};
            complete = translation.kP != 0.0 && allPositive(values);
        }

        @Override
        protected String getPhaseName() {
            return "Movement Limits";
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
            for (double[] maximum : maxima) {
                maximum[0] = 0.0;
                maximum[1] = 0.0;
            }
            trial = 0;
            selected = 0;
            measured = false;
            stage = LimitStage.TRANSLATION;
            headingHold = new PDSController(heading);
            headingHold.setAngularController();
            routine = new PDSRoutine(context, PDSRoutine.Axis.DRIVE);
            routine.start();
        }

        private boolean updateMeasurements() {
            if (measured) {
                return true;
            }

            switch (stage) {
                case TRANSLATION:
                    if (routine.update(context)) {
                        translation = copy(routine.getCoefficients());
                        context.getFollower().enableControllers();
                        context.getFollower().stop();
                        timer.reset();
                        stage = LimitStage.SETTLING;
                    }
                    break;
                case SETTLING:
                    context.getFollower().stop();
                    if (timer.milliseconds() >= SETTLE_TIME) {
                        if (trial >= LimitTrial.values().length) {
                            deriveValues();
                            measured = true;
                            return true;
                        }
                        heldHeading = context.getFollower().getPose().getHeading().getRad();
                        headingHold.reset();
                        timer.reset();
                        stage = LimitStage.RUNNING;
                    }
                    break;
                case RUNNING:
                    runTrial();
                    recordMaximums();
                    if (timer.milliseconds() >= RUN_TIME) {
                        context.getFollower().stop();
                        trial++;
                        timer.reset();
                        stage = LimitStage.SETTLING;
                    }
                    break;
            }

            String step = stage == LimitStage.TRANSLATION ? "Translational PDS" :
                    trial >= LimitTrial.values().length ? "Calculating" : LimitTrial.values()[trial].name();
            telemetry.addData("Step", step);
            telemetry.update();
            return false;
        }

        private void runTrial() {
            LimitTrial current = LimitTrial.values()[trial];
            double x = 0.0;
            double y = 0.0;
            double turn = 0.0;

            switch (current) {
                case FORWARD:
                    x = 1.0;
                    break;
                case BACKWARD:
                    x = -1.0;
                    break;
                case LEFT:
                    y = 1.0;
                    break;
                case RIGHT:
                    y = -1.0;
                    break;
                case COUNTERCLOCKWISE:
                    turn = 1.0;
                    break;
                case CLOCKWISE:
                    turn = -1.0;
                    break;
            }

            if (current != LimitTrial.COUNTERCLOCKWISE && current != LimitTrial.CLOCKWISE) {
                Angle currentHeading = context.getFollower().getPose().getHeading();
                double headingError = currentHeading.getShortestAngleTo(Angle.fromRad(heldHeading)).getRad();
                turn = Math.max(-1.0, Math.min(1.0, headingHold.calculate(headingError)));
            }

            context.getFollower().getDrivetrain().moveWithVectors(x, y, turn);
        }

        private void recordMaximums() {
            Pose velocity = context.getFollower().getVelocity();
            Pose acceleration = context.getFollower().getAcceleration();
            LimitTrial current = LimitTrial.values()[trial];
            double measuredVelocity;
            double measuredAcceleration;

            if (current == LimitTrial.FORWARD || current == LimitTrial.BACKWARD) {
                measuredVelocity = Math.abs(velocity.getX().getIn());
                measuredAcceleration = Math.abs(acceleration.getX().getIn());
            } else if (current == LimitTrial.LEFT || current == LimitTrial.RIGHT) {
                measuredVelocity = Math.abs(velocity.getY().getIn());
                measuredAcceleration = Math.abs(acceleration.getY().getIn());
            } else {
                measuredVelocity = Math.abs(velocity.getHeading().getRad());
                measuredAcceleration = Math.abs(acceleration.getHeading().getRad());
            }

            if (Double.isFinite(measuredVelocity)) {
                maxima[trial][0] = Math.max(maxima[trial][0], measuredVelocity);
            }
            if (Double.isFinite(measuredAcceleration)) {
                maxima[trial][1] = Math.max(maxima[trial][1], measuredAcceleration);
            }
        }

        private double weaker(LimitTrial first, LimitTrial second, int measurement) {
            return Math.min(maxima[first.ordinal()][measurement], maxima[second.ordinal()][measurement]);
        }

        private void deriveValues() {
            double fullForwardVelocity = weaker(LimitTrial.FORWARD, LimitTrial.BACKWARD, 0);
            double fullForwardAcceleration = weaker(LimitTrial.FORWARD, LimitTrial.BACKWARD, 1);
            double fullStrafeVelocity = weaker(LimitTrial.LEFT, LimitTrial.RIGHT, 0);
            double fullStrafeAcceleration = weaker(LimitTrial.LEFT, LimitTrial.RIGHT, 1);
            double fullAngularVelocity = weaker(LimitTrial.COUNTERCLOCKWISE, LimitTrial.CLOCKWISE, 0);
            double fullAngularAcceleration = weaker(LimitTrial.COUNTERCLOCKWISE, LimitTrial.CLOCKWISE, 1);
            double[] values = {fullForwardVelocity, fullForwardAcceleration, fullStrafeVelocity,
                    fullStrafeAcceleration, fullAngularVelocity, fullAngularAcceleration};

            if (!allPositive(values)) {
                throw new IllegalStateException("A movement limit measurement was zero.");
            }

            forwardVelocity = fullForwardVelocity * 0.95;
            forwardAcceleration = fullForwardAcceleration * 0.95;
            strafeVelocity = fullStrafeVelocity * 0.95;
            strafeAcceleration = fullStrafeAcceleration * 0.95;
            angularVelocity = fullAngularVelocity * 0.95;
            angularAcceleration = fullAngularAcceleration * 0.95;
            translationKV = 1.0 / fullForwardVelocity;
            translationKA = 1.0 / fullForwardAcceleration;
            angularKV = 1.0 / fullAngularVelocity;
            angularKA = 1.0 / fullAngularAcceleration;
            context.getFollower().setMovementTuning(translation, translationKV, translationKA, angularKV,
                    angularKA, forwardVelocity, strafeVelocity);
        }

        @Override
        protected void autoTune() {
            if (updateMeasurements()) {
                saveMovement();
                complete = true;
            }
        }

        @Override
        protected void manualTune() {
            if (!updateMeasurements()) {
                return;
            }

            if (gamepad1.dpadLeftWasPressed()) {
                selected = (selected + 12) % 13;
            }
            if (gamepad1.dpadRightWasPressed()) {
                selected = (selected + 1) % 13;
            }

            double direction = gamepad1.dpadUpWasPressed() ? 1.0 : gamepad1.dpadDownWasPressed() ? -1.0 : 0.0;
            if (direction != 0.0) {
                adjustSelected(direction);
                context.getFollower().setMovementTuning(translation, translationKV, translationKA, angularKV,
                        angularKA, forwardVelocity, strafeVelocity);
            }

            telemetry.addData("Selected", selectedName());
            telemetry.addData("Value", selectedValue());
            telemetry.addLine("Left/Right selects a value.");
            telemetry.addLine("Up/Down changes the value.");
            telemetry.addLine("A accepts the values.");
            telemetry.update();

            if (gamepad1.aWasPressed()) {
                saveMovement();
                complete = true;
            }
        }

        private String selectedName() {
            String[] names = {"Translation P", "Translation D", "Translation S", "Forward Velocity",
                    "Forward Acceleration", "Strafe Velocity", "Strafe Acceleration", "Angular Velocity",
                    "Angular Acceleration", "Translation kV", "Translation kA", "Angular kV", "Angular kA"};
            return names[selected];
        }

        private double selectedValue() {
            double[] values = {translation.kP, translation.kD, translation.kS, forwardVelocity,
                    forwardAcceleration, strafeVelocity, strafeAcceleration, angularVelocity,
                    angularAcceleration, translationKV, translationKA, angularKV, angularKA};
            return values[selected];
        }

        private void adjustSelected(double direction) {
            switch (selected) {
                case 0:
                    translation.kP = Math.max(0.0, translation.kP + direction * 0.01);
                    break;
                case 1:
                    translation.kD = Math.max(0.0, translation.kD + direction * 0.001);
                    break;
                case 2:
                    translation.kS = Math.max(0.0, translation.kS + direction * 0.005);
                    break;
                case 3:
                    forwardVelocity = Math.max(0.0, forwardVelocity + direction);
                    break;
                case 4:
                    forwardAcceleration = Math.max(0.0, forwardAcceleration + direction * 2.0);
                    break;
                case 5:
                    strafeVelocity = Math.max(0.0, strafeVelocity + direction);
                    break;
                case 6:
                    strafeAcceleration = Math.max(0.0, strafeAcceleration + direction * 2.0);
                    break;
                case 7:
                    angularVelocity = Math.max(0.0, angularVelocity + direction * 0.1);
                    break;
                case 8:
                    angularAcceleration = Math.max(0.0, angularAcceleration + direction * 0.2);
                    break;
                case 9:
                    translationKV = Math.max(0.0, translationKV + direction * 0.001);
                    break;
                case 10:
                    translationKA = Math.max(0.0, translationKA + direction * 0.0005);
                    break;
                case 11:
                    angularKV = Math.max(0.0, angularKV + direction * 0.005);
                    break;
                case 12:
                    angularKA = Math.max(0.0, angularKA + direction * 0.002);
                    break;
            }
        }

        @Override
        protected void reportResults() {
            telemetry.addData("Forward Velocity", forwardVelocity);
            telemetry.addData("Forward Acceleration", forwardAcceleration);
            telemetry.addData("Strafe Velocity", strafeVelocity);
            telemetry.addData("Strafe Acceleration", strafeAcceleration);
            telemetry.addData("Angular Velocity", angularVelocity);
            telemetry.addData("Angular Acceleration", angularAcceleration);
            telemetry.addData("Translation kV", translationKV);
            telemetry.addData("Translation kA", translationKA);
            telemetry.addData("Angular kV", angularKV);
            telemetry.addData("Angular kA", angularKA);
        }
    }

    private class Centripetal extends TuningPhase {
        private static final double ERROR_TARGET = 0.15;

        private BinarySearch search;
        private Path[] arcs;
        private int arc;
        private double errorSum;
        private int samples;
        private double meanError;
        private double manualStep;
        private boolean complete;

        Centripetal() {
            super(FollowerTuner.this.context);
            complete = centripetal > 0.0;
        }

        @Override
        protected String getPhaseName() {
            return "Centripetal";
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
            GeometryFactory factory = new GeometryFactory(context.getFollower()).setDistUnit(DistUnit.IN)
                    .setAngleUnit(AngleUnit.DEG);
            Pose start = factory.pose(0, 0, 0);
            Pose end = factory.pose(48, 0, 0);

            arcs = new Path[]{
                    factory.path(start, factory.arcPose(24, 24, 18), end)
                            .interpolateWith(InterpolationStyle.CONSTANT_START_HEADING).quickBuild(),
                    factory.path(end, factory.arcPose(24, -24, 18), start)
                            .interpolateWith(InterpolationStyle.CONSTANT_START_HEADING).quickBuild()
            };

            double fullStrafeAcceleration = Math.max(strafeAcceleration / 0.95, 0.001);
            double seed = centripetal > 0.0 ? centripetal : 1.0 / fullStrafeAcceleration;
            double upper = Math.max(seed * 2.0, 2.0 / fullStrafeAcceleration);
            search = new BinarySearch(0.0, upper, upper / 64.0);
            centripetal = manualMode ? seed : search.getGuess();
            manualStep = Math.max(seed * 0.05, 0.00001);
            context.getFollower().setCentripetalTuning(centripetal);
            context.getFollower().setDriveControllerEnabled(false);
            startTrial();
        }

        private void startTrial() {
            context.getFollower().stop();
            context.getFollower().setPose(Pose.zero());
            arc = 0;
            errorSum = 0.0;
            samples = 0;
            context.getFollower().follow(arcs[0]);
        }

        private void sampleError() {
            PathSegment segment = arcs[arc].getParametricPath();
            Vector current = context.getFollower().getPose().getVec();
            double t = segment.getBestT(current);
            if (t <= 0.15 || t >= 0.85) {
                return;
            }

            Vector target = segment.getPosition(t);
            Vector normal = PathSegment.calculateArcNormal(segment.getFirstDerivative(t),
                    segment.getSecondDerivative(t));
            double error = target.minus(current).dot(normal).getIn();
            if (Double.isFinite(error)) {
                errorSum += error;
                samples++;
            }
        }

        private boolean updateTrial() {
            if (context.getFollower().isBusy()) {
                sampleError();
                return false;
            }
            if (arc == 0) {
                arc = 1;
                context.getFollower().follow(arcs[1]);
                return false;
            }
            if (samples == 0) {
                throw new IllegalStateException("No centripetal error samples were recorded.");
            }
            meanError = errorSum / samples;
            return true;
        }

        private void finish() {
            context.getFollower().stop();
            context.getFollower().setDriveControllerEnabled(true);
            saveCentripetal();
            complete = true;
        }

        @Override
        protected void autoTune() {
            if (!updateTrial()) {
                return;
            }
            if (Math.abs(meanError) <= ERROR_TARGET) {
                finish();
                return;
            }

            boolean keepSearching = search.updateGuess(meanError > 0.0);
            centripetal = search.getGuess();
            context.getFollower().setCentripetalTuning(centripetal);
            if (keepSearching) {
                startTrial();
            } else {
                finish();
            }
        }

        @Override
        protected void manualTune() {
            boolean changed = false;
            if (gamepad1.dpadUpWasPressed()) {
                centripetal += manualStep;
                changed = true;
            }
            if (gamepad1.dpadDownWasPressed()) {
                centripetal = Math.max(0.0, centripetal - manualStep);
                changed = true;
            }
            if (gamepad1.dpadRightWasPressed()) {
                manualStep *= 2.0;
            }
            if (gamepad1.dpadLeftWasPressed()) {
                manualStep = Math.max(manualStep / 2.0, 0.000001);
            }

            if (changed) {
                context.getFollower().setCentripetalTuning(centripetal);
                startTrial();
            } else if (updateTrial()) {
                startTrial();
            }

            telemetry.addData("Centripetal", centripetal);
            telemetry.addData("Mean signed error", meanError);
            telemetry.addData("Step", manualStep);
            telemetry.addLine("Up/Down changes the gain.");
            telemetry.addLine("Left/Right changes the step.");
            telemetry.addLine("A accepts the value.");
            telemetry.update();

            if (gamepad1.aWasPressed()) {
                finish();
            }
        }

        @Override
        protected void reportResults() {
            telemetry.addData("Centripetal", centripetal);
            telemetry.addData("Mean signed error", meanError);
        }
    }

    private class VelocityFeedback extends TuningPhase {
        private static final int SEARCH_ROUNDS = 4;

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

        VelocityFeedback() {
            super(FollowerTuner.this.context);
            complete = allPositive(translationFeedback, angularFeedback);
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
            if (translation.kD > 0.0) {
                translationFeedback = translation.kD;
            }
            if (heading.kD > 0.0) {
                angularFeedback = heading.kD;
            }
            context.getFollower().setVelocityFeedbackTuning(translationFeedback, angularFeedback);
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
            center = axis == FeedbackAxis.TRANSLATION ? translationFeedback : angularFeedback;
            double feedforward = axis == FeedbackAxis.TRANSLATION ? translationKV : angularKV;
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
                translationFeedback = gains[candidate];
            } else {
                angularFeedback = gains[candidate];
            }
            context.getFollower().setVelocityFeedbackTuning(translationFeedback, angularFeedback);
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
        protected void autoTune() {
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
                translationFeedback = center;
                context.getFollower().setVelocityFeedbackTuning(translationFeedback, angularFeedback);
                startSearch(FeedbackAxis.ANGULAR);
            } else {
                angularFeedback = center;
                saveFeedback();
                complete = true;
            }
        }

        @Override
        protected void manualTune() {
            if (gamepad1.dpadLeftWasPressed() || gamepad1.dpadRightWasPressed()) {
                axis = axis == FeedbackAxis.TRANSLATION ? FeedbackAxis.ANGULAR : FeedbackAxis.TRANSLATION;
                startTest();
            }

            double direction = gamepad1.dpadUpWasPressed() ? 1.0 : gamepad1.dpadDownWasPressed() ? -1.0 : 0.0;
            if (direction != 0.0) {
                double base = axis == FeedbackAxis.TRANSLATION ? Math.max(translation.kD, translationKV) :
                        Math.max(heading.kD, angularKV);
                double adjustment = Math.max(base * 0.05, 0.00001);
                if (axis == FeedbackAxis.TRANSLATION) {
                    translationFeedback = Math.max(0.0, translationFeedback + direction * adjustment);
                } else {
                    angularFeedback = Math.max(0.0, angularFeedback + direction * adjustment);
                }
                context.getFollower().setVelocityFeedbackTuning(translationFeedback, angularFeedback);
                startTest();
            } else if (gamepad1.xWasPressed()) {
                startTest();
            } else if (updateTest()) {
                if (axis == FeedbackAxis.TRANSLATION) {
                    translationScore = lastScore;
                } else {
                    angularScore = lastScore;
                }
                startTest();
            }

            telemetry.addData("Selected", axis.name());
            telemetry.addData("Translation feedback", translationFeedback);
            telemetry.addData("Angular feedback", angularFeedback);
            telemetry.addData("Translation RMS error", translationScore);
            telemetry.addData("Angular RMS error", angularScore);
            telemetry.addLine("Left/Right selects a gain.");
            telemetry.addLine("Up/Down changes the gain.");
            telemetry.addLine("X restarts the current test.");
            telemetry.addLine("A accepts both gains.");
            telemetry.update();

            if (gamepad1.aWasPressed()) {
                context.getFollower().stop();
                saveFeedback();
                complete = true;
            }
        }

        @Override
        protected void reportResults() {
            telemetry.addData("Translation feedback", translationFeedback);
            telemetry.addData("Translation RMS error", translationScore);
            telemetry.addData("Angular feedback", angularFeedback);
            telemetry.addData("Angular RMS error", angularScore);
        }
    }
}
