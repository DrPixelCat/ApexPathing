package org.firstinspires.ftc.teamcode.tuning;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.concurrent.TimeUnit;

import core.ApexConfig;
import core.Follower;
import core.FollowerConstants;
import controllers.PDSController.PDSCoefficients;
import drivetrains.BaseDrivetrainConfig;
import localizers.BaseLocalizerConfig;
import geometry.Angle;
import geometry.Dist;
import geometry.Pose;
import geometry.Vector;
import paths.builders.Builder;
import paths.movements.Path;
import util.DistUnit;

import org.firstinspires.ftc.teamcode.Constants;

/**
 * Single unified automatic tuner capable of completely tuning a robot for Apex in minutes in just a single OpMode!
 * All you have to do is follow the telemetry instructions and press a couple buttons here and there
 * Once you have run this tuner, your robot is fully tuned and ready to go Path its way to the Peaks
 * @author Sohum Arora 22985 Paraducks
 */
@Configurable
@TeleOp(name = "Follower Tuner", group = "Apex Pathing Tuning")
public class FollowerTuner extends LinearOpMode {

    public static double kSGuess = 0.0;

    enum TuningState {
        AWAIT_CONFIRM,
        KS_SEARCH,
        STEP_RESPONSE,
        VELOCITY_FF,
        LATERAL_ACCEL,
        LATERAL_ACCEL_TEST,
        CONFIRM,
        SAVE
    }

    enum TuningPhase {
        HEADING,
        TRANSLATION,
        VELOCITY_FF,
        LATERAL_ACCEL,
        COMPLETE
    }

    enum TuningMode {
        AUTO,
        MANUAL
    }

    private TuningPhase phase = TuningPhase.HEADING;
    private TuningState state = TuningState.AWAIT_CONFIRM;
    private TuningMode mode = TuningMode.AUTO;

    private double headingP, headingD, headingS;
    private double translationP, translationD, translationS;
    private double velocityFF;
    private double maxLateralAccel = 40.0;
    private double headingToleranceDeg, distanceToleranceIn, tTolerance;

    private double ksMax = 0.2, ksMin = 0.0, ksGuess = 0.0, ksLastGuess = -1.0, ksMaxDeviation;
    private double stepMaxAccel, stepMaxVel, stepLastVel, stepLastTime, stepStartTime, stepTimeStamp, stepVelAtTimeStamp;
    private double accelMaxError;
    private boolean driftDetected;
    private boolean readyToRerun = false;

    private final ElapsedTime timer = new ElapsedTime();
    private final Constants baseConstants = new Constants();
    private final FollowerConstants followerConstants = new FollowerConstants();
    private Follower follower;

    private boolean lastA = false, lastB = false, lastY = false;
    private boolean isPaused = false;

    @Override
    public void runOpMode() throws InterruptedException {
        FollowerConstants defaults = baseConstants.followerConfig().getConstants();
        headingP = defaults.headingCoeffs.kP;
        headingD = defaults.headingCoeffs.kD;
        headingS = defaults.headingCoeffs.kS;
        translationP = defaults.driveCoeffs.kP;
        translationD = defaults.driveCoeffs.kD;
        translationS = defaults.driveCoeffs.kS;
        velocityFF = defaults.lateralKV;
        headingToleranceDeg = defaults.headingTolerance.getDeg();
        distanceToleranceIn = defaults.distanceTolerance.getIn();
        tTolerance = defaults.tTolerance;
        maxLateralAccel = defaults.maxLateralAccel > 10 ? defaults.maxLateralAccel : 40.0;

        boolean headingRun = defaults.headingCoeffs.kP != 0.0 || defaults.headingCoeffs.kD != 0.0 || defaults.headingCoeffs.kS != 0.0;
        boolean translationRun = defaults.driveCoeffs.kP != 0.0 || defaults.driveCoeffs.kD != 0.0 || defaults.driveCoeffs.kS != 0.0;
        boolean velocityFFRun = defaults.lateralKV != 0.0;
        boolean accelRun = defaults.maxLateralAccel > 10.0;

        while (opModeInInit()) {
            telemetry.addLine("Robot Initialized");
            telemetry.addLine("Tuning order:\n 1) Heading PDS \n 2) Translation PDS \n 3) Velocity FF \n 4) Max Lateral Accel");
            telemetry.addLine("Run the OpMode to proceed with the Heading Tuner");

            if (headingRun) telemetry.addLine("Heading tuner has already been run successfully and the values have been saved");
            if (translationRun) telemetry.addLine("Translation tuner has already been run successfully and the values have been saved");
            if (velocityFFRun) telemetry.addLine("Velocity FF tuner has already been run successfully and the values have been saved");
            if (accelRun) telemetry.addLine("Max Lateral Accel tuner has already been run successfully and the values have been saved");

            telemetry.addLine("Press 'A' (cross) to directly run the Translation Tuner if you have already run the Heading Tuner");
            telemetry.addLine("Press 'B' (circle) to directly run the Velocity FF Tuner if you have already run the Heading Tuner and Translation Tuner");
            telemetry.addLine("Once all of these 3 tuners are complete, press 'A' (circle) to run the Max Lateral Acceleration Tuner");
            telemetry.addLine("IMPORTANT: Do NOT run the tuners out of order");

            if (gamepad1.a) {
                phase = TuningPhase.TRANSLATION;
            } else if (gamepad1.b) {
                phase = TuningPhase.VELOCITY_FF;
            }

            telemetry.addData("Selected Phase", phase);
            telemetry.update();
        }

        updateFollowerConfig();
        follower = new Follower(customConfig, hardwareMap);

        waitForStart();
        lastA = false;
        lastB = false;
        lastY = false;

        while (opModeIsActive() && phase != TuningPhase.COMPLETE) {

            if (gamepad1.y && !lastY) {
                isPaused = !isPaused;
                if (!isPaused && state == TuningState.STEP_RESPONSE) {
                    stepLastTime = System.nanoTime();
                    timer.reset();
                }
            }
            if (isPaused && (state == TuningState.KS_SEARCH || state == TuningState.STEP_RESPONSE ||
                    state == TuningState.VELOCITY_FF || state == TuningState.LATERAL_ACCEL ||
                    state == TuningState.LATERAL_ACCEL_TEST)) {

                follower.teleOpDrive(0, 0, 0);
                telemetry.addLine("AUTO TUNER PAUSED. Press 'Y' to resume.");

                lastA = gamepad1.a;
                lastB = gamepad1.b;
                lastY = gamepad1.y;

                telemetry.update();
                continue;
            }

            switch (phase) {
                case HEADING:
                case TRANSLATION: {
                    boolean isAngular = phase == TuningPhase.HEADING;

                    switch (state) {
                        case AWAIT_CONFIRM:
                            telemetry.addLine(phase + " phase initialized");
                            telemetry.addLine("Press 'A' (cross) to TOGGLE mode.");
                            telemetry.addLine("Press 'B' (circle) to START tuning.");
                            telemetry.addData("Selected Mode", mode);

                            if (gamepad1.a && !lastA) {
                                mode = (mode == TuningMode.AUTO) ? TuningMode.MANUAL : TuningMode.AUTO;
                            } else if (gamepad1.b && !lastB) {
                                if (mode == TuningMode.AUTO) {
                                    resetKsSearch();
                                    isPaused = false;
                                    state = TuningState.KS_SEARCH;
                                } else {
                                    kSGuess = isAngular ? headingS : translationS;
                                    readyToRerun = false;
                                    state = TuningState.CONFIRM;
                                }
                            }
                            break;

                        case KS_SEARCH:
                            if (Math.abs(ksLastGuess - ksGuess) <= 0.01) {
                                if (isAngular) headingS = ksGuess;
                                else translationS = ksGuess;
                                resetStepResponse();
                                state = TuningState.STEP_RESPONSE;
                                break;
                            }

                            follower.setPose(new Pose(new Vector(Dist.of(0, DistUnit.IN), Dist.of(0, DistUnit.IN)), Angle.fromDeg(0)));
                            follower.update();
                            ksGuess = (ksMax + ksMin) / 2.0;
                            ksMaxDeviation = 0.0;
                            timer.reset();

                            while (opModeIsActive() && timer.time(TimeUnit.MILLISECONDS) < 500) {
                                follower.update();
                                double pos = isAngular
                                        ? follower.getPose().getHeading().getRad()
                                        : follower.getPose().getPos().getX().getIn();
                                ksMaxDeviation = Math.max(Math.abs(pos), ksMaxDeviation);
                                if (isAngular) follower.teleOpDrive(0, 0, ksGuess);
                                else follower.teleOpDrive(ksGuess, 0, 0);
                            }

                            if (ksMaxDeviation > 0.025) ksMax = ksGuess;
                            else ksMin = ksGuess;
                            ksLastGuess = ksGuess;

                            follower.teleOpDrive(0, 0, 0);
                            sleep(500);
                            break;

                        case STEP_RESPONSE:
                            if (timer.time(TimeUnit.MILLISECONDS) >= 2000) {
                                follower.teleOpDrive(0, 0, 0);
                                sleep(500);

                                double L = stepTimeStamp - (stepVelAtTimeStamp / stepMaxAccel);
                                double kP = 1.2 / (L * stepMaxAccel);
                                double kD = 0.6 / stepMaxAccel;

                                if (isAngular) {
                                    headingP = kP > 0 ? kP : 0.01;
                                    headingD = kD > 0 ? kD : 0.001;
                                } else {
                                    translationP = Double.isFinite(kP) && kP > 0 ? kP : 0.01;
                                    translationD = Double.isFinite(kD) && kD > 0 ? kD : 0.001;
                                }

                                updateFollowerConfig();
                                readyToRerun = false;
                                state = TuningState.CONFIRM;
                                break;
                            }

                            follower.update();
                            double curVel = isAngular
                                    ? follower.getVelocity().getHeading().getRad()
                                    : follower.getVelocity().getPos().getX().getIn();

                            double now = System.nanoTime();
                            double deltaT = (now - stepLastTime) / 1e9;
                            double deltaV = curVel - stepLastVel;
                            double accel = deltaT > 1e-6 ? deltaV / deltaT : 0.0;

                            if (accel > stepMaxAccel) {
                                stepMaxAccel = accel;
                                stepTimeStamp = (now - stepStartTime) / 1e9;
                                stepVelAtTimeStamp = curVel;
                            }

                            stepMaxVel = Math.max(curVel, stepMaxVel);
                            stepLastVel = curVel;
                            stepLastTime = now;

                            if (isAngular) follower.teleOpDrive(0, 0, 1.0);
                            else follower.teleOpDrive(1.0, 0, 0);
                            break;

                        case CONFIRM:
                            telemetry.addData("Current Phase", phase);
                            telemetry.addData("Robot Pose", follower.getPose().toString());

                            if (!readyToRerun) {
                                telemetry.addLine("Press 'A' (cross) to SAVE and advance.");
                                telemetry.addLine("Press 'B' (circle) to RERUN or ADJUST.");

                                if (mode == TuningMode.MANUAL) {
                                    telemetry.addLine("--- MANUAL TUNING ---");
                                    telemetry.addLine("Tune kSGuess via Config Panels. Drive to test.");
                                    if (isAngular) headingS = kSGuess;
                                    else translationS = kSGuess;
                                    updateFollowerConfig();
                                    telemetry.addData("Current kSGuess", kSGuess);
                                } else {
                                    if (isAngular) {
                                        telemetry.addData("Heading P", headingP);
                                        telemetry.addData("Heading D", headingD);
                                        telemetry.addData("Heading S", headingS);
                                    } else {
                                        telemetry.addData("Translation P", translationP);
                                        telemetry.addData("Translation D", translationD);
                                        telemetry.addData("Translation S", translationS);
                                    }
                                }

                                if (gamepad1.a && !lastA) {
                                    phase = isAngular ? TuningPhase.TRANSLATION : TuningPhase.VELOCITY_FF;
                                    state = TuningState.AWAIT_CONFIRM;
                                    mode = TuningMode.AUTO;
                                    resetKsSearch();
                                } else if (gamepad1.b && !lastB) {
                                    readyToRerun = true;
                                }
                            } else {
                                telemetry.addLine("Press 'A' (cross) to TOGGLE between AUTO and MANUAL mode.");
                                telemetry.addLine("Press 'B' (circle) to EXECUTE rerun.");
                                telemetry.addData("Selected Mode", mode);

                                if (gamepad1.a && !lastA) {
                                    mode = (mode == TuningMode.AUTO) ? TuningMode.MANUAL : TuningMode.AUTO;
                                    if (mode == TuningMode.MANUAL) {
                                        kSGuess = isAngular ? headingS : translationS;
                                    }
                                } else if (gamepad1.b && !lastB) {
                                    readyToRerun = false;
                                    if (mode == TuningMode.AUTO) {
                                        resetKsSearch();
                                        isPaused = false;
                                        state = TuningState.KS_SEARCH;
                                    }
                                }
                            }

                            follower.teleOpDrive(-gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x);
                            break;
                    }
                    break;
                }

                case VELOCITY_FF: {
                    switch (state) {
                        case AWAIT_CONFIRM:
                            telemetry.addLine("Phase: VELOCITY_FF initialized");
                            telemetry.addLine("Press 'A' (cross) to TOGGLE mode.");
                            telemetry.addLine("Press 'B' (circle) to START tuning.");
                            telemetry.addData("Selected Mode", mode);

                            if (gamepad1.a && !lastA) {
                                mode = (mode == TuningMode.AUTO) ? TuningMode.MANUAL : TuningMode.AUTO;
                            } else if (gamepad1.b && !lastB) {
                                if (mode == TuningMode.AUTO) {
                                    isPaused = false;
                                    state = TuningState.VELOCITY_FF;
                                } else {
                                    kSGuess = velocityFF;
                                    readyToRerun = false;
                                    state = TuningState.CONFIRM;
                                }
                            }
                            break;

                        case VELOCITY_FF:
                            follower.teleOpDrive(0, 1.0, 0);
                            sleep(1500);
                            double maxVel = Math.abs(follower.getVelocity().getPos().getX().getIn());
                            velocityFF = 1.0 / maxVel;
                            follower.teleOpDrive(0, 0, 0);
                            sleep(500);
                            updateFollowerConfig();
                            readyToRerun = false;
                            state = TuningState.CONFIRM;
                            break;

                        case CONFIRM:
                            telemetry.addData("Current Phase", phase);
                            telemetry.addData("Robot Pose", follower.getPose().toString());

                            if (!readyToRerun) {
                                telemetry.addLine("Press 'A' (cross) to SAVE and advance.");
                                telemetry.addLine("Press 'B' (circle) to RERUN or ADJUST.");

                                if (mode == TuningMode.MANUAL) {
                                    telemetry.addLine("--- MANUAL TUNING ---");
                                    telemetry.addLine("Tune kSGuess (kV) via Config Panels. Drive to test.");
                                    velocityFF = kSGuess;
                                    updateFollowerConfig();
                                    telemetry.addData("Current kV", velocityFF);
                                } else {
                                    telemetry.addData("Velocity FF (kV)", velocityFF);
                                }

                                if (gamepad1.a && !lastA) {
                                    phase = TuningPhase.LATERAL_ACCEL;
                                    state = TuningState.AWAIT_CONFIRM;
                                    mode = TuningMode.AUTO;
                                    maxLateralAccel = 50.0;
                                    driftDetected = false;
                                } else if (gamepad1.b && !lastB) {
                                    readyToRerun = true;
                                }
                            } else {
                                telemetry.addLine("Press 'A' (cross) to TOGGLE between AUTO and MANUAL mode.");
                                telemetry.addLine("Press 'B' (circle) to EXECUTE rerun.");
                                telemetry.addData("Selected Mode", mode);

                                if (gamepad1.a && !lastA) {
                                    mode = (mode == TuningMode.AUTO) ? TuningMode.MANUAL : TuningMode.AUTO;
                                    if (mode == TuningMode.MANUAL) kSGuess = velocityFF;
                                } else if (gamepad1.b && !lastB) {
                                    readyToRerun = false;
                                    if (mode == TuningMode.AUTO) {
                                        isPaused = false;
                                        state = TuningState.VELOCITY_FF;
                                    }
                                }
                            }

                            follower.teleOpDrive(-gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x);
                            break;
                    }
                    break;
                }

                case LATERAL_ACCEL: {
                    switch (state) {
                        case AWAIT_CONFIRM:
                            telemetry.addLine("Phase: LATERAL_ACCEL initialized");
                            telemetry.addLine("Press 'A' (cross) to TOGGLE mode.");
                            telemetry.addLine("Press 'B' (circle) to START tuning.");
                            telemetry.addData("Selected Mode", mode);

                            if (gamepad1.a && !lastA) {
                                mode = (mode == TuningMode.AUTO) ? TuningMode.MANUAL : TuningMode.AUTO;
                            } else if (gamepad1.b && !lastB) {
                                if (mode == TuningMode.AUTO) {
                                    isPaused = false;
                                    state = TuningState.LATERAL_ACCEL;
                                } else {
                                    kSGuess = maxLateralAccel;
                                    readyToRerun = false;
                                    state = TuningState.CONFIRM;
                                }
                            }
                            break;

                        case LATERAL_ACCEL:
                            if (driftDetected || maxLateralAccel > 300) {
                                if (!driftDetected) maxLateralAccel -= 20.0;
                                updateFollowerConfig();
                                readyToRerun = false;
                                state = TuningState.CONFIRM;
                                break;
                            }

                            updateFollowerConfig();
                            follower.setPose(new Pose(new Vector(Dist.of(0, DistUnit.IN), Dist.of(0, DistUnit.IN)), Angle.fromDeg(0)));

                            Pose start = follower.getPose();
                            Path testCurve = Builder.path(
                                    start,
                                    new Pose(start.getPos().plus(new Vector(Dist.of(30, DistUnit.IN), Dist.of(0, DistUnit.IN))), start.getHeading()),
                                    new Pose(start.getPos().plus(new Vector(Dist.of(30, DistUnit.IN), Dist.of(30, DistUnit.IN))), start.getHeading().plus(Angle.fromDeg(90))),
                                    new Pose(start.getPos().plus(new Vector(Dist.of(0, DistUnit.IN), Dist.of(30, DistUnit.IN))), start.getHeading().plus(Angle.fromDeg(180)))
                            ).build();

                            follower.follow(testCurve);
                            accelMaxError = 0;
                            state = TuningState.LATERAL_ACCEL_TEST;
                            break;

                        case LATERAL_ACCEL_TEST:
                            follower.update();
                            double err = follower.getPose().getPos().getMag().getIn();
                            if (err > accelMaxError) accelMaxError = err;

                            if (!follower.isBusy()) {
                                if (accelMaxError > 4.0) {
                                    driftDetected = true;
                                    maxLateralAccel -= 20.0;
                                } else {
                                    maxLateralAccel += 20.0;
                                    sleep(1000);
                                }
                                state = TuningState.LATERAL_ACCEL;
                            }
                            break;

                        case CONFIRM:
                            telemetry.addData("Current Phase", phase);
                            telemetry.addData("Robot Pose", follower.getPose().toString());

                            if (!readyToRerun) {
                                telemetry.addLine("Press 'A' (cross) to SAVE and finish.");
                                telemetry.addLine("Press 'B' (circle) to RERUN or ADJUST.");

                                if (mode == TuningMode.MANUAL) {
                                    telemetry.addLine("--- MANUAL TUNING ---");
                                    telemetry.addLine("Tune kSGuess (Max Lateral Accel) via Config Panels. Drive to test.");
                                    maxLateralAccel = kSGuess;
                                    updateFollowerConfig();
                                    telemetry.addData("Current Max Lateral Accel", maxLateralAccel);
                                } else {
                                    telemetry.addData("Max Lateral Accel", maxLateralAccel);
                                }

                                if (gamepad1.a && !lastA) {
                                    state = TuningState.SAVE;
                                } else if (gamepad1.b && !lastB) {
                                    readyToRerun = true;
                                }
                            } else {
                                telemetry.addLine("Press 'A' (cross) to TOGGLE between AUTO and MANUAL mode.");
                                telemetry.addLine("Press 'B' (circle) to EXECUTE rerun.");
                                telemetry.addData("Selected Mode", mode);

                                if (gamepad1.a && !lastA) {
                                    mode = (mode == TuningMode.AUTO) ? TuningMode.MANUAL : TuningMode.AUTO;
                                    if (mode == TuningMode.MANUAL) kSGuess = maxLateralAccel;
                                } else if (gamepad1.b && !lastB) {
                                    readyToRerun = false;
                                    if (mode == TuningMode.AUTO) {
                                        driftDetected = false;
                                        isPaused = false;
                                        state = TuningState.LATERAL_ACCEL;
                                    }
                                }
                            }

                            follower.teleOpDrive(-gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x);
                            break;

                        case SAVE:
                            saveConstantsToJson();
                            phase = TuningPhase.COMPLETE;
                            break;
                    }
                    break;
                }
            }

            lastA = gamepad1.a;
            lastB = gamepad1.b;
            lastY = gamepad1.y;
            telemetry.update();
        }

        while (opModeIsActive()) {
            telemetry.addData("Status", "All Tuning Cycles Complete! Configuration Saved to JSON.");
            telemetry.update();
            follower.teleOpDrive(0, 0, 0);
        }
    }

    private void resetKsSearch() {
        ksMax = 0.2;
        ksMin = 0.0;
        ksGuess = 0.0;
        ksLastGuess = -1.0;
        ksMaxDeviation = 0.0;
    }

    private void resetStepResponse() {
        stepMaxAccel = 0;
        stepMaxVel = 0;
        stepLastVel = 0;
        stepTimeStamp = 0;
        stepVelAtTimeStamp = 0;
        stepLastTime = System.nanoTime();
        stepStartTime = System.nanoTime();
        timer.reset();

        follower.setPose(new Pose(new Vector(Dist.of(0, DistUnit.IN), Dist.of(0, DistUnit.IN)), Angle.fromDeg(0)));
    }

    private void updateFollowerConfig() {
        followerConstants.headingCoeffs = new PDSCoefficients(headingP, headingD, headingS, 0);
        followerConstants.driveCoeffs = new PDSCoefficients(translationP, translationD, translationS, 0);
        followerConstants.lateralCoeffs = new PDSCoefficients(translationP, translationD, translationS, 0);
        followerConstants.lateralKV = velocityFF;
        followerConstants.headingTolerance = Angle.fromDeg(headingToleranceDeg);
        followerConstants.distanceTolerance = Dist.fromIn(distanceToleranceIn);
        followerConstants.tTolerance = tTolerance;
        followerConstants.maxLateralAccel = maxLateralAccel;
    }

    private final ApexConfig customConfig = new ApexConfig() {
        @Override
        public BaseDrivetrainConfig<?> drivetrainConfig() { return baseConstants.drivetrainConfig(); }
        @Override
        public BaseLocalizerConfig<?> localizerConfig() { return baseConstants.localizerConfig(); }
        @Override
        public FollowerConstants followerConfig() { return followerConstants; }
    };

    private void saveConstantsToJson() {
        String jsonPayload = "{\n" +
                "  \"headingP\": " + headingP + ",\n" +
                "  \"headingD\": " + headingD + ",\n" +
                "  \"headingS\": " + headingS + ",\n" +
                "  \"translationP\": " + translationP + ",\n" +
                "  \"translationD\": " + translationD + ",\n" +
                "  \"translationS\": " + translationS + ",\n" +
                "  \"velocityFF\": " + velocityFF + ",\n" +
                "  \"maxLateralAccel\": " + maxLateralAccel + "\n" +
                "}";

        try {
            File outputFolder = new File("/sdcard/FIRST");
            if (!outputFolder.exists()) outputFolder.mkdirs();
            FileWriter fileWriter = new FileWriter(new File(outputFolder, "FollowerConstants.json"));
            fileWriter.write(jsonPayload);
            fileWriter.close();
        } catch (IOException ignored) {}
    }
}