package org.firstinspires.ftc.teamcode.tuning.manual;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;

import controllers.PDSController.PDSCoefficients;
import drivetrains.Drivetrain;
import followers.BSplineFollower;
import followers.constants.BSplineFollowerConstants;
import localizers.Localizer;
import paths.Path;
import paths.PathBuilder;
import util.Pose;

/**
 * OpMode for tuning the BSpline follower with Panels. Matches the architecture of AxialTuner.
 * Hold X to execute the multi-stage B-Spline test path forward, and hold A to reset and drive back.
 * Adjust the PDS coefficients, feedforward, and tolerances in Panels.
 *
 * @author Sohum Arora - 22985 Paraducks
 */
@Configurable
@TeleOp(name = "BSpline Tuner", group = "Apex Pathing Tuning")
public class BSplineTuner extends OpMode {
    private Drivetrain drivetrain;
    private Localizer localizer;
    private BSplineFollower follower;
    private BSplineFollowerConstants followerConstants;
    private JoinedTelemetry fullTelem;

    // --- DASHBOARD TUNING VARIABLES ---
    public static double tP, tD, tS, tSDeadzone; // Translation PDS
    public static double hP, hD, hS, hSDeadzone; // Heading PDS
    public static double vFF;                    // Velocity Feedforward
    public static double headingTol;             // Heading Tolerance (Degrees)
    public static double distanceTol;            // Distance Tolerance (Inches)
    public static double tTol;                   // T-Parameter Tolerance

    private Path currentPath;
    private boolean pathActive = false;
    private boolean wasAtTarget = false;

    @Override
    public void init() {
        // Build constants, drivetrain, localizer, and telemetry
        Constants constants = new Constants();
        drivetrain = constants.buildOnlyDrivetrain(hardwareMap);
        localizer = constants.buildOnlyLocalizer(hardwareMap, Pose.zero());
        fullTelem = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);

        // Extract the constants specific to the BSpline follower
        followerConstants = (BSplineFollowerConstants) constants.setBSplineFollowerConstants();

        // Populate Dashboard variables with the initial values from your Constants file
        tP = followerConstants.translationCoeffs.kP;
        tD = followerConstants.translationCoeffs.kD;
        tS = followerConstants.translationCoeffs.kS;
        tSDeadzone = followerConstants.translationCoeffs.kSDeadzone;

        hP = followerConstants.headingCoeffs.kP;
        hD = followerConstants.headingCoeffs.kD;
        hS = followerConstants.headingCoeffs.kS;
        hSDeadzone = followerConstants.headingCoeffs.kSDeadzone;

        vFF = followerConstants.velocityFF;
        headingTol = Math.toDegrees(followerConstants.headingTolerance);
        distanceTol = followerConstants.distanceTolerance;
        tTol = followerConstants.tTolerance;

        // Build the follower
        follower = followerConstants.build(drivetrain, localizer);

        fullTelem.addLine(
                "Hold X to run the multi-stage B-Spline test path, and A to drive back to the start position."
        );
        fullTelem.update();
    }

    private void runPath(boolean forward) {
        if (!pathActive) {
            if (!forward) {
                currentPath = new PathBuilder(localizer.getPose())
                        .addControlPoints(new Pose(24, 24, Math.toRadians(90)), new Pose(0, 0, 0))
                        .build();
            } else {
                currentPath = new PathBuilder(localizer.getPose())
                        .addControlPoints(new Pose(24, 24, Math.toRadians(90)), new Pose(48, 0, 0))
                        .build();
            }
            follower.followPath(currentPath);
            pathActive = true;
        }
        follower.update();
    }

    @Override
    public void loop() {
        localizer.update();

        // Push Dashboard variable updates back into the active follower constants
        followerConstants.translationCoeffs = new PDSCoefficients(tP, tD, tS, tSDeadzone);
        followerConstants.headingCoeffs = new PDSCoefficients(hP, hD, hS, hSDeadzone);
        followerConstants.velocityFF = vFF;
        followerConstants.headingTolerance = Math.toRadians(headingTol);
        followerConstants.distanceTolerance = distanceTol;
        followerConstants.tTolerance = tTol;

        if (gamepad1.x) { // Run path forward when X is held
            runPath(true);
        } else if (gamepad1.a) { // Move back to start position when A is held
            runPath(false);
        } else {
            follower.stop();
            drivetrain.stop();
            pathActive = false;
        }

        boolean atTarget = pathActive && !follower.isBusy();
        if (atTarget && !wasAtTarget) { // Gamepad rumble and Led green when at target
            gamepad1.rumble(0.5, 0.5, 100);
            gamepad1.setLedColor(0, 1, 0, 300);
            pathActive = false;
        } else if (pathActive && !atTarget) { // Led red when not at target
            gamepad1.setLedColor(1, 0, 0, 100);
        }
        wasAtTarget = atTarget;

        fullTelem.addData("Target Path: ", pathActive ? "Active" : "Inactive");
        fullTelem.addData("Position X: ", localizer.getPose().getX());
        fullTelem.addData("Position Y: ", localizer.getPose().getY());
        fullTelem.addData("Heading: ", Math.toDegrees(localizer.getPose().getHeading()));
        fullTelem.addData("At Target: ", atTarget);
        fullTelem.addData("Drivetrain Output: ", drivetrain.toString());
        fullTelem.update();
    }
}