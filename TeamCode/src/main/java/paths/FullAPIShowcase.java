package paths;

import geometry.Angle;
import geometry.Pose;
import geometry.Vector;
import paths.builders.TankPathBuilder;
import paths.heading.HolonomicInterpolationStyle;
import paths.movements.FollowerMovement;
import paths.movements.Path;
import paths.movements.Turn;
import util.AngleUnit;
import util.DistUnit;
import util.GeometryFactory;

public class FullAPIShowcase {
    private final DistUnit distUnit = DistUnit.IN;
    private final AngleUnit angleUnit = AngleUnit.DEG;
    public GeometryFactory pose = new GeometryFactory(distUnit, angleUnit);
    private final Pose startPose;

    // Routine movements stored in sequence
    public Path smoothBlendPath;
    public Path constStartPath;
    public Path constEndPath;
    public Turn callbackTurn;
    public Path tangentForwardPath;
    public Path tankOptimalPath;
    public Path tangentCustomPath;
    public Path facingPointPath;
    public Path nodeBasedPath;

    public FullAPIShowcase(GeometryFactory.Mirror mirror) {
        pose.setMirror(mirror);
        startPose = Pose.Common.CENTER.get(); // (0, 0, 0)
        buildRoutine();
    }

    // region Dummy Actions
    public void startIntake() { /* Starts intake motors */ }

    public void deployOuttake() { /* Extends slides */ }

    public void dropElement() { /* Opens claw */ }

    public void finishRoutine() { /* Plays an LED animation */ }
    // endregion

    /**
     * A comprehensive showcase of every InterpolationStyle and generation method in the API.
     */
    private void buildRoutine() {

        // 1. SMOOTH_START_TO_END
        // Blends the heading linearly from the start pose to the end pose.
        smoothBlendPath = Builder.holonomicPath(
                        startPose,
                        pose.of(15, 0),
                        pose.of(25, 15, 90) // Target heading is 90 degrees
                )
                .interpolateWith(HolonomicInterpolationStyle.SMOOTH_START_TO_END)
                .profiledBuild();

        // ---------------------------------------------------------

        // 2. CONSTANT_START_HEADING
        // The robot locks its orientation to whatever heading it had when this path started.
        // Great for pure strafing. It will stay locked at 90 degrees from the previous path.
        constStartPath = Builder.holonomicPath(
                        smoothBlendPath.getEndPose(),
                        pose.of(25, 40),
                        pose.of(45, 40, 180) // APEX WARNING: The 180 target is ignored due to
                        // CONSTANT_START_HEADING
                )
                .interpolateWith(HolonomicInterpolationStyle.CONSTANT_START_HEADING)
                .addDistanceCallback(0.5, this::startIntake)
                .profiledBuild();

        // ---------------------------------------------------------

        // 3. CONSTANT_END_HEADING
        // The robot immediately targets the final heading of the segment and holds it for the
        // whole curve.
        constEndPath = Builder.holonomicPath(
                        constStartPath.getEndPose(),
                        pose.of(60, 20),
                        pose.of(60, 0, 270) // The robot will immediately pivot to face 270 while
                        // driving
                )
                .interpolateWith(HolonomicInterpolationStyle.CONSTANT_END_HEADING)
                .quickBuild(); // Bypasses physics profiling; relies purely on positional error

        // ---------------------------------------------------------

        // 4. POINT TURN WITH ANGULAR CALLBACK
        // Spins in place from 270 degrees up to 360/0 degrees.
        callbackTurn = Builder.turn(constEndPath.getEndPose())
                .turnTo(Angle.fromDeg(0))
                // Triggers exactly when the robot sweeps past the 315-degree mark during the spin
                .addAngularCallback(Angle.fromDeg(315), this::deployOuttake)
                .quickBuild(); // Quick build is recommended for turns unless dynamic physics are
        // strictly required

        // ---------------------------------------------------------

        // 5. TANGENT_FORWARD
        // The robot strictly faces the forward direction of travel along the path (like a car).
        tangentForwardPath = Builder.holonomicPath(
                        callbackTurn.getEndPose(),
                        pose.of(80, 0),
                        pose.of(100, 20, 90) // Target heading ignored; overridden by path tangent
                )
                .interpolateWith(HolonomicInterpolationStyle.TANGENT_FORWARD)
                .profiledBuild();

        // ---------------------------------------------------------

        // 6. TANGENT_OPTIMAL (TANK KINEMATICS)
        // Uses the TankPathBuilder to demonstrate drivetrain-specific logic.
        // Points either forward OR backward depending on which requires less physical rotation
        // at the start.
        tankOptimalPath = new TankPathBuilder(
                tangentForwardPath.getEndPose(),
                pose.of(120, 20),
                pose.arcPoseOf(120, 0, 10), // Dynamically constrains the corner
                pose.of(100, -20, 0)
        )
                .interpolateWith(TankInterpolationStyle.TANGENT_OPTIMAL)
                .addDistanceCallback(0.8, this::dropElement)
                .profiledBuild();

        // ---------------------------------------------------------

        // 7. TANGENT_CUSTOM
        // Follows the path tangent, but allows for a custom, fixed angular offset.
        tangentCustomPath = Builder.holonomicPath(
                        tankOptimalPath.getEndPose(),
                        pose.of(80, -20),
                        pose.of(60, 0, 0)
                )
                .interpolateWith(HolonomicInterpolationStyle.TANGENT_CUSTOM, Angle.fromDeg(90))
                .profiledBuild();

        // ---------------------------------------------------------

        // 8. FACING_POINT
        // Uses the same heading spline infrastructure as NODE_BASED, but populates it by aiming
        // at one fixed field point throughout the path. The 90-degree offset points the robot's
        // side at the point.
        facingPointPath = Builder.holonomicPath(
                        tangentCustomPath.getEndPose(),
                        pose.of(45, -10),
                        pose.of(30, 0)
                )
                .interpolateWith(HolonomicInterpolationStyle.FACING_POINT,
                        Vector.of(30, 30, distUnit), Angle.fromDeg(90))
                .profiledBuild();

        // ---------------------------------------------------------

        // 9. NODE_BASED (C2 CUBIC SPLINE HEADING)
        // Replaces the lambda function. Constructs a perfectly continuous heading spline
        // to control the orientation explicitly at specific distance percentages (s).
        nodeBasedPath = Builder.holonomicPath(
                        facingPointPath.getEndPose(),
                        pose.of(15, 0),
                        pose.of(0, 0, 0)
                )
                // Evaluates the shortest angular delta to prevent 360-degree snapback bugs
                .addHeadingNode(0.0, Angle.fromDeg(90))
                .addHeadingNode(0.5, Angle.fromDeg(270))
                .addHeadingNode(1.0, Angle.fromDeg(450)) // Forces a continuous relative rotation
                .addDistanceCallback(1.0, this::finishRoutine)
                .profiledBuild();
    }

    /**
     * Helper to retrieve the full, pre-compiled routine for the Follower's state machine.
     *
     * @return An array of sequenced FollowerMovements.
     */
    public FollowerMovement[] getAutoRoutine() {
        return new FollowerMovement[]{
                smoothBlendPath,
                constStartPath,
                constEndPath,
                callbackTurn,
                tangentForwardPath,
                tankOptimalPath,
                tangentCustomPath,
                facingPointPath,
                nodeBasedPath
        };
    }
}
