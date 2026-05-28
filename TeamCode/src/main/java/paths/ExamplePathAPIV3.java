package paths;

import paths.builders.PathBuilder;
import paths.builders.TurnBuilder;
import paths.movements.FollowerMovement;
import paths.movements.Path;
import paths.movements.Turn;

import util.Angle;
import util.Distance;
import util.Pose;
import util.PoseFactory;

public class ExamplePathAPIV3 {
    private Distance.Units distUnit = Distance.Units.INCHES;
    private Angle.Units angleUnit = Angle.Units.DEGREES;
    public PoseFactory pose = new PoseFactory(distUnit, angleUnit);
    private Pose startPose;

    // We can store our routine as a sequence of FollowerMovements
    public Path testPath;
    public Turn testTurn;

    public ExamplePathAPIV3(boolean mirror) {
        pose.setMirror(mirror);
        // Uses the builder's build method to apply units and mirroring
        startPose = pose.build(0, 0, 0);

        buildRoutine();
    }

    public void exampleCallback() {
        // This will run when the follower reaches a specific progression or angle
    }

    /**
     * A comprehensive showcase of the Movement Builder API.
     * GitHub snoopers: this is still subject to change. Please send suggestions if you have any!
     */
    private void buildRoutine() {

        // 1. THE CORE B-SPLINE
        // Demonstrating standard routing, auto-tightening, and educational warnings
        testPath = new PathBuilder(startPose)
                // A B-Spline can be created with 2 points in Apex because of ghost points that are added during construction
                .addControlPoints(
                        pose.at(15, 0),              // Standard waypoint
                        pose.at(25, 0, 90),          // INTENTIONAL WARNING: Apex will ignore this intermediate heading and warn the user!
                        pose.arcPoseAt(25, 25, 10),  // ArcEnforcement: Forces large, relaxed curves into a sharper turn with a 10in radius
                        pose.at(45, 25, 45)          // The final waypoint dictates the target heading for the end of this curve
                )

                // 2. DISTANCE CALLBACK: Triggers our custom function exactly halfway (s=0.5) down the curve
                .addDistanceCallback(0.5, this::exampleCallback)

                // 3. ANGULAR CALLBACK: Triggers precisely when the robot rotates past the 180-degree mark
                .addAngularCallback(new Angle(Math.PI), this::exampleCallback)

                // 4. ADVANCED LAMBDA INTERPOLATOR
                // Overrides the previous curve's heading logic with custom math.
                // Here, we command the robot to do a full 360-degree tornado spin over the course of the curve.
                .interpolateWith(s -> Angle.fromDeg(180 + (s * 360.0)))

                // 5. COMPILE: Locks the path, calculates all Look-Up Tables, and finalizes geometry.
                .build();

        // ---------------------------------------------------------

        // 6. THE TURN BUILDER
        // Seamlessly starts EXACTLY where the last path ended using .getEndPose()
        testTurn = new TurnBuilder(testPath.getEndPose())
                // Defines the final heading the robot should rotate to
                .turnTo(new Angle(Math.PI / 2))

                // Safety validated callback during the spin!
                .addAngularCallback(new Angle(Math.PI / 3), this::exampleCallback)

                // Locks the turn and finalizes the callback math
                .build();
    }

    /**
     * Optional helper to retrieve the full routine for a state machine.
     */
    public FollowerMovement[] getAutoRoutine() {
        return new FollowerMovement[] { testPath, testTurn };
    }
}