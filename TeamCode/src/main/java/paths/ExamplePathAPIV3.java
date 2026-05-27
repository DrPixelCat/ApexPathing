package paths;

import util.Angle;
import util.Distance;
import util.Pose;
import util.PoseFactory;

public class ExamplePathAPIV3 {
    private Distance.Units distUnit = Distance.Units.INCHES;
    private Angle.Units angleUnit = Angle.Units.DEGREES;
    public PoseFactory pose = new PoseFactory(distUnit, angleUnit);
    private Pose startPose;

    public ExamplePathAPIV3(boolean mirror) {
        pose.setMirror(mirror);
        // Uses the builder's build method to apply units and mirroring
        startPose = pose.build(0, 0, 0);
    }

    public void exampleCallback() {
        // This will run when the follower reaches 50% of the path segment
    }

    /**
     * A comprehensive showcase of every feature available in the BSplinePathBuilder API.
     * GitHub snoopers: this is still subject to change. Please send suggestions if you have any!
     */
    public Path testPath() {
        return new PathBuilder(startPose) // TODO: I almost wish startPose was in addControlPoints, but I can't think of a good way to do it :\
                // 1. THE CORE B-SPLINE: Demonstrating standard routing, auto-tightening, and educational warnings
                // A B-Spline can be created with 2 points in Apex because of ghost points that are added during construction
                .addControlPoints(
                        pose.at(15, 0),             // Standard waypoint
                        pose.at(25, 0, 90),         // INTENTIONAL WARNING: Apex will ignore this intermediate heading and warn the user!
                        pose.arcPoseAt(25, 25, 10),  // ArcEnforcement: Forces large, relaxed curves into a sharper turn with a 10in radius while maintaining C2 continuity
                        pose.at(45, 25, 45)         // The final waypoint dictates the target heading for the end of this curve
                )

                // 2. IN-LINE CALLBACK: Triggers our custom function exactly halfway (s=0.5) down the curve above
                .addCallback(0.5, this::exampleCallback) //TODO: Xenon plz review this I just made something rq

                // 3. POINT TURN: The robot stays at (45, 25) and rotates in place to face 180 degrees
                .turnTo(Angle.fromDeg(180))

                // 4. ADVANCED LAMBDA INTERPOLATOR: Overrides the previous curve's heading logic with custom math.
                // Here, we command the robot to do a full 360-degree tornado spin over the course of the curve.
                .interpolateWith(s -> Angle.fromDeg(180 + (s * 360.0)))

                // 5. COMPILE: Locks the path, calculates all Look-Up Tables, and finalizes geometry.
                .build();
    }
}