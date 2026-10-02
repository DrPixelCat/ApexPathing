package trajectory;

import geometry.Angle;
import geometry.DistUnit;
import geometry.PathPoint;
import geometry.PathSegment;
import geometry.Vector;
import feedforward.MotionParameters;
import paths.movements.Path;

/** Immutable physical target shared by online and stored trajectories. Units: inches, radians, seconds. */
public final class TrajectorySample {
    public final Vector velocity, acceleration, toPath, tangent, positionOnPath;
    public final double progressIn, remainingIn, pathT, angularVelocity, angularAcceleration;
    public final double angularProgressRad, remainingAngleRad;
    public final Angle heading;
    public final boolean overspeed, outsideGrid;

    public TrajectorySample(Vector velocity, Vector acceleration, Vector toPath, Vector tangent,
                            Vector positionOnPath, double progressIn, double remainingIn, double pathT,
                            Angle heading, double angularVelocity, double angularAcceleration,
                            boolean overspeed, boolean outsideGrid) {
        this(velocity, acceleration, toPath, tangent, positionOnPath, progressIn, remainingIn, pathT,
                heading, angularVelocity, angularAcceleration, overspeed, outsideGrid, 0, 0);
    }

    public TrajectorySample(Vector velocity, Vector acceleration, Vector toPath, Vector tangent,
                            Vector positionOnPath, double progressIn, double remainingIn, double pathT,
                            Angle heading, double angularVelocity, double angularAcceleration,
                            boolean overspeed, boolean outsideGrid, double angularProgressRad, double remainingAngleRad) {
        this.angularProgressRad = angularProgressRad; this.remainingAngleRad = remainingAngleRad;
        this.velocity = velocity; this.acceleration = acceleration; this.toPath = toPath;
        this.tangent = tangent; this.positionOnPath = positionOnPath;
        this.progressIn = progressIn; this.remainingIn = remainingIn; this.pathT = pathT;
        this.heading = heading; this.angularVelocity = angularVelocity;
        this.angularAcceleration = angularAcceleration;
        this.overspeed = overspeed; this.outsideGrid = outsideGrid;
    }

    public double tangentialVelocity() { return velocity.dot(tangent).getIn(); }
    public double tangentialAcceleration() { return acceleration.dot(tangent).getIn(); }
    public Vector normalAcceleration() { return acceleration.minus(tangent.times(tangentialAcceleration())); }

    public TrajectorySample withAngularMotion(double omega, double alpha) {
        return new TrajectorySample(velocity, acceleration, toPath, tangent, positionOnPath,
                progressIn, remainingIn, pathT, heading, omega, alpha, overspeed, outsideGrid,
                angularProgressRad, remainingAngleRad);
    }

    public static TrajectorySample onPath(Path path, double distance, Vector robot,
                                           MotionParameters motion, boolean overspeed) {
        return onPath(path, distance, robot, motion, overspeed, motion.getTangentialVel());
    }

    public static TrajectorySample onPath(Path path, double distance, Vector robot,
                                           MotionParameters motion, boolean overspeed, double normalVelocity) {
        PathSegment segment = path.getParametricPath();
        PathPoint point = segment.getPointAtDistance(distance);
        Vector tangent = point.getFirstDerivative().normalize();
        Vector normal = PathSegment.calculateLeftNormal(tangent);
        double v = motion.getTangentialVel();
        return new TrajectorySample(tangent.times(v),
                tangent.times(motion.getTangentialAccel()).plus(normal.times(normalVelocity * normalVelocity * point.getSignedCurvature())),
                point.getLocation().minus(robot), tangent, point.getLocation(),
                segment.getLengthIn() - point.getDistanceToEndIn(), point.getDistanceToEndIn(), point.getT(),
                path.getInterpolator().getHeadingTarg(point.getDistanceToEndIn(), point.getFirstDerivative(), segment.getFirstDerivative(1)),
                motion.getAngularVel(), motion.getAngularAccel(), overspeed, false);
    }

    public static Vector vector(double x, double y) { return Vector.of(x, y, DistUnit.IN); }
}
