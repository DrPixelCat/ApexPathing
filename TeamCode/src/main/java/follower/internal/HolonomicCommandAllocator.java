package follower.internal;

import follower.Follower;

import controllers.DriveController;
import controllers.DriveController.AllocatedCommand;
import drivetrains.BaseDrivetrain;
import drivetrains.Mecanum;
import geometry.Angle;
import geometry.Vector;

/**
 * Allocates holonomic correction, propulsion, and velocity commands into one motor-power budget.
 *
 * @author Joel H
 */
final class HolonomicCommandAllocator {
    static final class Result {
        final Vector drive;
        final double turn;
        final Follower.CommandDemand demand;
        final double[] wheelPowers;

        Result(Vector drive, double turn, Follower.CommandDemand demand) {
            this(drive, turn, demand, null);
        }
        Result(Vector drive, double turn, Follower.CommandDemand demand, double[] wheelPowers) {
            this.drive = drive;
            this.turn = turn;
            this.demand = demand;
            this.wheelPowers = wheelPowers;
        }
    }

    /** Preserve wheel-specific motor compensation, including the fourth wheel-power mode.
     * Reserve position correction first, then scale the entire feedforward together. */
    Result allocateWheels(Vector crossTrack, Vector tangent, Angle heading, double tangentCorrection,
                          double headingCorrection, double[] feedforward, double tangentVelocity,
                          double headingVelocity, Vector normalVelocity) {
        if (!(drivetrain instanceof Mecanum)) throw new IllegalStateException("Wheel model requires a mecanum drivetrain");
        Vector position = crossTrack.plus(tangent.times(tangentCorrection)).rotate(heading.times(-1));
        Vector velocity = tangent.times(tangentVelocity).plus(normalVelocity).rotate(heading.times(-1));
        double[] feedback = mix(position, headingCorrection), rate = mix(velocity, headingVelocity);
        double correctionDemand=peak(feedback), ffDemand=peak(feedforward), rateDemand=peak(rate);
        double correctionScale=1/Math.max(1,correctionDemand);
        double remaining=Math.max(0,1-correctionDemand*correctionScale);
        double ffScale=remainingScale(ffDemand,remaining);
        double rateScale=remainingScale(rateDemand,Math.max(0,remaining-ffDemand*ffScale));
        double[] powers=new double[4];
        for(int i=0;i<4;i++) powers[i]=feedback[i]*correctionScale+feedforward[i]*ffScale+rate[i]*rateScale;
        double x=(powers[0]+powers[1]+powers[2]+powers[3])/4;
        double y=(-powers[0]+powers[1]+powers[2]-powers[3])/4;
        double turn=(-powers[0]+powers[1]-powers[2]+powers[3])/4;
        Follower.CommandDemand demand = new Follower.CommandDemand(true, crossTrack.getMag().getIn(),
                Math.abs(tangentCorrection), Math.abs(headingCorrection), 0, Math.abs(tangentVelocity),
                Math.abs(headingVelocity), Math.abs((feedforward[0]+feedforward[1]+feedforward[2]+feedforward[3])/4),
                Math.abs((-feedforward[0]+feedforward[1]-feedforward[2]+feedforward[3])/4),
                correctionDemand,rateDemand,ffDemand,correctionDemand+rateDemand+ffDemand,
                correctionScale,ffScale,rateScale);
        return new Result(trajectory.TrajectorySample.vector(x,y),turn,demand,powers);
    }
    private static double[] mix(Vector body,double turn) {
        double x=body.getX().getIn(), y=body.getY().getIn();
        return new double[]{x-y-turn,x+y+turn,x+y-turn,x-y+turn};
    }
    private static double peak(double[] powers) {
        double max=0;for(double power:powers) max=Math.max(max,Math.abs(power));return max;
    }

    private final BaseDrivetrain<?> drivetrain;
    private final DriveController driveController;
    private final drivetrains.models.DriveModel physicalModel;

    HolonomicCommandAllocator(BaseDrivetrain<?> drivetrain, DriveController driveController, drivetrains.models.DriveModel model) {
        this.drivetrain = drivetrain;
        this.driveController = driveController;
        physicalModel=model;
    }

    private void requireHolonomic() {
        if (!drivetrain.isHolonomic()) {
            throw new IllegalStateException(
                    "A holonomic allocation was requested while the drivetrain was non-holonomic.");
        }
    }

    AllocatedCommand allocatePositionHold(Vector fieldCommand, Angle currentHeading,
                                          double availablePower) {
        requireHolonomic();
        return driveController.allocatePositionHold(fieldCommand, currentHeading, availablePower, physicalModel);
    }

    Result allocate(Vector crossTrackCorrection, Vector centripetalCorrection, Vector unitTangent,
                    Angle currentHeading, double tangentFeedback, double headingFeedback,
                    double tangentFeedforward, double tangentVelocityFeedback,
                    double headingVelocityFeedback, double headingFeedforward) {
        return allocate(crossTrackCorrection, centripetalCorrection, unitTangent, currentHeading,
                tangentFeedback, headingFeedback, tangentFeedforward, tangentVelocityFeedback,
                headingVelocityFeedback, headingFeedforward, unitTangent.times(0));
    }

    Result allocate(Vector crossTrackCorrection, Vector centripetalCorrection, Vector unitTangent,
                    Angle currentHeading, double tangentFeedback, double headingFeedback,
                    double tangentFeedforward, double tangentVelocityFeedback,
                    double headingVelocityFeedback, double headingFeedforward, Vector normalFeedforward) {
        return allocate(crossTrackCorrection, centripetalCorrection, unitTangent, currentHeading,
                tangentFeedback, headingFeedback, tangentFeedforward, tangentVelocityFeedback,
                headingVelocityFeedback, headingFeedforward, normalFeedforward, unitTangent.times(0));
    }

    Result allocate(Vector crossTrackCorrection, Vector centripetalCorrection, Vector unitTangent,
                    Angle currentHeading, double tangentFeedback, double headingFeedback,
                    double tangentFeedforward, double tangentVelocityFeedback,
                    double headingVelocityFeedback, double headingFeedforward, Vector normalFeedforward,
                    Vector centripetalFeedforward) {
        return allocate(crossTrackCorrection, centripetalCorrection, unitTangent, currentHeading,
                tangentFeedback, headingFeedback, tangentFeedforward, tangentVelocityFeedback,
                headingVelocityFeedback, headingFeedforward, normalFeedforward, centripetalFeedforward, Vector.zero());
    }

    Result allocate(Vector crossTrackCorrection, Vector centripetalCorrection, Vector unitTangent,
                    Angle currentHeading, double tangentFeedback, double headingFeedback,
                    double tangentFeedforward, double tangentVelocityFeedback,
                    double headingVelocityFeedback, double headingFeedforward, Vector normalFeedforward,
                    Vector centripetalFeedforward, Vector normalVelocityFeedback) {
        requireHolonomic();

        Vector feedbackRobot = crossTrackCorrection.plus(centripetalCorrection)
                .plus(unitTangent.times(tangentFeedback))
                .rotate(Angle.fromRad(-currentHeading.getRad()));
        double rawFeedbackDemand = physicalModel.commandDemand(feedbackRobot, headingFeedback);
        double feedbackScale = Math.max(1.0, rawFeedbackDemand);
        feedbackRobot = feedbackRobot.times(1.0 / feedbackScale);
        headingFeedback /= feedbackScale;
        double feedbackDemand = physicalModel.commandDemand(feedbackRobot, headingFeedback);

        Vector feedforwardRobot = unitTangent.times(tangentFeedforward).plus(normalFeedforward).plus(centripetalFeedforward)
                .rotate(Angle.fromRad(-currentHeading.getRad()));
        double rawHeadingFeedforwardDemand = Math.abs(headingFeedforward);
        double feedforwardDemand = physicalModel.commandDemand(feedforwardRobot, headingFeedforward);
        double remaining = Math.max(0.0, 1.0 - feedbackDemand);
        // The profile plans translation and rotation together. Preserve that ratio,
        // including angular braking, after reserving power for position correction.
        double feedforwardScale = remainingScale(feedforwardDemand, remaining);
        feedforwardRobot = feedforwardRobot.times(feedforwardScale);
        headingFeedforward *= feedforwardScale;
        remaining = Math.max(0.0,
                remaining - feedforwardDemand * feedforwardScale);

        Vector velocityFeedbackRobot = unitTangent.times(tangentVelocityFeedback).plus(normalVelocityFeedback)
                .rotate(Angle.fromRad(-currentHeading.getRad()));
        double velocityFeedbackDemand = physicalModel.commandDemand(velocityFeedbackRobot, headingVelocityFeedback);
        double rawHeadingVelocityDemand = Math.abs(headingVelocityFeedback);
        double velocityFeedbackScale = remainingScale(velocityFeedbackDemand, remaining);
        velocityFeedbackRobot = velocityFeedbackRobot.times(velocityFeedbackScale);
        headingVelocityFeedback *= velocityFeedbackScale;

        Follower.CommandDemand commandDemand = new Follower.CommandDemand(true,
                crossTrackCorrection.getMag().getIn(), Math.abs(tangentFeedback),
                Math.abs(headingFeedback * feedbackScale),
                centripetalCorrection.getMag().getIn() + centripetalFeedforward.getMag().getIn(),
                Math.abs(tangentVelocityFeedback), rawHeadingVelocityDemand,
                Math.abs(tangentFeedforward), rawHeadingFeedforwardDemand,
                rawFeedbackDemand, velocityFeedbackDemand, feedforwardDemand,
                rawFeedbackDemand + velocityFeedbackDemand + feedforwardDemand,
                1.0 / feedbackScale, feedforwardScale, velocityFeedbackScale);

        return new Result(
                feedbackRobot.plus(feedforwardRobot).plus(velocityFeedbackRobot),
                headingFeedback + headingFeedforward + headingVelocityFeedback,
                commandDemand);
    }

    static double normalizationScale(double x, double y, double turn, boolean anisotropic) {
        double translation = anisotropic ? Math.abs(x) + Math.abs(y) : Math.hypot(x, y);
        return Math.max(1.0, translation + Math.abs(turn));
    }

    static double remainingScale(double demand, double available) {
        return demand > available && demand > 1e-12 ? available / demand : 1.0;
    }

}
