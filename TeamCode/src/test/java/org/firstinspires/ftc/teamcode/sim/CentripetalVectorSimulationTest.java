package org.firstinspires.ftc.teamcode.sim;

import follower.Follower;
import follower.FollowerConstants;
import geometry.Vector;
import org.firstinspires.ftc.teamcode.apexpathing.Constants;
import org.firstinspires.ftc.teamcode.apexpathing.FollowerTuner;
import org.json.JSONObject;
import org.junit.Test;
import tuning.follower.TunerContext;
import tuning.follower.centripetal.CentripetalPhase;
import trajectory.TrajectorySample;
import paths.movements.Path;
import java.lang.reflect.Field;
import java.io.File;
import java.io.PrintWriter;
import feedforward.HolonomicFeedforward;
import static org.junit.Assert.*;

/** Traces the real tuner's centripetal vector through both traversal directions. */
public class CentripetalVectorSimulationTest {
    private static class Phase extends CentripetalPhase {
        Phase(TunerContext context) { super(context); }
        void begin() { init(); }
        boolean tick() { return autoTuned(); }
    }
    @Test(timeout = 60000) public void tunerVectorsPointTowardTheArcCenterOnBothLegs() throws Exception {
        Field singleton = FollowerConstants.class.getDeclaredField("instance"); singleton.setAccessible(true);
        Object previous = singleton.get(null);
        try {
            FollowerConstants constants = FollowerConstants.fromJson(new JSONObject().put("drivetrainType", "MECANUM"));
            singleton.set(null, constants);
            ApexSimulation.Hardware hardware = ApexSimulation.createHardware();
            Follower follower = new Follower(new Constants(), hardware.hardwareMap);
            SimulatorTuning.apply(constants);
            follower.setHeadingCoefficients(constants.angularCoeffs);
            follower.setDriveCoefficients(constants.translationalCoeffs);
            FollowerTuner opmode = new FollowerTuner();
            opmode.telemetry = new ApexSimTelemetry(frame -> { });
            TunerContext context = new TunerContext(opmode); context.setFollower(follower);
            Phase phase = new Phase(context); phase.begin();
            Field trial = CentripetalPhase.class.getDeclaredField("trialNumber"); trial.setAccessible(true);
            // Check the entire heading schedule, including the terminal blend region.
            // This fails with the former authored 0/90-degree endpoint headings.
            for (String name : new String[]{"forwardArc", "backwardArc"}) {
                Field field = CentripetalPhase.class.getDeclaredField(name); field.setAccessible(true);
                Path arc = (Path) field.get(phase);
                for (double t : new double[]{0, .25, .5, .75, .95, 1}) {
                    geometry.PathPoint point = arc.getParametricPath().getPointAtDistance(t * arc.getParametricPath().getLengthIn());
                    geometry.Angle heading = arc.getInterpolator().getHeadingTarg(point.getDistanceToEndIn(),
                            point.getFirstDerivative(), arc.getParametricPath().getFirstDerivative(1));
                    Vector body = point.getFirstDerivative().normalize().rotate(heading.times(-1));
                    assertEquals("Tuner heading must stay aligned with travel on " + name, 0, body.getY().getIn(), 1e-9);
                }
            }
            int outbound = 0, returning = 0;
            double maximumError = 0, minimumAlignment = 1;
            File traceFile = new File("build/centripetal-vector-trace.csv");
            traceFile.getParentFile().mkdirs();
            try (PrintWriter csv = new PrintWriter(traceFile)) {
                csv.println("return,t,error,measured_v,target_v,centripetal_in,axis_feedforward_in,heading_error,total_power_in");
                long deadline = System.nanoTime() + 50000000000L;
                while (trial.getInt(phase) == 1 && System.nanoTime() < deadline) {
                    StockSimTestSupport.step(hardware, .02); follower.update();
                    TrajectorySample target = follower.getTrajectoryTarget();
                    if (target != null && target.pathT > .25 && target.pathT < .75) {
                        Vector inward = TrajectorySample.vector(-16, 16).minus(target.positionOnPath).normalize();
                        Vector correction = follower.getCentripetalCorrection();
                        assertTrue("Centripetal command should be nonzero", correction.getMag().getIn() > 1e-8);
                        double alignment = inward.dot(correction.normalize()).getIn();
                        minimumAlignment = Math.min(minimumAlignment, alignment);
                        assertTrue("Centripetal command points away from center: " + correction, alignment > .99);
                        Vector axisPower = HolonomicFeedforward.translation(constants,
                                target.tangent.rotate(target.heading.times(-1)), target.tangentialVelocity(),
                                target.tangentialAcceleration(), constants.translationalKV).rotate(target.heading);
                        assertEquals("Axis feedforward must not push across the centripetal test arc",
                                0, axisPower.dot(inward).getIn(), .001);
                        double fl = follower.getDrivetrain().getLastFlPower(), fr = follower.getDrivetrain().getLastFrPower();
                        double bl = follower.getDrivetrain().getLastBlPower(), br = follower.getDrivetrain().getLastBrPower();
                        Vector total = TrajectorySample.vector((fl + fr + bl + br) / 4, (-fl + fr + bl - br) / 4)
                                .rotate(follower.getPose().getHeading());
                        csv.println((target.tangent.getX().getIn() < 0) + "," + target.pathT + ","
                                + follower.getCentripetalErrorIn() + "," + follower.getVelocity().getVec().dot(target.tangent).getIn()
                                + "," + target.tangentialVelocity() + "," + correction.dot(inward).getIn()
                                + "," + axisPower.dot(inward).getIn() + ","
                                + follower.getPose().getHeading().getShortestAngleTo(target.heading).getRad()
                                + "," + total.dot(inward).getIn());
                        maximumError = Math.max(maximumError, Math.abs(follower.getCentripetalErrorIn()));
                        if (target.tangent.getX().getIn() > 0) { outbound++; } else { returning++; }
                    }
                    phase.tick(); Thread.sleep(20);
                }
            }
            follower.stop();
            System.out.println("Centripetal trace: outbound=" + outbound + ", return=" + returning
                    + ", minimum inward alignment=" + minimumAlignment + ", maximum middle-arc error=" + maximumError);
            assertTrue("Outbound leg was not traced", outbound > 0);
            assertTrue("Return leg was not traced", returning > 0);
            assertTrue("Trial failed to finish", trial.getInt(phase) > 1);
        } finally { singleton.set(null, previous); }
    }
}
