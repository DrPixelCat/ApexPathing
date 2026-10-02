package core;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import drivetrains.Mecanum;
import drivetrains.Motor;
import geometry.Angle;
import geometry.DistUnit;
import geometry.Vector;
import org.junit.Test;
import static org.junit.Assert.assertEquals;

public class HolonomicCommandAllocatorTest {
    private final Motor unusedMotor = new Motor() {
        @Override public DcMotorEx build(HardwareMap hardwareMap) { return null; }
    };
    private final HolonomicCommandAllocator allocator = new HolonomicCommandAllocator(
            new Mecanum(new Mecanum.Constants().setFrontLeftMotor(unusedMotor)
                    .setFrontRightMotor(unusedMotor), null), null);

    private HolonomicCommandAllocator.Result allocate(double correction, double drive,
                                                       double turn, double velocity) {
        Vector zero = Vector.of(0, 0, DistUnit.IN);
        return allocator.allocate(zero, zero, Vector.of(1, 0, DistUnit.IN),
                Angle.fromRad(0), 0, correction, drive, velocity, 0, turn);
    }

    @Test public void saturatedFeedforwardPreservesTranslationAndBrakingRatio() {
        HolonomicCommandAllocator.Result result = allocate(0.2, 0.8, -0.4, 0.5);
        assertEquals(0.8 * 2 / 3, result.drive.getX().getIn(), 1e-12);
        assertEquals(0.2 - 0.4 * 2 / 3, result.turn, 1e-12);
        assertEquals(1, result.demand.correctionScale, 1e-12);
        assertEquals(2.0 / 3, result.demand.feedforwardScale, 1e-12);
        assertEquals(0, result.demand.velocityScale, 1e-12);
    }

    @Test public void unsaturatedCommandsRetainTheirFullValues() {
        HolonomicCommandAllocator.Result result = allocate(0.1, 0.2, -0.1, 0.15);
        assertEquals(0.35, result.drive.getX().getIn(), 1e-12);
        assertEquals(0, result.turn, 1e-12);
        assertEquals(1, result.demand.feedforwardScale, 1e-12);
        assertEquals(1, result.demand.velocityScale, 1e-12);
    }

    @Test public void correctionCanStillUseTheEntireBudget() {
        HolonomicCommandAllocator.Result result = allocate(2, 0.8, -0.4, 0.5);
        assertEquals(0, result.drive.getX().getIn(), 1e-12);
        assertEquals(1, result.turn, 1e-12);
        assertEquals(0.5, result.demand.correctionScale, 1e-12);
        assertEquals(0, result.demand.feedforwardScale, 1e-12);
    }
    @Test public void independentStrafeFeedforwardSharesTranslationAndTurningBudget() {
        Vector zero = Vector.of(0, 0, DistUnit.IN);
        HolonomicCommandAllocator.Result result = allocator.allocate(zero, zero,
                Vector.of(1, 0, DistUnit.IN), Angle.fromRad(0), 0, .1, .6, 0, 0, .2,
                Vector.of(0, .4, DistUnit.IN));
        assertEquals(.75, result.demand.feedforwardScale, 1e-9);
        assertEquals(.45, result.drive.getX().getIn(), 1e-9);
        assertEquals(.3, result.drive.getY().getIn(), 1e-9);
        assertEquals(.25, result.turn, 1e-9);
    }
}
