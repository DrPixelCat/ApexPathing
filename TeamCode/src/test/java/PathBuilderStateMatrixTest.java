import org.junit.Assert;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

import controllers.PDSController;
import core.FollowerConstants;
import feedforward.MotionParameters;
import feedforward.holonomic.mecanum.MecanumProfileGenerator;
import feedforward.holonomic.swerve.SwerveProfileGenerator;
import feedforward.tank.TankProfileGenerator;
import geometry.Angle;
import geometry.Dist;
import paths.builders.HolonomicPathBuilder;
import paths.builders.TankPathBuilder;
import paths.heading.HolonomicInterpolationStyle;
import paths.heading.TankInterpolationStyle;
import paths.movements.Path;
import util.AngleUnit;
import util.DistUnit;
import util.PoseFactory;

public class PathBuilderStateMatrixTest {
    private final PoseFactory poseFac = new PoseFactory(DistUnit.IN, AngleUnit.DEG);

    @Test
    public void allPathBuilderDrivetrainInterpolationStatesGenerateProfiles() {
        List<String> failures = new ArrayList<>();
        int casesRun = 0;

        for (TankInterpolationStyle style : TankInterpolationStyle.values()) {
            casesRun++;
            runCase(failures, "Tank/" + style, () -> {
                Path path = new TankPathBuilder(
                        poseFac.of(0, -50, 90),
                        poseFac.arcPoseOf(0, 50, 30),
                        poseFac.of(100, 50, 0))
                        .interpolateWith(style)
                        .quickBuild();
                MotionParameters[] profile =
                        new TankProfileGenerator(constants(FollowerConstants.DrivetrainType.TANK),
                                path).generate();
                assertFiniteProfile("Tank/" + style, profile);
            });
        }

        for (HolonomicInterpolationStyle style : HolonomicInterpolationStyle.values()) {
            casesRun++;
            runCase(failures, "CoaxialSwerve/" + style, () -> {
                Path path = buildHolonomicPath(style);
                MotionParameters[] profile =
                        new SwerveProfileGenerator(
                                constants(FollowerConstants.DrivetrainType.COAXIAL_SWERVE),
                                path).generate();
                assertFiniteProfile("CoaxialSwerve/" + style, profile);
            });

            casesRun++;
            runCase(failures, "Mecanum/" + style, () -> {
                Path path = buildHolonomicPath(style);
                MotionParameters[] profile =
                        new MecanumProfileGenerator(
                                constants(FollowerConstants.DrivetrainType.MECANUM),
                                path).generate();
                assertFiniteProfile("Mecanum/" + style, profile);
            });
        }

        if (!failures.isEmpty()) {
            Assert.fail("Path builder state matrix failures:\n" + joinFailures(failures));
        }

        System.out.println("PathBuilderStateMatrixTest passed " + casesRun + " states.");
    }

    private Path buildHolonomicPath(HolonomicInterpolationStyle style) {
        HolonomicPathBuilder builder = new HolonomicPathBuilder(
                poseFac.of(0, -50, 90),
                poseFac.arcPoseOf(0, 50, 30),
                poseFac.of(100, 50, 0));

        if (style == HolonomicInterpolationStyle.NODE_BASED) {
            return builder
                    .addHeadingNode(0.0, Angle.fromDeg(90))
                    .addHeadingNode(0.5, Angle.fromDeg(30))
                    .addHeadingNode(1.0, Angle.fromDeg(0))
                    .quickBuild();
        }

        if (style == HolonomicInterpolationStyle.TANGENT_CUSTOM) {
            return builder
                    .interpolateWith(style, Angle.fromDeg(15))
                    .quickBuild();
        }

        return builder
                .interpolateWith(style)
                .quickBuild();
    }

    private FollowerConstants constants(FollowerConstants.DrivetrainType drivetrainType) {
        return new FollowerConstants().inject(
                drivetrainType,
                new PDSController.PDSCoefficients(12, 0.8, 0.04, 0.0),
                new PDSController.PDSCoefficients(12, 1.3, 0.04, 0.0),
                2.5,
                0.0135,
                0.0026,
                0.011,
                0.0021,
                0.001,
                Dist.fromIn(72.5),
                Dist.fromIn(62.5),
                Dist.fromIn(58),
                Dist.fromIn(50),
                Angle.fromDeg(291),
                Angle.fromDeg(2000),
                Angle.fromDeg(0.5),
                Dist.fromIn(0.5)
        );
    }

    private void runCase(List<String> failures, String name, CaseBody body) {
        try {
            body.run();
        } catch (Throwable t) {
            failures.add(name + ": " + t.getClass().getSimpleName() + " - " +
                    t.getMessage() + topStackFrame(t));
        }
    }

    private void assertFiniteProfile(String name, MotionParameters[] profile) {
        Assert.assertNotNull(name + " profile should not be null", profile);
        Assert.assertTrue(name + " profile should contain samples", profile.length > 2);

        for (int i = 0; i < profile.length; i++) {
            MotionParameters point = profile[i];
            assertFinite(name, i, "velocity", point.getTangentialVel());
            assertFinite(name, i, "acceleration", point.getTangentialAccel());
            assertFinite(name, i, "angular velocity", point.getAngularVel());
            assertFinite(name, i, "angular acceleration", point.getAngularAccel());
            assertFinite(name, i, "motor power", point.getMotorPower());
        }
    }

    private void assertFinite(String name, int index, String field, double value) {
        Assert.assertTrue(name + " " + field + " should be finite at index " + index +
                ", was " + value, Double.isFinite(value));
    }

    private String joinFailures(List<String> failures) {
        StringBuilder sb = new StringBuilder();
        for (String failure : failures) {
            if (sb.length() > 0) {
                sb.append('\n');
            }
            sb.append(failure);
        }
        return sb.toString();
    }

    private String topStackFrame(Throwable t) {
        StackTraceElement[] trace = t.getStackTrace();
        if (trace.length == 0) {
            return "";
        }
        return " @ " + trace[0];
    }

    private interface CaseBody {
        void run();
    }
}