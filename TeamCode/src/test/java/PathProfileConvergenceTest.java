import org.junit.Test;
import org.knowm.xchart.BitmapEncoder;
import org.knowm.xchart.BitmapEncoder.BitmapFormat;
import org.knowm.xchart.XYChart;
import org.knowm.xchart.XYChartBuilder;
import org.knowm.xchart.XYSeries;
import org.knowm.xchart.style.Styler;

import org.junit.Assert;

import controllers.PDSController;
import core.FollowerConstants;
import drivetrains.Mecanum;
import drivetrains.Mecanum.MecanumDirectionalLut.DirectionalKinematics;
import feedforward.BaseProfileGenerator;
import feedforward.MotionParameters;
import feedforward.holonomic.mecanum.MecanumProfileGenerator;
import feedforward.holonomic.swerve.SwerveProfileGenerator;
import feedforward.tank.TankProfileGenerator;
import geometry.Angle;
import geometry.Dist;
import geometry.PathPoint;
import geometry.Vector;
import paths.builders.HolonomicPathBuilder;
import paths.builders.TankPathBuilder;
import paths.heading.HolonomicInterpolationStyle;
import paths.movements.Path;
import util.AngleUnit;
import util.DistUnit;
import util.PoseFactory;

public class PathProfileConvergenceTest {
    FollowerConstants dummyConstants = new FollowerConstants().inject(
            FollowerConstants.DrivetrainType.MECANUM,
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

    PoseFactory poseFac = new PoseFactory(DistUnit.IN, AngleUnit.DEG);

    Path path = new HolonomicPathBuilder(
            poseFac.of(0, -50, 90),
            poseFac.of(0, 50, 0))
            .profiledBuild();

    BaseProfileGenerator generator = createProfileGenerator(dummyConstants, path);

    @Test
    public void testProfileConvergenceAndGraph() {
        MotionParameters[] initialParams = generator.generateInitialProfile();
        MotionParameters[] convergedParams = generator.generate();

        PathPoint[] points = path.getGeneratedPoints();
        double pathLength = path.getParametricPath().getLengthIn();

        double[] sData = new double[points.length];

        double[] vInit = new double[points.length];
        double[] pInit = new double[points.length];

        double[] vConv = new double[points.length];
        double[] aConv = new double[points.length];
        double[] pConv = new double[points.length];
        double[] wConv = new double[points.length];
        double[] alphaConv = new double[points.length];

        double[] fullPower = constantArray(points.length, 1.0);
        double[] zeroLine = constantArray(points.length, 0.0);
        double[] velocityLimit = new double[points.length];
        double[] positiveAccelLimit = new double[points.length];
        double[] negativeAccelLimit = new double[points.length];
        double[] positiveAngularVelLimit = constantArray(
                points.length, dummyConstants.angularVelocityLimit.getRad()
        );
        double[] negativeAngularVelLimit = constantArray(
                points.length, -dummyConstants.angularVelocityLimit.getRad()
        );
        double[] positiveAngularAccelLimit = constantArray(
                points.length, dummyConstants.angularAccelerationLimit.getRad()
        );
        double[] negativeAngularAccelLimit = constantArray(
                points.length, -dummyConstants.angularAccelerationLimit.getRad()
        );
        Mecanum.MecanumDirectionalLut mecanumLimits =
                dummyConstants.drivetrainType == FollowerConstants.DrivetrainType.MECANUM
                        ? new Mecanum.MecanumDirectionalLut(
                        dummyConstants.forwardVelocityLimit.getIn(),
                        dummyConstants.forwardAccelerationLimit.getIn(),
                        dummyConstants.strafeVelocityLimit.getIn(),
                        dummyConstants.strafeAccelerationLimit.getIn()
                )
                        : null;
        Vector finalTangent = path.getParametricPath().getFirstDerivative(1.0);

        for (int i = 0; i < points.length; i++) {
            sData[i] = pathLength - points[i].getDistanceToEnd_in();

            vInit[i] = initialParams[i].getTangentialVel();
            pInit[i] = initialParams[i].getMotorPower();

            vConv[i] = convergedParams[i].getTangentialVel();
            aConv[i] = convergedParams[i].getTangentialAccel();
            pConv[i] = convergedParams[i].getMotorPower();
            wConv[i] = convergedParams[i].getAngularVel();
            alphaConv[i] = convergedParams[i].getAngularAccel();

            if (mecanumLimits != null) {
                Angle heading = path.getInterpolator().getHeadingTarg(
                        points[i].getDistanceToEnd_in(), points[i].getFirstDerivative(),
                        finalTangent
                );
                DirectionalKinematics dirK =
                        mecanumLimits.getKinematics(points[i].getFirstDerivative(), heading);
                velocityLimit[i] = dirK.maxVel;
                positiveAccelLimit[i] = dirK.maxAccel;
                negativeAccelLimit[i] = -dirK.maxAccel;
            } else {
                velocityLimit[i] = dummyConstants.forwardVelocityLimit.getIn();
                positiveAccelLimit[i] = dummyConstants.forwardAccelerationLimit.getIn();
                negativeAccelLimit[i] = -dummyConstants.forwardAccelerationLimit.getIn();
            }
        }

        double averageOptimizedPower = average(pConv);
        double maxOptimizedPower = max(pConv);
        String drivetrainName = drivetrainName(dummyConstants.drivetrainType);
        String velocityLimitLabel = mecanumLimits != null
                ? "Directional velocity limit" : "Forward velocity limit";
        String accelLimitLabel = mecanumLimits != null
                ? "directional accel limit" : "accel limit";

        XYChart transChart = new XYChartBuilder()
                .width(1000).height(460)
                .title(drivetrainName + " Profile - Translational State")
                .xAxisTitle("Arc Length s Along Path (in)")
                .build();
        styleChart(transChart);
        transChart.setYAxisGroupTitle(0, "Velocity (in/s)");
        transChart.setYAxisGroupTitle(1, "Tangential Accel (in/s^2)");
        transChart.getStyler().setYAxisGroupPosition(1, Styler.YAxisPosition.Right);
        transChart.getStyler().setYAxisMin(0, 0.0);
        transChart.getStyler().setYAxisMax(0, max(velocityLimit) * 1.12);
        transChart.getStyler().setYAxisMin(1, min(negativeAccelLimit) * 1.15);
        transChart.getStyler().setYAxisMax(1, max(positiveAccelLimit) * 1.15);
        addSeries(transChart, "Optimized velocity", sData, vConv, 0);
        addSeries(transChart, velocityLimitLabel, sData, velocityLimit, 0);
        addSeries(transChart, "Optimized tangential accel", sData, aConv, 1);
        addSeries(transChart, "+" + accelLimitLabel, sData, positiveAccelLimit, 1);
        addSeries(transChart, "-" + accelLimitLabel, sData, negativeAccelLimit, 1);
        addSeries(transChart, "Zero accel", sData, zeroLine, 1);

        XYChart angChart = new XYChartBuilder()
                .width(1000).height(460)
                .title(drivetrainName + " Profile - Heading State")
                .xAxisTitle("Arc Length s Along Path (in)")
                .build();
        styleChart(angChart);
        angChart.setYAxisGroupTitle(0, "Angular Velocity omega (rad/s)");
        angChart.setYAxisGroupTitle(1, "Angular Accel alpha (rad/s^2)");
        angChart.getStyler().setYAxisGroupPosition(1, Styler.YAxisPosition.Right);
        addSeries(angChart, "Optimized omega", sData, wConv, 0);
        addSeries(angChart, "+omega limit", sData, positiveAngularVelLimit, 0);
        addSeries(angChart, "-omega limit", sData, negativeAngularVelLimit, 0);
        addSeries(angChart, "Optimized alpha", sData, alphaConv, 1);
        addSeries(angChart, "+alpha limit", sData, positiveAngularAccelLimit, 1);
        addSeries(angChart, "-alpha limit", sData, negativeAngularAccelLimit, 1);
        addSeries(angChart, "Zero alpha", sData, zeroLine, 1);

        XYChart powerChart = new XYChartBuilder()
                .width(1000).height(460)
                .title(String.format(
                        "%s Profile - Power Utilization (avg %.3f, max %.3f)",
                        drivetrainName, averageOptimizedPower, maxOptimizedPower
                ))
                .xAxisTitle("Arc Length s Along Path (in)")
                .yAxisTitle("Normalized Motor Utilization")
                .build();
        styleChart(powerChart);
        powerChart.getStyler().setYAxisMin(0.0);
        powerChart.getStyler().setYAxisMax(1.12);
        addSeries(powerChart, "Optimized utilization", sData, pConv, 0);
        addSeries(powerChart, "Full-power target", sData, fullPower, 0);

        System.out.println("--- GENERATOR DEBUG REPORT ---");
        System.out.println(generator.getLastDebugReport().getSummary());

        try {
            BitmapEncoder.saveBitmap(transChart, "Translational_Kinematics", BitmapFormat.PNG);
            BitmapEncoder.saveBitmap(angChart, "Angular_Kinematics", BitmapFormat.PNG);
            BitmapEncoder.saveBitmap(powerChart, "Power_Utilization", BitmapFormat.PNG);
            System.out.println("Success! Check your project folder for the PNG graphs.");
        } catch (Exception e) {
            System.out.println("Failed to save charts: " + e.getMessage());
        }
    }

    @Test
    public void testMecanumAndTankProfilesGenerateFiniteOutput() {
        Path mecanumPath = new HolonomicPathBuilder(
                poseFac.of(0, -50, 90),
                poseFac.arcPoseOf(0, 50, 30),
                poseFac.of(100, 50, 0))
                .interpolateWith(HolonomicInterpolationStyle.TANGENT_FORWARD)
                .quickBuild();
        MotionParameters[] mecanumProfile =
                generateProfile(constantsWithDrivetrain(FollowerConstants.DrivetrainType.MECANUM),
                        mecanumPath);
        assertUsableProfile("mecanum", mecanumProfile);

        Path tankPath = new TankPathBuilder(
                poseFac.of(0, -50, 90),
                poseFac.arcPoseOf(0, 50, 30),
                poseFac.of(100, 50, 0))
                .quickBuild();
        MotionParameters[] tankProfile =
                generateProfile(constantsWithDrivetrain(FollowerConstants.DrivetrainType.TANK),
                        tankPath);
        assertUsableProfile("tank", tankProfile);
    }

    private void styleChart(XYChart chart) {
        chart.getStyler().setLegendPosition(Styler.LegendPosition.OutsideE);
        chart.getStyler().setMarkerSize(0);
        chart.getStyler().setPlotGridLinesVisible(true);
        chart.getStyler().setPlotContentSize(0.92);
        chart.getStyler().setChartPadding(12);
        chart.getStyler().setXAxisDecimalPattern("0");
        chart.getStyler().setYAxisDecimalPattern("0.###");
        chart.getStyler().setDefaultSeriesRenderStyle(XYSeries.XYSeriesRenderStyle.Line);
        chart.getStyler().setLegendSeriesLineLength(44);
    }

    private void addSeries(XYChart chart, String name, double[] xData, double[] yData,
                           int yAxisGroup) {
        XYSeries series = chart.addSeries(name, xData, yData);
        series.setYAxisGroup(yAxisGroup);
    }

    private double[] constantArray(int length, double value) {
        double[] output = new double[length];
        for (int i = 0; i < length; i++) {
            output[i] = value;
        }
        return output;
    }

    private double average(double[] values) {
        double sum = 0.0;
        for (double value : values) {
            sum += value;
        }
        return values.length == 0 ? 0.0 : sum / values.length;
    }

    private double max(double[] values) {
        double max = 0.0;
        for (double value : values) {
            max = Math.max(max, value);
        }
        return max;
    }

    private double min(double[] values) {
        double min = 0.0;
        for (double value : values) {
            min = Math.min(min, value);
        }
        return min;
    }

    private BaseProfileGenerator createProfileGenerator(FollowerConstants constants, Path path) {
        switch (constants.drivetrainType) {
            case COAXIAL_SWERVE:
                requirePathType(path, Path.PathType.HOLONOMIC, constants.drivetrainType);
                return new SwerveProfileGenerator(constants, path);
            case MECANUM:
                requirePathType(path, Path.PathType.HOLONOMIC, constants.drivetrainType);
                return new MecanumProfileGenerator(constants, path);
            case TANK:
                requirePathType(path, Path.PathType.TANK, constants.drivetrainType);
                return new TankProfileGenerator(constants, path);
            default:
                throw new IllegalArgumentException(
                        "Unsupported convergence test drivetrain: " +
                                constants.drivetrainType
                );
        }
    }

    private MotionParameters[] generateProfile(FollowerConstants constants, Path path) {
        return createProfileGenerator(constants, path).generate();
    }

    private void requirePathType(Path path, Path.PathType expected,
                                 FollowerConstants.DrivetrainType drivetrainType) {
        if (path.getPathType() != expected) {
            throw new IllegalArgumentException(
                    "Convergence test drivetrain " + drivetrainType +
                            " requires a " + expected + " path, but got " + path.getPathType()
            );
        }
    }

    private String drivetrainName(FollowerConstants.DrivetrainType drivetrainType) {
        switch (drivetrainType) {
            case COAXIAL_SWERVE:
                return "Swerve";
            case MECANUM:
                return "Mecanum";
            case TANK:
                return "Tank";
            default:
                return drivetrainType.name();
        }
    }

    private FollowerConstants constantsWithDrivetrain(
            FollowerConstants.DrivetrainType drivetrainType) {
        return new FollowerConstants().inject(
                drivetrainType,
                dummyConstants.headingCoeffs,
                dummyConstants.translationalCoeffs,
                dummyConstants.velocityFeedbackGain,
                dummyConstants.translationalKV,
                dummyConstants.translationalKA,
                dummyConstants.angularKV,
                dummyConstants.angularKA,
                dummyConstants.Kcentripetal,
                dummyConstants.forwardVelocityLimit,
                dummyConstants.forwardAccelerationLimit,
                dummyConstants.strafeVelocityLimit,
                dummyConstants.strafeAccelerationLimit,
                dummyConstants.angularVelocityLimit,
                dummyConstants.angularAccelerationLimit,
                dummyConstants.headingTolerance,
                dummyConstants.distanceTolerance
        );
    }

    private void assertUsableProfile(String name, MotionParameters[] profile) {
        Assert.assertTrue(name + " profile should contain samples", profile.length > 2);

        double maxVelocity = 0.0;
        double maxPower = 0.0;
        for (MotionParameters point : profile) {
            Assert.assertTrue(name + " velocity must be finite",
                    Double.isFinite(point.getTangentialVel()));
            Assert.assertTrue(name + " acceleration must be finite",
                    Double.isFinite(point.getTangentialAccel()));
            Assert.assertTrue(name + " angular velocity must be finite",
                    Double.isFinite(point.getAngularVel()));
            Assert.assertTrue(name + " angular acceleration must be finite",
                    Double.isFinite(point.getAngularAccel()));
            Assert.assertTrue(name + " motor power must be finite",
                    Double.isFinite(point.getMotorPower()));
            Assert.assertTrue(name + " velocity should not go negative",
                    point.getTangentialVel() >= -1e-6);

            maxVelocity = Math.max(maxVelocity, point.getTangentialVel());
            maxPower = Math.max(maxPower, point.getMotorPower());
        }

        Assert.assertTrue(name + " should move", maxVelocity > 1.0);
        Assert.assertTrue(name + " should stay near normalized power", maxPower <= 1.05);
    }

    @Test
    public void testMecanumDirectionSpecificVelocityLimit() {
        MotionParameters[] forwardProfile = generateStraightMecanumProfile(90);
        MotionParameters[] diagonalProfile = generateStraightMecanumProfile(45);

        double forwardMaxVelocity = maxVelocity(forwardProfile);
        double diagonalMaxVelocity = maxVelocity(diagonalProfile);
        double forwardAverageVelocity = averageVelocity(forwardProfile);
        double diagonalAverageVelocity = averageVelocity(diagonalProfile);

        Assert.assertTrue("45-degree mecanum profile should have a lower peak velocity",
                diagonalMaxVelocity < forwardMaxVelocity - 1.0);
        Assert.assertTrue("45-degree mecanum profile should be slower overall",
                diagonalAverageVelocity < forwardAverageVelocity - 1.0);
    }

    @Test
    public void testMecanumDirectionalKinematicsOrdering() {
        Mecanum.MecanumDirectionalLut limits = createMecanumLimits();

        DirectionalKinematics forward =
                limits.getKinematics(Vector.of(1, 0, DistUnit.IN), Angle.zero());
        DirectionalKinematics diagonal =
                limits.getKinematics(Vector.fromPolar(Dist.fromIn(1), Angle.fromDeg(45)),
                        Angle.zero());
        DirectionalKinematics strafe =
                limits.getKinematics(Vector.of(0, 1, DistUnit.IN), Angle.zero());

        Assert.assertEquals(dummyConstants.forwardVelocityLimit.getIn(), forward.maxVel, 1e-6);
        Assert.assertEquals(dummyConstants.forwardAccelerationLimit.getIn(), forward.maxAccel,
                1e-6);
        Assert.assertEquals(dummyConstants.strafeVelocityLimit.getIn(), strafe.maxVel, 1e-6);
        Assert.assertEquals(dummyConstants.strafeAccelerationLimit.getIn(), strafe.maxAccel,
                1e-6);

        Assert.assertTrue("strafe velocity should be below forward",
                strafe.maxVel < forward.maxVel);
        Assert.assertTrue("diagonal velocity should be below pure strafe",
                diagonal.maxVel < strafe.maxVel);
        Assert.assertTrue("strafe accel should be below forward",
                strafe.maxAccel < forward.maxAccel);
        Assert.assertTrue("diagonal accel should be below pure strafe",
                diagonal.maxAccel < strafe.maxAccel);
    }

    @Test
    public void testMecanumDirectionalKinematicsInterpolatesBetweenDegrees() {
        Mecanum.MecanumDirectionalLut limits = createMecanumLimits();

        DirectionalKinematics belowBoundary =
                limits.getKinematics(Vector.fromPolar(Dist.fromIn(1), Angle.fromDeg(44.49)),
                        Angle.zero());
        DirectionalKinematics aboveBoundary =
                limits.getKinematics(Vector.fromPolar(Dist.fromIn(1), Angle.fromDeg(44.51)),
                        Angle.zero());

        Assert.assertTrue("velocity should not jump across integer-degree boundary",
                Math.abs(aboveBoundary.maxVel - belowBoundary.maxVel) < 0.01);
        Assert.assertTrue("accel should not jump across integer-degree boundary",
                Math.abs(aboveBoundary.maxAccel - belowBoundary.maxAccel) < 0.01);
    }

    private MotionParameters[] generateStraightMecanumProfile(double headingDeg) {
        Path straightPath = new HolonomicPathBuilder(
                poseFac.of(0, -50, headingDeg),
                poseFac.of(0, 50, headingDeg))
                .quickBuild();
        return generateProfile(constantsWithDrivetrain(FollowerConstants.DrivetrainType.MECANUM),
                straightPath);
    }

    private Mecanum.MecanumDirectionalLut createMecanumLimits() {
        return new Mecanum.MecanumDirectionalLut(
                dummyConstants.forwardVelocityLimit.getIn(),
                dummyConstants.forwardAccelerationLimit.getIn(),
                dummyConstants.strafeVelocityLimit.getIn(),
                dummyConstants.strafeAccelerationLimit.getIn()
        );
    }

    private double maxVelocity(MotionParameters[] profile) {
        double maxVelocity = 0.0;
        for (MotionParameters point : profile) {
            maxVelocity = Math.max(maxVelocity, point.getTangentialVel());
        }
        return maxVelocity;
    }

    private double averageVelocity(MotionParameters[] profile) {
        double sum = 0.0;
        for (MotionParameters point : profile) {
            sum += point.getTangentialVel();
        }
        return profile.length == 0 ? 0.0 : sum / profile.length;
    }
}