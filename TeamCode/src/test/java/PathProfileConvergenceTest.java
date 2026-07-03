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
import feedforward.MotionParameters;
import feedforward.holonomic.mecanum.MecanumProfileGenerator;
import feedforward.holonomic.swerve.SwerveProfileGenerator;
import feedforward.tank.TankProfileGenerator;
import geometry.Angle;
import geometry.Dist;
import geometry.PathPoint;
import paths.builders.HolonomicPathBuilder;
import paths.builders.TankPathBuilder;
import paths.heading.HolonomicInterpolationStyle;
import paths.heading.TankInterpolationStyle;
import paths.movements.Path;
import util.AngleUnit;
import util.DistUnit;
import util.PoseFactory;

public class PathProfileConvergenceTest {
    FollowerConstants dummyConstants = new FollowerConstants().inject(
            FollowerConstants.DrivetrainType.COAXIAL_SWERVE,
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
            poseFac.of(0, 50, -90))
//            poseFac.of(100, 50, 0))
            .addHeadingNode(0.25, Angle.fromDeg(180))
            .addHeadingNode(0.5, Angle.fromDeg(0))
            .profiledBuild();

    SwerveProfileGenerator generator = new SwerveProfileGenerator(dummyConstants, path);

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
        double[] forwardVelLimit = constantArray(
                points.length, dummyConstants.forwardVelocityLimit.getIn()
        );
        double[] positiveAccelLimit = constantArray(
                points.length, dummyConstants.forwardAccelerationLimit.getIn()
        );
        double[] negativeAccelLimit = constantArray(
                points.length, -dummyConstants.forwardAccelerationLimit.getIn()
        );
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

        for (int i = 0; i < points.length; i++) {
            sData[i] = pathLength - points[i].getDistanceToEnd_in();

            vInit[i] = initialParams[i].getTangentialVel();
            pInit[i] = initialParams[i].getMotorPower();

            vConv[i] = convergedParams[i].getTangentialVel();
            aConv[i] = convergedParams[i].getTangentialAccel();
            pConv[i] = convergedParams[i].getMotorPower();
            wConv[i] = convergedParams[i].getAngularVel();
            alphaConv[i] = convergedParams[i].getAngularAccel();
        }

        double averageOptimizedPower = average(pConv);
        double maxOptimizedPower = max(pConv);

        XYChart transChart = new XYChartBuilder()
                .width(1000).height(460)
                .title("Swerve Profile - Translational State")
                .xAxisTitle("Arc Length s Along Path (in)")
                .build();
        styleChart(transChart);
        transChart.setYAxisGroupTitle(0, "Velocity (in/s)");
        transChart.setYAxisGroupTitle(1, "Tangential Accel (in/s^2)");
        transChart.getStyler().setYAxisGroupPosition(1, Styler.YAxisPosition.Right);
        transChart.getStyler().setYAxisMin(0, 0.0);
        transChart.getStyler().setYAxisMax(0, dummyConstants.forwardVelocityLimit.getIn() * 1.12);
        transChart.getStyler().setYAxisMin(
                1, -dummyConstants.forwardAccelerationLimit.getIn() * 1.15
        );
        transChart.getStyler().setYAxisMax(
                1, dummyConstants.forwardAccelerationLimit.getIn() * 1.15
        );
        addSeries(transChart, "Initial velocity", sData, vInit, 0);
        addSeries(transChart, "Optimized velocity", sData, vConv, 0);
        addSeries(transChart, "Forward velocity limit", sData, forwardVelLimit, 0);
        addSeries(transChart, "Optimized tangential accel", sData, aConv, 1);
        addSeries(transChart, "+accel limit", sData, positiveAccelLimit, 1);
        addSeries(transChart, "-accel limit", sData, negativeAccelLimit, 1);
        addSeries(transChart, "Zero accel", sData, zeroLine, 1);

        XYChart angChart = new XYChartBuilder()
                .width(1000).height(460)
                .title("Swerve Profile - Heading State")
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
                        "Swerve Profile - Power Utilization (avg %.3f, max %.3f)",
                        averageOptimizedPower, maxOptimizedPower
                ))
                .xAxisTitle("Arc Length s Along Path (in)")
                .yAxisTitle("Normalized Motor Utilization")
                .build();
        styleChart(powerChart);
        powerChart.getStyler().setYAxisMin(0.0);
        powerChart.getStyler().setYAxisMax(1.12);
        addSeries(powerChart, "Initial utilization", sData, pInit, 0);
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
                new MecanumProfileGenerator(dummyConstants, mecanumPath).generate();
        assertUsableProfile("mecanum", mecanumProfile);

        Path tankPath = new TankPathBuilder(
                poseFac.of(0, -50, 90),
                poseFac.arcPoseOf(0, 50, 30),
                poseFac.of(100, 50, 0))
                .quickBuild();
        MotionParameters[] tankProfile = new TankProfileGenerator(dummyConstants, tankPath).generate();
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
}
