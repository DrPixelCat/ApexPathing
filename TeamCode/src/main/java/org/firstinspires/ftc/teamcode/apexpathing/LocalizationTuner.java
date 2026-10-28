package org.firstinspires.ftc.teamcode.apexpathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import core.Follower;
import geometry.Angle;
import geometry.Pose;
import localizers.BaseLocalizer;
import localizers.PodCalibrationReading;
import tuning.TunerContext;

/** Hand-motion localizer calibration and filter convergence capture. Never commands robot motion. */
@TeleOp(name = "Localization Tuner (Advanced)", group = "Apex Pathing")
public class LocalizationTuner extends LinearOpMode {
    private static final String[] POD_HEADER = {"time_s", "trial", "kind", "localizer", "channel",
            "travel_in", "delta_travel_in", "reference_sweep_rad", "reported_sweep_rad",
            "configured_offset_in", "measured_offset_in"};

    @Override public void runOpMode() {
        TunerContext context = new TunerContext(this);
        try {
            context.setFollower(new Follower(new Constants(), hardwareMap, true));
            context.getFollower().stop();
            context.getFollower().disableControllers();
            context.openLog("localizer-tuning");
            telemetry.addData("CSV folder", context.getLog().getDirectory().getAbsolutePath());
            telemetry.addLine("Hand-motion pod and Kalman tuning. Motors remain stopped.");
            telemetry.addLine("Keep still during hardware IMU calibration, then press Start.");
            telemetry.update();
            waitForStart();
            if (!opModeIsActive()) { return; }

            BaseLocalizer<?> localizer = context.getFollower().getLocalizer();
            double referenceDegrees = 360.0;
            boolean recording = false;
            boolean adapting = false;
            int trial = 0;
            double previousHeading = 0, reportedSweep = 0;
            PodCalibrationReading[] start = null;
            String lastResult = "No pod trial recorded";
            localizer.setIsTuning(false);
            while (opModeIsActive()) {
                context.getFollower().getDrivetrain().stop();
                localizer.update();
                double heading = localizer.getPose().getHeading().getRad();
                if (recording) { reportedSweep += Angle.wrap(heading - previousHeading); }
                previousHeading = heading;
                PodCalibrationReading[] pods = localizer.getPodCalibrationReadings();

                if (!recording) {
                    if (gamepad1.dpadUpWasPressed()) { referenceDegrees += 90; }
                    if (gamepad1.dpadDownWasPressed()) { referenceDegrees -= 90; }
                    if (gamepad1.dpadLeftWasPressed()) { referenceDegrees = -Math.abs(referenceDegrees); }
                    if (gamepad1.dpadRightWasPressed()) { referenceDegrees = Math.abs(referenceDegrees); }
                    if (gamepad1.bWasPressed()
                            && localizer.getVelocityFilterMode() == BaseLocalizer.VelocityFilterMode.ADAPTIVE_KALMAN) {
                        adapting = !adapting;
                        localizer.setIsTuning(adapting);
                        context.getLog().event("localizer", adapting ? "adaptation_enabled" : "noise_frozen", "");
                    }
                    if (gamepad1.yWasPressed()) {
                        localizer.setPose(Pose.zero());
                        context.getLog().event("localizer", "pose_and_filter_reset", "Pod baselines reset");
                        pods = localizer.getPodCalibrationReadings();
                        previousHeading = 0;
                    }
                    if (gamepad1.xWasPressed() && pods.length > 0 && Math.abs(referenceDegrees) >= 90) {
                        start = pods;
                        reportedSweep = 0;
                        trial++;
                        recording = true;
                        context.getLog().event("pods", "trial_start", "Physical rotation degrees=" + referenceDegrees);
                    }
                } else if (gamepad1.aWasPressed()) {
                    for (int i = 0; i < pods.length; i++) {
                        double measured = pods[i].measuredOffset(start[i].travelIn, Math.toRadians(referenceDegrees));
                        logPod(context, localizer, trial, "result", pods[i], start[i].travelIn,
                                referenceDegrees, reportedSweep, measured);
                    }
                    lastResult = "Trial " + trial + " saved to pod_offsets.csv; inspect before applying offsets.";
                    context.getLog().event("pods", "trial_complete", lastResult);
                    recording = false;
                }

                context.sample("localizer", recording ? "pod_rotation" : adapting ? "filter_adapting" : "filter_frozen");
                for (int i = 0; i < pods.length; i++) {
                    logPod(context, localizer, trial, recording ? "sample" : "configured", pods[i],
                            recording ? start[i].travelIn : pods[i].travelIn, referenceDegrees,
                            reportedSweep, Double.NaN);
                }
                telemetry.clearAll();
                context.addInterfaceHeader();
                telemetry.addData("Localizer", localizer.getClass().getSimpleName());
                telemetry.addData("Filter", localizer.getVelocityFilterMode());
                telemetry.addData("Noise adaptation", adapting ? "ON: raw and filtered are logged" : "OFF: fixed Q/R; estimates continue");
                telemetry.addLine(recording ? "Rotate by hand about the marked robot center; A ends the trial."
                        : "X: begin pod trial; Dpad Up/Down: angle +/-90; Left: CW; Right: CCW");
                telemetry.addData("Physical reference angle (deg)", referenceDegrees);
                telemetry.addData("Reported rotation (deg)", Math.toDegrees(reportedSweep));
                telemetry.addLine("Use a marked angle measured externally; keep the robot center fixed.");
                telemetry.addLine("B: learn/freeze Q/R; Y: reset pose/filter while idle.");
                telemetry.addLine("Move by hand through rest, translation, and rotation to capture convergence.");
                if (pods.length == 0) { telemetry.addLine("Pod offset capture unavailable for this localizer; filter logging remains active."); }
                telemetry.addLine(lastResult);
                telemetry.update();
                sleep(20);
            }
            if (recording) { context.getLog().event("pods", "trial_aborted", "Stop before operator confirmation; no result"); }
        } finally {
            try {
                if (context.getFollower() != null) {
                    context.getFollower().getLocalizer().setIsTuning(false);
                    context.getFollower().stop();
                }
            } finally { context.closeLog(); }
        }
    }

    private void logPod(TunerContext context, BaseLocalizer<?> localizer, int trial, String kind,
                        PodCalibrationReading pod, double startTravel, double degrees,
                        double reportedSweep, double measured) {
        context.getLog().write("pod_offsets", POD_HEADER, context.getLog().seconds(), trial, kind,
                localizer.getClass().getSimpleName(), pod.name, pod.travelIn, pod.travelIn - startTravel,
                Math.toRadians(degrees), reportedSweep, pod.configuredOffsetIn, measured);
    }
}
