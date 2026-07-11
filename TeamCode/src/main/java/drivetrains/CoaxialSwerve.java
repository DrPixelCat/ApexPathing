package drivetrains;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Objects;

import controllers.PDSController;
import controllers.PDSController.PDSCoefficients;
import geometry.Angle;
import geometry.Dist;
import util.MotorFactory;

/**
 * Coaxial swerve drivetrain controller
 *
 * @author Xander Haemel - 31616 404 Not Found
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public class CoaxialSwerve extends BaseDrivetrain<CoaxialSwerve.Constants> {
    private final static double pi2 = 2 * Math.PI;
    private final static double piOver2 = Math.PI / 2;

    private final CRServo flServo, frServo, blServo, brServo;
    private final AnalogInput flEncoder, frEncoder, blEncoder, brEncoder;
    private final PDSController flSteerController, frSteerController, blSteerController,
            brSteerController;
    private final double voltageToRad; // Radians = voltage * this
    private final double[] offsetAnglesRad; // Per-module radians added to raw encoder angle

    public CoaxialSwerve(Constants constants, HardwareMap hardwareMap) {
        super(constants, hardwareMap, DrivetrainType.COAXIAL_SWERVE);

        // Make sure all motors, servos, and encoders are configured (front motors are checked in super())
        if (Objects.equals(constants.blMotorConfig, null) || Objects.equals(constants.brMotorConfig, null)) {
            throw new IllegalArgumentException(
                    "Back left and right motor configurations must be provided for a coaxial swerve drivetrain"
            );
        }
        if (Objects.equals(constants.flServoName, null) || Objects.equals(constants.frServoName, null) ||
                Objects.equals(constants.blServoName, null) || Objects.equals(constants.brServoName, null)) {
            throw new IllegalArgumentException(
                    "Servo names must be provided for all 4 modules in a coaxial swerve drivetrain"
            );
        }
        if (Objects.equals(constants.flEncoderName, null) || Objects.equals(constants.frEncoderName, null) ||
                Objects.equals(constants.blEncoderName, null) || Objects.equals(constants.brEncoderName, null)) {
            throw new IllegalArgumentException(
                    "Encoder names must be provided for all 4 modules in a coaxial swerve drivetrain"
            );
        }

        flServo = hardwareMap.get(CRServo.class, constants.flServoName);
        frServo = hardwareMap.get(CRServo.class, constants.frServoName);
        blServo = hardwareMap.get(CRServo.class, constants.blServoName);
        brServo = hardwareMap.get(CRServo.class, constants.brServoName);
        flEncoder = hardwareMap.get(AnalogInput.class, constants.flEncoderName);
        frEncoder = hardwareMap.get(AnalogInput.class, constants.frEncoderName);
        blEncoder = hardwareMap.get(AnalogInput.class, constants.blEncoderName);
        brEncoder = hardwareMap.get(AnalogInput.class, constants.brEncoderName);

        flSteerController = new PDSController(constants.steeringCoefficients);
        frSteerController = new PDSController(constants.steeringCoefficients);
        blSteerController = new PDSController(constants.steeringCoefficients);
        brSteerController = new PDSController(constants.steeringCoefficients);

        voltageToRad = (2 * Math.PI) / flEncoder.getMaxVoltage();
        offsetAnglesRad = new double[]{
                constants.flOffsetAngle.getRad(), constants.frOffsetAngle.getRad(),
                constants.blOffsetAngle.getRad(), constants.brOffsetAngle.getRad()
        };
    }

    @Override
    public void moveWithVectors(double x, double y, double turn) {
        // Coaxial swerve kinematics explanation:
        // https://www.chiefdelphi.com/t/paper-4-wheel-independent-drive-independent-steering-swerve/107383
        turn *= -1; // Clockwise turn angle

        double strafeFront = y + turn * constants.wheelbaseRatio;
        double strafeRear = y - turn * constants.wheelbaseRatio;
        double forwardLeft = x + turn * constants.trackWidthRatio;
        double forwardRight = x - turn * constants.trackWidthRatio;

        double flPower = Math.sqrt(Math.pow(strafeFront, 2) + Math.pow(forwardLeft, 2));
        double frPower = Math.sqrt(Math.pow(strafeFront, 2) + Math.pow(forwardRight, 2));
        double blPower = Math.sqrt(Math.pow(strafeRear, 2) + Math.pow(forwardLeft, 2));
        double brPower = Math.sqrt(Math.pow(strafeRear, 2) + Math.pow(forwardRight, 2));

        double flAngleTarget = Math.atan2(strafeFront, forwardLeft);
        double frAngleTarget = Math.atan2(strafeFront, forwardRight);
        double blAngleTarget = Math.atan2(strafeRear, forwardLeft);
        double brAngleTarget = Math.atan2(strafeRear, forwardRight);

        double[] moduleAngles = moduleAngles();
        double flAngle = moduleAngles[0];
        double frAngle = moduleAngles[1];
        double blAngle = moduleAngles[2];
        double brAngle = moduleAngles[3];

        double flError = wrapError(flAngleTarget, flAngle);
        double frError = wrapError(frAngleTarget, frAngle);
        double blError = wrapError(blAngleTarget, blAngle);
        double brError = wrapError(brAngleTarget, brAngle);

        // If the error > 90 degrees, reverse the wheel direction and subtract 180 from the target angle
        if (Math.abs(flError) > piOver2) {
            flPower *= -1;
            flError -= Math.copySign(Math.PI, flError);
        }
        if (Math.abs(frError) > piOver2) {
            frPower *= -1;
            frError -= Math.copySign(Math.PI, frError);
        }
        if (Math.abs(blError) > piOver2) {
            blPower *= -1;
            blError -= Math.copySign(Math.PI, blError);
        }
        if (Math.abs(brError) > piOver2) {
            brPower *= -1;
            brError -= Math.copySign(Math.PI, brError);
        }

        // Steering controllers need a fresh update every loop even when the sampled error happens
        // to be identical. Skipping an update leaves stale CRServo power applied indefinitely.
        flServo.setPower(flSteerController.calculateFromError(flError));
        frServo.setPower(frSteerController.calculateFromError(frError));
        blServo.setPower(blSteerController.calculateFromError(blError));
        brServo.setPower(brSteerController.calculateFromError(brError));

        setPowers(flPower, frPower, blPower, brPower);
    }

    /** @return the error wrapped to the range [-pi, pi] to ensure the shortest path is taken */
    private double wrapError(double target, double current) {
        double errorRaw = target - current;
        return errorRaw - (pi2 * Math.round(errorRaw / pi2));
    }

    public double[] rawAngles() {
        return new double[]{
                Angle.normalize(flEncoder.getVoltage() * voltageToRad),
                Angle.normalize(frEncoder.getVoltage() * voltageToRad),
                Angle.normalize(blEncoder.getVoltage() * voltageToRad),
                Angle.normalize(brEncoder.getVoltage() * voltageToRad)
        };
    }

    public double[] moduleAngles() {
        double[] raw = rawAngles();
        for (int i = 0; i < raw.length; i++) raw[i] = Angle.normalize(raw[i] + offsetAnglesRad[i]);
        return raw;
    }

    public PDSCoefficients getSteeringGains() {
        return constants.steeringCoefficients;
    }

    /** Holds every module at one angle with drive motors disabled; intended only for tuning. */
    public void aimModules(Angle target) {
        double[] angles = moduleAngles();
        PDSController[] controllers = {flSteerController, frSteerController,
                blSteerController, brSteerController};
        CRServo[] servos = {flServo, frServo, blServo, brServo};
        for (int i = 0; i < servos.length; i++) {
            servos[i].setPower(controllers[i].calculateFromError(
                    wrapError(target.getRad(), angles[i])));
        }
        setPowers(0, 0, 0, 0);
    }

    public void stopSteering() {
        flServo.setPower(0); frServo.setPower(0); blServo.setPower(0); brServo.setPower(0);
    }

    /** Configuration class for Coaxial Swerve drivetrain. */
    public static class Constants extends BaseDrivetrainConstants<Constants> {
        public String flServoName, frServoName, blServoName, brServoName = "defaultServoName";
        public String flEncoderName, frEncoderName, blEncoderName, brEncoderName =
                "defaultEncoderName";

        public PDSCoefficients steeringCoefficients = new PDSCoefficients();

        public Angle flOffsetAngle = Angle.zero();
        public Angle frOffsetAngle = Angle.zero();
        public Angle blOffsetAngle = Angle.zero();
        public Angle brOffsetAngle = Angle.zero();
        public Dist wheelbase = Dist.fromIn(14); // Front pod to back pod spacing
        public Dist trackWidth = Dist.fromIn(14); // Left pod to right pod spacing
        public Dist diagonalDist;
        public double wheelbaseRatio; // Wheelbase / diagonal distance
        public double trackWidthRatio; // Track width / diagonal distance

        @Override
        public CoaxialSwerve build(HardwareMap hardwareMap) {
            diagonalDist = wheelbase.hypot(trackWidth);
            wheelbaseRatio = wheelbase.div(diagonalDist).getIn();
            trackWidthRatio = trackWidth.div(diagonalDist).getIn();

            return new CoaxialSwerve(this, hardwareMap);
        }

        /** Sets the front left motor configuration. */
        public Constants setFrontLeftMotor(MotorFactory motorFactory) {
            this.flMotorConfig = motorFactory;
            return this;
        }

        /** Sets the front right motor configuration. */
        public Constants setFrontRightMotor(MotorFactory motorFactory) {
            this.frMotorConfig = motorFactory;
            return this;
        }

        /** Sets the back left motor configuration. */
        public Constants setBackLeftMotor(MotorFactory motorFactory) {
            this.blMotorConfig = motorFactory;
            return this;
        }

        /** Sets the back right motor configuration. */
        public Constants setBackRightMotor(MotorFactory motorFactory) {
            this.brMotorConfig = motorFactory;
            return this;
        }

        /** Sets the names of the steering servos. */
        public Constants setServoNames(String fl, String fr, String bl, String br) {
            this.flServoName = fl;
            this.frServoName = fr;
            this.blServoName = bl;
            this.brServoName = br;
            return this;
        }

        /** Sets the names of the steering encoders. */
        public Constants setEncoderNames(String fl, String fr, String bl, String br) {
            this.flEncoderName = fl;
            this.frEncoderName = fr;
            this.blEncoderName = bl;
            this.brEncoderName = br;
            return this;
        }

        /** Sets the coefficients for the steering PDS controllers. */
        public Constants setSteeringCoefficients(PDSCoefficients coefficients) {
            this.steeringCoefficients = coefficients;
            return this;
        }

        /** Sets the encoders reported angle when the wheel is facing forward. */
        public Constants setOffsetAngle(Angle offsetAngle) {
            this.flOffsetAngle = offsetAngle;
            this.frOffsetAngle = offsetAngle;
            this.blOffsetAngle = offsetAngle;
            this.brOffsetAngle = offsetAngle;
            return this;
        }

        /** Sets the independently calibrated absolute-encoder offset for each module. */
        public Constants setOffsetAngles(Angle fl, Angle fr, Angle bl, Angle br) {
            this.flOffsetAngle = fl;
            this.frOffsetAngle = fr;
            this.blOffsetAngle = bl;
            this.brOffsetAngle = br;
            return this;
        }

        /** Sets the wheelbase of the chassis (front to back pod wheel spacing). */
        public Constants setWheelbase(Dist wheelbase) {
            this.wheelbase = wheelbase; return this;
        }

        /** Sets the track width of the chassis (left to right pod wheel spacing). */
        public Constants setTrackWidth(Dist trackWidth) {
            this.trackWidth = trackWidth; return this;
        }
    }
}
