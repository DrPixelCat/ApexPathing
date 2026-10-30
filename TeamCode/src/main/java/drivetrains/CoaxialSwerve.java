package drivetrains;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import controllers.FeedbackController;
import controllers.FeedbackController.Coefficients;
import paths.geometry.Angle;
import paths.geometry.Dist;

/**
 * Coaxial swerve drivetrain controller
 *
 * @author Xander Haemel - 31616 404 Not Found
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public class CoaxialSwerve extends BaseDrivetrain<CoaxialSwerve.Constants> {
    private final Module frontLeft, frontRight, backLeft, backRight;

    private static final class Module {
        final CRServo servo;
        final AnalogInput encoder;
        final FeedbackController controller;
        final double offset;
        final double encoderSign;

        Module(HardwareMap hardware, String servoName, String encoderName, Coefficients coefficients, double offset,
               boolean servoReverse, boolean encoderReverse) {
            servo = hardware.get(CRServo.class, servoName);
            servo.setDirection(servoReverse ? com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE
                    : com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD);
            encoder = hardware.get(AnalogInput.class, encoderName);
            controller = new FeedbackController(coefficients);
            this.offset = offset;
            encoderSign = encoderReverse ? -1 : 1;
        }

        double command(double forward, double strafe) {
            double angle = encoderSign * encoder.getVoltage() * 2 * Math.PI / encoder.getMaxVoltage() + offset;
            double error = Angle.wrap(Math.atan2(strafe, forward) - angle);
            double power = Math.hypot(forward, strafe);
            if (Math.abs(error) > Math.PI / 2) {
                power = -power;
                error -= Math.copySign(Math.PI, error);
            }
            double steeringPower = controller.p().d().s().calculate(error).get();
            if (!Double.isFinite(steeringPower)) {
                throw new IllegalStateException("Invalid swerve steering measurement");
            }
            servo.setPower(Math.max(-1, Math.min(1, steeringPower)));
            return power;
        }

        void stop() {
            servo.setPower(0);
            controller.reset();
        }
    }

    public CoaxialSwerve(Constants constants, HardwareMap hardwareMap) {
        super(constants, hardwareMap, DrivetrainType.COAXIAL_SWERVE);

        double offset = constants.offsetAngle.getRad();
        frontLeft = new Module(hardwareMap, constants.flServoName, constants.flEncoderName, constants.steeringCoefficients, offset + constants.moduleOffsets[0], constants.servoReversed[0], constants.encoderReversed[0]);
        frontRight = new Module(hardwareMap, constants.frServoName, constants.frEncoderName, constants.steeringCoefficients, offset + constants.moduleOffsets[1], constants.servoReversed[1], constants.encoderReversed[1]);
        backLeft = new Module(hardwareMap, constants.blServoName, constants.blEncoderName, constants.steeringCoefficients, offset + constants.moduleOffsets[2], constants.servoReversed[2], constants.encoderReversed[2]);
        backRight = new Module(hardwareMap, constants.brServoName, constants.brEncoderName, constants.steeringCoefficients, offset + constants.moduleOffsets[3], constants.servoReversed[3], constants.encoderReversed[3]);
    }

    @Override
    public void moveWithVectors(double x, double y, double turn) {
        // Coaxial swerve kinematics explanation:
        // https://www.chiefdelphi.com/t/paper-4-wheel-independent-drive-independent-steering-swerve/107383
        if (!Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(turn)) {
            stop();
            throw new IllegalArgumentException("Drive commands must be finite");
        }
        if (x == 0 && y == 0 && turn == 0) { stop(); return; }
        double cwTurn = -turn;

        double strafeFront = y + cwTurn * constants.wheelbaseRatio;
        double strafeRear = y - cwTurn * constants.wheelbaseRatio;
        double forwardLeft = x + cwTurn * constants.trackWidthRatio;
        double forwardRight = x - cwTurn * constants.trackWidthRatio;

        try {
            setPowers(frontLeft.command(forwardLeft, strafeFront), frontRight.command(forwardRight, strafeFront),
                    backLeft.command(forwardLeft, strafeRear), backRight.command(forwardRight, strafeRear));
        } catch (RuntimeException failure) {
            stop();
            throw failure;
        }
    }

    /** @return the error wrapped to the range [-pi, pi] to ensure the shortest path is taken */
    @Override public void stop() {
        super.stop();
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    /** Configuration class for Coaxial Swerve drivetrain. */
    public static class Constants extends BaseDrivetrainConstants<Constants> {
        /** Per-module calibration in FL, FR, BL, BR order; offsets are radians. */
        public final double[] moduleOffsets = new double[4];
        public final boolean[] servoReversed = new boolean[4], encoderReversed = new boolean[4];
        public String flServoName, frServoName, blServoName, brServoName;
        public String flEncoderName, frEncoderName, blEncoderName, brEncoderName;

        public Coefficients steeringCoefficients =
                new Coefficients().p(0.0).d(0.0).s(0.0);

        public Angle offsetAngle = Angle.zero(); // Encoder value facing forward position
        public Dist wheelbase = Dist.fromIn(14); // Front pod to back pod spacing
        public Dist trackWidth = Dist.fromIn(14); // Left pod to right pod spacing
        public Dist diagonalDist;
        public double wheelbaseRatio; // Wheelbase / diagonal distance
        public double trackWidthRatio; // Track width / diagonal distance

        @Override
        public CoaxialSwerve build(HardwareMap hardwareMap) {
            for (double offset : moduleOffsets) {
                if (!Double.isFinite(offset)) { throw new IllegalArgumentException("Module offsets must be finite"); }
            }
            if (wheelbase == null || trackWidth == null || !Double.isFinite(wheelbase.getIn())
                    || !Double.isFinite(trackWidth.getIn()) || wheelbase.getIn() <= 0 || trackWidth.getIn() <= 0
                    || offsetAngle == null || offsetAngle.isNotFinite() || steeringCoefficients == null) {
                throw new IllegalArgumentException("Swerve geometry and steering configuration must be finite and positive");
            }
            diagonalDist = wheelbase.hypot(trackWidth);
            wheelbaseRatio = wheelbase.div(diagonalDist).getIn();
            trackWidthRatio = trackWidth.div(diagonalDist).getIn();

            if (flMotorConfig == null || frMotorConfig == null || blMotorConfig == null ||
                    brMotorConfig == null) {
                throw new IllegalArgumentException(
                        "All 4 motor configs must be provided for a coaxial swerve drivetrain."
                );
            }
            if (flServoName == null || frServoName == null || blServoName == null ||
                    brServoName == null) {
                throw new IllegalArgumentException(
                        "All 4 servo names must be provided for a coaxial swerve drivetrain."
                );
            }
            if (flEncoderName == null || frEncoderName == null || blEncoderName == null ||
                    brEncoderName == null) {
                throw new IllegalArgumentException(
                        "All 4 encoder names must be provided for a coaxial swerve drivetrain."
                );
            }

            return new CoaxialSwerve(this, hardwareMap);
        }

        /** Sets the front left motor configuration. */
        public Constants setFrontLeftMotor(Motor Motor) {
            this.flMotorConfig = Motor;
            return this;
        }

        /** Sets the front right motor configuration. */
        public Constants setFrontRightMotor(Motor Motor) {
            this.frMotorConfig = Motor;
            return this;
        }

        /** Sets the back left motor configuration. */
        public Constants setBackLeftMotor(Motor Motor) {
            this.blMotorConfig = Motor;
            return this;
        }

        /** Sets the back right motor configuration. */
        public Constants setBackRightMotor(Motor Motor) {
            this.brMotorConfig = Motor;
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

        /** Sets the coefficients for the steering feedback controllers. */
        public Constants setSteeringCoefficients(Coefficients coefficients) {
            this.steeringCoefficients = coefficients;
            return this;
        }

        /** Sets the encoders reported angle when the wheel is facing forward. */
        public Constants setOffsetAngle(Angle offsetAngle) {
            this.offsetAngle = offsetAngle;
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
