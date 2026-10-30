package localizers;

import static com.qualcomm.robotcore.util.TypeConversion.byteArrayToInt;

import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import org.json.JSONObject;

import paths.geometry.Dist;
import paths.geometry.Pose;
import paths.geometry.Vector;
import paths.geometry.AngleUnit;
import paths.geometry.DistUnit;
import paths.geometry.GeometryFactory;

/**
 * goBILDA Pinpoint Odometry Computer localizer (uses custom driver class)
 *
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public class Pinpoint extends BaseLocalizer<Pinpoint.Constants> {
    private final Driver pinpoint;

    public enum EncoderDirection { FORWARD, REVERSED }
    public enum GoBildaPods { goBILDA_SWINGARM_POD,  goBILDA_4_BAR_POD }

    public Pinpoint(Constants constants, HardwareMap hardwareMap) {
        super(constants);

        if (constants.name == null || constants.name.trim().isEmpty()
                || constants.offsets == null || !constants.offsets.isFinite()
                || constants.customEncoderResolution == null
                || !Double.isFinite(constants.customEncoderResolution.getIn())
                || constants.customEncoderResolution.getIn() < 0
                || !Double.isFinite(constants.angularScalar) || constants.angularScalar < 0
                || constants.xPodDirection == null || constants.yPodDirection == null
                || (constants.customEncoderResolution.getIn() == 0 && constants.encoderResolution == null)) {
            throw new IllegalArgumentException("Pinpoint requires valid device, offsets, resolution, directions and yaw scalar");
        }
        pinpoint = hardwareMap.get(Driver.class, constants.name);
        pinpoint.setOffsets(constants.offsets);
        pinpoint.setEncoderDirections(constants.xPodDirection, constants.yPodDirection);
        if (constants.customEncoderResolution.getIn() == 0) {
            pinpoint.setEncoderResolution(constants.encoderResolution);
        } else {
            pinpoint.setEncoderResolution(constants.customEncoderResolution);
        }
        if (constants.angularScalar != 0) { pinpoint.setYawScalar(constants.angularScalar); }
        pinpoint.resetPosAndIMU();
    }

    @Override
    public void update() {
        pinpoint.update();
        if (!pinpoint.isLastSampleValid()) { markMeasurementInvalid(); return; }
        pose = pinpoint.getPosition();
        calculate(pinpoint.getVelocity(), pinpoint.getSampleTimeNanos());
    }

    /** Returns the raw X/Y pod encoder positions from the last bulk update. */
    public int[] getPodTicks() { return pinpoint.getEncoderPositions(); }

    @Override
    public void setPose(Pose newPose) {
        pinpoint.setPosition(newPose);
        resetKinematicEstimate(newPose);
    }

    /** Configuration class for goBILDA Pinpoint localizer. */
    public static class Constants implements BaseLocalizerConstants<Constants> {
        public String name;
        public Vector offsets = Vector.zero();
        public EncoderDirection xPodDirection = EncoderDirection.FORWARD;
        public EncoderDirection yPodDirection = EncoderDirection.FORWARD;
        public GoBildaPods encoderResolution = GoBildaPods.goBILDA_4_BAR_POD;
        public Dist customEncoderResolution = Dist.zero(); // Overrides encoderResolution if != 0
        public double angularScalar = 0; // Overrides the default

        @Override
        public JSONObject getCalibrationValues() {
            try {
                return CalibrationJson.vector(offsets)
                        .put("customEncoderResolutionIn", customEncoderResolution.getIn())
                        .put("angularScalar", angularScalar)
                        .put("xPodDirection", xPodDirection.name())
                        .put("yPodDirection", yPodDirection.name());
            } catch (Exception e) { throw new IllegalStateException(e); }
        }

        @Override
        public void applyCalibrationValues(JSONObject values) {
            offsets = CalibrationJson.vector(values, offsets);
            customEncoderResolution = Dist.fromIn(CalibrationJson.finite(values,
                    "customEncoderResolutionIn", customEncoderResolution.getIn()));
            angularScalar = CalibrationJson.finite(values, "angularScalar", angularScalar);
            xPodDirection = direction(values, "xPodDirection", xPodDirection);
            yPodDirection = direction(values, "yPodDirection", yPodDirection);
        }

        @Override
        public Pinpoint build(HardwareMap hardwareMap) {
            if (name == null) {
                throw new IllegalArgumentException(
                        "Pinpoint name is not set in the localizer constants."
                );
            }
            return new Pinpoint(this, hardwareMap);
        }

        /** Sets the name of the Pinpoint in the hardware map. */
        public Constants setName(String name) {
            this.name = name;
            return this;
        }

        /** Sets the X and Y pod offsets of the Pinpoint */
        public Constants setOffsets(double xOffset, double yOffset, DistUnit distanceUnit) {
            this.offsets = Vector.of(xOffset, yOffset, distanceUnit);
            return this;
        }

        /** Sets the direction of the X and Y encoders of the Pinpoint. */
        public Constants setEncoderDirections(EncoderDirection xPodDirection,
                                              EncoderDirection yPodDirection) {
            this.xPodDirection = xPodDirection;
            this.yPodDirection = yPodDirection;
            return this;
        }

        /** Sets the encoder resolution of the Pinpoint in ticks per unit using goBILDA pods. */
        public Constants setEncoderResolution(GoBildaPods encoderResolution) {
            this.encoderResolution = encoderResolution;
            return this;
        }

        /** Sets the encoder resolution of the Pinpoint in ticks per unit. */
        public Constants setEncoderResolution(Dist customEncoderResolution) {
            this.customEncoderResolution = customEncoderResolution;
            return this;
        }

        /** Sets the angular scalar of the Pinpoint. It is not recommended to change this values. */
        public Constants setAngularScalar(double angularScalar) {
            this.angularScalar = angularScalar;
            return this;
        }

        private static EncoderDirection direction(JSONObject values, String key,
                                                  EncoderDirection fallback) {
            try { return EncoderDirection.valueOf(values.optString(key, fallback.name())); }
            catch (IllegalArgumentException ignored) { return fallback; }
        }
    }

    /**
     * Driver class for the goBILDA Pinpoint Odometry Computer. This code is a modified version
     * of the original goBILDA Pinpoint Driver created by Ethan Doak from goBILDA. It has been
     * modified to better suit the needs of Apex Pathing. The original code can be found on the
     * goBILDA
     * <a href="https://github.com/goBILDA-Official/FtcRobotController-Add-Pinpoint/">GitHub</a>.
     *
     * @author Ethan Doak - goBILDA
     * @author Dylan B. - 18597 RoboClovers - Delta
     */
    @SuppressWarnings("ConstantExpression")
    @I2cDeviceType
    @DeviceProperties(
            name = "goBILDA® Pinpoint Odometry Computer",
            xmlTag = "goBILDAPinpoint",
            description = "goBILDA® Pinpoint Odometry Computer (IMU Sensor Fusion for 2 Wheel " +
                    "Odometry), optimized for Apex Pathing"
    )
    public static class Driver extends I2cDeviceSynchDevice<I2cDeviceSynchSimple> {
        private boolean lastSampleValid;
        private long sampleTimeNanos;
        private int deviceStatus;

        private final GeometryFactory geometryFactory = new GeometryFactory()
                .setDistUnit(DistUnit.MM)
                .setAngleUnit(AngleUnit.RAD);
        private Pose pose = geometryFactory.pose();
        private Pose velocity = geometryFactory.pose();

        private int loopTime = 0;
        private int encoderX;
        private int encoderY;

        private static final float goBILDA_SWINGARM_POD = 13.26291192f; // goBILDA Swingarm Pod TPM
        private static final float goBILDA_4_BAR_POD = 19.89436789f; // goBILDA 4 Bar Pod TPM
        public static final byte DEFAULT_ADDRESS = 0x31; // I2C address of the device

        public Driver(I2cDeviceSynchSimple deviceClient, boolean deviceClientIsOwned) {
            super(deviceClient, deviceClientIsOwned);

            this.deviceClient.setI2cAddress(I2cAddr.create7bit(DEFAULT_ADDRESS));
            super.registerArmingStateCallback(false);
        }

        @Override
        protected synchronized boolean doInitialize() {
            ((LynxI2cDeviceSynch) deviceClient).setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);
            return true;
        }

        @Override
        public Manufacturer getManufacturer() { return Manufacturer.GoBilda; }

        @Override
        public String getDeviceName() { return "goBILDA® Pinpoint Odometry Computer"; }

        /** I2C registers */
        private enum Register {
            DEVICE_ID(1),
            DEVICE_VERSION(2),
            DEVICE_STATUS(3),
            DEVICE_CONTROL(4),
            LOOP_TIME(5),
            X_ENCODER_VALUE(6),
            Y_ENCODER_VALUE(7),
            X_POSITION(8),
            Y_POSITION(9),
            H_ORIENTATION(10),
            X_VELOCITY(11),
            Y_VELOCITY(12),
            H_VELOCITY(13),
            MM_PER_TICK(14),
            X_POD_OFFSET(15),
            Y_POD_OFFSET(16),
            YAW_SCALAR(17),
            BULK_READ(18);

            private final int bVal;

            Register(int bVal) { this.bVal = bVal; }
        }

        /**
         * Writes an int to the i2c device
         *
         * @param i the integer to write to the register
         */
        private void writeInt(int i) {
            deviceClient.write(Register.DEVICE_CONTROL.bVal, TypeConversion.intToByteArray(i,
                    ByteOrder.LITTLE_ENDIAN));
        }

        /**
         * Converts a byte array to a float value
         *
         * @param byteArray byte array to transform
         * @return the float value stored by the byte array
         */
        private float byteArrayToFloat(byte[] byteArray) {
            return ByteBuffer.wrap(byteArray).order(ByteOrder.LITTLE_ENDIAN).getFloat();
        }

        /**
         * Converts a float to a byte array
         *
         * @param value the float array to convert
         * @return the byte array converted from the float
         */
        private byte[] floatToByteArray(float value) {
            return ByteBuffer.allocate(4).order(ByteOrder.LITTLE_ENDIAN).putFloat(value).array();
        }

        /**
         * Writes a byte array to a register on the i2c device
         *
         * @param reg the register to write to
         * @param bytes the byte array to write
         */
        private void writeByteArray(Register reg, byte[] bytes) {
            deviceClient.write(reg.bVal, bytes);
        }

        /**
         * Writes a float to a register on the i2c device
         *
         * @param reg the register to write to
         * @param f the float to write
         */
        private void writeFloat(Register reg, float f) {
            byte[] bytes = ByteBuffer.allocate(4).order(ByteOrder.LITTLE_ENDIAN)
                    .putFloat(f).array();
            deviceClient.write(reg.bVal, bytes);
        }

        /** Reads one complete packet; invalid packets never refresh the accepted state. */
        public void update() {
            lastSampleValid = false;
            byte[] bytes = deviceClient.read(Register.BULK_READ.bVal, 40);
            if (bytes == null || bytes.length != 40) { return; }
            ByteBuffer packet = ByteBuffer.wrap(bytes).order(ByteOrder.LITTLE_ENDIAN);
            deviceStatus = packet.getInt();
            loopTime = packet.getInt();
            int newEncoderX = packet.getInt();
            int newEncoderY = packet.getInt();
            Pose newPose = geometryFactory.pose(packet.getFloat(), packet.getFloat(), packet.getFloat());
            Pose newVelocity = geometryFactory.pose(packet.getFloat(), packet.getFloat(), packet.getFloat());
            // Register 3: READY bit, followed by calibration and missing-pod fault bits.
            if (deviceStatus != 1 || loopTime <= 0 || !finite(newPose) || !finite(newVelocity)) { return; }
            pose = newPose;
            velocity = newVelocity;
            encoderX = newEncoderX;
            encoderY = newEncoderY;
            sampleTimeNanos = System.nanoTime();
            lastSampleValid = true;
        }

        private static boolean finite(Pose value) {
            return value.getVec().isFinite() && !value.getHeading().isNotFinite();
        }

        public boolean isLastSampleValid() { return lastSampleValid; }
        public long getSampleTimeNanos() { return sampleTimeNanos; }
        public int getDeviceStatus() { return deviceStatus; }

        /** Returns raw X/Y encoder counts captured by the latest bulk read. */
        public int[] getEncoderPositions() { return new int[] {encoderX, encoderY}; }

        /**
         * Sets the odometry pod positions relative to the point that the odometry computer
         * tracks around. The most common tracking position is the center of the robot.
         *
         * @param offset a Vector containing the X and Y offsets of the odometry pods
         */
        public void setOffsets(Vector offset) {
            writeFloat(Register.X_POD_OFFSET, (float) offset.getX().getMm());
            writeFloat(Register.Y_POD_OFFSET, (float) offset.getY().getMm());
        }

        /**
         * Resets the current position to 0,0,0 and recalibrates the Odometry Computer's internal
         * IMU. <strong> Robot MUST be stationary </strong> <br><br> The device takes a large number
         * of samples, and uses those as the gyroscope zero-offset. This takes approximately 0.25
         * seconds.
         */
        public void resetPosAndIMU() { writeInt(2); }

        /**
         * Can reverse the direction of each encoder.
         *
         * @param xEncoder FORWARD or REVERSED, X (forward) pod should increase when the robot is
         *                 moving forward
         * @param yEncoder FORWARD or REVERSED, Y (strafe) pod should increase when the robot is
         *                 moving left
         */
        public void setEncoderDirections(EncoderDirection xEncoder, EncoderDirection yEncoder) {
            if (xEncoder == EncoderDirection.FORWARD) {
                writeInt(32);
            } else if (xEncoder == EncoderDirection.REVERSED) {
                writeInt(16);
            }

            if (yEncoder == EncoderDirection.FORWARD) {
                writeInt(8);
            } else if (yEncoder == EncoderDirection.REVERSED) {
                writeInt(4);
            }
        }

        /**
         * If you're using goBILDA odometry pods, the ticks-per-mm values are stored here for
         * easy access.<br><br>
         *
         * @param pods goBILDA_SWINGARM_POD or goBILDA_4_BAR_POD
         */
        public void setEncoderResolution(GoBildaPods pods) {
            if (pods == GoBildaPods.goBILDA_SWINGARM_POD) {
                writeByteArray(Register.MM_PER_TICK, floatToByteArray(goBILDA_SWINGARM_POD));
            } else if (pods == GoBildaPods.goBILDA_4_BAR_POD) {
                writeByteArray(Register.MM_PER_TICK, floatToByteArray(goBILDA_4_BAR_POD));
            }
        }

        /**
         * Sets the encoder resolution in ticks per mm of the odometry pods. <br>
         * You can find this number by dividing the counts-per-revolution of your encoder by the
         * circumference of the wheel.
         *
         * @param ticksPerUnit the ticks per distance unit of your odometry pod encoders.
         */
        public void setEncoderResolution(Dist ticksPerUnit) {
            double resolution = ticksPerUnit.getMm();
            writeByteArray(Register.MM_PER_TICK, floatToByteArray((float) resolution));
        }

        /**
         * Tuning this value should be unnecessary. <br> The goBILDA Odometry Computer has a
         * per-device tuned yaw offset already applied when you receive it.<br><br> This is a scalar
         * that is applied to the gyro's yaw value. Increasing it will mean it will report more than
         * one degree for every degree the sensor fusion algorithm measures.
         *
         * @param yawOffset A scalar for the robot's heading.
         */
        public void setYawScalar(double yawOffset) {
            writeByteArray(Register.YAW_SCALAR, floatToByteArray((float) yawOffset));
        }

        /**
         * Send a position that the Pinpoint should use to track your robot relative to.
         *
         * @param pos a Pose2D describing the robot's new position.
         */
        public void setPosition(Pose pos) {
            writeByteArray(Register.X_POSITION, floatToByteArray((float) pos.getX().getMm()));
            writeByteArray(Register.Y_POSITION, floatToByteArray((float) pos.getY().getMm()));
            writeByteArray(Register.H_ORIENTATION,
                    floatToByteArray((float) pos.getHeading().getRad()));
            pose = pos;
            velocity = Pose.zero();
            lastSampleValid = false;
        }

        /**
         * @return a Pose2D containing the estimated position of the robot
         */
        public Pose getPosition() { return pose; }

        /**
         * @return a Pose2D containing the estimated velocity of the robot, velocity is unit per
         * second.
         */
        public Pose getVelocity() { return velocity; }
    }
}
