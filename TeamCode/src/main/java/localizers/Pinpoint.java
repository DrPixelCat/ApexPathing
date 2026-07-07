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
import java.util.Arrays;
import java.util.Objects;

import geometry.Dist;
import geometry.Vector;
import geometry.Pose;
import util.DistUnit;
import util.AngleUnit;
import util.PoseFactory;

/**
 * goBILDA Pinpoint Odometry Computer localizer (uses custom driver class)
 *
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public class Pinpoint extends BaseLocalizer<Pinpoint.Constants> {
    private final Driver pinpoint;

    public enum EncoderDirection { FORWARD, REVERSED }
    public enum GoBildaPods { goBILDA_SWINGARM_POD,  goBILDA_4_BAR_POD }

    public Pinpoint(Constants config, HardwareMap hardwareMap) {
        super(config);

        pinpoint = hardwareMap.get(Pinpoint.Driver.class, config.name);
        pinpoint.setOffsets(config.offsets);
        pinpoint.setEncoderDirections(config.xPodDirection, config.yPodDirection);
        if (config.customEncoderResolution.getIn() != 0) {
            pinpoint.setEncoderResolution(config.customEncoderResolution);
        } else {
            pinpoint.setEncoderResolution(config.encoderResolution);
        }
        if (config.yawScalar != 0) { pinpoint.setYawScalar(config.yawScalar); }
        pinpoint.resetPosAndIMU();
    }

    @Override
    public void update() {
        pinpoint.update();
        pose = pinpoint.getPosition();
        velocity = pinpoint.getVelocity();
        calculate(UpdateType.ACCELERATION);
    }

    @Override
    public void setPose(Pose newPose) {
        pinpoint.setPosition(newPose);
    }

    /** Configuration class for goBILDA Pinpoint localizer. */
    public static class Constants extends BaseLocalizerConstants<Constants> {
        public String name = "defaultPinpointNName";
        public Vector offsets = Vector.zero();
        public EncoderDirection xPodDirection = EncoderDirection.FORWARD;
        public EncoderDirection yPodDirection = EncoderDirection.FORWARD;
        public GoBildaPods encoderResolution = GoBildaPods.goBILDA_4_BAR_POD;
        public Dist customEncoderResolution = Dist.zero(); // Overrides encoderResolution if != 0
        public double yawScalar = 0; // Overrides the default

        @Override
        public Pinpoint build(HardwareMap hardwareMap) {
            if (Objects.equals(this.name, "defaultPinpointName")) {
                throw new IllegalArgumentException("Pinpoint name is not set in the localizer config.");
            }
            return new Pinpoint(this, hardwareMap);
        }

        /** Sets the name of the Pinpoint in the hardware map. */
        public Constants setName(String name) { this.name = name; return this; }

        /** Sets the X and Y pod offsets of the Pinpoint */
        public Constants setOffsets(double xOffset, double yOffset, DistUnit distanceUnit) {
            this.offsets = Vector.of(xOffset, yOffset, distanceUnit); return this;
        }

        /** Sets the X and Y pod offsets of the Pinpoint express as a Vector */
        public Constants setOffsets(Vector offsets) { this.offsets = offsets; return this; }

        /** Sets the direction of the X and Y encoders of the Pinpoint. */
        public Constants setEncoderDirections(EncoderDirection xPodDirection, EncoderDirection yPodDirection) {
            this.xPodDirection = xPodDirection; this.yPodDirection = yPodDirection; return this;
        }

        /** Sets the encoder resolution of the Pinpoint in ticks per mm using goBILDA pods. */
        public Constants setEncoderResolution(GoBildaPods encoderResolution) {
            this.encoderResolution = encoderResolution; return this;
        }

        /** Sets the encoder resolution of the Pinpoint in ticks per mm. */
        public Constants setEncoderResolution(Dist customEncoderResolution) {
            this.customEncoderResolution = customEncoderResolution; return this;
        }

        /** Sets the yaw scalar of the Pinpoint. It is not recommended to change this values. */
        public Constants setYawScalar(double yawScalar) {
            this.yawScalar = yawScalar; return this;
        }
    }

    /**
     * Driver class for the goBILDA Pinpoint Odometry Computer. This code is a modified version of the
     * original goBILDA Pinpoint Driver created by Ethan Doak from goBILDA. It has been modified to
     * better suit the needs of Apex Pathing. The original code can be found on the goBILDA
     * <a href="https://github.com/goBILDA-Official/FtcRobotController-Add-Pinpoint/">GitHub</a>.
     *
     * @author Ethan Doak - goBILDA
     * @author Dylan B. - 18597 RoboClovers Delta
     */
    @I2cDeviceType
    @DeviceProperties(
            name = "goBILDA® Pinpoint Odometry Computer",
            xmlTag = "goBILDAPinpoint",
            description = "goBILDA® Pinpoint Odometry Computer (IMU Sensor Fusion for 2 Wheel Odometry), optimized for Apex Pathing"
    )
    public static class Driver extends I2cDeviceSynchDevice<I2cDeviceSynchSimple> {
        private final PoseFactory pose = new PoseFactory(DistUnit.MM, AngleUnit.RAD);
        private int loopTime = 0;
        private float xPosition = 0;
        private float yPosition = 0;
        private float hOrientation = 0;
        private float xVelocity = 0;
        private float yVelocity = 0;
        private float hVelocity = 0;

        private static final float goBILDA_SWINGARM_POD = 13.26291192f; // goBILDA Swingarm Pod TPM
        private static final float goBILDA_4_BAR_POD = 19.89436789f; // goBILDA 4 Bar Pod TPM
        public static final byte DEFAULT_ADDRESS = 0x31; // I2C address of the device

        public Driver(I2cDeviceSynchSimple deviceClient, boolean deviceClientIsOwned) {
            super(deviceClient, deviceClientIsOwned);

            this.deviceClient.setI2cAddress(I2cAddr.create7bit(DEFAULT_ADDRESS));
            super.registerArmingStateCallback(false);
        }

        @Override
        public Manufacturer getManufacturer() { return Manufacturer.Other; }

        @Override
        protected synchronized boolean doInitialize() {
            ((LynxI2cDeviceSynch)(deviceClient)).setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);
            return true;
        }

        @Override
        public String getDeviceName() { return "goBILDA® Pinpoint Odometry Computer"; }

        // I2C registers
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
         * @param reg the register to write the int to
         * @param i the integer to write to the register
         */
        private void writeInt(final Register reg, int i){
            deviceClient.write(reg.bVal, TypeConversion.intToByteArray(i,ByteOrder.LITTLE_ENDIAN));
        }

        /**
         * Converts a byte array to a float value
         * @param byteArray byte array to transform
         * @param byteOrder order of byte array to convert
         * @return the float value stored by the byte array
         */
        private float byteArrayToFloat(byte[] byteArray, ByteOrder byteOrder){
            return ByteBuffer.wrap(byteArray).order(byteOrder).getFloat();
        }

        /**
         * Converts a float to a byte array
         * @param value the float array to convert
         * @return the byte array converted from the float
         */
        private byte [] floatToByteArray (float value, ByteOrder byteOrder) {
            return ByteBuffer.allocate(4).order(byteOrder).putFloat(value).array();
        }

        /**
         * Writes a byte array to a register on the i2c device
         * @param reg the register to write to
         * @param bytes the byte array to write
         */
        private void writeByteArray (Register reg, byte[] bytes){ deviceClient.write(reg.bVal,bytes); }

        /**
         * Writes a float to a register on the i2c device
         * @param reg the register to write to
         * @param f the float to write
         */
        private void writeFloat (Register reg, float f){
            byte[] bytes = ByteBuffer.allocate(4).order(ByteOrder.LITTLE_ENDIAN).putFloat(f).array();
            deviceClient.write(reg.bVal,bytes);
        }

        /**
         * Confirm that the number received is a number, and does not include a change above the threshold
         * @param oldValue the reading from the previous cycle
         * @param newValue the new reading
         * @param threshold the maximum change between this reading and the previous one
         * @param bulkUpdate true if we are updating the loopTime variable. If not it should be false.
         * @return newValue if the position is good, oldValue otherwise
         */
        private Float isPositionCorrupt(float oldValue, float newValue, int threshold, boolean bulkUpdate){
            boolean noData = bulkUpdate && (loopTime < 1);
            boolean isCorrupt = noData || Float.isNaN(newValue) || Math.abs(newValue - oldValue) > threshold;

            if (!isCorrupt) { return newValue; }

            return oldValue;
        }

        /**
         * Confirm that the number received is a number, and does not include a change above the threshold
         * @param oldValue the reading from the previous cycle
         * @param newValue the new reading
         * @param threshold the velocity allowed to be reported
         * @return newValue if the velocity is good, oldValue otherwise
         */
        private Float isVelocityCorrupt(float oldValue, float newValue, int threshold){
            boolean isCorrupt = Float.isNaN(newValue) || Math.abs(newValue) > threshold;

            if (!isCorrupt) { return newValue; }

            return oldValue;
        }

        /**
         * Call this once per loop to read new data from the Odometry Computer.
         */
        public void update() {
            // Set thresholds well above any normal values to detect corruption
            final int positionThreshold = 5000;
            final int headingThreshold = 120;
            final int velocityThreshold = 10000;
            final int headingVelocityThreshold = 120;

            float oldPosX = xPosition;
            float oldPosY = yPosition;
            float oldPosH = hOrientation;
            float oldVelX = xVelocity;
            float oldVelY = yVelocity;
            float oldVelH = hVelocity;

            byte[] bArr = deviceClient.read(Register.BULK_READ.bVal, 40);
            loopTime = byteArrayToInt(Arrays.copyOfRange  (bArr, 4, 8),  ByteOrder.LITTLE_ENDIAN);
            xPosition = byteArrayToFloat(Arrays.copyOfRange(bArr, 16,20), ByteOrder.LITTLE_ENDIAN);
            yPosition = byteArrayToFloat(Arrays.copyOfRange(bArr, 20,24), ByteOrder.LITTLE_ENDIAN);
            hOrientation = byteArrayToFloat(Arrays.copyOfRange(bArr, 24,28), ByteOrder.LITTLE_ENDIAN);
            xVelocity = byteArrayToFloat(Arrays.copyOfRange(bArr, 28,32), ByteOrder.LITTLE_ENDIAN);
            yVelocity = byteArrayToFloat(Arrays.copyOfRange(bArr, 32,36), ByteOrder.LITTLE_ENDIAN);
            hVelocity = byteArrayToFloat(Arrays.copyOfRange(bArr, 36,40), ByteOrder.LITTLE_ENDIAN);

            /*
             * Check to see if any of the floats we have received from the device are NaN or are too large
             * if they are, we return the previously read value and alert the user via the DeviceStatus Enum.
             */
            xPosition = isPositionCorrupt(oldPosX, xPosition, positionThreshold, true);
            yPosition = isPositionCorrupt(oldPosY, yPosition, positionThreshold, true);
            hOrientation = isPositionCorrupt(oldPosH, hOrientation, headingThreshold, true);
            xVelocity = isVelocityCorrupt(oldVelX, xVelocity, velocityThreshold);
            yVelocity = isVelocityCorrupt(oldVelY, yVelocity, velocityThreshold);
            hVelocity = isVelocityCorrupt(oldVelH, hVelocity, headingVelocityThreshold);
        }

        /**
         * Sets the odometry pod positions relative to the point that the odometry computer tracks around.
         * The most common tracking position is the center of the robot.
         * @param offset a Vector containing the X and Y offsets of the odometry pods
         */
        public void setOffsets(Vector offset) {
            writeFloat(Register.X_POD_OFFSET, (float) offset.getX().getM());
            writeFloat(Register.Y_POD_OFFSET, (float) offset.getY().getM());
        }

        /**
         * Resets the current position to 0,0,0 and recalibrates the Odometry Computer's internal IMU.
         * <strong> Robot MUST be stationary </strong> <br><br>
         * Device takes a large number of samples, and uses those as the gyroscope zero-offset. This takes approximately 0.25 seconds.
         */
        public void resetPosAndIMU() { writeInt(Register.DEVICE_CONTROL, 1<<1); }

        /**
         * Can reverse the direction of each encoder.
         * @param xEncoder FORWARD or REVERSED, X (forward) pod should increase when the robot is moving forward
         * @param yEncoder FORWARD or REVERSED, Y (strafe) pod should increase when the robot is moving left
         */
        public void setEncoderDirections(EncoderDirection xEncoder, EncoderDirection yEncoder) {
            if (xEncoder == EncoderDirection.FORWARD){ writeInt(Register.DEVICE_CONTROL, 1<<5); }
            else if (xEncoder == EncoderDirection.REVERSED) { writeInt(Register.DEVICE_CONTROL, 1<<4); }

            if (yEncoder == EncoderDirection.FORWARD){ writeInt(Register.DEVICE_CONTROL, 1<<3); }
            else if (yEncoder == EncoderDirection.REVERSED){ writeInt(Register.DEVICE_CONTROL, 1<<2); }
        }

        /**
         * If you're using goBILDA odometry pods, the ticks-per-mm values are stored here for easy access.<br><br>
         * @param pods goBILDA_SWINGARM_POD or goBILDA_4_BAR_POD
         */
        public void setEncoderResolution(GoBildaPods pods) {
            if (pods == GoBildaPods.goBILDA_SWINGARM_POD) {
                writeByteArray(Register.MM_PER_TICK, (floatToByteArray(goBILDA_SWINGARM_POD, ByteOrder.LITTLE_ENDIAN)));
            } else if (pods == GoBildaPods.goBILDA_4_BAR_POD) {
                writeByteArray(Register.MM_PER_TICK, (floatToByteArray(goBILDA_4_BAR_POD, ByteOrder.LITTLE_ENDIAN)));
            }
        }

        /**
         * Sets the encoder resolution in ticks per mm of the odometry pods. <br>
         * You can find this number by dividing the counts-per-revolution of your encoder by the circumference of the wheel.
         * @param ticksPerUnit the ticks per distance unit of your odometry pod encoders.
         */
        public void setEncoderResolution(Dist ticksPerUnit) {
            double resolution = ticksPerUnit.getMm();
            writeByteArray(Register.MM_PER_TICK, (floatToByteArray((float) resolution, ByteOrder.LITTLE_ENDIAN)));
        }

        /**
         * Tuning this value should be unnecessary.<br>
         * The goBILDA Odometry Computer has a per-device tuned yaw offset already applied when you receive it.<br><br>
         * This is a scalar that is applied to the gyro's yaw value. Increasing it will mean it will report more than one degree for every degree the sensor fusion algorithm measures. <br><br>
         * You can tune this variable by rotating the robot a large amount (10 full turns is a good starting place) and comparing the amount that the robot rotated to the amount measured.
         * Rotating the robot exactly 10 times should measure 3600°. If it measures more or less, divide moved amount by the measured amount and apply that value to the Yaw Offset.<br><br>
         * If you find that to get an accurate heading number you need to apply a scalar of more than 1.05, or less than 0.95, your device may be bad. Please reach out to tech@gobilda.com
         * @param yawOffset A scalar for the robot's heading.
         */
        public void setYawScalar(double yawOffset) {
            writeByteArray(Register.YAW_SCALAR, (floatToByteArray((float) yawOffset, ByteOrder.LITTLE_ENDIAN)));
        }

        /**
         * Send a position that the Pinpoint should use to track your robot relative to. You can use this to
         * update the estimated position of your robot with new external sensor data, or to run a robot
         * in field coordinates. <br><br>
         * This overrides the current position. <br><br>
         * <strong>Using this feature to track your robot's position in field coordinates:</strong> <br>
         * When you start your code, send a Pose2D that describes the starting position on the field of your robot. <br>
         * Say you're on the red alliance, your robot is against the wall and closer to the audience side,
         * and the front of your robot is pointing towards the center of the field.
         * You can send a setPosition with something like -600mm x, -1200mm Y, and 90 degrees. The pinpoint would then always
         * keep track of how far away from the center of the field you are. <br><br>
         * <strong>Using this feature to update your position with additional sensors: </strong><br>
         * Some robots have a secondary way to locate their robot on the field. This is commonly
         * Apriltag localization in FTC, but it can also be something like a distance sensor.
         * Often these external sensors are absolute (meaning they measure something about the field)
         * so their data is very accurate. But they can be slower to read, or you may need to be in a very specific
         * position on the field to use them. In that case, spend most of your time relying on the Pinpoint
         * to determine your location. Then when you pull a new position from your secondary sensor,
         * send a setPosition command with the new position. The Pinpoint will then track your movement
         * relative to that new, more accurate position.
         * @param pos a Pose2D describing the robot's new position.
         */
        public void setPosition(Pose pos) {
            writeByteArray(Register.X_POSITION, (floatToByteArray((float) pos.getX().getMm(), ByteOrder.LITTLE_ENDIAN)));
            writeByteArray(Register.Y_POSITION, (floatToByteArray((float) pos.getY().getMm(), ByteOrder.LITTLE_ENDIAN)));
            writeByteArray(Register.H_ORIENTATION, (floatToByteArray((float) pos.getHeading().getRad(), ByteOrder.LITTLE_ENDIAN)));
        }

        /**
         * @return a Pose2D containing the estimated position of the robot
         */
        public Pose getPosition() { return pose.of(xPosition, yPosition, hOrientation); }

        /**
         * @return a Pose2D containing the estimated velocity of the robot, velocity is unit per second
         */
        public Pose getVelocity() { return pose.of(xVelocity, yVelocity, hVelocity); }
    }
}