package setup.hardware;

import follower.Follower;
import drivetrains.BaseDrivetrain;
import localizers.ThreeWheel;
import org.firstinspires.ftc.teamcode.apexpathing.Constants;
import org.json.JSONObject;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;
import support.FakeHardware;
import static org.junit.Assert.*;

public class SavedSetupIntegrationTest {
    @Rule public TemporaryFolder temp = new TemporaryFolder();

    @Test public void nextFollowerLoadsHardwareGeometryDirectionsAndFiltersWithoutJavaEdits() throws Exception {
        String previous = System.getProperty("apexpathing.storageDirectory");
        System.setProperty("apexpathing.storageDirectory", temp.getRoot().getAbsolutePath());
        try {
            Constants configuration = new Constants(); // OpMode discovery can instantiate it before setup is saved.
            RobotProfile p = RobotSetupWorkflowTest.profile("THREE_WHEEL");
            p.set("drive", "TANK_2"); p.set("ticksPerInch", "500"); p.set("offsetX", "10"); p.set("offsetY", "-3");
            p.set("forwardReverse", "true");
            p.filter = new RobotSetupWorkflowTest.Hardware().captureFilter(); p.filterMeasured = true;
            new SetupStore(temp.getRoot()).save(p);
            FakeHardware map = new FakeHardware();
            map.motor("frontLeftMotor"); map.motor("frontRightMotor");
            FakeHardware.MotorState left = map.motor("forward"), right = map.motor("right"), strafe = map.motor("strafe");
            Follower follower = Follower.create(configuration, map, true);
            assertEquals(BaseDrivetrain.DrivetrainType.TANK, follower.getDrivetrain().getDrivetrainType());
            assertTrue(follower.getLocalizer() instanceof ThreeWheel);
            assertEquals(.15, follower.getLocalizer().getXKalmanTuning().measurementVariance, 0);
            assertEquals(42, follower.getLocalizer().getXKalmanTuning().processVariance, 0);
            // A positive physical point turn with a reversed left channel.
            left.ticks = -2500; right.ticks = -2500; strafe.ticks = -1500;
            follower.update();
            assertEquals(1, follower.getPose().getHeading().getRad(), 1e-9);
            assertEquals(0, follower.getPose().getX().getIn(), 1e-9);
            assertEquals(0, follower.getPose().getY().getIn(), 1e-9);
            p.set("offsetX", "14"); new SetupStore(temp.getRoot()).save(p);
            assertEquals(14, ((ThreeWheel.Constants) configuration.localizerConstants()).offsets.getX().getIn(), 0);
            assertNull(new Constants(false).robotProfile());
        } finally {
            if (previous == null) { System.clearProperty("apexpathing.storageDirectory"); }
            else { System.setProperty("apexpathing.storageDirectory", previous); }
        }
    }
}
