package setup;

import org.json.JSONObject;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;
import java.io.File;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import static org.junit.Assert.*;

public class SetupStoreTest {
    @Rule public TemporaryFolder temp = new TemporaryFolder();

    @Test public void replacementsRetainPreviousProfileAndRecoverInterruptedRename() throws Exception {
        SetupStore store = new SetupStore(temp.newFolder());
        RobotProfile first = new RobotProfile(); store.save(first);
        RobotProfile second = first.copy(); second.set("offsetX", "4"); store.save(second);
        File backup = new File(store.getFile().getPath() + ".bak");
        assertEquals(0, RobotProfile.fromJson(new JSONObject(RobotSetupWorkflowTest.read(backup))).number("offsetX"), 0);
        assertEquals(4, store.load().number("offsetX"), 0);
        assertTrue(store.getFile().delete());
        assertEquals("Interrupted replacement recovers the prior complete profile", 0, store.load().number("offsetX"), 0);
    }

    @Test public void corruptAndFutureProfilesDoNotSilentlyLoadDefaults() throws Exception {
        SetupStore store = new SetupStore(temp.newFolder());
        Files.write(store.getFile().toPath(), "{broken".getBytes(StandardCharsets.UTF_8));
        try { store.load(); fail("must reject corruption"); } catch (IOException expected) { }
        JSONObject json = new RobotProfile().toJson().put("version", 999);
        Files.write(store.getFile().toPath(), json.toString().getBytes(StandardCharsets.UTF_8));
        try { store.load(); fail("must reject unsupported version"); } catch (IOException expected) { }
    }

    @Test public void failedReplacementDoesNotModifySavedProfile() throws Exception {
        SetupStore store = new SetupStore(temp.newFolder()); store.save(new RobotProfile());
        byte[] previous = Files.readAllBytes(store.getFile().toPath());
        File backup = new File(store.getFile().getPath() + ".bak"); assertTrue(backup.mkdir());
        assertTrue(new File(backup, "unavailable").createNewFile());
        RobotProfile next = new RobotProfile(); next.set("offsetX", "8");
        try { store.save(next); fail("backup failure should block replacement"); } catch (IOException expected) { }
        assertArrayEquals(previous, Files.readAllBytes(store.getFile().toPath()));
    }

    @Test public void rejectsDuplicateDevicesParallelMountingAndIncompleteNoise() throws Exception {
        RobotProfile p = new RobotProfile(); p.set("fr", p.get("fl"));
        try { p.validate(); fail("duplicate drive motor"); } catch (IllegalArgumentException expected) { }
        p = RobotSetupWorkflowTest.profile("TWO_WHEEL"); p.set("logo", "UP"); p.set("usb", "DOWN");
        try { p.validate(); fail("parallel hub axes"); } catch (IllegalArgumentException expected) { }
        p = new RobotProfile(); p.filterMeasured = true;
        try { p.validate(); fail("missing filter data"); } catch (IllegalArgumentException expected) { }
        p.filter = new JSONObject().put("velocity", new JSONObject());
        try { p.validate(); fail("incomplete axis data"); } catch (IllegalArgumentException expected) { }
    }

    @Test public void repairingCorruptionPreservesTheLastReadableBackup() throws Exception {
        SetupStore store = new SetupStore(temp.newFolder());
        RobotProfile first = new RobotProfile(); first.set("offsetX", "4"); store.save(first);
        RobotProfile second = first.copy(); second.set("offsetX", "8"); store.save(second);
        Files.write(store.getFile().toPath(), "{broken".getBytes(StandardCharsets.UTF_8));
        RobotProfile recovered = store.loadPrevious(); assertEquals(4, recovered.number("offsetX"), 0);
        store.save(recovered);
        assertEquals(4, store.load().number("offsetX"), 0);
        assertEquals(4, store.loadPrevious().number("offsetX"), 0);
    }
}
