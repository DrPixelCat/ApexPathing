package tuning.logging;

import java.io.File;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.Locale;

import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

import static org.junit.Assert.*;

public class CsvLogTest {
    @Rule public TemporaryFolder temp = new TemporaryFolder();

    @Test public void escapesTextAndKeepsNumbersIndependentOfDeviceLocale() throws Exception {
        File file = temp.newFile();
        Locale previous = Locale.getDefault();
        try {
            Locale.setDefault(Locale.GERMANY);
            try (CsvLog log = new CsvLog(file, "number", "text", "missing")) {
                log.row(1.25, "comma, quote\" and\nnewline", null);
            }
        } finally { Locale.setDefault(previous); }
        String text = new String(Files.readAllBytes(file.toPath()), StandardCharsets.UTF_8);
        assertTrue(text.contains("1.25,\"comma, quote\"\" and\nnewline\","));
    }

    @Test public void sessionsAreUniqueAndCloseRetainsAnInterruptedRunsLastRow() throws Exception {
        File first;
        try (TuningLog log = new TuningLog(temp.getRoot(), "test")) {
            first = log.getDirectory();
            log.write("samples", new String[]{"time_s", "value"}, 0.0, 7);
        }
        try (TuningLog second = new TuningLog(temp.getRoot(), "test")) {
            assertNotEquals(first, second.getDirectory());
        }
        assertEquals(2, Files.readAllLines(new File(first, "samples.csv").toPath()).size());
    }

    @Test public void refusesToContinueWhenLogStorageCannotBeCreated() throws Exception {
        File parent = temp.newFile();
        try {
            new TuningLog(parent, "test");
            fail("Must report unavailable storage");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().contains("Cannot create CSV folder"));
        }
    }

    @Test public void rejectsMalformedRowBeforeWritingIt() throws Exception {
        File file = temp.newFile();
        try (CsvLog log = new CsvLog(file, "a", "b")) {
            try { log.row(1); fail("wrong row width"); }
            catch (IllegalArgumentException expected) { /* keep the CSV rectangular */ }
            log.row(1, 2);
        }
        assertEquals(2, Files.readAllLines(file.toPath()).size());
    }
}
