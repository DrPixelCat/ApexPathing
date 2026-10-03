package tuning;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

import storage.ApexStorage;

/** Small failure-tolerant CSV writer for tuner evidence captured on robot or desktop. */
public final class TuningCsvWriter {
    private final File file;
    private final FileWriter writer;
    private final int autoFlushRows;
    private final StringBuilder pending;
    private String error;
    private int rowsSinceFlush;

    public static TuningCsvWriter open(String prefix, String... header) {
        return open(prefix, 25, header);
    }

    /** Opens a writer whose caller will flush at safe points outside time-critical motion. */
    public static TuningCsvWriter openBuffered(String prefix, String... header) {
        return open(prefix, 0, header);
    }

    private static TuningCsvWriter open(String prefix, int autoFlushRows, String... header) {
        try {
            File directory = ApexStorage.getDirectory();
            if (!directory.exists() && !directory.mkdirs()) {
                throw new IOException("Could not create " + directory.getAbsolutePath());
            }
            String timestamp = new SimpleDateFormat(
                    "yyyyMMdd_HHmmss_SSS", Locale.US).format(new Date());
            File file = new File(directory, prefix + "_" + timestamp + ".csv");
            TuningCsvWriter result = new TuningCsvWriter(
                    file, new FileWriter(file), autoFlushRows);
            result.writeRow((Object[]) header);
            return result;
        } catch (IOException e) {
            return new TuningCsvWriter(e.getMessage());
        }
    }

    private TuningCsvWriter(File file, FileWriter writer, int autoFlushRows) {
        this.file = file;
        this.writer = writer;
        this.autoFlushRows = autoFlushRows;
        this.pending = autoFlushRows == 0 ? new StringBuilder() : null;
    }

    private TuningCsvWriter(String error) {
        this.file = null;
        this.writer = null;
        this.autoFlushRows = 0;
        this.pending = null;
        this.error = error;
    }

    public void writeRow(Object... values) {
        if (writer == null) { return; }
        try {
            StringBuilder row = new StringBuilder();
            for (int i = 0; i < values.length; i++) {
                if (i > 0) { row.append(','); }
                row.append(escape(values[i]));
            }
            row.append('\n');
            if (pending != null) {
                pending.append(row);
            } else {
                writer.write(row.toString());
            }
            // Periodic flushing bounds data loss if Stop is pressed without adding filesystem
            // latency to every 50 Hz controller iteration.
            rowsSinceFlush++;
            if (autoFlushRows > 0 && rowsSinceFlush >= autoFlushRows) { flush(); }
        } catch (IOException e) {
            error = e.getMessage();
        }
    }

    public void flush() {
        if (writer == null) { return; }
        try {
            if (pending != null && pending.length() > 0) {
                writer.write(pending.toString());
                pending.setLength(0);
            }
            writer.flush();
            rowsSinceFlush = 0;
        } catch (IOException e) {
            error = e.getMessage();
        }
    }

    public void close() {
        if (writer == null) { return; }
        try {
            flush();
            writer.close();
        } catch (IOException e) {
            error = e.getMessage();
        }
    }

    public String getPath() { return file == null ? "Unavailable" : file.getAbsolutePath(); }

    public String getError() { return error; }

    private static String escape(Object value) {
        String text = value == null ? "" : String.valueOf(value);
        if (text.indexOf(',') < 0 && text.indexOf('"') < 0 &&
                text.indexOf('\n') < 0 && text.indexOf('\r') < 0) {
            return text;
        }
        return '"' + text.replace("\"", "\"\"") + '"';
    }
}
