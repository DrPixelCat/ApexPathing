package setup.logging;

import java.io.BufferedWriter;
import java.io.Closeable;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.nio.charset.StandardCharsets;

/** Buffered, locale-independent CSV. I/O failures abort the tuning run instead of losing data silently. */
public final class CsvLog implements Closeable {
    private final BufferedWriter writer;
    private final int columns;
    private long lastFlush = System.nanoTime();

    public CsvLog(File file, String... header) throws IOException {
        writer = new BufferedWriter(new OutputStreamWriter(new FileOutputStream(file), StandardCharsets.UTF_8));
        columns = header.length;
        row((Object[]) header);
        flush();
    }

    public void row(Object... values) {
        if (values.length != columns) {
            throw new IllegalArgumentException("CSV row has " + values.length + " columns; expected " + columns);
        }
        try {
            for (int i = 0; i < values.length; i++) {
                if (i > 0) { writer.write(','); }
                String value = values[i] == null ? "" : values[i].toString();
                if (value.contains(",") || value.contains("\"") || value.contains("\n") || value.contains("\r")) {
                    writer.write('"');
                    writer.write(value.replace("\"", "\"\""));
                    writer.write('"');
                } else {
                    writer.write(value);
                }
            }
            writer.newLine();
            if (System.nanoTime() - lastFlush >= 1_000_000_000L) { flush(); }
        } catch (IOException e) {
            throw new IllegalStateException("Cannot write tuning CSV", e);
        }
    }

    public void flush() {
        try {
            writer.flush();
            lastFlush = System.nanoTime();
        } catch (IOException e) {
            throw new IllegalStateException("Cannot flush tuning CSV", e);
        }
    }

    @Override public void close() throws IOException { writer.close(); }
}
