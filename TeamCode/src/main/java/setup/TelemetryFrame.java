package setup;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

/** Text collected during an update, published without running the workflow again. */
public final class TelemetryFrame {
    private final Telemetry telemetry;
    private final List<Row> rows = new ArrayList<>();

    public TelemetryFrame(Telemetry telemetry) { this.telemetry = telemetry; }

    public void clearAll() {
        rows.clear();
        telemetry.clearAll();
    }
    public void addLine(String line) { rows.add(new Row(null, line)); }
    public void addData(String caption, Object value) { rows.add(new Row(caption, String.valueOf(value))); }
    public void addData(String caption, String format, Object... values) {
        addData(caption, String.format(Locale.US, format, values));
    }
    public List<String> view() {
        List<String> lines = new ArrayList<>();
        for (Row row : rows) {
            lines.add(row.caption == null ? row.value : row.caption + " " + row.value);
        }
        return Collections.unmodifiableList(lines);
    }

    public void update() {
        for (Row row : rows) {
            if (row.caption == null) { telemetry.addLine(row.value); }
            else { telemetry.addData(row.caption, row.value); }
        }
        telemetry.update();
    }

    private static final class Row {
        final String caption;
        final String value;

        Row(String caption, String value) { this.caption = caption; this.value = value; }
    }
}
