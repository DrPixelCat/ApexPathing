package org.firstinspires.ftc.teamcode.sim;

import setup.TelemetryFrame;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.*;

public class TelemetryFrameTest {
    @Test public void snapshotsAreReadOnlyAndPublicationPreservesLabelledValues() {
        List<String> published = new ArrayList<>();
        ApexSimTelemetry telemetry = new ApexSimTelemetry(published::add);
        telemetry.setMsTransmissionInterval(0);
        TelemetryFrame frame = new TelemetryFrame(telemetry);
        frame.addLine("Recording");
        frame.addData("Samples", 12);
        List<String> snapshot = frame.view();
        assertThrows(UnsupportedOperationException.class, () -> snapshot.add("changed"));
        assertTrue(published.isEmpty());
        frame.update();
        assertEquals(1, published.size());
        assertTrue(published.get(0).contains("Samples 12"));
        frame.clearAll();
        assertEquals(2, snapshot.size());
        assertTrue(frame.view().isEmpty());
    }

    @Test public void legacyPhaseTelemetryCanStillAddLinesDuringAnUpdate() {
        List<String> published = new ArrayList<>();
        ApexSimTelemetry telemetry = new ApexSimTelemetry(published::add);
        telemetry.setMsTransmissionInterval(0);
        TelemetryFrame frame = new TelemetryFrame(telemetry);
        frame.clearAll();
        telemetry.addLine("Custom phase details");
        frame.addLine("Shared screen");
        frame.update();
        assertTrue(published.get(0).contains("Custom phase details"));
        assertTrue(published.get(0).contains("Shared screen"));
    }
}
