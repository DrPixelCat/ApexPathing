package org.firstinspires.ftc.teamcode.apexpathing;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class AutoTestSequenceTest {
    @Test
    public void autoTestStopsAfterOutboundCurve() {
        assertEquals(AutoTest.AutoState.COMPLETE,
                AutoTest.nextState(AutoTest.AutoState.OUTBOUND_CURVE));
    }

    @Test
    public void terminalAutoStatesCannotAdvance() {
        assertEquals(AutoTest.AutoState.COMPLETE,
                AutoTest.nextState(AutoTest.AutoState.COMPLETE));
        assertEquals(AutoTest.AutoState.FAILED,
                AutoTest.nextState(AutoTest.AutoState.FAILED));
    }
}
