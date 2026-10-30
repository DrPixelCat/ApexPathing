# Workflow cleanup validation

Validated on October 28, 2026. Existing workspace changes were retained. The
follower's internal lifecycle, measurement algorithms, units, coefficients,
sample validation, loop pauses, and numerical test tolerances were not changed.

## Passing focused suite

The normal Android Gradle test task passed **119 tests** across 27 suites:

```powershell
.\gradlew.bat :TeamCode:testDebugUnitTest --tests "setup.*" --tests "*TunerScreenWorkflowTest" --tests "*TelemetryFrameTest" --tests "*RobotSetupSimulationTest" --tests "*FollowerTunerSequenceTest" --console=plain
```

Coverage includes setup drafts and Save, failed saves, accepted calibration,
motor-test holds and release after navigation, timed screen boundaries,
read-only rendering, OpMode STOP, collection cleanup, CSV flushing, controller
policy restoration, and calibration/optimizer calculations.

New regressions exercise held confirmation through both tuner phase loops,
STOP during measurement with buffered evidence, and setup cleanup when the
hardware stop operation throws. Existing assertions were retained; obsolete
tests for the removed generic engine were deleted.

Compilation and `git diff --check` passed. The rebuilt runtime jar contains only
`WorkflowText` in the `workflow` package: the old engine and screen adapters are
absent.

## Simulation comparison

The pre-cleanup runtime was retained for comparison, without reverting the
workspace. The final measurement simulation selection completed **17 tests:
14 passed and 3 failed**.

| Suite | Tests | Baseline failures | Cleanup failures |
| --- | ---: | ---: | ---: |
| AccelerationFeedforwardPhaseSimulationTest | 2 | 1 | 0 |
| FeedforwardPhaseSimulationTest | 4 | 3 | 3 |
| LocalizationTunerWorkflowTest | 11 | 2 | 0 |

The remaining failures are the same feedforward moving-friction assertions:

| Test | Expected drive kS | Baseline measured | Cleanup measured |
| --- | ---: | ---: | ---: |
| linearMecanumSysIdFitsBothAxes | 0.06 | 0.28713 | 0.27257 |
| tankSysIdRecoversLowFrictionModel | 0.06 | 0.26275 | 0.26310 |
| linearMecanumSysIdIdentifiesDifferentMovingFriction | 0.03 | 0.27680 | 0.27029 |

No new failure appeared in this selection. The baseline acceleration failure
lacked sufficient valid unsaturated samples; the two baseline localization
failures timed out waiting for filter-collection stages. Their subsequent passes
are variable simulation outcomes, not evidence that their underlying issues were
fixed. No assertions or tolerances were relaxed and no acquisition retries were
added.

The simulation selection can be run through the normal task:

```powershell
.\gradlew.bat :TeamCode:testDebugUnitTest --tests "*LocalizationTunerWorkflowTest" --tests "*FeedforwardPhaseSimulationTest" --tests "*AccelerationFeedforwardPhaseSimulationTest" --console=plain
```

## Other observed failures and validation limits

The two manual SysId telemetry tests fail at their existing ten-line display
limit in both implementations. The mandatory STOP guidance makes their initial
idle frame eleven lines. Those assertions were not weakened. A matched
three-test comparison also verified that `stopDiscardsUnacceptedManualEdits`
passes in both implementations. An earlier interrupted mixed run failed that
test's phase-menu timing assertion; that result remains timing-sensitive and is
not claimed fixed. The unrelated full-follower simulation selected by that
exploratory run was canceled, not reported as passing.

The broader baseline also failed both TankTuningWorkflowSimulationTest cases:
the full tuning sequence did not complete, and the pre-tuned route exceeded its
cross-track limit (2.522003). Those full route tests were not repeated after the
cleanup; tank acquisition is covered by the focused measurement comparisons.

The long baseline reported a `ClassNotFoundException` for the obsolete
StateMachineTest when incremental compilation removed that test while the old
run was still executing. This is an obsolete test-loading artifact, separate
from its eight behavioral simulation failures. The final normal Gradle run has
no such error. One normal build attempt encountered Windows jar locks from the
running simulation; the normal build and focused tests passed after it exited.

Generated evidence remains under `TeamCode/build/`: normal focused results in
`test-results/testDebugUnitTest`, final simulations in
`test-results/cleanupSimulationTest`, paired telemetry comparisons in
`test-results/cleanupBaselineTelemetryTest` and `test-results/cleanupTelemetryTest`,
and saved baseline results in `workflow-baseline/test-results`. Temporary
validation scripts were removed.
