# Straight path with rotation: feedforward diagnosis

Before the coupled wheel correction, the 80-inch, 0-to-90-degree stored-path test
had a real combined-motion model failure. The initial run turned only 9.14 degrees; the instrumented repeat
turned 10.27 degrees, with all feedback and endpoint blending at zero. Independent
plant and localizer measurements agree. Small variations come from host-timed execution.

The validated correction and current graphs are documented in
[MecanumWheelFeedforward.md](MecanumWheelFeedforward.md). The findings below
record the original axis model and the evidence used to select that correction.

## Findings

1. **A tiny strafe component activates full axis drag compensation.**
   `FollowerConstants.characterizedTranslation` inverts the forward and strafe
   maps independently. At 16 in/s, changing target heading from exactly zero to
   0.001 degrees changes strafe power from 0 to -0.466625. The commanded lateral
   velocity is only -0.000279 in/s. A controlled probe then measures approximately
   -101.76 in/s2 lateral acceleration. The independent strafe friction offset is
   being added as though the chassis were moving purely sideways. With both axes
   active, these drag terms do not represent the stock chassis response.

2. **Angular braking switches to a different, incompatible model.**
   `MotionCharacterization.power` falls back to legacy gains when velocity and
   acceleration have opposite signs. At 0.47 rad/s, requested turn power is
   0.063418 at zero angular acceleration, but 0.027185 at -0.001 rad/s2: a 57.1%
   drop for a negligible reference change. While translating at 16 in/s, actual
   deceleration is -0.945284 rad/s2 instead of the requested -0.001. In the
   instrumented path, turn power drops from about 0.063 to 0.027 around
   40 inches. Allocation scale is 1 on both sides of the transition.

3. **Turning authority depends on the other wheel motion.**
   The angular response was characterized in isolation. Keeping the same angular
   velocity and turn power, zero forward speed gives +0.185593 rad/s2 angular
   acceleration; adding 16 in/s forward travel changes that to -0.373213, and
   32 in/s changes it to -0.746759. None of these probes clips. The upstream motor
   response depends on each wheel's velocity and power sign; combining the three
   independent axis inverses cannot capture those interactions. This affects the
   acceleration/cruise portion as well as the braking transition.

4. **A heading error amplifies translation error.**
   The follower constructs calibrated body-axis demands using target heading,
   rotates them to the field, then the allocator rotates them using actual heading.
   Once rotation lags, those demands correspond to different assumed wheel
   velocities. At target heading 45 degrees but actual heading zero, the current
   16 in/s translation feedforward produces +40.49 in/s2 forward acceleration and
   -22.81 in/s2 lateral acceleration in a controlled probe. Even with matching
   target/actual heading, the independent drag composition has a residual.

## What was checked

`StraightTwistFeedforwardDiagnosticTest` probes the unmodified registered stock
FTCodeSim motor/chassis mechanism at imposed reference velocities. Each probe is
0.1 ms, with no follower, allocator, feedback or wall-clock pacing. It verifies
that the requested four wheel commands reach the motors and remain below clipping.
It does not read plant coefficients to construct a replacement model.

The actual path test now logs all four wheel commands, requested turn feedforward,
allocated turn power, allocated body-axis powers and feedforward demand. The final
instrumented run delivered turn feedforward within 0.000925 normalized power of
the requested value times allocation scale; 129 of 138 samples had full allocation.
`StockSimulatorPlantTest` also confirms that the concrete upstream plant and motor
coefficients remain unchanged. Allocation and measurement error do not account
for the confirmed model failures.

The previous `complete=true` diagnostic meant the follower was no longer busy.
The endpoint remained far outside the 2-degree heading tolerance; execution ended
through the 0.5-second stuck watchdog. The log now calls this `followerStopped`.
Passing this diagnostic validates execution and feedback isolation, not accuracy.

## A measured correction experiment

At 16 in/s forward and 0.47 rad/s angular speed, a local three-axis response
matrix was fitted from small command perturbations. No live tracking error was
fed into a controller. A new command, evaluated in an independent probe, changed
forward/turn demand from (0.345284, 0.063418) to (0.341402, 0.086789). Both linear
and angular acceleration residuals fell below 0.000001 in their respective units.

This original local experiment verified the feasibility of a coupled response
at one moving state. The subsequent measured wheel model validates combined
states and whole-path execution as described in the linked correction report.
The required model properties were to make translation drag depend on the
combined motion direction, account for simultaneous wheel motion in turning
authority, and share continuous signed braking regimes between planning and
execution. Merely validating peak motor power cannot establish these properties.

## Reproduction and artifacts

```powershell
./gradlew.bat :TeamCode:testDebugUnitTest --tests org.firstinspires.ftc.teamcode.sim.StraightTwistFeedforwardDiagnosticTest
$env:APEX_AUTOTEST_GRAPH_DIR = Join-Path (Get-Location) 'TeamCode/build/path-power-graphs/straight-twist-diagnosis/live-run'
./gradlew.bat :TeamCode:testDebugUnitTest --tests org.firstinspires.ftc.teamcode.sim.StraightTwistFeedforwardSimulationTest --tests org.firstinspires.ftc.teamcode.sim.StockSimulatorPlantTest
```

`TeamCode/build/path-power-graphs/straight-twist-diagnosis` contains
`controlled-probes.csv`, `summary.txt`, `Angular_Coupling.png`, and
`Tiny_Heading_Strafe_Power.png`. Its `live-run` directory contains the instrumented
path trace and the original motion/power charts, plus `_Turn_Power.png` comparing
requested and delivered signed turn demand.

The path-test command now exercises the corrected model; it does not reproduce
the historical failure. Controlled axis-model probes still reproduce the original
root-cause evidence. Corrected path tests assert motion accuracy and feedback
isolation, including execution through the planned stop.
