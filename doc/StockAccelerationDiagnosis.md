# Stock FTCodeSim acceleration plateau diagnosis

On October 2, 2026, independent state probes were run through the stock `SimulatedMecanum`
and registered stock motors. Their equations and coefficients were not changed. The
diagnostic is `StockAccelerationDiagnosticTest`; outputs are under
`TeamCode/build/stock-ftcodesim-retuned-20261002/acceleration-diagnosis`.

## Main cause: incorrect high-speed acceleration authority

Apex does account for speed-dependent power consumption. `HolonomicDriveModel.powerEvaluator`
uses the calibrated affine feedforward relation `u = kS + kV*v + kA*a` for forward powered
motion, and `PathProfileGenerator` limits its predicted normalized utilization to 1.0.
However, that fitted relation is insufficient to characterize this stock simulator across
its full powered acceleration envelope.

For straight forward motion with positive velocity and positive motor power, the actual
loaded stock motor/chassis branch gives `a = (183 - 2.30769230769*v)*u - 49`, with speed in
in/s, acceleration in in/s², and normalized motor power `u`. The duty-dependent back-EMF
term couples power and velocity. This expression describes the existing upstream branch;
it is not a replacement plant or a proposed production controller. It was checked against
actual library stepping at all diagnostic speeds and powers, within 1e-7 in/s².

| Speed (in/s) | Affine fit's acceleration at full power (in/s²) | Stock measured acceleration at full power (in/s²) |
|---|---:|---:|
| 20 | 112.952 | 87.846 |
| 30 | 97.391 | 64.769 |
| 40 | 81.829 | 41.692 |
| 50 | 66.268 | 18.615 |
| 52.884 | 61.780 | 11.961 |

The profiler also imposes the configured 94.734 in/s² acceleration limit, so the first two
fit-derived ceilings are additionally capped. That cap does not resolve the high-speed
mismatch. A measured peak acceleration with a fixed 80% margin cannot substitute for a
validated acceleration-versus-speed envelope.

The recorded Auto Test's initial straight acceleration provides independent saturation
evidence: at 0.564208 s the target velocity was 41.753 in/s, the raw velocity was 36.123 in/s,
the target acceleration was 79.290 in/s², and both logged front motor commands were 1.0.
The straight-line symmetry gives equal commands on all wheels. No heading or lateral motion
was being requested there. Feedforward demand was 1.001 and velocity-feedback allocation
was zero. The physical deficit therefore cannot be repaired by increasing feedback gain.

## Additional cause: holding-power bias and short cruise intervals

At 52.884 in/s, the affine feedforward predicts holding power 0.673325. The stock plant
requires 0.803799. Proportional velocity feedback needs a nonzero velocity error to supply
that missing power.

A fixed-heading experiment held the same 52.884 in/s target for 1.8 simulated seconds,
starting at 44 in/s. The same feedback gain and actual stock simulator were used in both
cases. With the fitted base power the final speed was 51.098738 in/s; with the stock-state
holding power as the diagnostic input it was 52.881542 in/s. These are physical simulation
results from the library, not integration of a separately implemented plant. The second
input is only a diagnostic control experiment; production feedforward remains unchanged.

In Auto Test, the logged exact flat-top samples span approximately 0.280 s, 0.240 s and
0.120 s. During the first of these the raw speed rose from 44.124 to 49.286 in/s before
deceleration began. The plateau speed is reachable, but the inaccurate ramp leaves a deficit
and the short cruise does not provide enough recovery time.

## Implication for the next change

The first correction should be a measured, conservative positive-acceleration envelope
as a function of speed and direction, validated with independent powered runs. The profile
must preserve room for tracking corrections and combined heading/lateral demand. Feedforward
validation should include high-speed holding-power and full-power acceleration probes;
the existing high overall SysId R² does not establish feasibility at those conditions.
The simulator must remain unchanged. No production profiler, feedforward, allocator or
braking implementation was changed during this investigation.

`Full_Power_Authority.png` compares fit predictions with stock measurements.
`Plateau_Hold.png` shows the two stock-plant holding experiments. Raw measurements are
`forward-authority.csv` and `plateau-hold.csv`; the command and results are preserved in
`graph-gradle.log` and `run-results.txt`.
