# Fresh SysId manual validation

The sections below record the original joint-fit baseline, before the staged
identification change. The original harness capped physics catch-up time and updated
physics concurrently with localization; its profile errors can include timing artifacts.
The direct holding-power probe is independent of those timing artifacts. The current
harness advances all elapsed physics time immediately before the tuner sensor frame.

On October 5, 2026, `FreshSysIdManualValidationTest` ran the production automatic
drive and heading SysId phases from empty storage on the stock FTCodeSim mecanum
setup, then ran both directions of each production manual phase without editing
the identified gains. Only simulator positioning and gamepad inputs were supplied
by the harness; the motor/chassis physics and tuner algorithms were unchanged.

Two independent fresh runs reproduced acceleration lag followed by cruise overspeed.
The final run reported:

| Trial | Acceleration mean signed velocity error | Cruise mean signed velocity error |
| --- | --- | --- |
| Drive forward | -0.97238 in/s | +2.25261 in/s |
| Drive backward | +1.00640 in/s | -2.23308 in/s |
| Heading CCW | -0.21075 rad/s | +0.11489 rad/s |
| Heading CW | +0.21346 rad/s | -0.10573 rad/s |

All four trials completed. Drive command magnitude stayed below 0.561, so power
clipping was not responsible. Independent integration of the recorded measured
velocity reproduced acceleration scores within 0.009 in/s and cruise scores within
0.001 in/s (small capture-time differences). Direct simulator velocity also showed
drive cruise overspeed: approximately +2.14/-2.17 in/s over the same scored intervals.
Cruise scoring began around 1.056/1.063 seconds, when the measured speed first crossed
24 in/s. Braking was excluded from these scores.

The final fresh drive fit was kS=0.2407863252, kV=0.00817537585,
kA=0.00513595369, with R-squared=0.998375. It passes the existing fit checks but
commands `kS + kV*24 = 0.43699535` during cruise. An independent stock-state probe
at exactly 24 in/s measured +6.767329 in/s² acceleration under that command.
The same unmodified physics measured zero acceleration with holding power
0.38396624. The fitted cruise command is therefore approximately 13.8% too high.

That run's fitted gains did not reproduce the stock holding-power probe. This
does not establish that the identification procedure is invalid: measurement
alignment and harness timing must be ruled out before assigning a cause. A high
fit R-squared alone does not establish independent tracking accuracy. The earlier manual UI tests used
hand-supplied gains and checked controls and lifecycle; they did not establish
fresh automatic-fit tracking accuracy.

Evidence is written to `TeamCode/build/fresh-sysid-manual-validation`: fitted
constants, automatic measurement CSVs, manual traces, reported results, independent
score calculations, direct holding-power probes, and drive/heading plots. The new
diagnostic test also asserts that the stock holding-power equation agrees with
actual library stepping. No production algorithm or saved simulator calibration
was changed during this investigation.

## Displacement-based comparison

A subsequent fresh-tuning run compared both reference types using exactly the same
identified gains. The displacement reference was the same trapezoid indexed by
measured progress: `v(s) = min(vMax, sqrt(2*a*s), sqrt(2*a*(length-s)))`.
Drive used 48 inches, 24 in/s and 24 in/s², with acceleration/cruise/braking boundaries
at 12 and 36 inches. Heading used 3 radians, 1.5 rad/s and 1.5 rad/s², with boundaries
at 0.75 and 2.25 radians. Feedback remained disabled. Spatial scoring split sample
intervals at the phase boundaries and excluded braking.

| Trial | Time acceleration error | Spatial acceleration error | Time cruise error | Spatial cruise error |
| --- | --- | --- | --- | --- |
| Drive forward (in/s) | -0.93606 | -0.51943 | +2.23779 | +2.38947 |
| Drive backward (in/s) | +0.96731 | +0.51334 | -2.07023 | -2.13539 |
| Heading CCW (rad/s) | -0.21400 | -0.09049 | +0.11770 | +0.18494 |
| Heading CW (rad/s) | +0.21428 | +0.09038 | -0.12298 | -0.17367 |

The spatial reference reduced acceleration lag but did not eliminate cruise
overspeed. Scoring the entire spatial cruise phase, including time below target,
still produced +2.32834/-2.05860 in/s for drive and +0.17940/-0.16773 rad/s for
heading. Direct simulator velocity confirmed the spatial drive overspeed at
+2.24945/-2.05237 in/s. Maximum command magnitudes were below 0.560 for drive
and 0.168 for heading, so there was no clipping.

Both spatial drive trials reached the 48-inch termination threshold. The angular
trials stalled during braking at 2.65401 and 2.67997 radians and hit the eight-second
diagnostic timeout before reaching the 3-radian endpoint. They still supplied full
acceleration/cruise evidence; timeout is a separate completion limitation, not a
successful endpoint stop. Braking mean velocity error was not scored.

The existing `LinePowerSimulationTest.storedLineWithoutFeedback` was also run without
changes. Its real stored-path follower completed the 80-inch line with simulator
velocity RMSE 0.355444 in/s and endpoint error 0.222 inches, with zero feedback and
endpoint blend. That baseline uses `SimulatorTuning`'s saved calibration, including
motion characterization, rather than the fresh three-coefficient fit. Its success
therefore does not contradict the isolated fresh-gain results.

The paired test and existing stored-line test finished successfully. Spatial traces
and scores are in `displacement-traces.csv` and `displacement-results.csv` in the
same evidence directory. No production profile or feedforward code was changed.

## Withdrawn staged identification experiment

An experimental production change measured settled holding power at two speeds in each
direction to identify kS/kV, fits dynamic acceleration for an initial kA, then refines
kA with two profiles while holding kS/kV fixed. Separate forward/reverse profiles
must meet acceleration and full-cruise error limits before the automatic gains can
be accepted. This combines ideas from Road Runner's ramp/profile workflow and
WPILib's dynamic identification and response diagnostics; it is Apex code rather
than either library's implementation.

Eighteen focused unit checks passed. Fresh end-to-end acceptance remains unverified:
some runs improved holding power substantially, but the corrected harness still
encountered rejected measurements/refinement and a shutdown exception. Historical
successful runs are not evidence that the final staged implementation is validated.
No real robot measurements have been collected, and the simulator's motor physics
has not been changed. Profile timing and model mismatch both need to be distinguished
before drawing conclusions about expected hardware performance. This experiment has
now been removed. The original four-run joint fit is restored while the implementation
is compared against established libraries; see `SysIdRoadRunnerComparison.md`.
