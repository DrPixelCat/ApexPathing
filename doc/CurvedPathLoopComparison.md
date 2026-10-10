# Curved-path follower update timing: Apex versus Pedro

Measured October 8, 2026, on the local Windows desktop with Temurin OpenJDK 21.0.11
HotSpot. This measures production follower `update` CPU work with replayed sensor
states and stub motor communication. It includes path projection/planning, feedback,
power allocation, and the production mecanum drivetrain code. Sensor acquisition,
localization filtering, telemetry, and physical motor communication are excluded.

**Apex quickBuild was 2.06–5.57 times faster than the tested Pedro Foresight snapshot.
Apex buildToMemory was another 3.60–5.70 times faster than quickBuild**, across these
12 workloads. These are desktop measurements, not Control Hub loop-time predictions.

## Median update time

All times are microseconds per individual follower update. Each cell is the median
of five round medians, with 25,600 measured updates in each round.

| Curve | Heading | Input speed (in/s) | Apex quickBuild (µs) | Apex buildToMemory (µs) | Pedro Foresight (µs) | Pedro / quickBuild | quickBuild / buildToMemory |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Wide corner | Constant | 8 | 3.60 | 1.00 | 8.60 | 2.39× | 3.60× |
| Wide corner | Constant | 30 | 4.40 | 1.10 | 11.60 | 2.64× | 4.00× |
| Wide corner | Tangent | 8 | 5.00 | 1.10 | 10.30 | 2.06× | 4.55× |
| Wide corner | Tangent | 30 | 4.80 | 0.90 | 10.20 | 2.12× | 5.33× |
| S-curve | Constant | 8 | 3.70 | 1.00 | 17.10 | 4.62× | 3.70× |
| S-curve | Constant | 30 | 4.40 | 1.20 | 24.50 | 5.57× | 3.67× |
| S-curve | Tangent | 8 | 5.70 | 1.00 | 21.30 | 3.74× | 5.70× |
| S-curve | Tangent | 30 | 7.40 | 1.30 | 29.20 | 3.95× | 5.69× |
| Tight corner | Constant | 8 | 4.50 | 1.10 | 12.40 | 2.76× | 4.09× |
| Tight corner | Constant | 30 | 4.30 | 1.00 | 11.40 | 2.65× | 4.30× |
| Tight corner | Tangent | 8 | 5.30 | 1.10 | 11.10 | 2.09× | 4.82× |
| Tight corner | Tangent | 30 | 4.90 | 0.90 | 11.60 | 2.37× | 5.44× |

The stored mode is 8.60–22.46 times faster than Pedro in these workloads. That is a
whole follower computation comparison under the stated I/O exclusions; the earlier
sampler-only comparison does not describe the cost of the remaining control work.

## Tail timing and allocation

These rows use tangent heading at 30 in/s. p95 is the median of the five rounds'
individual-call p95 values. Allocation is HotSpot thread allocation per update,
averaged within each round and then median across rounds; kB means 1,000 bytes.

| Curve | Method | Median (µs) | p95 (µs) | Allocated kB/update |
| --- | --- | ---: | ---: | ---: |
| Wide corner | Apex quickBuild | 4.80 | 11.50 | 14.15 |
| Wide corner | Apex buildToMemory | 0.90 | 2.00 | 5.33 |
| Wide corner | Pedro Foresight | 10.20 | 24.40 | 29.88 |
| S-curve | Apex quickBuild | 7.40 | 14.40 | 14.28 |
| S-curve | Apex buildToMemory | 1.30 | 2.40 | 5.40 |
| S-curve | Pedro Foresight | 29.20 | 49.70 | 60.88 |
| Tight corner | Apex quickBuild | 4.90 | 10.40 | 14.02 |
| Tight corner | Apex buildToMemory | 0.90 | 2.10 | 5.40 |
| Tight corner | Pedro Foresight | 11.60 | 23.50 | 30.72 |

Host scheduling and GC caused substantial variation. For example, the median of
round means for the tangent S-curve at 30 in/s was 19.62 µs for quickBuild, 2.72 µs
for buildToMemory, and 59.66 µs for Pedro. The median table should not be interpreted
as average wall-clock cost or a worst-case bound. `summary.csv` preserves means and
minimum/maximum round medians, and `results.csv` preserves each measured round.

## Construction cost and stored memory

Construction is separate from update timing. These are tangent-heading paths, using
five warmup builds and the median of nine measured builds per method. Apex includes
builder creation; Pedro includes native Bézier and compound path construction from
already prepared control points. The mathematical spline-to-Bézier conversion is
setup work and is excluded from Pedro's reported build time.

| Curve | quickBuild (ms) | buildToMemory (ms) | Pedro path build (ms) | Packed Apex field bytes |
| --- | ---: | ---: | ---: | ---: |
| Wide corner | 0.1171 | 17.7555 | 0.2269 | 1,058,400 |
| S-curve | 0.0684 | 12.5261 | 0.1599 | 960,480 |
| Tight corner | 0.0546 | 6.6027 | 0.1541 | 230,496 |

The memory figure covers the packed field, not total retained heap. The stored mode
uses the default one-inch grid spacing and 12-inch padding. quickBuild uses an
explicit 20-inch planning horizon. buildToMemory buys lower update CPU cost with
greater startup cost and retained storage. The two modes also compute different
references: quickBuild replans using measured motion; buildToMemory samples a
permanent nominal field. This benchmark does not establish equivalent tracking.

## Matched geometry and replay method

Apex uses its native `HolonomicPathBuilder` and cubic B-spline. Each spline span is
converted exactly to a native Pedro cubic Bézier, then combined with `CompoundCurve`.
The conversion verifies 101 positions per span against the original spline, within
`1e-8` inches. This avoids comparing visibly different curves built from the same
waypoints. Each library retains its own arc-length lookup and closest-point code.

Waypoints, in inches:

- Wide corner: `(0,0) → (80,0) → (80,80)`, two cubic spans.
- S-curve: `(0,0) → (40,40) → (80,-40) → (120,0)`, three cubic spans.
- Tight corner: `(0,0) → (24,0) → (24,24)`, two cubic spans.

The S-curve has more pieces and more complex geometry. Pedro's compound-curve
projection searches every piece; Apex projects against its native whole-path LUT
hierarchy. That code difference is a plausible contributor to the larger S-curve
gap, but this run is not a profiler-based attribution of every microsecond. A single
native Pedro Bézier or a different representation could produce different results.

Each replay traverses 256 precomputed states from 10% to 90% of Apex arc length.
Positions, world-frame tangent velocity, heading, and angular velocity are supplied
identically to both followers. Constant heading is zero; tangent heading follows
the curve, with angular velocity equal to signed curvature times input speed.
Elapsed time is distance increment divided by input speed, with Apex's injected
monotonic clock advancing accordingly and Pedro receiving that delta time explicitly.
This is prescribed-state replay, not a closed-loop robot simulation.

Both sides use 60 in/s translation limits and 100 in/s² acceleration limits. Apex
also uses 8 rad/s and 20 rad/s² angular limits, translational kV/kA/kS of
0.01/0.005/0.03 and angular kV/kA/kS of 0.1/0.02/0.02. Both use heading PID
1/0/0.01 and translation PID 0.05/0/0.001, with each library's own controller
implementation. Pedro uses explicit braking-model coefficients and its Foresight
controller. These configurations exercise the actual algorithms, rather than
claiming equivalent physical tuning or acceleration commands.

Motor interfaces are the same Java proxy stubs behind each library's production
mecanum drivetrain; power caches and allocation remain active. Apex uses its
untuned/tuning constructor to permit this synthetic configuration. Localization
updates return precomputed states, and telemetry/loggers are disabled.

For each workload, 80 replay sweeps warm the implementations before five measured
rounds of 100 sweeps. Each sweep resets the movement outside timing and primes one
update outside timing. Normal traversal cache misses remain in measured updates.
Method order rotates between sweeps and rounds. Only the `update()` / `update(dt)`
call is timed; state assignment and movement setup are outside the timing interval.
Every sweep asserts that the follower remains actively following and produces a
finite command. All checks passed. The final run measured 4,608,000 updates in total.

## Versions, artifacts, and reproduction

Pedro is the **locally cached `core` and `revhub` 3.0.0-SNAPSHOT**, using Foresight.
This report makes no claim about Pedro 2.x or another 3.0 snapshot. SHA-256:

- Pedro core: `EE5D37F8A54DD9DED56BBA66017C2891D0C584EDF28F8B6A3D3835E5F37A893F`.
- Pedro revhub classes: `BC727D7F7A507D9518E5F7750CD6EF3ACA5B84192AD82D84DBAE8881AE0AC589`.

Apex was compiled from **all current working-tree production Java sources**,
including existing uncommitted changes. HEAD was
`b594eff9056bd703b056c25f8de8283f36d4be35`; HEAD alone does not identify these sources.
`metadata.json` records every compiled production source hash and the runtime.
No production code or existing tests were edited for this comparison.

Local source, raw CSV, summary, argument files, and logs are in
[`work/curve-loop-comparison/`](../work/curve-loop-comparison/). That directory is
ignored by Git. The initial fixed-20ms run is retained as a diagnostic and excluded
from every reported number; the final run uses speed-consistent timestamps.

Run from the repository root on this machine, with the cached dependencies present:

```powershell
javac '@work/curve-loop-comparison/production-compile.args'
javac '@work/curve-loop-comparison/harness-compile.args'
java '@work/curve-loop-comparison/run.args'
python work/curve-loop-comparison/summarize.py
```

The argument files are machine-specific. Compilation bypasses the Android Gradle
plugin and uses cached dependency jars. The JVM uses `-Xms512m -Xmx1024m`. Five
rounds share one JVM; this is a local microbenchmark, not an independent multi-host
or Android benchmark. Real Control Hub profiling is required to compare physical
OpMode loops with localization, telemetry, hardware I/O, and Android GC included.
