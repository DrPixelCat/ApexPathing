# Online planning performance

These optimizations build on the earlier half-density online window. They keep its
exact window boundaries, midpoint, constraint transitions, and full-resolution
retry when the coarse solve has no reachable or controllable state.

## Implementation

- `CubicSpline1D` caches its current interval. Repeated queries take constant time,
  neighboring forward queries advance once, and larger or backward jumps use binary
  search. Exact knots retain the previous left-interval convention. The cursor is
  only a hint; concurrent queries of a shared spline still select the correct interval.
- `PathSegment` builds a bounding-box hierarchy over the geometry LUT. Each online
  traversal retains its previous closest index as a seed. Bounds certify the same
  global closest LUT node as the original scan, including after localization jumps
  and near adjacent route branches. Newton refinement is unchanged.
- `ConstraintSchedule` sorts once and compiles prefix limits with stable ordering
  for equal-position constraints. Window queries enumerate only visible transitions.
  Adding a constraint or changing a built-in constraint's `setS` invalidates the
  schedule. Custom constraints without change hooks remain live.
- Retained geometry nodes cache their solver limits, heading derivatives, and power
  evaluators. Visible authored transitions also retain their geometry and solver samples
  after the first evaluation. Heading revisions, constraint revisions, geometry replacement, calibrated
  model replacement, and relevant physical limits refresh the cache. Custom heading
  and generator implementations default to live evaluation. Power coefficients and
  requested power limits remain live in the signed evaluators.
- Window construction merges ordered geometry, boundary, and constraint streams into
  reusable primitive arrays. Solver controllability storage and acceleration split
  storage are also reused. Online queries avoid full speed arrays and per-node square
  roots, while retaining every future-transition validation needed for the coarse retry.
- Eligible mecanum and tank power branches resolve to affine functions of acceleration
  at a fixed speed. Intersecting their signed power inequalities replaces inner ternary
  and binary searches. Calibrated branches qualify only when the entire interval stays
  in one measured-model branch. Candidates are checked with the actual signed evaluator.
  Branch changes and swerve power retain the generic convex search with a 32-step cap
  and tight tolerance exits. Calibrated holonomic evaluation uses scalar body-axis powers
  instead of temporary vectors, including during overspeed recovery.

## Complexity

Let N be the full geometry LUT size, W the online window size, C the authored constraint
count, H the heading-node count, K the visible constraint transitions, and R the search
cap (32).

| Work | Before this round | After this round |
| --- | --- | --- |
| Closest LUT point | O(N) | Usually O(log N); O(N) worst case, globally verified |
| Heading interval lookup | O(H) | O(1) for cursor hits, O(log H) for jumps |
| Constraint preparation/application | O(C log C + WC) each window | O(C log C) on mutation; O(log C + K) transition selection and O(log C) per uncached node |
| Window ordering | O(W log W) with boxed tree entries | O(log N + log C + W + K), reusable arrays |
| Sample preparation | Repeated for every node | Cached interior nodes; moving boundaries and new transitions computed fresh |
| Coupled solve | O(WR²) worst case | O(WR) on affine branches; O(WR²) generic worst case |

The generic worst-case bound remains, including difficult projection geometry and
unsupported power branches. A traversal retains O(N + C + maximum window size) cache
and workspace memory. Cache initialization and invalidation have a one-time cost;
the benchmark measures warmed queries. No spatial velocity field is introduced.

## Verification and reproduction

The regression selection includes geometry, profiles, constraints/reversal, core,
controllers, trajectory, tuning math, profile convergence, and the Apex, Auto Test,
dual-actuated, and traversal lifecycle simulations. Differential checks compare the
projection hierarchy against the former full scan over randomized queries and nearby
route branches, analytic power candidates against independently evaluated signed power,
and online workspace results against full trajectory solves. Tests also cover mutable
built-in constraints, custom mutable constraints, and heading/physical-limit invalidation.

`trajectory.OnlinePlanningBenchmarkTest` uses an explicit 20-inch horizon, 3,000 warmup
queries and 10,000 measured queries per case. It reports median and p95 latency and
HotSpot thread allocation bytes. Ordinary cases use 8 in/s; recovery uses 80 in/s.
The before snapshot is the planner after the earlier resolution reduction, with this
round's production sources replaced by saved originals in an isolated build directory.
Both snapshots run the same benchmark source and desktop runtime.

## Measured results

The final run measured the following warmed sampler costs. The before column already
includes the earlier reduction in online point density; these savings are from the
additional optimizations in this round.

| Case | Median before (µs) | Median after (µs) | Bytes/query before | Bytes/query after |
| --- | ---: | ---: | ---: | ---: |
| 100-inch line | 8.3 | 8.3 | 22,252 | 9,158 |
| 1,000-inch line | 9.5 | 8.2 | 22,403 | 8,992 |
| 1,000-inch line with dense heading nodes | 23.5 | 8.4 | 22,055 | 9,371 |
| 256 authored constraints | 17.4 | 9.2 | 36,407 | 9,112 |
| Curved path | 69.4 | 28.4 | 46,669 | 9,587 |
| Calibrated 1,000-inch line | 11.1 | 8.8 | 46,431 | 9,032 |
| Overspeed recovery | 10.9 | 9.6 | 23,716 | 11,008 |

Allocation fell by approximately 54–81% per query. Median latency improved most for
dense headings, many constraints, and curved paths; the short straight-line case
was unchanged in this run. CSV files also retain p95 latency, LUT size, and final
window size. The ordinary line window uses 10 nodes, the many-constraint case uses
16, and the curved case ends with a 25-node full-density retry.

The final focused regression run passed 218 tests. The broader simulation selection
passed 242 of 244 tests; both failures were Auto Test average loop-time fixture limits.
Both online and stored versions of that same fixture also exceeded their loop-time
limits with the saved pre-change planner (37.31 ms and 46.39 ms respectively).
The fixture thresholds were left unchanged. Geometry, power feasibility, constraint
mutation, heading cache invalidation, coupled-solver equivalence, and the other
selected simulation checks passed.

These are desktop sampler measurements, not Android robot-controller timings. JIT,
garbage collection, and host scheduling affect latency, especially p95. Timing is not
used as a benchmark assertion. The Auto Test simulation has its own timing assertions,
as described above. Robot-controller profiling remains the appropriate way to measure
the change in complete follower-loop time.
