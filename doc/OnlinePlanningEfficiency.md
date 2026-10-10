# Full-resolution online planning efficiency

Online windows now retain every geometry LUT point on the first solve, together with
exact start/midpoint/end and authored constraint transitions. The coarse-first solve
and recursive full-resolution retry have been removed. This eliminates failures caused
by skipping the existing geometry nodes, without hiding physical infeasibility.
The geometry LUT remains a discrete approximation; this change does not prove continuous
feasibility for arbitrary curves or nonconvex custom power models.

The shared solver narrows the backward search using the kinematic identity
`qNext = q + 2*a*ds`, where `q` is squared speed. Because `a >= -maxAccel`, a feasible
predecessor must satisfy `q <= qNextCeiling + 2*maxAccel*ds`. Necessary angular
acceleration bounds also restrict the search using `alpha = headingSecond*q + headingPrime*a`
and the interval's possible successor speeds. These bounds preserve the signs of
heading derivatives and allow translation acceleration to cancel heading acceleration.
The bounds use each interval's physical/authored acceleration ceilings and apply equally to straight,
curved, heading-changing and constrained paths. It can eliminate a numerical search
when the bounded state is already feasible. Power and angular constraints still apply,
and every future forward transition is validated before publishing the first command.
Custom transition rules can leave the new predecessor-bound hook unbounded.

All drivetrain generators also bypass acceleration-branch splitting when the greatest
kinematically allowed acceleration already passes the signed power evaluator.
Existing interval power checks remain active. The online planner reuses the first
window's immutable geometry point when constructing its published trajectory sample.
Swerve's eligible affine power branches use quadratic boundary roots for the vector
magnitude plus absolute angular demand. Candidates are verified against unsquared
physical demand; ambiguous floating-point or model-branch cases retain the numerical search.
No solve-frequency, horizon-length, or disturbance-response shortcuts were introduced.

## Desktop measurements

The same benchmark was compiled against saved pre-change production classes and the
current production classes. Each case has 10,000 warmup calls and three rounds of
10,000 measured calls, timed in blocks of 50. Values below are the median of the three
round medians, in microseconds per call. All use a 20-inch horizon and 8 in/s measured
tangent velocity, over the same ten-inch route section beginning at 40% progress.
Curved rows use a separate longer run with 100,000 warmup and 150,000 measured calls
per case to check the more variable curved-path timings.

| Path | Mecanum before → after | Tank before → after | Swerve before → after |
| --- | ---: | ---: | ---: |
| 100-inch line | 7.83 → 2.27 | 9.95 → 2.41 | 10.11 → 2.24 |
| 1,000-inch line | 7.66 → 2.96 | 9.73 → 2.40 | 14.15 → 2.56 |
| Curve (longer run) | 31.30 → 13.42 | 28.52 → 11.26 | 45.42 → 27.06 |
| 256 constraints | 8.53 → 3.17 | 12.04 → 2.72 | 14.42 → 3.72 |
| Heading-change fixture | 12.07 → 4.11 | 9.76 → 2.35 | 16.76 → 5.14 |

Tank paths use tangent heading, so the tank heading-change fixture retains tangent
heading rather than independently rotating along the line. All 15 measured workloads
improved in this comparison. Curves improved by 40–61%; other measured cases by 61–82%.
These are desktop HotSpot measurements, with visible host scheduling and GC variation,
not guarantees for every possible route, calibration or Android controller. Full-resolution
windows contain more nodes, so some workloads may gain less or regress.

Raw CSVs, source, machine-specific argument files and logs are in
`work/online-planner-efficiency/`. Gradle's uncached Android plugin prevented an offline
Gradle run; current production and selected test sources were compiled with javac using
the cached SDK and dependency jars.

## Correctness checks

- 94 trajectory, geometry and path regression tests passed, including a randomized
  comparison of bounded versus unbounded backward solves, and independent signed-demand
  checks of mecanum, tank and swerve analytic power bounds. Swerve quadratic bounds were
  also checked against dense feasibility samples over 1,000 random coefficient sets.
- Full-resolution single-solve checks cover all drivetrain enum values and both
  dual-actuated profiles, with straight, curved and constrained routes.
- A separate probe compared 2,400 states against the saved full-resolution solver:
  mecanum/tank/swerve, straight/curved/heading-changing/constrained/reversed routes,
  legacy/asymmetric calibrated models, and speeds 0/8/30/80 in/s. All 684 overspeed
  classifications matched. Maximum absolute acceleration difference was `1.702e-8`
  in/s², angular acceleration difference `3.31e-10` rad/s², and motor-power difference
  `9.2e-11`; velocities matched at printed precision. This reference comparison uses
  the same geometry resolution on both sides;
  commands may differ from the old coarse path because the sampling grid changed.
- Genuine infeasibility remains a reported error; overspeed still uses braking recovery.

The broader 15-test follower/simulation selection was not clean: five failures also
occurred with the saved baseline (two lifecycle fixtures and three tank fixtures).
An online dual-mode endpoint stall appeared in the combined optimized run, but the
isolated online dual-mode test passed against both versions. The original planner
with only full-resolution sampling enabled also showed a different stored-mode stall.
The final unmodified online dual-mode regression also passed in an isolated run.
These real-time simulation fixtures have state/timing sensitivity; the broader run
does not establish complete simulation success. Logs preserve the failures, and no
fixture tolerances or follower completion policies were relaxed.
