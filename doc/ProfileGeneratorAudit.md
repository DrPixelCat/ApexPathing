# Feedforward profile audit and fixes — October 2, 2026

The reproduced mecanum constraint violations and unnecessary power charges have
been fixed. The generator now solves signed acceleration and power together using
backward controllability and forward reachability, then separately validates power
and kinematics. The configured ceiling is described in [ProfilePowerLimit.md](ProfilePowerLimit.md).

The evidence supports nearly maximal profiles in the tested sampled cases. It does
not establish global optimality for arbitrary paths or a bound on the continuous
trajectory's optimality gap.

## Sources and corrections

| Issue | Source | Correction |
| --- | --- | --- |
| Directional acceleration violations | Reachability used the next sample's geometry; output acceleration belongs to the segment starting at the current sample. | Both passes evaluate the actual starting state and its local constraints. |
| Angular acceleration violations | Scalar acceleration bounds were evaluated at a different speed and did not enforce the full signed coupling. | Intersect the admissible acceleration interval with `-alphaMax <= f'' v² + f' a <= alphaMax`. |
| Feasible speeds excluded | Initial ceilings assumed `a = 0`, although acceleration can cancel heading demand or reduce motor demand. | Keep physical, authored, and measured-domain speed ceilings; evaluate acceleration-dependent feasibility during transitions. |
| Diagonal and strafe power overstated | Directional LUT multipliers were applied to feedforward coefficients in addition to the motor mixer's body-axis charge. | Use the LUT for physical speed/acceleration limits and shared signed feedforward for power. |
| Curved-path power mismatch | Planning charged scalar normal force; execution reserved normal force separately, preventing signed axis cancellation. | Combine tangent and normal vectors before charging `abs(x) + abs(y) + abs(turn)` and allocating profiled feedforward. |
| False convergence report | The old flag tested estimated power only. | Report power feasibility, kinematic feasibility, maximum constraint ratio, duration, and effective power ceiling; reject a failed final validation. |
| Authored constraints applied in insertion order | An earlier constraint added later could replace the latest active constraint. | Sort by path progression before applying active overrides, including reversed paths. |

`HolonomicFeedforward` supplies the reference translation and heading calculations
for planning and execution. Runtime normal force still responds to measured speed;
tracking errors, feedback, endpoint blending, and an explicit planning-only kV
override can make actual commands differ from the zero-error reference model.

Tank and swerve profiles use the coupled transition solver too. Their respective
traction and translation power models remain drivetrain-specific. Legacy tank quick
profiles continue to use the scalar dynamic-profile solver.

## Before and after

The local audit independently reconstructs directional acceleration, heading
derivatives, and four mecanum wheel commands. It uses the same seven path cases
before and after the fixes, with power ceiling 1.0 and no characterization.

| Path | Old duration (s) | New duration (s) | Old maximum a / limit | New maximum a / limit | Old maximum alpha / limit | New maximum alpha / limit |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Forward, 60 in | 1.959726 | 1.959726 | 1.000000 | 1.000000 | 0 | 0 |
| Fixed 45° heading, 60 in | 2.482123 | 2.476726 | 1.000000 | 1.000000 | 0 | 0 |
| Strafe, 60 in | 2.191550 | 2.191041 | 1.000000 | 1.000000 | 0 | 0 |
| Facing a point, 100 in | 3.116723 | 3.093241 | **1.071777** | 1.000000 | 0.972930 | 0.989985 |
| Constant-heading rounded corner | 3.827523 | 3.688916 | 1.000000 | 1.000000 | 0 | 0 |
| Tangent-heading rounded corner | 3.466912 | 3.467458 | 1.000000 | 1.000000 | 0.741631 | 0.741632 |
| Smooth 180° heading change, alpha limit 1 rad/s² | 3.919203 | 3.953030 | **1.010569** | 1.000000 | **1.345891** | 1.000000 |

All new cases have zero sampled acceleration violations and stay within the power
ceiling. The constant-heading corner is approximately 3.62% faster. The tight
heading case takes longer because its previous profile was infeasible. The tangent
corner is approximately 0.016% slower after correcting its power computation and
removing the old 0.001 power allowance.

The old facing-point profile had 14 directional acceleration violations. The old
tight-heading profile had 37 angular acceleration violations and three tangential
acceleration violations. Both are covered by committed regression tests.

A separate coordinate-increase search, using the original physical ceilings and
independent adjacent-transition checks, found at most **0.000000731 s** of duration
improvement in the new audit cases. This is a local diagnostic, not a global proof.
An analytic straight-path regression additionally checks the maximal sampled
rest-to-rest acceleration envelope. Power-limited straight tests verify saturation
at a configured 0.7 ceiling across six headings using independent wheel commands.

The local cancellation witness uses `f' = 1`, `f'' = 1`, `alphaMax = 1`:
`v = sqrt(2), a = -2` produces `alpha = 0` and reference power 0.029448232.
The old zero-acceleration ceiling excluded this state; the new physical ceiling
permits it. This is a local feasible-state witness, not a complete trajectory.

## Interpreting maximal power or an active constraint

The relevant power value is the configured ceiling, which can be below 1.0.
Acceleration, angular limits, authored limits, measured velocity domains, and
required future braking can all prevent using that ceiling.

A literal equality at every sample is too strict: endpoints, intervals spanning an
acceleration/braking switch, and neighboring reachable states can leave apparent
instantaneous slack. The stronger objective is to minimize feasible traversal time,
rather than to spend extra power without increasing useful progress.

The new solver starts from physical ceilings, propagates the states that can still
reach the endpoint, and takes the greatest feasible next speed within that set.
Its interval representation assumes controllable speeds form a connected interval
starting at zero and transition feasibility is monotone on that interval. It does
not represent disconnected feasible regions from arbitrary nonconvex models.

Final feasibility is checked at generated samples. The existing LUT interpolation
and geometry between samples have no continuous-time feasibility proof. Smaller
sampling distances can reduce discretization error, but these results do not imply
every interpolated command satisfies every physical constraint. Characterization
also remains an empirical motor-duty model, not a measurement of electrical watts.

## Validation and reproduction

Committed tests cover the reported failures, analytic acceleration saturation,
independent wheel power, out-of-order/reversed constraints, signed mandatory
deceleration, and 18 deterministic curved/changing-heading cases alternating legacy
and asymmetric measured motor models. They also cover the configurable power API,
boosted boundary states, and combined normal-force allocation.

The final selected regression run passes 94 of 95 tests, including the
current-calibration mecanum autonomous simulation, which completes all five
movements in approximately 15 seconds within its existing tracking and timing
limits. Its one failure is the standalone open-loop heading feedforward acceptance
test: six cases exceed the braking RMSE threshold. That test directly drives the
simulated plant without invoking the changed generator or allocator, and its failure
also reproduces in the isolated original-code build.

Two other separately investigated simulation tests still fail: the old
corner-tracking fixture and the dual-actuated reverse-startup assertion. Both
failures reproduce with the original HEAD generator/runtime in an isolated build.
These three preexisting simulation failures remain outside the generator fixes;
their tests and tuning fixtures were not weakened to make this change pass.

Local audit sources are under `work/feedforward-audit/src`, outputs under
`work/feedforward-audit/results`, and baseline outputs under
`work/feedforward-audit/results-before`. That workspace folder is ignored by Git.
Run the diagnostic with:

```powershell
.\gradlew.bat :TeamCode:testDebugUnitTest --offline `
  --tests FeedforwardOptimalityAuditTest `
  -I work/feedforward-audit/audit.init.gradle
```

Audit coefficients: legacy kV = 0.0135, kA = 0.0026, angular kV = 0.011,
angular kA = 0.0021, kCentripetal = 0.001, zero static feedforward; forward v/a
limits 72.5/62.5, strafe v/a limits 58/50, angular velocity limit 291°/s,
angular acceleration limit 2000°/s² except for the identified tight-heading case.
