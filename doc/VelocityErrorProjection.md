# Velocity feedback during acceleration and braking

**Archived experiment.** Normal following has been restored to
[current-time proportional velocity feedback](VelocityFeedback.md). The projection
helper, configuration field and experiment test harness have been removed.
The equations, comparison results and existing graphs below record the experiment;
they do not describe the active controller.

The controller strengthens correction when the robot lags the planned change in
velocity, and softens correction when it leads that change. Both corrections still
pull measured velocity toward the current reference. Feedforward continues to
provide the planned acceleration; this rule only adjusts tangential velocity feedback.

## Geometry and units

Let `e = v_target - v_measured` at the same reference/measurement timestamp, and
let `a` be the slope of that reference interval. A time/velocity distance needs
explicit axis scales: seconds and in/s cannot be added directly.

Use `T = velocityErrorProjectionTimeSeconds` (default 0.050 s) for a horizontal
coordinate unit and `V0 = 1 in/s` for a vertical coordinate unit:

```text
x = (t - t_reference) / T
y = (v - v_target) / V0
m = a*T/V0
reference line: y = m*x
measured point: (0, -e/V0)
```

Projecting the measured point onto the line gives:

```text
x_closest = -m*(e/V0) / (1 + m*m)
y_closest = m*x_closest
r = 1 / sqrt(1 + m*m)
e_line = r*e
```

`e_line` is the signed perpendicular distance, converted back to velocity units.
It is different from the vertical velocity difference to the projected point,
which would be `e/(1+m*m)`. The sign is essential: overspeed must still request
negative corrective authority, and underspeed positive authority.

The 50 ms setting controls the geometric scaling. It does not sample a future
velocity, delay the feedback, or move the feedforward target. Larger `T` makes
the state-dependent adjustment stronger. `T = 0` disables it and restores ordinary
proportional velocity control. The configuration accepts finite values in [0, 1] s.

## The five motion states

The final rule uses raw error as its base:

```text
e_adjusted = e + abs(e - e_line) * sign(a)
u_velocity = k_velocity * e_adjusted
```

Equivalently, apply a positive gain multiplier `g`:

| State | Error sign | Multiplier g | Corrective action |
| --- | --- | --- | --- |
| Underspeed, accelerating | e > 0, a > 0 | 2 - r | Boost positive authority |
| Overspeed, accelerating | e < 0, a > 0 | r | Soften negative authority |
| Flat reference | a = 0 | 1 | Ordinary P feedback |
| Overspeed, braking | e < 0, a < 0 | 2 - r | Boost negative authority |
| Underspeed, braking | e > 0, a < 0 | r | Soften positive authority, reducing braking |

Thus `a` and `e` with the same sign indicate lagging motion; opposite signs
indicate leading motion. The implementation compares signs without multiplying
potentially large inputs. Zero error requests zero authority.

At `|a| = 40 in/s^2`, `T = .05 s`, `r = 1/sqrt(5) = .4472`.
A 4 in/s error becomes 6.211 in/s in a boosted state and 1.789 in/s in a softened
state, retaining its original sign. As `a` approaches zero, both multipliers
approach one smoothly; no arbitrary acceleration threshold is required.

Using `e_line` as the base of the absolute-difference equation instead could
reverse correction for leading motion when `|a*T/V0| > sqrt(3)`. That earlier
draft does not implement the five-state intent and is not the final controller.

## Convergence and practical limits

For an exact scalar acceleration inverse, synchronous measurements and held
commands, with no saturation or reference stop clamp:

```text
e[k+1] = (1 - h*b*k_velocity*g[k])*e[k] - h*d[k]
```

Here `b` is acceleration per corrective authority, `h` is the control interval,
and `d` is unmodelled acceleration. Since `0 < r <= 1`, softened gain is positive
and boosted gain is in [1, 2). In this model the error contracts without changing
sign when `0 < h*b*k_velocity*g <= 1`. The online follower reapplies its existing
timing bound after scaling, so a boost cannot bypass that limit. The bound includes
known sensor age; it is a safeguard rather than a full stability proof for delayed
or filtered sensors, changing tangents, wheel saturation or nonlinear motor dynamics.

With the simulator's `b*k_velocity = 8.17065 /s`, the 40 in/s^2 example gives
approximate continuous error time constants of 79 ms when boosted, 274 ms when
softened, and 122 ms for ordinary P control. Faster catch-up trades against slower
cleanup of errors classified as leading motion. This is gain scheduling, not
prediction or explicit noise filtering.

The adjustment relative to ordinary P always points along planned acceleration.
For symmetric, zero-mean measured error fluctuations at fixed `a`, it produces
`mean(u_velocity) = k_velocity*(1-r)*sign(a)*mean(abs(e))`, even when `mean(e)=0`.
That is a mathematical consequence of treating leading and lagging errors
differently. The Apex comparison does not isolate how much measurement noise
contributes to its observed tracking differences.

Online reference acceleration is retrieved from the same timestamped interval
as the velocity target, including delayed measurements. Stored execution uses
the stored sample's velocity and acceleration. Lateral and heading feedback keep
their existing gains. Feedforward-only execution applies no velocity correction.

## Apex twisting-path comparison

The same production follower, native simulated sensor and unchanged plant execute
the first Apex path with heading nodes 83 degrees at 30%, -71 degrees at 65%, and
the 146-degree end heading. Control and physics share a deterministic clock.
Every case uses zero velocity-target preview. Baseline scaling is disabled; the
candidate uses the selected 50 ms scale.

| Case | Current-reference velocity RMS, P → scaled (in/s) | Independent nominal-envelope RMS, P → scaled (in/s) | Endpoint error, P → scaled (in) |
| --- | --- | --- | --- |
| Online, clean | .230 → .280 | .590 → .611 | .043 → .127 |
| Online, extra 4 in/s^2 drag | .401 → .449 | .619 → .612 | .073 → .025 |
| Online, 5 in/s speed loss and 10/30/20 ms timing | .485 → .543 | .942 → .935 | .083 → .057 |
| Stored, clean | .373 → .421 | .396 → .445 | .294 → .288 |

All eight runs complete the path and its callback and satisfy the existing
velocity, heading, lateral and endpoint assertions. Traversal times remain
9.94–10.03 s. Velocity RMS excludes endpoint blending above 5%; graphs include
the whole trace, and normal endpoint completion can stop the robot before full rest.

The five-state behavior is verified, but these results do not establish a tracking
improvement from the selected 50 ms scale. Current-reference RMS worsens in every
case, and the nominal envelope comparison is mixed. Some transitions show larger
error peaks. This should be treated as a controller behavior change requiring
further tuning, not a validated accuracy upgrade. Physical robot behavior has
not been measured by these tests.

`PointSlopeVelocitySimulationTest` writes velocity/error overlays, individual
reference/velocity and heading graphs, power graphs, detailed traces and
`PointSlope_Comparison.csv`. Power is peak normalized motor command, not watts.
The graphs compare actual plant velocities at each run's path progress. The nominal
envelope is generated independently; the error graph compares each run to its own
current-time reference.

Artifacts are under `TeamCode/build/path-power-graphs/velocity-point-slope-five-state`.
The OpMode CSV also logs raw error, adjusted error, reference acceleration and
projection time for diagnosis on the robot. Mathematical regressions cover all five
states, explicit orthogonal projection, sign preservation, contraction in the scalar
model, disabled scaling, delayed-slope alignment and the post-boost timing bound.

Final verification ran 168 tests: 167 passed, none failed, and one wall-timed
feedforward-only diagnostic was skipped by its existing 50 ms scheduling guard
after a 96.9 ms host interval. All four deterministic feedforward-only twist
variants passed with velocity RMS .214 in/s and zero feedback.

The first broad run exposed a test-fixture omission: power traces deliberately
exclude the completing update, but the full-stop assertion also excluded that
update's retained terminal target. The fixture now checks scheduled distance and
zero terminal target velocity independently of its active-command power trace.
Its distance, motion and timing thresholds are unchanged. The final wall-timed
run reports the complete 80-inch target and zero terminal target velocity, but
its motion accuracy remains ineligible due to the host interval above.
`test-reports` preserves the first run; `test-reports-final` preserves final results.

The archived CSV traces, graphs and test reports remain in the artifact directory
above. The former experiment harness is no longer part of the active regression suite.
