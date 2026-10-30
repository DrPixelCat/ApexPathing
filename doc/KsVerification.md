# kS boundary correction and verification

The automatic feedforward tuner now retains the applied command from the last measurement classified as stationary. When the next measurement crosses the movement threshold, it saves the retained command instead of the command that produced the crossing. The retained command resets at the beginning of each acquisition run. Positive and negative acquisitions use the same rule.

The existing movement criteria are unchanged: 0.5 in/s for drive and 0.05 rad/s for heading. Thus "stationary" means below the detector threshold, not mathematical zero velocity. Localization delay and noise still limit physical boundary accuracy.

Deterministic full-stock-chassis checks (0.001 power increments held for 0.5 seconds, direct chassis velocity observations) produced:

| Axis | Last classified-stationary power, both signs | First detected-moving power, both signs | Analytic zero-speed boundary |
| --- | ---: | ---: | ---: |
| Drive | 0.270 | 0.271 | 0.267760 |
| Heading | 0.053 | 0.054 | 0.050557 |

The detector's saved result is within approximately 0.0025 power of the ideal boundary in these checks. This verifies the revised selection rule; it does not measure the error introduced by a real localizer or replay the complete interactive tuning UI.

There are two separately saved quantities in the current implementation:

- `translationalCoeffs.kS` / `angularCoeffs.kS`: the measured breakaway compensation, with the smaller of the two directional measurements selected.
- `translationalFeedforwardKS` / `angularFeedforwardKS`: the intercept of a joint moving-data kS/kV/kA regression. The boundary correction does not change this regression.

A second experiment supplied the production regression with unfiltered complete-stock-chassis measurements from four forward/reverse quasistatic/dynamic runs. Drive fitted kS was **0.209727**, compared with the 0.267760 low-speed boundary; heading fitted kS was **0**, compared with 0.050557. This exposes the distinction between an affine moving-data intercept and a directly observed breakaway boundary on this nonlinear plant. Earlier complete UI runs used different sampling/filtering and returned different fitted intercepts; these experiments must not be treated as identical runs.

Verification: **14 tests ran; 13 passed and one failed**. The boundary checks, existing regression tests, and plant-equation checks passed. `KsPlantValidationTest.driveAndHeadingKsAgreeWithLowSpeedPlantAuthority` remains a failing diagnostic for the joint regression's disagreement with the low-speed boundary. Its tolerance was not relaxed to turn the result green.

Results: `TeamCode/build/test-results/simulatorPlantAudit/`; command log: `tmp/simulator-plant-audit/ks-verification.log`.

No conclusion that both saved forms of kS are verified is warranted yet. Deciding whether feedforward kS should also use the observed stationary boundary is a separate behavioral decision from correcting the breakaway detector.
