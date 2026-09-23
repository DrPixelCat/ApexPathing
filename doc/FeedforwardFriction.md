# Feedforward ramp tuning

Drive and turn each use a rising-only power ramp at 0.1 power per second, up to
0.9. The ramp stops for operator input, invalid localization, the 96-inch / 4-pi
radian travel limit, or the end of the ramp. There is no falling-power segment
or early reversal based on speed.

Before motion is detected, stationary samples determine the friction threshold.
kS is the greatest delivered power observed while still stationary. At the first
positive moving sample, kS is frozen for the remainder of that run. Motion is
detected using filtered velocity above 0.1 in/s for drive or 0.01 rad/s for turn.
These thresholds distinguish motion from localization noise; measurement delay
and ramp increments limit the precision of the breakaway estimate.

After motion begins, the tuner fits only kV:

`kV = sum(v * (power - kS)) / sum(v * v)`

kS stays fixed when moving samples are excluded or restored during review. The
fit must still meet the sample-count, speed-span, positive-kV and R-squared checks.
Raw CSV rows identify their role as friction or kV samples. kA and the separate
position-controller breakaway settings remain unchanged.

This procedure measures breakaway kS rather than a moving-friction intercept.
It does not separate acceleration or nonlinear drag from kV during the rising
ramp. The earlier paired-ramp results do not validate this replacement procedure.

The mecanum simulator retains its original motor equation and discontinuities.
Its configured moving Coulomb fraction is 0.06 and its nominal breakaway fraction
is 0.08. In FTCodeSim, naturalDeceleration controls moving Coulomb friction,
whereas staticFriction is an acceleration threshold near rest. These settings
are unchanged by the restoration of rising-only tuning.
