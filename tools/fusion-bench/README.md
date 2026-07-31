# fusion-bench

A host-side benchmark for the tracker's orientation fusion. It runs the real
VQF filter — the same `lib/vqf` the firmware ships — over a dataset and reduces
the result to a handful of numbers, so that a change to fusion or calibration
can be argued about with evidence instead of impressions.

Before this existed, `ci/build.py` proved the firmware *compiled* and nothing
asserted anything about whether it *worked*. The regression detector was a user
noticing they were resetting more often.

## Quick start

Needs a C++17 compiler and nothing else. No PlatformIO, no embedded toolchain,
no Python.

```sh
cd tools/fusion-bench
make test     # self-tests for the harness itself
make suite    # run the standard benchmark
make check    # run it against the committed baseline (this is the CI gate)
```

Subcommands, once built:

```sh
./build/fusion-bench suite [--baseline FILE] [--write-baseline FILE] [--json FILE]
./build/fusion-bench gen TRAJECTORY OUT.csv [options]
./build/fusion-bench run DATASET.csv [options]
./build/fusion-bench sweep DATASET.csv PARAM FROM TO STEPS
./build/fusion-bench gyro-scale DATASET.csv
./build/fusion-bench noise DATASET.csv [options]
```

`gen` options — the error model to inject:

| flag | default | meaning |
| --- | --- | --- |
| `--duration S` | 60 | capture length, seconds |
| `--rate HZ` | 250 | sample rate |
| `--seed N` | 1 | RNG seed; identical seeds give byte-identical datasets |
| `--gyro-bias DPS` | 0.15 | constant gyroscope bias |
| `--gyro-noise DPS` | 0.03 | per-sample gyroscope white noise, 1σ |
| `--gyro-scale FRAC` | 0 | fractional scale error, e.g. `0.01` for 1% |
| `--accel-bias MS2` | 0.02 | constant accelerometer bias |
| `--accel-noise MS2` | 0.02 | **per-axis** accelerometer white noise, 1σ |
| `--with-mag` | off | emit a magnetometer column |

Note `--accel-noise` is per axis, while the noise figures quoted for real parts
are usually vector magnitude — √3 larger for three comparable axes. Mixing the
two moves any threshold derived from them by 73%.

`run` options: `--json FILE`, `--stock` (use VQF's own defaults, which is what
every real device runs), `--mag`, and the four tuning knobs `--tau-acc`,
`--rest-th-gyr`, `--rest-th-acc`, `--rest-min-t`. The knobs also compose with
`sweep`. `sweep` PARAM is one of `tau-acc`, `rest-th-gyr`, `rest-th-acc`,
`rest-min-t`.

`make suite` takes well under a second, which is the point: it can run on every
PR without anyone resenting it.

## What you get today

The suite generates its own datasets, so it works with no hardware and no
downloads. Each case isolates one thing:

| case | what it isolates |
| --- | --- |
| `static-tuned` | drift with a realistic residual gyro bias, using `DefaultVQFParams`; doubles as the rest-cliff guard (see below) |
| `static-stock` | the same input with VQF's own default parameters — **this is what every real device runs** |
| `tilted` | inclination tracking away from level |
| `yaw-sweep` | heading tracking through sustained yaw motion |
| `tumble` | general multi-axis motion |
| `return-to-origin` | net-zero rotation; whatever heading error remains is accumulated error |
| `scale-error-1pct` | a 1% gyro scale error with no bias — the error term rest-gated bias calibration cannot see |
| `walk` | limb-like motion with linear acceleration disturbing the gravity reference |

Synthetic data is not a substitute for real captures, but it is not a weaker
substitute either — it is a *different and complementary* tool. A real capture
contains every error term at once and no ground truth. A synthetic dataset has
exact ground truth and contains exactly the error terms you asked for, which is
the only way to attribute a change to a cause. Use synthetic data to reason,
real data to confirm.

## The metrics

**`heading_drift_deg_per_min`** — the headline number. Slope of a line fitted to
the estimated heading while the tracker is stationary. This is what determines
how often a user has to reset.

**`total_error_deg_rms` / `heading_error_deg_rms` / `inclination_error_deg_rms`**
— the standard decomposition, available only when the dataset carries a
reference orientation. Computed after removing the single best *constant*
heading offset: a 6-DoF estimator has no absolute heading reference, so its
absolute yaw is arbitrary and penalising it would measure nothing. A constant
*tilt* offset is not forgiven, because gravity is observable and getting it
wrong is a real error.

**`final_heading_error_deg`** — heading error on the last sample. For a
trajectory whose net rotation is zero this is the accumulated-error number the
physical return-to-origin bench test produces.

**`tilt_error_deg_rms`** — rotate the measured specific force into the world
frame; it should point straight up. Needs no reference at all. Exact while
stationary, degrades under linear acceleration (which is why `walk` scores ~5.7°
and that is *correct*, not a bug).

**`jitter_deg_rms`** — angular step between consecutive estimates while
stationary. Isolates high-frequency noise from slow drift.

**`first_rest_sec`** — when VQF first declared the tracker at rest. `-1` means
never, which is a red flag: rest-gated gyro bias estimation never ran.

### A note on rest detection

Drift and jitter are only meaningful while the tracker is not moving, so the
harness finds the longest stationary segment itself. It requires *both* low
gyro variation and low gyro magnitude, because neither works alone:

- A stationary tracker with an uncorrected bias reads a constant non-zero rate.
  A magnitude-only test would classify exactly the trackers we most want to
  measure as "moving".
- A constant-rate rotation has near-zero variation. A variation-only test would
  classify a steadily turning tracker as stationary.

Both mistakes were made and caught by the self-tests while writing this, which
is a reasonable advertisement for `make test`.

## Worked example: what the harness is for

`DefaultVQFParams` in `src/sensors/SensorFusion.h` sets `restThAcc = 0.06`,
against VQF's own default of `1.418598`. Running the suite:

```
static-tuned   drift  -3.5777 deg/min   total 2.073 deg
static-stock   drift  -0.0166 deg/min   total 0.158 deg
```

The tuned parameters are ~200× worse on this dataset. `first_rest_sec` explains
why: it is `-1` for the tuned run. Rest is *never detected*, so rest-gated gyro
bias estimation never runs, so the bias survives as drift.

Sweeping isolates the responsible parameter in one command:

```sh
./build/fusion-bench sweep build/static.csv rest-th-acc 0.04 0.20 9
```

```
0.04000   -3.59099
0.06000   -3.59099     <- the value the firmware ships
0.08000   -0.02089
0.10000   -0.01763
```

A cliff, with the shipped value on the wrong side of it. Varying the
accelerometer noise shows where the cliff moves to:

| accel noise (m/s², 1σ) | rest detected? | drift °/min |
| --- | --- | --- |
| 0.002 | yes, at 2.0 s | −0.032 |
| 0.005 | yes, at 2.0 s | −0.032 |
| 0.010 | yes, at 2.0 s | −0.032 |
| 0.020 | **never** | −3.583 |
| 0.050 | **never** | −3.584 |

So `restThAcc = 0.06` works only if per-sample accelerometer noise stays below
roughly 0.015 m/s². An LSM6DSV at moderate bandwidth sits around 0.006–0.012
m/s² — on the safe side, but not by much, and mechanical vibration eats the
margin. If it is ever exceeded, drift degrades by two orders of magnitude
*silently*.

**This was a hypothesis, and Bench Test A has now falsified the alarming part
of it.** Measured on a real LSM6DSV over a 28-minute static capture:

| | tuned (`restThAcc = 0.06`) | stock (`1.418598`) |
| --- | --- | --- |
| `first_rest_sec` | 2.25 s | 2.66 s |
| `heading_drift_deg_per_min` | 0.0187 | 0.0186 |

Rest is detected in both cases and the drift difference is noise. The measured
per-sample accelerometer noise was **0.00686 m/s²**, against a cliff located
between 0.010 and 0.020 — so the real part sits about 1.5× clear of it.

The *mechanism* is real: put `restThAcc` below the noise floor and rest is never
detected, bias estimation never runs, and drift degrades by two orders of
magnitude. What was wrong was the risk estimate, because the synthetic noise
model (0.020 m/s²) was about 3× pessimistic and therefore landed on the wrong
side of a cliff the hardware is clear of.

That is the harness working as intended, and it is worth being blunt about the
lesson: a synthetic model can put you confidently on the wrong side of a real
threshold. Synthetic data is the right tool for *finding* a cliff and a poor one
for deciding whether you are near it. Use it to reason, then measure.

### The cliff, located exactly

The account above leaves the cliff bracketed between two sweep points and the
margin stated as "about 1.5×". Both can be sharpened, because the cliff is not
something to bisect for — it is computable in closed form from a capture.

Rest requires `restMinT / accTs` *consecutive* samples whose residual is under
the threshold, since one sample over it resets the timer. So the smallest
threshold that admits rest at all is the quietest such window's peak:

    min over all windows of ( max residual within that window )

`fusion-bench noise` computes that sliding-window minimum-of-maximum directly.
It agrees with bisecting actual runs to within 0.06% across sample rates from
100–250 Hz, rest windows from 2–6 s, and noise from 0.002–0.05 m/s²
(`testNoiseFloorPredictsTheRestCliffExactly`).

```sh
./build/fusion-bench noise capture.csv            # what this device runs
./build/fusion-bench noise capture.csv --stock    # ditto, explicitly
```

```
accelerometer noise (m/s^2, 1 sigma):
  per axis     x 0.00340  y 0.00320  z 0.00510
  vector       0.00686

minimum restThAcc that admits rest:
  anywhere              0.01924  (includes the filter's startup transient)
  once settled          0.02359  (= 3.44 x vector sigma) <- configure against this
configured restThAcc:   1.418598  (60.1x the settled minimum)
```

Three things this changes about the story above:

**The margin is 60×, not 1.5×.** The 1.5× figure compared the measured noise to
the *cliff bracket* found synthetically. The right comparison is measured noise
to the threshold actually in force, and the threshold in force is stock
`1.418598` — because `DefaultVQFParams` is unreachable (see the comment on it in
`src/sensors/SensorFusion.h`). Were the tuned `0.06` wired up, its margin on this
part would be about 2.7×: above the cliff, but thin.

**Two bounds, not one.** A threshold between them catches rest during the
filter's startup transient and never again — VQF's low-pass is a plain running
mean for the first `restFilterTau`, so early residuals are artificially small.
`first_rest_sec` looks healthy while bias estimation silently stops after boot,
which is worse than the outright failure because the usual diagnostic misses it.

**The multiplier is not a constant, so don't hardcode one.** It grows with the
rest window in samples:

| rate | `restMinT` | N | cliff / per-axis σ |
| --- | --- | --- | --- |
| 100 Hz | 2.0 s | 200 | 3.23 |
| 250 Hz | 2.0 s | 500 | 3.24 |
| 250 Hz | 8.0 s | 2000 | 3.82 |

Near 3.3 across the 100–250 Hz accelerometer rates this firmware uses, which is
what makes "keep it well above the noise floor" quantitative. Above roughly
500 Hz it departs from anything the noise alone explains — 35σ at 1 kHz with
`restMinT` 8 s, where a peak-of-N argument predicts about 4. VQF is compiled
single-precision (`VQF_SINGLE_PRECISION`, `vqf.h`) and at 1 kHz the rest filter's
normalised cutoff is fc/fs ≈ 2e-4, so the low-pass stops tracking DC. Out of
range for this firmware; recorded so nobody extrapolates the multiplier there.

The threshold is *not* what rejects motion, incidentally. On `walk`, `yaw-sweep`
and `tumble`, rest is never detected at any accelerometer threshold including
stock's — `restThGyr` and `biasClip` do that work. Being generous with
`restThAcc` costs little; being stingy risks the silent failure.

### Both sides of the cliff, on real captures

Everything above is either synthetic or a static bench test. A tracker on a
shin, walking, is a different question, and it became answerable only once raw
samples could be captured over the network
(kmatzen/SlimeVR-Server#36, #41). Two real captures from an LSM6DSV on a
WEMOS D1 Mini — ten seconds stationary, fourteen seconds walking in place.

**The low side, located in threshold space:**

| `restThAcc` | rest detected | heading drift |
| --- | --- | --- |
| ≤ 0.015 | never | **23.35 °/min** |
| ≥ 0.020 | at 2.00 s | **2.78 °/min** |

The cliff sits between 0.015 and 0.020, and `fusion-bench noise` predicts
0.01518 from the same capture — inside the bracket, which is the closed-form
estimator above validated against a direct sweep on hardware rather than against
generated data.

The shipped 0.06 is about **4× clear of it**.

**The penalty is smaller than the synthetic figure.** Falling off the cliff cost
8.4× here (23.35 against 2.78 °/min), not the ~200× the generated static dataset
showed. Both are real; the mechanism is the same and the magnitude depends on
how much uncorrected bias the device has. Quote the mechanism, not the multiple.

**The high side, on real gait:** the claim above — that `restThAcc` is not what
rejects motion — holds, and holds harder than stated. On the walking capture
rest is never detected at *any* setting of any of the three parameters:

| swept on the walk | range tried | rest detected |
| --- | --- | --- |
| `restThAcc` | 0.02 → 1.418598 (stock) | never |
| `restThGyr` | 0.5 → 100 °/s | never |
| `restMinT` | 2.0 → 0.10 s | never |

So it is not `restThGyr` doing the work either. **Real walking never presents a
quiet window at all** — the shank rotates continuously through the gait cycle,
and there is no hundred-millisecond stretch where both residuals are small. Rest
detection cannot false-positive during gait however it is tuned.

Worth recording that this is a case where the synthetic conclusion was *right*.
The generated `walk` trajectory reached the same answer as a real leg, and the
existing advice — be generous with `restThAcc`, stinginess risks the silent
failure — survives contact with hardware unchanged.

**What is still untested** is the case where a limb is held still while the body
accelerates: a vehicle, a lift, a tracker on a torso that is being carried. That
is the one situation where the accelerometer residual is disturbed while the
gyroscope is quiet, and therefore the one where `restThAcc` would be the guard
rather than an irrelevance. Nothing here says anything about it.

## Bench tests on real hardware

These are the physical protocols. They are deliberately over-specified: the
value of a bench test is entirely in it being repeatable, and "hold it still for
a bit" is not repeatable.

**First, capture a dataset** — see "Capturing from a tracker" below. Do not
substitute eyeballing the GUI; that is the situation this tool exists to
replace.

General rules for every test:

- **Run each test 5 times.** Report mean ± standard deviation. A single run is
  not a measurement — IMU noise is stochastic and run-to-run spread is often
  larger than the effect you are looking for.
- **Use the same physical tracker** for all comparisons. Part-to-part variation
  between two nominally identical IMUs is frequently larger than the change
  being tested.
- **Let the tracker reach thermal equilibrium** — 10 minutes powered on —
  before starting, except in Test D where the whole point is that it has not.
- **Record the ambient temperature** and put it in the dataset's `# note`
  header. Gyro bias is temperature dependent; a comparison across a 5 °C swing
  is not a comparison.
- **Never compare numbers from different datasets.** A metric is only meaningful
  against the same input.

### Test A — static drift (start here)

Measures: `heading_drift_deg_per_min`, `tilt_error_deg_rms`, `first_rest_sec`.

1. Put the tracker on a solid surface — a desk, not a couch, not a shelf that
   shares a wall with a door. Foam underneath helps isolate building vibration.
2. Orient it flat, z-axis up. Leave it alone.
3. Power on, wait 10 minutes for thermal equilibrium.
4. Start logging. **Do not touch the desk for 30 minutes.** Walking heavily
   nearby is enough to trip rest detection.
5. `./build/fusion-bench run capture.csv`
6. `./build/fusion-bench noise capture.csv` — the noise floor of this part and
   how much margin its rest threshold has.

What to look for:

- `first_rest_sec` should be a small positive number. **If it is `-1`, stop —
  nothing else in the run means anything**, because bias estimation never ran.
  That alone confirms or refutes the `restThAcc` hypothesis above.
- From `noise`, the configured threshold should be comfortably above the *settled*
  minimum — at least 2×. A positive `first_rest_sec` with a margin under 1× means
  rest was reached only during the filter's startup transient and will not be
  reached again, which `run` alone cannot tell you.
- `heading_drift_deg_per_min` is typically 0.5–3 °/min for a 6-DoF tracker.
- `tilt_error_deg_rms` should be well under 1°. It has an absolute gravity
  reference, so unlike heading it should not drift at all.

### Test B — two-tracker differential (the most sensitive test, and it needs no ground truth)

Measures relative heading error between two trackers. **Do this one.** Their
relative orientation is constant *by construction*, so any change in relative
yaw is pure error, and no reference of any kind is required.

1. Bolt two trackers rigidly to one plate — 3D-printed, plywood, whatever, as
   long as it does not flex. Both flat, aligned, screwed down, not taped.
2. Power both, wait 10 minutes.
3. Start logging on both simultaneously.
4. Leave the plate still for 2 minutes. Then move it around by hand for 5
   minutes: slow turns, fast turns, tilts, a couple of sharp taps. Then set it
   down still for 2 more minutes.
5. Compare the two heading traces.

The relative yaw should be constant. Plot it. What you are looking for is not
the average but the *shape*: a ramp means one tracker is drifting faster; a step
at a sharp movement means one lost heading during dynamics.

This is also the acceptance test for the sample-timestamp work: on a rigid mount
the true lag between the two is zero by definition, so any measured lag between
their angular-velocity traces is pure clock skew.

### Test C — return to origin

Measures accumulated heading error, which is the thing users actually feel.

1. Build a fixture with a hard mechanical stop: a 3D-printed cradle against two
   walls, or a block of wood with a screwed-on lip. It must return to the *same*
   pose repeatably — this is the whole experiment, so do not freehand it.
2. Seat the tracker, log 10 seconds still.
3. Lift, rotate through a fixed sequence for 60 seconds — say 10 full turns
   about each axis in order, at a comfortable speed.
4. Return it to the fixture. Log 30 seconds still.
5. Compare heading in step 2 against heading in step 4.

That difference is accumulated error. It is directly comparable to
`final_heading_error_deg` from the synthetic `return-to-origin` case.

### Test C2 — gyroscope scale factor

Measures the error term rest-gated bias calibration cannot see. Bias dominates a
*stationary* sensor and is estimated at rest; scale factor produces error
proportional to rotation **angle** and is therefore invisible at rest, because
there is no rotation to scale. A 1% error puts you 1.8° out after a 180° turn,
however good the bias estimate is.

`fusion-bench gyro-scale` estimates it from an ordinary capture — no turntable
required. Whenever the tracker is still, the accelerometer gives the gravity
direction exactly; between two such pauses the gyroscope says how far it thinks
the tracker turned, and that prediction can be checked against where gravity
actually ended up. A scale error makes the prediction miss.

**Capture protocol.** Roughly two minutes, by hand, no fixture:

1. Rest the tracker on the desk for 2 seconds. **The pauses are the measurement**
   — without them there is no reference.
2. Turn it about 90° about one axis, taking about a second.
3. Rest 2 seconds again.
4. Repeat 10–15 times, **varying which axis you turn about**, and make sure the
   tracker ends up genuinely tilted rather than always flat.

```sh
./build/fusion-bench gyro-scale capture.csv
```

```
rest-to-rest transitions used: 12
gravity prediction error: 2.140 deg before, 0.089 deg after
observability (deg per 1% of scale): x 0.412  y 0.388  z 0.244

scale (multiply measured rate by this):
  x 0.97087  y 1.02041  z 0.98522
gyroscope reads high by: x +3.000%  y -2.000%  z +1.500%
```

**What it cannot see, and why it will tell you so.** Rotation *about* the gravity
vector does not move gravity, so it carries no scale information at all. A
capture of a tracker spun while sitting flat is worthless for this however long
it runs. The estimator reports per-axis observability and refuses rather than
returning a confident number built from noise:

```
no usable estimate: axis Z was never rotated in a way that moves gravity
```

It exits non-zero when it refuses, so it can be scripted. Running it on the
static Bench Test A capture correctly reports `fewer than two settled rest
periods` — that capture has no motion at all.

**Relationship to the turntable.** The turntable measures a full known rotation
about one axis and is the better reference for a single axis. This is
complementary: it needs no fixture, covers all three axes in one capture, and is
something a user can actually be asked to do.

#### Storing the result on the tracker

The estimate is only useful once the tracker applies it. Type the three numbers
from the `scale (multiply measured rate by this)` line back over the serial
console:

```
SET GYROSCALE 0 0.97087 1.02041 0.98522
```

```
[INFO ] CMD SET GYROSCALE OK: Sensor 0 scale set to 0.9709 1.0204 0.9852
[INFO ] Reboot to apply -- the running fusion holds its own copy
```

The first argument is the sensor ID as printed by `GET INFO`. **Reboot after
setting it** — the calibration is read once at sensor init, so the running
fusion keeps its old copy until then.

To read back what is stored:

```
GET GYROSCALE
```

To undo it without destroying anything else:

```
SET GYROSCALE 0 RESET
```

`RESET` restores unity. Use it rather than `DELCAL`, which erases the gyroscope
and accelerometer *bias* calibration too — that is the part the tracker
re-learns slowly at rest, and throwing it away costs far more than the scale
factor is worth.

**Why this is a one-off and bias is not.** Scale error is a property of the part
that is stable over its life; bias drifts with temperature and age, which is why
the firmware re-estimates it continuously at rest and why there is no equivalent
`SET GYROBIAS`.

**What it will refuse.** Values outside 0.90–1.10 are rejected, as is anything
that is not a number:

```
[ERROR] CMD SET GYROSCALE ERROR: Y axis 10.0000 is outside 0.90..1.10
[INFO ] A real gyroscope is within a few percent of unity; check for a misplaced decimal point
```

Those bounds are the same range the estimator searches, so a value outside them
is not something it could have produced — it is a typo. This matters more than
it sounds: a gyroscope scaled by ten does not look like a bad calibration, it
looks like broken hardware, and the fix is not obvious once the tracker is
reassembled.

If the command reports `Sensor 0 does not use runtime calibration`, the board is
using one of the older fixed calibration formats, which has no error-model
matrix to write into.

### Test C3 — accelerometer bias and scale, on the tracker

Unlike everything else in this section, this one needs no capture and no host
tool. The tracker runs the fit itself:

```
CALIBRATE ACCEL
```

Then hold it still with each axis pointing up in turn — six positions, any
order. It captures each one on its own as soon as it sees the tracker settled in
a position it has not already recorded, so there is nothing to press:

```
[INFO ] Guided accelerometer calibration started for sensor 0
[INFO ] Hold the tracker still with each axis pointing up in turn; it will capture on its own
[INFO ] Next: hold the tracker with +X up (0 of 6 captured)
[INFO ] Capturing +Z up -- hold still
[INFO ] Captured +Z up
[INFO ] Next: hold the tracker with +X up (1 of 6 captured)
...
[INFO ] Captured -Y up -- all 6 positions done, fitting
[INFO ] Accel bias: 0.118 -0.079 0.052
[INFO ] Accel scale: 1.0298 0.9702 1.0101
[INFO ] Accelerometer calibration applied
```

Applied immediately, no reboot — unlike `SET GYROSCALE`, which has to defer
because the number comes from outside. Here the fit just ran against this
sensor's own samples, so the running copy is updated in place.

`CALIBRATE CANCEL` abandons a run, and it gives up on its own after two minutes
with no progress. Add a sensor ID (`CALIBRATE ACCEL 1`) to address one IMU; with
none, every sensor calibrates from the same six placements, which is what you
want on an extension holding two.

**What it fits, and what it deliberately does not.** Bias and per-axis scale
only. The matrix it writes is diagonal.

That is narrower than "six positions determines bias, scale *and* misalignment
by least squares", which is what issue #5 proposed and what the textbook
procedure is usually described as doing. Working through it, the textbook is
wrong about the perfectly-executed case, and in an interesting direction: the
full quadric fit has columns for `xy`, `xz` and `yz`, and holding exactly one
axis vertical makes every one of those products zero. All three columns vanish
and the solve is **singular** — not ill-conditioned, singular. It is only
hand-placement error that makes the full fit succeed at all, which means the
misalignment terms a six-position fit reports are estimated from how badly you
held the tracker.

Inventing cross-axis coupling is the worst possible direction to be wrong in:
it converts pitch and roll into spurious yaw, and yaw is the unobservable axis
on a 6-DoF tracker. So the on-device flow fits the six unknowns six positions
genuinely determine and declines to guess the other six. Misalignment stays with
the host path below, where a capture can cover orientations that actually
observe it.

**Coverage still matters, for a subtler reason than you would expect.** Given
noiseless samples, even badly thinned coverage — four positions in one plane
plus a pair lifted 20° out of it — fits *exactly*. Thin coverage does not fail
loudly. What it does is amplify noise, measured on synthetic data with 0.02 m/s²
surviving the block average:

| out-of-plane coverage | worst Z-scale error |
|---|---|
| full six positions | 0.23% |
| pair lifted 35° | 1.06% |
| pair lifted 20° | 3.3% |

At 3.3% the calibration is adding more scale error than it removes. The fit
refuses sample sets whose direction spread falls below threshold for exactly
this reason, and the tolerance on each hold (20° off-axis) exists to keep you
well clear of it.

**What it will refuse.** Two independent checks, and they mean different things.
The fit itself refuses when the six positions did not span three dimensions — no
model is recoverable. The plausibility check refuses when a model *was*
recovered and describes a part no accelerometer could be: scale outside
0.90–1.10, misalignment above 0.10, or bias above 10% of gravity. Either way the
previous calibration survives untouched, because a slightly uncalibrated tracker
is worth much more than a confidently wrong one.

**For the full matrix**, including misalignment, use `SET LOGRAW` to capture a
session covering orientations off the axes and fit it on the host — that path
observes the cross terms this one cannot.

### The tracker also calibrates itself, quietly

Since the six-position procedure only exists to put the sensor in varied
orientations, and ordinary use eventually does that anyway, the tracker runs the
same fit continuously in the background. Every still moment is a constraint —
a stationary accelerometer measures gravity, and gravity has a known magnitude.

```
GET ACCELCAL
```

```
Sensor[0] online accel estimate: 41.3 observations, spread 0.291, all axes both ways: yes
  online: bias 0.1187 -0.0793 0.0521  scale 1.0298 0.9701 1.0102
  stored: bias 0.1187 -0.0793 0.0521  scale 1.0298 0.9701 1.0102 (from this estimator)
```

**It only applies itself where nothing better exists.** A model from `CALIBRATE
ACCEL` or `SET GYROSCALE` is something you chose; the estimator defers to it
permanently and says so. It will, however, keep refreshing *its own* previous
answer — that is the point of being online, since bias moves with temperature
while scale does not.

**What makes it fit on a microcontroller** is that the batch fit's normal
equations are a sum over samples, so there is no history to keep. The whole
estimator is 368 bytes and constant — for comparison the guided collector needs
288 bytes just to hold its 24 samples. It is not a port of the offline
estimator (which re-integrates the capture per candidate); it is the same
algorithm with the loop turned inside out.

**Three gates decide what counts as an observation**, and the first is the one
that makes the whole thing work:

- **Novelty.** A tracker left on a desk overnight is at rest for seven million
  samples in *one* orientation. Counted, they would swamp every other direction
  ever seen — and everything would look healthy while it happened, because the
  count would be enormous and the system beautifully conditioned about a single
  point. Coverage is the only thing that would be wrong, and coverage is exactly
  what a count cannot measure. So an observation is only taken somewhere the
  tracker has not already been, and a long rest is worth one.
- **Magnitude**, because rest detection reports "not moving", which is not the
  same as "measuring gravity".
- **Forgetting**, so the bias estimate reflects current conditions rather than
  an average over a day of them.

**It does not converge from wear alone, and that is not a tuning problem.** A
shin tracker sees gravity inside a narrow cone however far its wearer walks, so
the axis whose scale and bias are confounded stays confounded — separating them
needs that axis measured pointing both up and down. What completes the picture
is the tracker being taken off, set down and stored on different faces, which
happens daily but is not walking. Until then it reports what it has and refuses
to solve. If you want a calibration now, run `CALIBRATE ACCEL`.

**One board does not have it.** `BOARD_GLOVE_IMU_SLIMEVR_DEV` builds every IMU
driver into a 1280 kB partition and was already 99.8% full before this existed —
2672 bytes spare, against roughly 17 kB for the flow. About 5 kB of that is the
soft-float `double` library, which this is the first code on that board to need;
the rest is the collector and the fit. No amount of trimming closes a gap that
size, so it is compiled out there (`-D DISABLE_GUIDED_ACCEL_CALIBRATION`) rather
than shrunk into something that no longer works. `CALIBRATE ACCEL` says so
rather than failing silently. Every other board has it, and the same flag
governs the online estimator above — a board that cannot afford the procedure
certainly cannot afford an estimator that runs all the time.

### Test D — temperature ramp

The firmware has temperature-gradient compensation and nothing currently proves
it works. This is that proof.

1. Put the tracker in a fridge (not a freezer) for 30 minutes, in a sealed bag
   with a desiccant packet so condensation does not form on the board.
2. Take it out, immediately place it on the bench, power on, start logging.
3. Log continuously for 45 minutes as it warms to room temperature.
4. Plot `heading_drift` over 5-minute windows against the logged temperature.

A tracker with working temperature compensation shows drift roughly independent
of the thermal gradient. One without shows drift that tracks the gradient and
settles as the temperature does. The difference is obvious on a plot and
invisible in a single number, which is why this test outputs a curve.

### Optional: free ground truth

Two ways to get a real reference without a mocap lab:

- **BROAD** (Berlin Robust Orientation Attitude Dataset) — free, optical-mocap
  ground truth, purpose-built for benchmarking orientation filters, and VQF's
  own paper evaluates on it, so there are published numbers to compare against.
  Convert to the CSV format below and run it. This validates the *algorithm*; it
  says nothing about this firmware's drivers or calibration.
- **Lighthouse** — rigidly co-mount a tracker with a Vive Tracker or controller
  and log both. Sub-degree, free if you already have base stations, and it
  captures real human motion rather than bench motion.

## Capturing from a tracker

The firmware can stream raw samples over serial in exactly the format below, so
capturing a dataset is just redirecting the serial port to a file. There is no
converter step and no host-side dependency.

### 1. Build with logging enabled

Add to the `[env]` section of `platformio.ini`, or pass via
`PLATFORMIO_BUILD_FLAGS`:

```ini
build_flags =
  ${env.build_flags}
  -D RAW_SAMPLE_LOGGING
  -D serialBaudRate=460800
```

Raise the baud rate. An LSM6DSV at 240 Hz gyro / 120 Hz accel produces roughly
9 KB/s, and 115200 baud carries about 11.5 KB/s — it fits, but with only ~20%
headroom, and a faster IMU will not fit at all. **If the link saturates, serial
writes block and the sample timing is perturbed, which corrupts the very thing
you are measuring.**

460800 is the recommended rate: 5× the headroom needed, and verified working.
921600 produced unreadable output on the CH340 adapter tested, so prefer 460800
unless you have confirmed the higher rate on your own hardware.

Only one IMU is logged. If your board has two, select which with
`-D RAW_SAMPLE_LOGGING_SENSOR_ID=1` (default 0).

This is a debug build. Do not ship it — the serial stream runs continuously.

### 2. Capture

```sh
# Linux / macOS. Substitute your port; 'ls /dev/tty.*' or 'ls /dev/ttyUSB*'.
stty -f /dev/tty.usbserial-0001 921600 raw
cat /dev/tty.usbserial-0001 > capture.csv
```

Let it run for the duration the test calls for, then Ctrl-C.

### 3. Run

```sh
./build/fusion-bench run capture.csv
```

If the capture picked up ordinary firmware log lines, they are skipped and the
count is reported on stderr. A large skip count means something is wrong —
usually a saturated serial link — and the numbers should not be trusted.

### What gets logged, and why raw

Raw uncalibrated integer counts, with the scale factors in the header.

- Raw is what the sensor actually produced. Everything else is derived from it,
  so a raw capture can be replayed under *different* calibration — which is the
  entire point when the thing being evaluated is the calibration, as in the
  `restThAcc` example above.
- Formatting integers is much cheaper on an ESP8266 than formatting floats, and
  this runs in the sample path. Perturbing sample timing would corrupt the
  measurement.

Note this means a replay measures the **uncalibrated** sensor. It will show more
drift than the tracker actually exhibits, because the runtime bias calibration
is not applied. That is deliberate: it isolates the sensor from the calibration
so the two can be evaluated separately.

Accelerometer and gyroscope rows are separate, because the two run at different
rates (120 Hz and 240 Hz on an LSM6DSV) and resampling them onto a common tick
would bake an assumption into the data instead of leaving it to the analysis.

Row timestamps are **nominal** — derived from the configured sample period —
because that is what the on-device fusion integrates, so replaying them
reproduces what the filter saw. Periodic `# t_real` comments carry the true
elapsed time alongside sample counts, so the configured and actual ODR can be
compared. That difference is exactly the error the runtime sample-rate
calibration exists to correct, and it is otherwise invisible.

## Dataset format

Plain CSV with a small comment header. Diffable, hand-editable, trivially
produced by a firmware serial logger or by numpy.

```
# slimevr-imu-log v1
# gyr_ts 0.0016
# acc_ts 0.0016
# note   static bench, tracker flat on desk, 21.5 C
t_us,ax,ay,az,gx,gy,gz,qw,qx,qy,qz
0,0.01,0.02,9.80,0.0011,0.0,0.0,1,0,0,0
```

Required: `t_us, ax, ay, az, gx, gy, gz`.
Optional: `mx, my, mz` (magnetometer), `qw, qx, qy, qz` (reference orientation),
`temp` (°C).

Units are m/s² and rad/s by default — what VQF expects. A file may instead hold
raw sensor counts and declare `# acc_scale` / `# gyr_scale` / `# mag_scale`
header directives, which the reader multiplies through. This is what the
firmware emits.

**An empty field means "no sample", which is not the same as a sample reading
zero.** Real IMUs run the accelerometer and gyroscope at different rates, so a
capture interleaves rows:

```
t_us,ax,ay,az,gx,gy,gz
4166,,,,10,-20,30
8333,100,-200,8192,,,
8333,,,,11,-21,31
```

Each sensor is updated only on rows that carry it. Repeating the last
accelerometer sample onto every gyroscope row would double-count the
accelerometer correction and change what the filter does. For the same reason
the gyroscope timestep is measured between consecutive *gyroscope* rows, not
between consecutive rows of any kind.

Timestamps are used as-is rather than assuming the nominal rate, so jitter and
dropped samples in a real capture are modelled rather than silently smoothed
away.

## Setting tolerances

`make baseline` writes a starting tolerance of 5% per metric. **Replace these
with measured values.** The procedure:

1. Run the suite 20 times. Because the synthetic data is deterministic, all 20
   are identical — the spread is zero and a tight tolerance is legitimate.
2. For metrics computed from *real* captures, run the physical test 5 times and
   use 3σ of the observed spread.

Tolerance set from taste rather than measurement will either flap — and a
flapping CI check gets muted, which is worse than having none — or be so loose
it catches nothing.

## CI

`make check` exits non-zero if any metric moves outside tolerance, and prints a
`baseline | current | delta` table either way. Report the table on every PR, not
just failures: many changes here are trade-offs, and a reviewer needs to see the
trade rather than a bare pass/fail.

## Layout

```
src/quatmath.h    quaternion/vector helpers, deterministic PRNG
src/dataset.*     CSV reader/writer
src/synth.*       synthetic trajectory generation with exact ground truth
src/metrics.*     VQF runner and metric computation
src/main.cpp      CLI
tests/selftest.cpp  tests for the harness itself
baseline.txt      committed expected values
```

`tests/selftest.cpp` tests the *measuring instrument*, not the firmware — that
the quaternion algebra is right, that the error decomposition separates heading
from inclination correctly, and that runs are bit-reproducible. A benchmark
whose own arithmetic is wrong is worse than no benchmark, because it produces
confident numbers.

## Formatting

The repo's `format` CI job runs clang-format 17 over the whole tree, and
formatting differs between clang-format versions, so pin it:

```sh
pip install clang-format==17.0.6
make format        # apply
make format-check  # verify, as CI does
```

## Bring-up: BMM350 over the LSM6DSV sensor hub

**Status: the sensor-hub half runs on hardware; the BMM350 half has never been
answered by a real part.** Those are separate claims and worth keeping separate.
The LSM6DSV master has been driven on silicon — it configures, issues I2C cycles
and reports their outcome correctly (see "Confirmed working on silicon" below),
and getting there fixed two real register bugs. What no board has done yet is
*acknowledge*: the only Blueberry tested is the `[NO_MAG]` variant with U6
unpopulated, so every BMM350 register value below — taken from the Bosch
datasheet (BST-BMM350-DS001) and `BMM350_SensorAPI` — is still unexercised.

Work through the milestones in order — each isolates a different failure, and
stopping at the first that fails tells you exactly where the problem is.

Build with `serialDebug` set to `true` in `src/debug.h` so the driver's log
output is visible.

### Hardware bring-up results (CheeseCake "Blueberry", LSM6DSV + BMM350)

Run on a real board. Build with `-D LSM6DS_SHUB_DEBUG` for these diagnostics.

**The sensor hub works.** It configures, runs I2C cycles, and reports their
outcome. **The magnetometer does not respond**: every address tried returns
`STATUS_MASTER = 0x09`, which is `SENS_HUB_ENDOP | SLAVE0_NACK` — the cycle
completed and nothing acknowledged.

```
[ERROR] Aux device 0x14 did not acknowledge (STATUS_MASTER=0x09)
```

That is a hardware answer, and a definite one. A pass-through probe bridging
SDX/SCX onto the host I2C bus independently finds nothing. The board designer's
caution sheet says the magnetometer *"is an experimental and might not work
functionally"*, and the board ships in `MAG` and `[NO_MAG]` BOM variants.

**Resolved: the board tested had no magnetometer fitted.** The `[NO_MAG]` BOM
variant leaves U6 unpopulated, and an empty footprint NACKs exactly like a
faulty or unpowered part. The driver's behaviour here is correct -- it detects
no magnetometer, logs why, and continues with the IMU.

If you are debugging a board that *should* have one, measure **1.8 V across
C15** (U8's output, feeding the BMM350's VDD) before suspecting the part: an
unpowered device is indistinguishable from an absent one at the bus level.

#### Two datasheet bugs this found

Both were in this driver, both silent, and each masked the other. Found by
reading DS13476 Rev 2 directly after inference from the LSM6DSO family
conventions had produced two confidently wrong diagnoses.

1. **`STATUS_MASTER_MAINPAGE` is `0x48`, not `0x39`.** On LSM6DSV, `0x39` is
   `UI_OUTZ_H_A_OIS_DualC` — an OIS output register, which reads ~0 on a
   stationary tracker. Polling it meant end-of-operation was never observed no
   matter what the hub did.

2. **`SLV0_CONFIG` bits [7:5] are `SHUB_ODR`, whose reset default is `100` =
   120 Hz.** Writing the register with just a `numop` value cleared them to
   `000` = **1.875 Hz**, a 533 ms cycle — longer than the original 50 ms
   timeout and marginal even against 500 ms.

Together these produced `STATUS_MASTER = 0x00` with neither ENDOP nor NACK,
which reads as "the master never starts". That inference was correct given the
data and wrong about the cause, and it sent the investigation at the OIS/SPI2
pin muxing and the trigger polarity — both of which were fine.

`STATUS_MASTER` is also read-to-clear, so its value has to be captured inside
the poll loop at the moment ENDOP is seen. Reading it afterwards returns zero
and makes every transaction look clean.

#### Confirmed working on silicon

- LSM6DSV detected and streaming; measured timesteps 0.004161 s gyro /
  0.008323 s accel, exactly the nominal 240/120 Hz.
- `SupportsMags` true for LSM6DSV, so the magnetometer path is compiled in.
- Sensor-hub bank entry and exit: `WHO_AM_I` reads `0x70` outside and `0x00`
  inside, both directions.
- Configuration writes land: `SLV0_ADD=0xf9`, `SLV0_SUBADD`, `SLV0_CONFIG=0x81`
  (120 Hz | 1 read), `MASTER_CONFIG=0x44`.
- Hub cycles complete and report NACK status correctly.

#### Also verified against the datasheet

- `START_CONFIG = 0` selects the accel/gyro data-ready trigger (table 428). An
  earlier experiment flipping this was wrong.
- `SLV0_ADD` is the 7-bit address in bits [7:1] with `rw_0` in bit 0 (table 430).
- `AUX_SENS_ON = 00` means one sensor (table 428).
- `STATUS_MASTER`: `SENS_HUB_ENDOP` bit 0, `SLAVE0_NACK` bit 3 (table 456).

### Milestone 1 — the hub can read the magnetometer's chip ID

This is the decisive test, and it validates most of the risk in one shot: bank
switching, the slave address, the read sequence, `STATUS_MASTER` polling, and
the aux bus wiring.

Watch the serial log during startup for:

```
[INFO ] [MagDriver] Trying mag BMM350!
[INFO ] [MagDriver] Found mag BMM350! Initializing
```

- **"Found mag BMM350"** — the sensor hub read path works. Move to milestone 2.
- **"Trying" but never "Found"** — `readAux` returned something other than
  `0x33`, i.e. the hub completed a cycle and the part did not answer as
  expected. On a Blueberry the remaining causes are, in order: **U6 is not
  populated** (the `[NO_MAG]` BOM variant — by far the most likely, and what
  happened here); the part is present but unpowered, which is
  indistinguishable from absent at the bus level, so measure 1.8 V across C15
  first; or the part is fitted and faulty, which the designer's caution sheet
  explicitly allows for.
- **"Aux read ... timed out"** — the hub is not completing transactions at all.
  The hub is clocked by the accelerometer, so confirm the IMU is streaming
  normally first.

Three causes this list used to name have since been eliminated, and are kept
here only so they are not re-investigated: the address strap (ADSEL is tied to
GND on this board — see below — so `0x14` is right), the aux pull-ups (`setAuxId`
now calls `enableAuxPullups()`, so `readAux` gets them too, not just
`startAuxPolling`), and `STATUS_MASTER_MAINPAGE` (corrected from `0x39` to
`0x48`).

#### Verified from the board files, without hardware

Read out of the EasyEDA project in the CheeseCake repo
(`006-LSM6DSV 「Blueberry」/LSM6DSV_MAG_JST125.epro`, sheet net list) rather than
inferred, so these are settled for this board and need no bring-up time:

- **ADSEL (U6 pin B2) is tied to GND**, so the I2C address is `0x14`. The `0x15`
  strap hypothesis does not apply here.
- **The aux bus is point-to-point.** `SCX` and `SDX` have exactly two endpoints
  each — the LSM6DSV and U6 — so nothing else loads them, and there is no
  external pull-up anywhere on either net. `IF_CFG.SHUB_PU_EN` is therefore
  mandatory on this board, not an optimisation.
- **U6's SDA/SCK go only to the LSM6DSV's `SDX`/`SCX`**, never to the host bus.
  The ESP8266 cannot reach the magnetometer directly, so the sensor hub is the
  only route — there is no simpler bit-banged alternative to fall back on. (A
  pass-through probe can bridge the two buses, which is how the NACK above was
  independently confirmed.)
- **VDDIO (B3) is on 3V3 and VDD (C1) on 1V8**, the latter from U8
  (AP7343D-18FS4-7B). VDDIO shares the LSM6DSV's supply, so aux-bus logic levels
  match with no shifting; only the analog supply is 1.8 V, which is why C15 is
  the thing to measure.
- **INT (A1) is marked no-connect**, so data-ready interrupt from the mag is not
  available. Polling via the hub is the only option, which is what the driver
  does.

### Milestone 2 — the magnetometer is configured

If detection works but the part never produces sensible data, the setup writes
are the next suspect. `writeAux` uses the same machinery as `readAux`, so if
milestone 1 passed the mechanism is sound — but verify by reading back
`PMU_CMD_STATUS_0` (`0x07`) after init and confirming normal mode is latched.

### Milestone 3 — data streaming

The FIFO decode is now implemented. `bulkRead` handles the sensor-hub tags and
reassembles the split slave-0/slave-1 reads, skipping each transaction's dummy
bytes, then hands a sample to `processMagSample`.

The byte-level assembly lives in `src/sensors/softfusion/drivers/magfifo.h`,
deliberately free of any hardware dependency so it can be unit tested — see
`testMagFifo24BitSplit` and friends in `tests/selftest.cpp`. That logic *is*
verified: the BMM350's 24-bit split layout, sign extension at the 24-bit
boundaries, and recovery when a slave-1 word is dropped all have tests, and the
tests were checked against a deliberate mutation to confirm they are not
vacuous.

What is **not** verified is the part that touches the device: the FIFO tag
values. They come from ST's `lsm6dsox_reg.h` enum — slave 0 is `0x0E`, slave 1
`0x0F`, and a slave NACK is `0x19`. Tags `0x01`–`0x03` in that same enum match
what this driver already relies on for gyro, accel and temperature on LSM6DSV,
which is good evidence the numbering carries over, but it is evidence rather
than confirmation.

If milestones 1 and 2 pass and no magnetometer data arrives, the tag values are
the first thing to check. A `Sensor hub reported a NACK` line in the log means
the tags are right and the auxiliary sensor is not responding — a different
problem, and a much easier one.

### Milestone 4 — factory trim (OTP) compensation

Implemented. On detection the driver reads all 32 OTP words through the sensor
hub, decodes the trim, and applies offset, per-axis sensitivity and cross-axis
compensation to every sample.

Look for this line at startup:

```
[INFO ] [MagDriver] Read BMM350 factory trim data
```

If instead you see `Failed to read BMM350 trim data; continuing uncompensated`,
the magnetometer still works — `bmm350Compensate` falls back to nominal scaling,
so a bad OTP read costs accuracy rather than the whole sensor. The OTP readout
costs several sensor-hub transactions per word and the hub is clocked by the
accelerometer, so all 32 words take a noticeable fraction of a second; that is
why it happens once at startup and never per-sample.

The arithmetic — OTP word decoding, sign extension of the 8- and 12-bit packed
fields, and the compensation formula — is in
`src/sensors/softfusion/drivers/bmm350comp.h`, deliberately hardware-free so it
can be unit tested. It is covered by 40 checks in `tests/selftest.cpp`,
including the `offset_y`/`offset_z` fields that straddle two OTP words, which
are the easiest thing in the decoder to get wrong and the hardest to notice.
Those tests were confirmed non-vacuous by mutation.

Cross-axis compensation is the part that matters most for a tracker: it rotates
the measured field vector, and a rotated vector is a wrong heading — an error no
amount of downstream filtering can undo. Per-axis sensitivity has the same
character. Both are now corrected.

### Known limitation: temperature drift is not corrected

The TCO and TCS terms need the BMM350's own die temperature, which lives at
registers `0x3A`–`0x3C`, immediately after the magnetometer data. Reading it
would extend the sensor-hub burst from 9 to 12 data bytes, which no longer fits
two slaves and would raise the FIFO word rate.

Until then the compensation is evaluated at the trim's reference temperature,
where both terms are defined to vanish. This is a well-defined degradation
rather than a fudge: everything affecting field *direction* is corrected, and
what is dropped is the second-order drift of offset and sensitivity with
temperature.

Wiring it up needs a third sensor-hub slave and a lower magnetometer ODR to keep
FIFO pressure sane — 25 Hz is ample for heading.

For heading purposes this matters less than it sounds: VQF needs a consistent
field *direction*, and hard- and soft-iron correction is a separate step
regardless. Uncompensated output should still yield usable heading. It will
degrade VQF's magnetic-disturbance rejection, though, since that compares field
norm against a learned reference and cross-axis error makes the norm vary with
orientation. Worth revisiting once streaming works.
