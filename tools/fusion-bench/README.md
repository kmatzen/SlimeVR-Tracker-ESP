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

`make suite` takes well under a second, which is the point: it can run on every
PR without anyone resenting it.

## What you get today

The suite generates its own datasets, so it works with no hardware and no
downloads. Each case isolates one thing:

| case | what it isolates |
| --- | --- |
| `static-tuned` | drift with a realistic residual gyro bias, using `DefaultVQFParams` |
| `static-stock` | the same input with VQF's own default parameters |
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

**This is a hypothesis, not a conclusion.** It rests on a white-noise
accelerometer model, and real noise is not white. What is robust is the
*structure*: there is a cliff, and the shipped value is near it. Confirming it
on real hardware is Bench Test A below, and that is exactly the point — the
harness turns "these constants look suspicious" into a specific, cheap,
falsifiable measurement.

## Bench tests on real hardware

These are the physical protocols. They are deliberately over-specified: the
value of a bench test is entirely in it being repeatable, and "hold it still for
a bit" is not repeatable.

**Prerequisite:** all of these need raw sample logging in the firmware, which
does not exist yet — see the raw-logging issue. Until then the suite above runs
on synthetic data only. Do not skip the prerequisite by eyeballing the GUI;
that is the situation this tool exists to replace.

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

What to look for:

- `first_rest_sec` should be a small positive number. **If it is `-1`, stop —
  nothing else in the run means anything**, because bias estimation never ran.
  That alone confirms or refutes the `restThAcc` hypothesis above.
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

Units are m/s² and rad/s — what VQF expects, so nothing is rescaled anywhere in
the harness.

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
