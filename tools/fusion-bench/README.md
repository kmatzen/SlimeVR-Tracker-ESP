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
  -D serialBaudRate=921600
```

Raise the baud rate. An LSM6DSV at 240 Hz gyro / 120 Hz accel produces roughly
9 KB/s, and 115200 baud carries about 11.5 KB/s — it fits, but with only ~20%
headroom, and a faster IMU will not fit at all. **If the link saturates, serial
writes block and the sample timing is perturbed, which corrupts the very thing
you are measuring.** 921600 leaves plenty of margin.

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

**Status: compiles, never run against hardware.** The register values come from
the LSM6DSV datasheet, ST's `lsm6dsv_reg.h`, and Bosch's `bmm350_defs.h`, but
none of the sequences have been executed on a real part. Work through these
milestones in order — each one isolates a different failure, and stopping at the
first that fails tells you exactly where the problem is.

Build with `serialDebug` set to `true` in `src/debug.h` so the driver's log
output is visible.

### Hardware bring-up results (CheeseCake "Blueberry", LSM6DSV + BMM350)

Run on a real board. Build with `-D LSM6DS_SHUB_DEBUG` to reproduce these
diagnostics.

**Confirmed working on silicon:**

- LSM6DSV detected and streaming; measured timesteps 0.004161 s gyro /
  0.008323 s accel, i.e. exactly the nominal 240/120 Hz.
- `IMUConsts::SupportsMags` is true for LSM6DSV, so the magnetometer path is
  compiled in and `MagDriver` runs its detection loop. This was the gate that
  previously discarded the whole path.
- Sensor-hub bank entry *and* exit: `WHO_AM_I` reads `0x70` outside the bank and
  `0x00` inside it, both ways, on every switch.
- Every hub configuration write lands: `SLV0_ADD=0xf9` (= `0x7c<<1|1`),
  `SLV0_SUBADD=0x00`, `SLV0_CONFIG=0x01`, `MASTER_CONFIG=0x44`.

**Not working:** the hub never performs a transaction. `STATUS_MASTER` reads
`0x00` from both the main-page mirror and the in-bank register — crucially,
neither `SENS_HUB_ENDOP` *nor* any NACK bit. A hub that ran a cycle against an
unresponsive bus would set a NACK; reading exactly zero means the master never
starts a cycle at all. A pass-through probe, which bridges SDX/SCX onto the host
I2C bus, found no device at `0x14`, `0x15` or any other address tried.

**Hypotheses tested on hardware and refuted.** Each was applied, the register
write was verified to have taken effect, and the failure was unchanged:

| Hypothesis | Test | Result |
| --- | --- | --- |
| Aux bus has no pull-ups | Enable `SHUB_PU_EN` before every transaction, not just before polling | Was a real defect, fixed; failure unchanged |
| SDX/SCX held by the OIS/SPI2 interface | `OIS_CTRL_FROM_UI=1`, pulse `SPI2_RESET`, `UI_CTRL1_OIS=0x00` (verified reading back `0x00`) | Refuted |
| One hub cycle is slower than the timeout | Raise the timeout from 50 ms to 500 ms | Refuted |
| `START_CONFIG` polarity inverted vs LSM6DSO | Flip bit 5 (`MASTER_CONFIG=0x64`, verified in readback) | Refuted |
| Bank switch not taking effect | Read `WHO_AM_I` inside and outside the bank | Bank works both ways |
| Config writes not landing | Read back all four hub registers | All correct |

Note what a dead auxiliary bus would look like: the master would still run a
cycle and set `SENS_HUB_ENDOP` together with `SLAVE0_NACK`. Reading exactly
`0x00` is a stronger statement than "nothing answered" -- the master is not
starting. That points at the LSM6DSV side rather than the BMM350 side, and it is
why the hardware hypotheses below are listed after the firmware ones despite the
pass-through probe finding nothing.

**Remaining hypotheses, in the order worth testing:**

1. **No 1.8 V rail.** The BMM350's VDD comes from U8 (AP7343D-18). Measure across
   C15: it should read 1.8 V. This is a 30-second check with a multimeter and
   would settle both this and the next item. U6 and U8 being populated does not
   prove the rail is up.
2. **Solder joint on U6.** The BMM350 is a small package; populated is not the
   same as connected. A missing SDX/SCX or GND joint gives exactly this
   signature.
3. **Pin multiplexing.** On LSM6DSV the SDX/SCX pins are shared with the
   OIS/SPI2 auxiliary interface. If they default to that function the hub cannot
   drive them, which would explain the master never starting. This is the most
   likely remaining *firmware* cause and the next thing to try in code.

Note the board designer's own caution sheet says *"The magnetometer is an
experimental and might not work functionally"*, and the board ships in `MAG` and
`[NO_MAG]` BOM variants — so a non-functional magnetometer is a documented
possibility rather than a surprise.

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
  `0x33`. Most likely causes, in order: the board straps the BMM350's address
  pin high, so it is at `0x15` not `0x14` (change `deviceId` in the
  `MagDefinition`); the aux bus needs the internal pull-ups that
  `startAuxPolling` enables but `readAux` does not; `STATUS_MASTER_MAINPAGE`
  (`0x39`) is wrong for this part, so `waitForEndOp` returns before the
  transaction completes.
- **"Aux read ... timed out"** — the hub is not completing transactions at all.
  The hub is clocked by the accelerometer, so confirm the IMU is streaming
  normally first.

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
