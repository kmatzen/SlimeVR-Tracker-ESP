// Accelerometer noise floor, and the rest threshold it implies.
//
// `restThAcc` is the switch that turns rest-gated gyroscope bias estimation on
// and off, and bias estimation is the single most valuable thing in the
// calibration path -- on the Bench Test A capture it removes about three orders
// of magnitude of heading drift (0.4626 deg/s of bias, ~27 deg/min if left
// alone, down to 0.0187 deg/min). Setting the threshold below the
// accelerometer's noise floor turns that off, and turns it off *silently*:
// nothing in the logs says the calibration never ran.
//
// ## What the threshold is compared against
//
// Not a standard deviation over a window. VQF low-pass filters the
// accelerometer (2nd-order Butterworth, time constant `restFilterTau`) and
// compares the magnitude of the residual between the raw sample and that
// low-pass output against `restThAcc`, every sample:
//
//     |acc[n] - lowpass(acc)[n]|  >=  restThAcc   ->  restT = 0, not at rest
//     otherwise                                   ->  restT += accTs
//
// Rest is declared once `restT` reaches `restMinT`. So a single sample over the
// threshold resets the clock, and rest requires N = restMinT / accTs
// *consecutive* samples all under it. That makes the threshold a bound on the
// peak of the residual over a window, not on its RMS -- which is why the safe
// value sits several sigma above the noise floor rather than near it.
//
// ## The cliff is exactly computable
//
// Because rest needs one run of N consecutive samples under the threshold, the
// smallest threshold that admits rest at all is
//
//     min over all windows of ( max residual within that window )
//
// a sliding-window minimum-of-maximum over the residual series. That is not an
// estimate of the cliff; it is the cliff, and `requiredThreshold` reports it.
// `measuredCliffAgrees()` in the self-test checks it against the value found by
// bisecting actual runs.
//
// This is what makes the threshold derivable rather than guessable. The residual
// series comes from VQF itself, via getRelativeRestDeviations(), rather than
// from a reimplementation of the same filter here -- a second copy of the
// Butterworth would be free to disagree with the one that ships.
//
// ## Why the multiplier is not a constant
//
// It is tempting to write `restThAcc = k * noise` with a fixed k. Measured, k
// depends on the window length in samples, because a longer window is a longer
// run of draws that must all come in under the bound:
//
//     rate    restMinT   N      cliff / per-axis sigma
//     100 Hz  2.0 s      200    3.23
//     250 Hz  2.0 s      500    3.24
//     250 Hz  8.0 s      2000   3.82
//
// Over the accelerometer rates this firmware actually runs -- 100 Hz on BMI270
// and ICM42688, 120-250 Hz elsewhere -- k lands near 3.3. Above roughly 500 Hz
// the relationship departs sharply from anything the noise alone explains
// (35 sigma at 1 kHz with restMinT 8 s, where a Gaussian peak-of-N argument
// predicts about 4). VQF is compiled single-precision (`VQF_SINGLE_PRECISION`,
// vqf.h:13) and at 1 kHz the rest filter's normalised cutoff is fc/fs ~ 2e-4,
// so the low-pass stops tracking DC. Out of range for this firmware, recorded
// so the next person does not extrapolate the multiplier there.
//
// Prefer `requiredThreshold` over any multiplier. It needs no assumption about
// the noise being white, Gaussian, or stationary, and real accelerometer noise
// is none of those.
#pragma once

#include <cstddef>
#include <string>

#include "dataset.h"
#include "metrics.h"

namespace fb {

struct AccelNoiseResult {
	/// Per-axis white-noise sigma in m/s^2, estimated from successive
	/// differences: sigma = stddev(x[n] - x[n-1]) / sqrt(2). Deliberately
	/// independent of VQF's filter, so it stays meaningful if the filter
	/// changes, and insensitive to slow drift or a constant bias.
	double axisSigma[3] = {0, 0, 0};

	/// Vector-magnitude noise, sqrt(sum of per-axis variances). This is the
	/// quantity to compare against `restThAcc`, which bounds a 3-axis residual
	/// magnitude. Reported separately because per-axis and vector-magnitude
	/// figures differ by sqrt(3) and confusing them moves any derived threshold
	/// by 73%.
	double vectorSigma = 0;

	/// The statistic `restThAcc` is actually compared against: the magnitude of
	/// the accelerometer residual against VQF's rest low-pass. Taken from VQF.
	double residualRms = 0;
	double residualPeak = 0;

	/// Smallest `restThAcc` that admits rest anywhere on this capture -- the
	/// sliding-window minimum-of-maximum described above, over the whole
	/// residual series. Rest is impossible below this value and possible at or
	/// above it, which makes it an exact predictor of `first_rest_sec == -1`.
	double requiredThreshold = 0;

	/// The same bound, but ignoring the filter's startup transient.
	///
	/// These differ, and the difference matters. For the first `restFilterTau`
	/// seconds VQF's "low-pass output" is a plain running mean of every sample
	/// so far rather than a Butterworth output (`filterVec` in vqf.cpp), and a
	/// running mean over few samples sits very close to those samples -- the
	/// residual against the first sample is identically zero. So the startup
	/// window is artificially quiet, and a threshold can be low enough to catch
	/// rest there and nowhere else.
	///
	/// That is a real behaviour, not an artefact: such a tracker reports rest
	/// shortly after boot and then never again, because every settled window
	/// exceeds the threshold. Since it cannot recover rest after any motion, it
	/// is not a configuration worth shipping.
	///
	/// So: `requiredThreshold` is what predicts the observed cliff, and
	/// `settledThreshold` is the number to configure against.
	double settledThreshold = 0;

	/// Rest window in samples, N = restMinT / accTs.
	size_t restWindowSamples = 0;

	/// The `restThAcc` the measurement ran with, resolved from `BenchParams`
	/// (so `--stock` reports VQF's own default rather than the bench's).
	/// Reported here so callers do not have to repeat that resolution.
	double configuredThreshold = 0;

	/// Accelerometer rows seen. Rest cannot be reached at all when this is
	/// below `restWindowSamples`, however quiet the capture is.
	size_t accelSamples = 0;

	/// `settledThreshold` divided by `vectorSigma` -- the multiplier this
	/// capture implies, for comparison with the table above. Uses the settled
	/// bound, since that is the one a configuration should be derived from.
	double sigmaMultiple = 0;

	/// False when the capture cannot support the measurement at all.
	bool valid = false;

	/// Human-readable reason when `valid` is false.
	std::string reason;
};

/// Ratio of configured threshold to `requiredThreshold` below which a
/// configuration is reported as having no useful margin. A threshold only just
/// above the cliff admits rest on *this* capture and would lose it to a
/// slightly noisier part, a higher bandwidth setting, or a vibrating mount.
constexpr double kMinThresholdMargin = 2.0;

/// Fewest accelerometer rows worth measuring noise from.
constexpr size_t kMinNoiseSamples = 200;

/// Measures the noise floor and the implied threshold. `p` supplies the filter
/// configuration (`restFilterTau` via VQF's defaults, plus `restMinT` and
/// `restThAcc`), so the residual series is the one the configuration under test
/// would actually produce.
AccelNoiseResult measureAccelNoise(const Dataset& ds, const BenchParams& p);

}  // namespace fb
