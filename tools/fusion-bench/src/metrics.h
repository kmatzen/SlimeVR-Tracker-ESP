// Runs VQF over a dataset and reduces the result to a small set of numbers.
//
// Two families of metric are produced:
//
//   * Ground-truth metrics, when the dataset carries a reference orientation.
//     These are the standard decomposition used in the orientation-estimation
//     literature (total / heading / inclination), computed after removing the
//     single best constant heading offset -- a 6-DoF estimator has no absolute
//     heading reference, so its absolute yaw is arbitrary and penalising it
//     would measure nothing.
//
//   * Self-consistency metrics, which need no reference at all and are what the
//     desk-bench protocol in README.md produces. Heading drift is the headline
//     number: it is what determines how often a user has to reset.
#pragma once

#include <string>
#include <vector>

#include "dataset.h"
#include "quatmath.h"

namespace fb {

struct RunResult {
	std::vector<Quat> est;
	// Wall-clock offset, in seconds from the first sample, at which VQF first
	// reported rest. Negative if rest was never detected.
	double firstRestSec = -1;
	// Final gyroscope bias estimate, deg/s.
	Vec3 finalBiasDps;
	// Fraction of samples during which VQF reported a magnetic disturbance.
	double magDistFraction = 0;
	bool usedMag = false;
};

struct Metrics {
	std::string dataset;
	size_t sampleCount = 0;
	double durationSec = 0;
	double sampleRateHz = 0;

	// Self-consistency (no reference required).
	//
	// Heading drift and jitter are only meaningful while the tracker is
	// stationary -- fitting a line to the yaw of a tracker that is genuinely
	// rotating measures the motion, not the error. Both are therefore computed
	// over the longest contiguous rest segment, and are only reported when that
	// segment is long enough to support an estimate.
	bool hasDriftEstimate = false;
	double restSecondsUsed = 0;
	double headingDriftDegPerMin = 0;
	double jitterDegRms = 0;

	// Tilt is computed over every sample. It is exact at rest and degrades
	// under linear acceleration, which is itself informative.
	double tiltErrorDegRms = 0;
	double tiltErrorDegMax = 0;

	double firstRestSec = -1;
	double finalBiasDps = 0;

	// Reference-based. Only meaningful when hasGroundTruth is true.
	bool hasGroundTruth = false;
	double totalErrorDegRms = 0;
	double totalErrorDegMax = 0;
	double headingErrorDegRms = 0;
	double inclinationErrorDegRms = 0;
	// Heading error on the final sample. For a trajectory whose net rotation is
	// zero this is the accumulated-error number the physical return-to-origin
	// bench test produces.
	double finalHeadingErrorDeg = 0;
};

// Rest detection needs two criteria together, and neither works alone:
//
//   * Variation about the local mean. Necessary because a stationary tracker
//     with an uncorrected bias reads a constant non-zero rate, and a pure
//     magnitude test would classify exactly the trackers we most want to
//     measure as "moving".
//
//   * Absolute magnitude. Necessary because a *constant-rate* rotation also has
//     near-zero variation, so a pure variation test would classify a steadily
//     turning tracker as stationary.
//
// The magnitude bound is loose enough to admit any plausible uncorrected bias
// and far below sustained real motion; the variation bound then rejects the
// slow steady turns that would otherwise slip under it.
constexpr double kRestVariationDps = 0.5;
constexpr double kRestMagnitudeDps = 5.0;
// Shortest rest segment from which a drift rate will be estimated.
constexpr double kMinDriftSegmentSec = 5.0;

// VQF tuning knobs the bench is allowed to vary.
//
// Defaults mirror SlimeVR::Sensors::DefaultVQFParams. Note what that does *not*
// mean: those tuned parameters are unreachable in the firmware, so this is a
// hypothetical configuration rather than any shipped device's. The convenience
// SensorFusion constructor that reads them is only called under
// `#if !MPU_USE_DMPMAG`, and MPU_USE_DMPMAG is hardcoded to 1 -- MPU9250 runs
// SensorFusionDMP, which has no VQF in it at all.
//
// So `--stock` is the setting that measures what every real device does, and the
// default measures the alternative. Both are worth keeping: the delta between
// them is the acceptance test for issue #4.
struct BenchParams {
	double tauAcc = 2.0;
	double restMinT = 2.0;
	double restThGyr = 0.6;
	double restThAcc = 0.06;
	bool useMag = false;
	bool stock = false;  // ignore the above, use VQF's own defaults
};

RunResult runFusion(const Dataset& ds, const BenchParams& p);
Metrics computeMetrics(const Dataset& ds, const RunResult& run);
std::string metricsToJson(const Metrics& m);

}  // namespace fb
