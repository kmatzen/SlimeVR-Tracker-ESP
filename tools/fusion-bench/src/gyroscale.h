// Gyroscope scale-factor estimation from a raw capture.
//
// Scale factor is the error term rest-gated bias calibration cannot see. Bias
// dominates a stationary sensor and is measured at rest; scale factor produces
// error proportional to rotation *angle* and is therefore invisible at rest,
// because there is no rotation to scale. A 1% error puts you 1.8 degrees out
// after a 180 degree turn, however good the bias estimate is.
//
// ## The reference
//
// The turntable protocol in README.md measures this against a known mechanical
// rotation. That works but needs a fixture. This estimator instead uses a
// reference every capture already contains: gravity.
//
// Whenever the tracker is at rest, the accelerometer gives the gravity
// direction in the body frame exactly. Between two rest periods the gyroscope
// says how far it thinks the body turned, and that prediction can be checked
// against where gravity actually ended up:
//
//     g_end_predicted = dR' * g_start
//
// A scale error makes dR too large or too small, so the predicted gravity
// misses. Fitting the scale that minimises that miss, across many rest-to-rest
// transitions, recovers the scale factor with no fixture at all.
//
// ## What it cannot see
//
// Rotation *about* the gravity vector does not move gravity, so it contributes
// nothing. A capture of a tracker spun about the vertical axis while sitting
// flat carries no scale information whatsoever, however long it runs.
//
// This is the same shape of problem as the kinematic heading estimator: the
// geometry sometimes says nothing, and the honest response is to report that
// rather than return a confident number built from noise. Observability is
// measured per axis and reported, and `valid` is false when the data does not
// constrain the fit.
#pragma once

#include <cstddef>
#include <string>

#include "dataset.h"

namespace fb {

struct GyroScaleResult {
	/// Multiplier to apply to measured rate: corrected = scale * measured.
	/// Below 1 means the gyroscope reads high.
	double scale[3] = {1.0, 1.0, 1.0};

	/// How strongly the data constrains each axis, as the rise in residual
	/// produced by a 1% perturbation of that axis, in degrees. Near zero means
	/// the axis was never exercised in a way that moves gravity.
	double observability[3] = {0, 0, 0};

	/// Mean gravity-prediction error before and after fitting, degrees.
	double residualBeforeDeg = 0;
	double residualAfterDeg = 0;

	/// Rest-to-rest transitions used.
	int segments = 0;

	/// True when enough segments were found and every axis is observable.
	bool valid = false;

	/// Human-readable reason when `valid` is false.
	std::string reason;
};

/// Minimum rest-to-rest transitions before a fit is attempted.
constexpr int kMinScaleSegments = 4;

/// Minimum per-axis observability, in degrees of residual per 1% of scale.
/// Below this the axis is not meaningfully constrained by the capture.
constexpr double kMinScaleObservability = 0.05;

/// Rest periods shorter than this are ignored; gravity needs a settled reading.
constexpr double kRestSegmentSeconds = 0.3;

/// Transitions turning less than this carry too little signal to be worth using.
constexpr double kMinTransitionDeg = 20.0;

GyroScaleResult estimateGyroScale(const Dataset& ds);

}  // namespace fb
