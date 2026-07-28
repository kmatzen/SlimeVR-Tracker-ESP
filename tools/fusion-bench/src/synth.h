// Synthetic dataset generation.
//
// The point of these is that the ground truth is known exactly: a body-frame
// angular velocity is defined analytically, integrated with the exponential map
// to produce the reference orientation, and the sensor readings are then
// derived from that reference and corrupted with a controlled error model.
// Nothing is inferred, so any error the harness reports is genuinely the
// estimator's.
//
// This is what makes the benchmark runnable in CI with no hardware, no captured
// data, and no external dataset download -- and it is also the only way to
// measure a specific error term (bias alone, scale factor alone) in isolation,
// which real captures cannot do because they contain every error at once.
#pragma once

#include <string>

#include "dataset.h"

namespace fb {

struct SynthParams {
	double durationSec = 60.0;
	double rateHz = 250.0;
	uint64_t seed = 1;

	// Error model. Defaults are representative of a consumer MEMS IMU after
	// the firmware's existing rest-gated bias calibration has run.
	double gyroBiasDps = 0.15;  // constant gyroscope bias, deg/s
	double gyroNoiseDps = 0.03;  // per-sample white noise, deg/s
	double gyroScaleErr = 0.0;  // fractional, e.g. 0.01 for 1%
	double accelBias = 0.02;  // constant accelerometer bias, m/s^2
	double accelNoise = 0.02;  // per-sample white noise, m/s^2

	// Emit a magnetometer column. The field is a clean, constant world-frame
	// vector -- useful for exercising the 9-DoF path, not for claiming anything
	// about real indoor magnetic environments.
	bool withMag = false;
};

// Known trajectories: "static", "static-tilted", "yaw-sweep", "tumble",
// "return-to-origin", "walk".
bool listTrajectories(std::string& out);
bool generate(
	const std::string& trajectory,
	const SynthParams& p,
	Dataset& out,
	std::string& error
);

}  // namespace fb
