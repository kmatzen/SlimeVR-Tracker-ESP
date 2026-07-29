/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2026 SlimeVR Contributors

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
	FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
	IN THE SOFTWARE.
*/

#pragma once

#include <cmath>
#include <cstdlib>

// Validation for the `SET GYROSCALE` serial command.
//
// Split out from serialcommands.cpp with no Arduino dependency so it can be
// unit tested on a host -- see tools/fusion-bench/tests/selftest.cpp. The
// command itself is a handful of lines; what is worth testing is everything it
// has to refuse.

namespace SlimeVR::Configuration {

/**
 * Bounds on an acceptable gyroscope scale factor.
 *
 * These match the range the offline estimator searches
 * (tools/fusion-bench, `gyro-scale`), which is deliberate: a value outside it
 * is not something the estimator could have produced, so it is a typo rather
 * than a measurement. A misplaced decimal point -- `10` for `1.0` -- would
 * otherwise be accepted and would make the tracker's output look like a
 * hardware fault rather than a bad calibration.
 *
 * Real MEMS gyroscope scale error after factory trim is well under a percent
 * for a good part and a few percent for a poor one, so 10% is already generous.
 */
constexpr float kGyroScaleMin = 0.90f;
constexpr float kGyroScaleMax = 1.10f;

enum class GyroScaleStatus {
	Ok,
	/// Not a number at all, or trailing garbage after one.
	Unparseable,
	/// Parsed, but outside [kGyroScaleMin, kGyroScaleMax].
	OutOfRange,
};

/**
 * Strict float parse.
 *
 * `atof` returns 0.0 for unparseable input, which here would be silently
 * treated as "scale factor zero" -- in range as far as a naive check is
 * concerned only because zero is not, but the failure mode is worth closing
 * explicitly rather than relying on the range test to catch it. Trailing
 * characters are rejected too, so `1.0x` does not become `1.0`.
 */
inline bool parseGyroScaleValue(const char* text, float& out) {
	if (text == nullptr || *text == '\0') {
		return false;
	}
	char* end = nullptr;
	const float value = std::strtof(text, &end);
	if (end == text || end == nullptr || *end != '\0') {
		return false;
	}
	if (!std::isfinite(value)) {
		return false;
	}
	out = value;
	return true;
}

/// Parses three axis values, reporting which one failed via `badAxis` (0..2).
inline GyroScaleStatus
parseGyroScale(const char* const text[3], float out[3], int& badAxis) {
	for (int i = 0; i < 3; i++) {
		float value = 0;
		if (!parseGyroScaleValue(text[i], value)) {
			badAxis = i;
			return GyroScaleStatus::Unparseable;
		}
		if (value < kGyroScaleMin || value > kGyroScaleMax) {
			badAxis = i;
			out[i] = value;
			return GyroScaleStatus::OutOfRange;
		}
		out[i] = value;
	}
	badAxis = -1;
	return GyroScaleStatus::Ok;
}

/**
 * True if a stored gyroscope matrix carries cross-axis (misalignment) terms.
 *
 * Only meaningful when the stored model is valid; an all-zero matrix from a
 * never-calibrated config has no off-diagonal content to lose.
 */
inline bool gyroModelHasMisalignment(const float gM[9]) {
	static constexpr int offDiagonal[6] = {1, 2, 3, 5, 6, 7};
	for (const int i : offDiagonal) {
		if (gM[i] != 0.0f) {
			return true;
		}
	}
	return false;
}

/**
 * Fills in the two error-model matrices for a gyroscope-scale-only calibration.
 *
 * `gM` becomes diag(scale) exactly -- any previous cross-axis terms are
 * dropped, because the offline estimator produces three scale factors and has
 * nothing to say about misalignment. Overwriting is the predictable choice, but
 * it can silently discard part of a fitted model, so the return value reports
 * whether that happened and the caller is expected to say so out loud.
 *
 * `aM` is left alone if the stored model was already valid, and is otherwise
 * set to identity. That part is not defensive padding, it is the main reason
 * this function exists. Storing a gyroscope scale means setting
 * `errorModelValid`, and that flag governs *both* matrices. A sensor config
 * that has never had a model fitted carries an all-zero `A_M` -- harmless while
 * the flag is false, because the reader substitutes identity, but the moment
 * the flag is set that zero matrix becomes live and multiplies every
 * accelerometer sample to zero. The tracker would go dead in a way that reads
 * as a hardware fault.
 *
 * @return true if fitted misalignment terms were discarded.
 */
inline bool buildGyroScaleModel(
	const float scale[3],
	bool errorModelWasValid,
	float aM[9],
	float gM[9]
) {
	constexpr float identity[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};

	if (!errorModelWasValid) {
		for (int i = 0; i < 9; i++) {
			aM[i] = identity[i];
		}
	}

	const bool discarded = errorModelWasValid && gyroModelHasMisalignment(gM);

	for (int i = 0; i < 9; i++) {
		gM[i] = identity[i];
	}
	gM[0] = scale[0];
	gM[4] = scale[1];
	gM[8] = scale[2];

	return discarded;
}

}  // namespace SlimeVR::Configuration
