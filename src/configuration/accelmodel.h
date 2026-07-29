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

#include "../sensors/softfusion/errormodel.h"

// Storing a fitted accelerometer error model into a sensor calibration.
//
// Split out with no Arduino dependency so it can be unit tested on a host --
// see tools/fusion-bench/tests/selftest.cpp. As with gyroscalecmd.h, the
// interesting part is not the assignment, it is everything that has to be
// refused or repaired first.

namespace SlimeVR::Configuration {

/**
 * Bounds on a plausible accelerometer error model.
 *
 * A six-position fit can succeed numerically and still be wrong -- held badly,
 * captured on a moving surface, or run on a sensor that was changing range
 * mid-capture. The result is then applied to every sample forever, so the
 * distance between "a poor calibration" and "a tracker that reads nonsense" is
 * only as wide as this check.
 *
 * The numbers describe what a real part can be, not what the fit can emit:
 *
 *  - **Scale** is trimmed at the factory to well under a percent on a good MEMS
 *    accelerometer and a few percent on a poor one. 10% is already generous.
 *  - **Misalignment** between die, package and board runs 0.5-2 degrees.
 *    0.1 as an off-diagonal term is about 5.7 degrees.
 *  - **Bias** is a few tenths of a m/s^2 on a part worth calibrating. 10% of
 *    gravity is roughly 1 m/s^2.
 *
 * A fit outside these is not a badly calibrated tracker, it is a failed
 * measurement, and the honest response is to keep the previous calibration and
 * say so.
 */
constexpr float kAccelScaleMin = 0.90f;
constexpr float kAccelScaleMax = 1.10f;
constexpr float kAccelMisalignmentMax = 0.10f;
constexpr float kAccelBiasFraction = 0.10f;

enum class AccelModelStatus {
	Ok,
	/// A non-finite entry: the fit diverged.
	NotFinite,
	/// A diagonal term outside [kAccelScaleMin, kAccelScaleMax].
	ScaleOutOfRange,
	/// An off-diagonal term above kAccelMisalignmentMax.
	MisalignmentOutOfRange,
	/// A bias term above kAccelBiasFraction of the expected magnitude.
	BiasOutOfRange,
};

/**
 * Checks a fitted model against what a real accelerometer can plausibly be.
 *
 * @param norm  Expected magnitude the model was fitted against, e.g. gravity.
 */
inline AccelModelStatus
checkAccelModel(const SlimeVR::Sensors::SoftFusion::ErrorModel& model, float norm) {
	for (int i = 0; i < 9; i++) {
		if (!std::isfinite(model.m[i])) {
			return AccelModelStatus::NotFinite;
		}
	}
	for (int i = 0; i < 3; i++) {
		if (!std::isfinite(model.bias[i])) {
			return AccelModelStatus::NotFinite;
		}
	}

	static constexpr int diagonal[3] = {0, 4, 8};
	for (const int i : diagonal) {
		if (model.m[i] < kAccelScaleMin || model.m[i] > kAccelScaleMax) {
			return AccelModelStatus::ScaleOutOfRange;
		}
	}

	static constexpr int offDiagonal[6] = {1, 2, 3, 5, 6, 7};
	for (const int i : offDiagonal) {
		if (std::fabs(model.m[i]) > kAccelMisalignmentMax) {
			return AccelModelStatus::MisalignmentOutOfRange;
		}
	}

	for (int i = 0; i < 3; i++) {
		if (std::fabs(model.bias[i]) > kAccelBiasFraction * norm) {
			return AccelModelStatus::BiasOutOfRange;
		}
	}

	return AccelModelStatus::Ok;
}

inline const char* accelModelStatusToString(AccelModelStatus status) {
	switch (status) {
		case AccelModelStatus::Ok:
			return "ok";
		case AccelModelStatus::NotFinite:
			return "the fit diverged";
		case AccelModelStatus::ScaleOutOfRange:
			return "a scale factor is outside 0.90..1.10";
		case AccelModelStatus::MisalignmentOutOfRange:
			return "a misalignment term exceeds 0.10";
		case AccelModelStatus::BiasOutOfRange:
			return "a bias exceeds 10% of gravity";
	}
	return "unknown";
}

/**
 * Whether an online estimate is allowed to overwrite what is stored.
 *
 * A deliberate calibration always wins. `CALIBRATE ACCEL` and `SET GYROSCALE`
 * are things a user chose to do, and an estimator quietly undoing them would be
 * the worst kind of surprise -- the tracker would appear to drift back to where
 * it was before the user fixed it.
 *
 * The estimator may, however, replace *its own* previous estimate. That is the
 * whole point of it being online: it has to keep up with a bias that moves.
 * Without the provenance flag the two cases are indistinguishable, and the
 * estimator would apply once and then be locked out by the flag it had just
 * set.
 */
inline bool onlineEstimateMayApply(bool errorModelValid, bool errorModelFromOnline) {
	return !errorModelValid || errorModelFromOnline;
}

/**
 * Whether a new estimate differs enough from the stored one to be worth saving.
 *
 * Not a numerical nicety -- a flash-wear guard. The estimate moves a little
 * with every observation, and a tracker that persisted each one would write to
 * LittleFS several times a day for its whole life. The thresholds are the point
 * below which a difference cannot matter to tracking: 0.01 m/s^2 is a thousandth
 * of gravity, and 0.001 of scale is a tenth of the smallest error worth
 * correcting.
 *
 * The applied (RAM) copy is updated regardless; only persistence waits for a
 * material change.
 */
inline bool accelModelDiffersMaterially(
	const SlimeVR::Sensors::SoftFusion::ErrorModel& model,
	const float storedM[9],
	const float storedBias[3]
) {
	constexpr float biasEpsilon = 0.01f;
	constexpr float scaleEpsilon = 0.001f;
	for (int i = 0; i < 3; i++) {
		if (std::fabs(model.bias[i] - storedBias[i]) > biasEpsilon) {
			return true;
		}
	}
	static constexpr int diagonal[3] = {0, 4, 8};
	for (const int i : diagonal) {
		if (std::fabs(model.m[i] - storedM[i]) > scaleEpsilon) {
			return true;
		}
	}
	return false;
}

/**
 * Writes a fitted accelerometer model into a sensor calibration.
 *
 * The bias goes to `aOff` and the matrix to `aM`, which is exactly how the
 * sample path consumes them: `corrected = A_M * (raw * AScale - A_off)`.
 *
 * `gM` is the reason this is a function rather than six assignments. Storing an
 * accelerometer model means setting `errorModelValid`, and that flag governs
 * *both* matrices. A sensor config that has never had a model fitted carries an
 * all-zero `G_M`; harmless while the flag is false, because the reader
 * substitutes identity, but the moment the flag is set that zero matrix is live
 * and multiplies every gyroscope sample to zero. The tracker would hold a fixed
 * orientation and read as a hardware fault.
 *
 * This is the mirror image of the hazard `buildGyroScaleModel` guards, and it
 * is the same rule: `errorModelValid` is one flag over two matrices, so
 * whichever one you are not writing has to be made safe on the way past.
 */
inline void storeAccelModel(
	const SlimeVR::Sensors::SoftFusion::ErrorModel& model,
	bool errorModelWasValid,
	float aM[9],
	float gM[9],
	float aOff[3],
	bool accelCalibrated[3]
) {
	constexpr float identity[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};

	if (!errorModelWasValid) {
		for (int i = 0; i < 9; i++) {
			gM[i] = identity[i];
		}
	}

	for (int i = 0; i < 9; i++) {
		aM[i] = model.m[i];
	}
	for (int i = 0; i < 3; i++) {
		aOff[i] = model.bias[i];
		accelCalibrated[i] = true;
	}
}

}  // namespace SlimeVR::Configuration
