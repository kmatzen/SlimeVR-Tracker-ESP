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

#include <cstdint>

#include "../../../configuration/SensorConfig.h"
#include "../../../configuration/accelmodel.h"
#include "../../../consts.h"
#include "../sixposition.h"
#include "logging/Logger.h"

// The parts of the guided six-position flow that do not depend on which IMU is
// fitted -- which is nearly all of it.
//
// `RuntimeCalibrator` is a template over the IMU driver, so anything written
// inside it is emitted once per driver. That is the right trade for the sample
// path, where the scale factor and raw sample type genuinely differ, and the
// wrong one for prompting a user and fitting a model, where nothing differs at
// all. Left in the template this flow cost about 1.7 kB per driver, and the
// boards that instantiate the most drivers are the ones with the least flash to
// spare: BOARD_GLOVE_IMU_SLIMEVR_DEV sits at 99.8% full before this feature
// exists.
//
// So the driver-dependent shim stays in the template -- scaling a raw sample,
// reading the clock, saving through the sensor's own config slot -- and
// everything else lives here, once.

namespace SlimeVR::Sensors::RuntimeCalibration::SixPositionReport {

constexpr float kGravity = static_cast<float>(CONST_EARTH_GRAVITY);

inline void promptForNext(
	Logging::Logger& logger,
	const SoftFusion::SixPositionCollector& collector
) {
	const int next = collector.nextPosition();
	if (next < 0) {
		return;
	}
	logger.info(
		"Next: hold the tracker with %s (%d of %d captured)",
		SoftFusion::SixPositionCollector::positionName(next),
		static_cast<int>(collector.capturedCount()),
		static_cast<int>(SoftFusion::kSixPositionCount)
	);
}

inline void logStarted(Logging::Logger& logger, uint8_t sensorId) {
	logger.info("Guided accelerometer calibration started for sensor %d", sensorId);
	logger.info(
		"Hold the tracker still with each axis pointing up in turn; it will "
		"capture on its own"
	);
}

inline void logCancelled(Logging::Logger& logger) {
	logger.info(
		"Guided accelerometer calibration cancelled; the stored calibration is "
		"unchanged"
	);
}

inline void logTimedOut(
	Logging::Logger& logger,
	const SoftFusion::SixPositionCollector& collector
) {
	logger.error(
		"Guided accelerometer calibration timed out after %d of %d positions",
		static_cast<int>(collector.capturedCount()),
		static_cast<int>(SoftFusion::kSixPositionCount)
	);
}

/// Narrates one collector event. `wasCapturing` is the position that was active
/// before the sample was offered, which is the one an event refers to.
inline void logEvent(
	Logging::Logger& logger,
	SoftFusion::SixPositionEvent event,
	int wasCapturing,
	const SoftFusion::SixPositionCollector& collector
) {
	switch (event) {
		case SoftFusion::SixPositionEvent::None:
			return;
		case SoftFusion::SixPositionEvent::Started:
			logger.info(
				"Capturing %s -- hold still",
				SoftFusion::SixPositionCollector::positionName(collector.activePosition(
				))
			);
			return;
		case SoftFusion::SixPositionEvent::Disturbed:
			logger.info(
				"Movement during %s -- that position will be retried",
				SoftFusion::SixPositionCollector::positionName(wasCapturing)
			);
			return;
		case SoftFusion::SixPositionEvent::Captured:
			logger.info(
				"Captured %s",
				SoftFusion::SixPositionCollector::positionName(wasCapturing)
			);
			promptForNext(logger, collector);
			return;
		case SoftFusion::SixPositionEvent::Complete:
			logger.info(
				"Captured %s -- all %d positions done, fitting",
				SoftFusion::SixPositionCollector::positionName(wasCapturing),
				static_cast<int>(SoftFusion::kSixPositionCount)
			);
			return;
	}
}

/**
 * Fits the collected positions and writes the result into `calibration`, or
 * explains why not.
 *
 * Two independent refusals, and they mean different things. `fit` failing is
 * about the *data*: the positions did not determine a model. `checkAccelModel`
 * failing is about the *answer*: a model was determined and it describes a part
 * no accelerometer could be. Either way the previous calibration survives,
 * because a tracker that is slightly uncalibrated is worth much more than one
 * confidently wrong.
 *
 * The collector is stopped either way, so the caller cannot loop on it.
 *
 * @return true if `calibration` was modified and should be saved.
 */
inline bool fitAndStore(
	Logging::Logger& logger,
	SoftFusion::SixPositionCollector& collector,
	Configuration::RuntimeCalibrationSensorConfig& calibration
) {
	SoftFusion::ErrorModel model;
	const bool fitted = collector.fit(kGravity, model);
	collector.abort();

	if (!fitted) {
		logger.error(
			"Accelerometer fit refused: the six positions did not determine a model"
		);
		logger.info("Hold each axis closer to vertical and run CALIBRATE ACCEL again");
		return false;
	}

	const auto status = Configuration::checkAccelModel(model, kGravity);
	if (status != Configuration::AccelModelStatus::Ok) {
		logger.error(
			"Accelerometer fit rejected: %s",
			Configuration::accelModelStatusToString(status)
		);
		logger.info("The stored calibration is unchanged");
		return false;
	}

	Configuration::storeAccelModel(
		model,
		calibration.errorModelValid,
		calibration.A_M,
		calibration.G_M,
		calibration.A_off,
		calibration.accelCalibrated
	);
	calibration.errorModelValid = true;
	// Deliberate: from here on the online estimator defers to this rather than
	// refreshing it, because a user chose to run the procedure.
	calibration.errorModelFromOnline = false;

	logger.info("Accel bias: %f %f %f", model.bias[0], model.bias[1], model.bias[2]);
	logger.info("Accel scale: %f %f %f", model.m[0], model.m[4], model.m[8]);
	return true;
}

/**
 * Copies a freshly fitted model into the running calibration.
 *
 * Applied without a reboot, unlike `SET GYROSCALE`. That command has to defer
 * because the value is typed in from a host tool and nothing re-reads it; here
 * the fit just ran against this sensor's own samples, so the running copy can
 * be updated in place.
 */
inline void applyToActive(
	Logging::Logger& logger,
	const Configuration::RuntimeCalibrationSensorConfig& calibration,
	Configuration::RuntimeCalibrationSensorConfig& active,
	bool announce = true
) {
	for (size_t i = 0; i < 9; i++) {
		active.A_M[i] = calibration.A_M[i];
		active.G_M[i] = calibration.G_M[i];
	}
	for (size_t i = 0; i < 3; i++) {
		active.A_off[i] = calibration.A_off[i];
		active.accelCalibrated[i] = true;
	}
	active.errorModelValid = true;
	// The online path applies repeatedly and quietly; only a deliberate
	// calibration is worth a line in the log every time.
	if (announce) {
		logger.info("Accelerometer calibration applied");
	}
}

inline void logStoredButNotApplied(Logging::Logger& logger) {
	logger.info("Stored, but not applied: calibration is disabled for this sensor");
}

}  // namespace SlimeVR::Sensors::RuntimeCalibration::SixPositionReport
