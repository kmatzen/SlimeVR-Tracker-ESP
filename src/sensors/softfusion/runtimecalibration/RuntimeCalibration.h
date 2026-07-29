/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2024 Gorbit99 & SlimeVR Contributors

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
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/

#pragma once

#include <vector3.h>

#include <cstdint>

#include "../../../GlobalVars.h"
#include "../../../calibration.h"
#include "../../../configuration/Configuration.h"
#include "../../../configuration/accelmodel.h"
#include "../sixposition.h"
#include "AccelBiasCalibrationStep.h"
#include "GyroBiasCalibrationStep.h"
#include "MotionlessCalibrationStep.h"
#include "NullCalibrationStep.h"
#include "SampleRateCalibrationStep.h"
#include "configuration/SensorConfig.h"
#include "logging/Logger.h"
#include "sensors/SensorFusion.h"
#include "sensors/softfusion/CalibrationBase.h"

namespace SlimeVR::Sensors::RuntimeCalibration {

template <typename IMU>
class RuntimeCalibrator : public Sensors::CalibrationBase<IMU> {
public:
	static constexpr bool HasUpsideDownCalibration = false;

	using Base = Sensors::CalibrationBase<IMU>;
	using Self = RuntimeCalibrator<IMU>;
	using Consts = typename Base::Consts;
	using RawSensorT = typename Consts::RawSensorT;
	using RawVectorT = typename Consts::RawVectorT;

	RuntimeCalibrator(
		SensorFusion& fusion,
		IMU& imu,
		uint8_t sensorId,
		Logging::Logger& logger,
		SensorToggleState& toggles
	)
		: Base{fusion, imu, sensorId, logger, toggles} {
		calibration.T_Ts = Consts::getDefaultTempTs();
		activeCalibration.T_Ts = Consts::getDefaultTempTs();
	}

	bool calibrationMatches(const Configuration::SensorConfig& sensorCalibration
	) final {
		return sensorCalibration.type
				== SlimeVR::Configuration::SensorConfigType::RUNTIME_CALIBRATION
			&& (sensorCalibration.data.sfusion.ImuType == IMU::Type)
			&& (sensorCalibration.data.sfusion.MotionlessDataLen
				== Base::MotionlessCalibDataSize());
	}

	void assignCalibration(const Configuration::SensorConfig& sensorCalibration) final {
		calibration = sensorCalibration.data.runtimeCalibration;
		activeCalibration = sensorCalibration.data.runtimeCalibration;
		if (!toggles.getToggle(SensorToggles::CalibrationEnabled)) {
			activeCalibration.gyroPointsCalibrated = 0;
			for (size_t i = 0; i < 3; i++) {
				activeCalibration.G_off1[i] = 0;
				activeCalibration.G_off2[i] = 0;
			}

			for (size_t i = 0; i < 3; i++) {
				activeCalibration.accelCalibrated[i] = false;
				activeCalibration.A_off[i] = 0;
			}
		} else {
			calculateZROChange();
		}

		currentStep = &nullCalibrationStep;
	}

	void begin() final {
		startupMillis = millis();

		gyroBiasCalibrationStep.swapCalibrationIfNecessary();

		currentStep = &sampleRateCalibrationStep;
		currentStep->start();
		nextCalibrationStep = CalibrationStepEnum::SAMPLING_RATE;

		calculateZROChange();

		printCalibration();
	}

	/**
	 * Begins the guided six-position accelerometer calibration.
	 *
	 * Everything else in this class runs unprompted, in the background, while
	 * the tracker is being worn. This one cannot: the accelerometer error model
	 * needs the sensor held in six specific orientations, and no amount of
	 * patience will produce those on their own. So it is the one calibration a
	 * user has to be walked through, and the one that has to be asked for.
	 */
	void startCalibration(int calibrationType) final {
		if (calibrationType != CALIBRATION_TYPE_INTERNAL_ACCEL) {
			return;
		}

		// The normal step machine is suspended for the duration. Both want the
		// accelerometer stream and both want the tracker at rest, and the
		// guided flow is the one with a user waiting on it.
		currentStep->cancel();
		isCalibrating = false;

		sixPosition.begin();
		sixPositionLastProgressMillis = millis();

		logger.info("Guided accelerometer calibration started for sensor %d", sensorId);
		logger.info(
			"Hold the tracker still with each axis pointing up in turn; it will "
			"capture on its own"
		);
		promptForNextPosition();
	}

	void cancelCalibration() final {
		if (!sixPosition.isRunning()) {
			return;
		}
		sixPosition.abort();
		logger.info(
			"Guided accelerometer calibration cancelled; the stored calibration is "
			"unchanged"
		);
	}

	void tick() final {
		if (sixPosition.isRunning()) {
			// The only thing tick() does during the guided flow is give up. The
			// tracker is on a desk with someone watching a serial log, and a
			// procedure that waits forever for a position they have stopped
			// trying to reach is worse than one that says so.
			if (millis() - sixPositionLastProgressMillis
				> sixPositionTimeoutSeconds * 1e3) {
				logger.error(
					"Guided accelerometer calibration timed out after %d of %d "
					"positions",
					static_cast<int>(sixPosition.capturedCount()),
					static_cast<int>(SoftFusion::kSixPositionCount)
				);
				sixPosition.abort();
			}
			return;
		}

		if (sixPosition.getState()
			== SoftFusion::SixPositionCollector::State::Complete) {
			finishSixPositionCalibration();
			return;
		}

		if (skippedAStep && !lastTickRest && fusion.getRestDetected()) {
			computeNextCalibrationStep();
			skippedAStep = false;
		}

		if (millis() - startupMillis < initialStartupDelaySeconds * 1e3) {
			return;
		}

		if (!fusion.getRestDetected() && currentStep->requiresRest()) {
			if (isCalibrating) {
				currentStep->cancel();
				isCalibrating = false;
			}

			lastTickRest = fusion.getRestDetected();
			return;
		}

		if (!isCalibrating) {
			isCalibrating = true;
			currentStep->start();
		}

		if (currentStep->requiresRest() && !currentStep->restDetectionDelayElapsed()) {
			lastTickRest = fusion.getRestDetected();
			return;
		}

		auto result = currentStep->tick();

		switch (result) {
			case CalibrationStep<RawSensorT>::TickResult::DONE:
				if (nextCalibrationStep == CalibrationStepEnum::SAMPLING_RATE) {
					stepCalibrationForward(true, false);
					break;
				}
				stepCalibrationForward();
				break;
			case CalibrationStep<RawSensorT>::TickResult::SKIP:
				stepCalibrationForward(false, false);
				break;
			case CalibrationStep<RawSensorT>::TickResult::CONTINUE:
				break;
		}

		lastTickRest = fusion.getRestDetected();
	}

	void scaleAccelSample(sensor_real_t accelSample[3]) final {
		const sensor_real_t d[3] = {
			accelSample[0] * Consts::AScale - activeCalibration.A_off[0],
			accelSample[1] * Consts::AScale - activeCalibration.A_off[1],
			accelSample[2] * Consts::AScale - activeCalibration.A_off[2],
		};
		applyErrorMatrix(activeCalibration.A_M, d, accelSample);
	}

	float getAccelTimestep() final { return activeCalibration.A_Ts; }

	void scaleGyroSample(sensor_real_t gyroSample[3]) final {
		const sensor_real_t d[3] = {
			static_cast<sensor_real_t>(
				Consts::GScale * (gyroSample[0] - activeCalibration.G_off1[0])
			),
			static_cast<sensor_real_t>(
				Consts::GScale * (gyroSample[1] - activeCalibration.G_off1[1])
			),
			static_cast<sensor_real_t>(
				Consts::GScale * (gyroSample[2] - activeCalibration.G_off1[2])
			),
		};
		applyErrorMatrix(activeCalibration.G_M, d, gyroSample);
	}

	/**
	 * Applies the scale and misalignment matrix to an already bias-corrected
	 * and scaled sample.
	 *
	 * Identity is the overwhelmingly common case -- no device has a fitted
	 * model yet -- so it is short-circuited rather than paying nine multiplies
	 * and six adds per sample per sensor at up to 240 Hz.
	 */
	static void applyErrorMatrix(
		const float m[9],
		const sensor_real_t in[3],
		sensor_real_t out[3]
	) {
		// Treated as identity in two cases: an actual identity matrix, and an
		// all-zero diagonal.
		//
		// The second is the important one. A default-constructed config zeroes
		// this array, and a zero matrix would multiply every sample to zero --
		// silencing the sensor completely rather than degrading it. No
		// legitimate fitted model has a zero on the diagonal, since that would
		// describe an axis with no sensitivity at all, so this cannot mask a
		// real calibration.
		const bool unset = m[0] == 0.0f && m[4] == 0.0f && m[8] == 0.0f;
		const bool identity = m[0] == 1.0f && m[4] == 1.0f && m[8] == 1.0f
						   && m[1] == 0.0f && m[2] == 0.0f && m[3] == 0.0f
						   && m[5] == 0.0f && m[6] == 0.0f && m[7] == 0.0f;
		if (unset || identity) {
			out[0] = in[0];
			out[1] = in[1];
			out[2] = in[2];
			return;
		}
		out[0] = m[0] * in[0] + m[1] * in[1] + m[2] * in[2];
		out[1] = m[3] * in[0] + m[4] * in[1] + m[5] * in[2];
		out[2] = m[6] * in[0] + m[7] * in[1] + m[8] * in[2];
	}

	float getGyroTimestep() final { return activeCalibration.G_Ts; }

	float getTempTimestep() final { return activeCalibration.T_Ts; }

	const uint8_t* getMotionlessCalibrationData() final {
		return activeCalibration.MotionlessData;
	}

	void signalOverwhelmed() final {
		if (isCalibrating) {
			currentStep->signalOverwhelmed();
		}
	}

	void provideAccelSample(const RawSensorT accelSample[3]) final {
		if (sixPosition.isRunning()) {
			feedSixPosition(accelSample);
			return;
		}

		if (isCalibrating) {
			currentStep->processAccelSample(accelSample);
		}
	}

	void provideGyroSample(const RawSensorT gyroSample[3]) final {
		if (isCalibrating) {
			currentStep->processGyroSample(gyroSample);
		}
	}

	void provideTempSample(float tempSample) final {
		if (isCalibrating) {
			currentStep->processTempSample(tempSample);
		}
	}

	void calculateZROChange() {
		if (activeCalibration.gyroPointsCalibrated < 2) {
			activeZROChange = IMU::TemperatureZROChange;
		}

		float diffX = (activeCalibration.G_off2[0] - activeCalibration.G_off1[0])
					* Consts::GScale;
		float diffY = (activeCalibration.G_off2[1] - activeCalibration.G_off1[1])
					* Consts::GScale;
		float diffZ = (activeCalibration.G_off2[2] - activeCalibration.G_off1[2])
					* Consts::GScale;

		float maxDiff
			= std::max(std::max(std::abs(diffX), std::abs(diffY)), std::abs(diffZ));

		activeZROChange = 0.1f / maxDiff
						/ (activeCalibration.gyroMeasurementTemperature2
						   - activeCalibration.gyroMeasurementTemperature1);
	}

	float getZROChange() final { return activeZROChange; }

private:
	enum class CalibrationStepEnum {
		NONE,
		SAMPLING_RATE,
		MOTIONLESS,
		GYRO_BIAS,
		ACCEL_BIAS,
	};

	void computeNextCalibrationStep() {
		if (!calibration.motionlessCalibrated && Base::HasMotionlessCalib) {
			nextCalibrationStep = CalibrationStepEnum::MOTIONLESS;
			currentStep = &motionlessCalibrationStep;
		} else if (calibration.gyroPointsCalibrated == 0) {
			nextCalibrationStep = CalibrationStepEnum::GYRO_BIAS;
			currentStep = &gyroBiasCalibrationStep;
			// } else if (!accelBiasCalibrationStep.allAxesCalibrated()) {
			// 	nextCalibrationStep = CalibrationStepEnum::ACCEL_BIAS;
			// 	currentStep = &accelBiasCalibrationStep;
		} else {
			nextCalibrationStep = CalibrationStepEnum::GYRO_BIAS;
			currentStep = &gyroBiasCalibrationStep;
		}
	}

	void stepCalibrationForward(bool print = true, bool save = true) {
		currentStep->cancel();
		switch (nextCalibrationStep) {
			case CalibrationStepEnum::NONE:
				return;
			case CalibrationStepEnum::SAMPLING_RATE:
				nextCalibrationStep = CalibrationStepEnum::MOTIONLESS;
				currentStep = &motionlessCalibrationStep;
				if (print) {
					printCalibration(CalibrationPrintFlags::TIMESTEPS);
				}
				break;
			case CalibrationStepEnum::MOTIONLESS:
				nextCalibrationStep = CalibrationStepEnum::GYRO_BIAS;
				currentStep = &gyroBiasCalibrationStep;
				if (print) {
					printCalibration(CalibrationPrintFlags::MOTIONLESS);
				}
				break;
			case CalibrationStepEnum::GYRO_BIAS:
				if (calibration.gyroPointsCalibrated == 1) {
					// nextCalibrationStep = CalibrationStepEnum::ACCEL_BIAS;
					// currentStep = &accelBiasCalibrationStep;
					nextCalibrationStep = CalibrationStepEnum::GYRO_BIAS;
					currentStep = &gyroBiasCalibrationStep;
				}

				if (print) {
					printCalibration(CalibrationPrintFlags::GYRO_BIAS);
				}
				break;
			case CalibrationStepEnum::ACCEL_BIAS:
				nextCalibrationStep = CalibrationStepEnum::GYRO_BIAS;
				currentStep = &gyroBiasCalibrationStep;

				if (print) {
					printCalibration(CalibrationPrintFlags::ACCEL_BIAS);
				}

				if (!accelBiasCalibrationStep.allAxesCalibrated()) {
					skippedAStep = true;
				}
				break;
		}

		isCalibrating = false;

		if (save) {
			saveCalibration();
		}
	}

	void promptForNextPosition() {
		const int next = sixPosition.nextPosition();
		if (next < 0) {
			return;
		}
		logger.info(
			"Next: hold the tracker with %s (%d of %d captured)",
			SoftFusion::SixPositionCollector::positionName(next),
			static_cast<int>(sixPosition.capturedCount()),
			static_cast<int>(SoftFusion::kSixPositionCount)
		);
	}

	void feedSixPosition(const RawSensorT accelSample[3]) {
		// Uncorrected on purpose. The fit measures the error the sample path is
		// about to remove, so feeding it samples that already had a model
		// applied would fit the residual of the old model and compound the two.
		const float scaled[3] = {
			static_cast<float>(accelSample[0]) * static_cast<float>(Consts::AScale),
			static_cast<float>(accelSample[1]) * static_cast<float>(Consts::AScale),
			static_cast<float>(accelSample[2]) * static_cast<float>(Consts::AScale),
		};

		const int wasCapturing = sixPosition.activePosition();
		const auto event = sixPosition.feed(
			scaled,
			fusion.getRestDetected(),
			static_cast<float>(CONST_EARTH_GRAVITY)
		);

		switch (event) {
			case SoftFusion::SixPositionEvent::None:
				return;
			case SoftFusion::SixPositionEvent::Started:
				logger.info(
					"Capturing %s -- hold still",
					SoftFusion::SixPositionCollector::positionName(
						sixPosition.activePosition()
					)
				);
				break;
			case SoftFusion::SixPositionEvent::Disturbed:
				logger.info(
					"Movement during %s -- that position will be retried",
					SoftFusion::SixPositionCollector::positionName(wasCapturing)
				);
				break;
			case SoftFusion::SixPositionEvent::Captured:
				logger.info(
					"Captured %s",
					SoftFusion::SixPositionCollector::positionName(wasCapturing)
				);
				promptForNextPosition();
				break;
			case SoftFusion::SixPositionEvent::Complete:
				logger.info(
					"Captured %s -- all %d positions done, fitting",
					SoftFusion::SixPositionCollector::positionName(wasCapturing),
					static_cast<int>(SoftFusion::kSixPositionCount)
				);
				break;
		}

		sixPositionLastProgressMillis = millis();
	}

	/**
	 * Fits the collected positions and stores the result, or explains why not.
	 *
	 * Two independent refusals, and they mean different things. `fit` failing
	 * is about the *data*: the positions did not span three dimensions, so no
	 * model is recoverable. `checkAccelModel` failing is about the *answer*: a
	 * model was recovered and it describes a part no accelerometer could be.
	 * Either way the previous calibration survives, because a tracker that is
	 * slightly uncalibrated is worth much more than one confidently wrong.
	 */
	void finishSixPositionCalibration() {
		constexpr auto gravity = static_cast<float>(CONST_EARTH_GRAVITY);

		SoftFusion::ErrorModel model;
		const bool fitted = sixPosition.fit(gravity, model);
		sixPosition.abort();

		if (!fitted) {
			logger.error(
				"Accelerometer fit refused: the six positions did not span enough "
				"directions"
			);
			logger.info(
				"Hold each axis closer to vertical and run CALIBRATE ACCEL again"
			);
			return;
		}

		const auto status = Configuration::checkAccelModel(model, gravity);
		if (status != Configuration::AccelModelStatus::Ok) {
			logger.error(
				"Accelerometer fit rejected: %s",
				Configuration::accelModelStatusToString(status)
			);
			logger.info("The stored calibration is unchanged");
			return;
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
		saveCalibration();

		logger
			.info("Accel bias: %f %f %f", model.bias[0], model.bias[1], model.bias[2]);
		logger.info("Accel scale: %f %f %f", model.m[0], model.m[4], model.m[8]);

		// Applied without a reboot, unlike SET GYROSCALE. That command has to
		// defer because the value is typed in from a host tool and nothing
		// re-reads it; here the fit just ran against this sensor's own samples,
		// so the running copy can be updated in place.
		if (!toggles.getToggle(SensorToggles::CalibrationEnabled)) {
			logger.info(
				"Stored, but not applied: calibration is disabled for this sensor"
			);
			return;
		}
		for (size_t i = 0; i < 9; i++) {
			activeCalibration.A_M[i] = calibration.A_M[i];
			activeCalibration.G_M[i] = calibration.G_M[i];
		}
		for (size_t i = 0; i < 3; i++) {
			activeCalibration.A_off[i] = calibration.A_off[i];
			activeCalibration.accelCalibrated[i] = true;
		}
		activeCalibration.errorModelValid = true;
		logger.info("Accelerometer calibration applied");
	}

	void saveCalibration() {
		SlimeVR::Configuration::SensorConfig calibration{};
		calibration.type
			= SlimeVR::Configuration::SensorConfigType::RUNTIME_CALIBRATION;
		calibration.data.runtimeCalibration = this->calibration;
		configuration.setSensor(sensorId, calibration);
		configuration.save();
	}

	enum class CalibrationPrintFlags {
		TIMESTEPS = 1,
		MOTIONLESS = 2,
		GYRO_BIAS = 4,
		ACCEL_BIAS = 8,
	};

	static constexpr CalibrationPrintFlags PrintAllCalibration
		= CalibrationPrintFlags::TIMESTEPS | CalibrationPrintFlags::MOTIONLESS
		| CalibrationPrintFlags::GYRO_BIAS | CalibrationPrintFlags::ACCEL_BIAS;

	void printCalibration(CalibrationPrintFlags toPrint = PrintAllCalibration) {
		if (any(toPrint & CalibrationPrintFlags::TIMESTEPS)) {
			if (activeCalibration.sensorTimestepsCalibrated) {
				logger.info(
					"Calibrated timesteps: Accel %f, Gyro %f, Temperature %f",
					activeCalibration.A_Ts,
					activeCalibration.G_Ts,
					activeCalibration.T_Ts
				);
			} else {
				logger.info("Sensor timesteps not calibrated");
			}
		}

		if (Base::HasMotionlessCalib
			&& any(toPrint & CalibrationPrintFlags::MOTIONLESS)) {
			if (calibration.motionlessCalibrated) {
				logger.info("Motionless calibration done");
			} else {
				logger.info("Motionless calibration not done");
			}
		}

		if (any(toPrint & CalibrationPrintFlags::GYRO_BIAS)) {
			if (calibration.gyroPointsCalibrated != 0) {
				logger.info(
					"Calibrated gyro bias at %fC: %f %f %f",
					calibration.gyroMeasurementTemperature1,
					calibration.G_off1[0],
					calibration.G_off1[1],
					calibration.G_off1[2]
				);
			} else {
				logger.info("Gyro bias not calibrated");
			}

			if (calibration.gyroPointsCalibrated == 2) {
				logger.info(
					"Calibrated gyro bias at %fC: %f %f %f",
					calibration.gyroMeasurementTemperature2,
					calibration.G_off2[0],
					calibration.G_off2[1],
					calibration.G_off2[2]
				);
			}
		}

		if (any(toPrint & CalibrationPrintFlags::ACCEL_BIAS)) {
			if (accelBiasCalibrationStep.allAxesCalibrated()) {
				logger.info(
					"Calibrated accel bias: %f %f %f",
					calibration.A_off[0],
					calibration.A_off[1],
					calibration.A_off[2]
				);
			} else if (accelBiasCalibrationStep.anyAxesCalibrated()) {
				logger.info(
					"Partially calibrated accel bias: %f %f %f",
					calibration.A_off[0],
					calibration.A_off[1],
					calibration.A_off[2]
				);
			} else {
				logger.info("Accel bias not calibrated");
			}
		}
	}

	CalibrationStepEnum nextCalibrationStep = CalibrationStepEnum::SAMPLING_RATE;

	static constexpr float initialStartupDelaySeconds = 5;
	uint64_t startupMillis = millis();

	SampleRateCalibrationStep<RawSensorT> sampleRateCalibrationStep{activeCalibration};
	MotionlessCalibrationStep<IMU, RawSensorT> motionlessCalibrationStep{
		calibration,
		sensor
	};
	GyroBiasCalibrationStep<RawSensorT> gyroBiasCalibrationStep{calibration};
	AccelBiasCalibrationStep<RawSensorT> accelBiasCalibrationStep{
		calibration,
		static_cast<float>(Consts::AScale)
	};
	NullCalibrationStep<RawSensorT> nullCalibrationStep{calibration};

	CalibrationStep<RawSensorT>* currentStep = &nullCalibrationStep;

	/// Long enough to turn a tracker over six times with fumbling, short enough
	/// that an abandoned session does not leave the normal background
	/// calibration suspended indefinitely. Measured from the last progress, not
	/// from the start, so a slow user is never cut off mid-procedure.
	static constexpr float sixPositionTimeoutSeconds = 120;

	SoftFusion::SixPositionCollector sixPosition;
	uint32_t sixPositionLastProgressMillis = 0;

	bool isCalibrating = false;
	bool skippedAStep = false;
	bool lastTickRest = false;

	SlimeVR::Configuration::RuntimeCalibrationSensorConfig calibration{
		// let's create here transparent calibration that doesn't affect input data
		.ImuType = {IMU::Type},
		.MotionlessDataLen = {Base::MotionlessCalibDataSize()},

		.sensorTimestepsCalibrated = false,
		.A_Ts = IMU::AccTs,
		.G_Ts = IMU::GyrTs,
		.M_Ts = IMU::MagTs,
		.T_Ts = 0,

		.motionlessCalibrated = false,
		.MotionlessData = {},

		.gyroPointsCalibrated = 0,
		.gyroMeasurementTemperature1 = 0,
		.G_off1 = {0.0, 0.0, 0.0},
		.gyroMeasurementTemperature2 = 0,
		.G_off2 = {0.0, 0.0, 0.0},

		.accelCalibrated = {false, false, false},
		.A_off = {0.0, 0.0, 0.0},
	};

	float activeZROChange = 0;

	Configuration::RuntimeCalibrationSensorConfig activeCalibration = calibration;

	using Base::fusion;
	using Base::logger;
	using Base::sensor;
	using Base::sensorId;
	using Base::toggles;
};

}  // namespace SlimeVR::Sensors::RuntimeCalibration
