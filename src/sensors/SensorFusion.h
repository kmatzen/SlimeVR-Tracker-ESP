#ifndef SLIMEVR_SENSORFUSION_H
#define SLIMEVR_SENSORFUSION_H

#include "globals.h"
#include "sensor.h"

#define SENSOR_DOUBLE_PRECISION 0

#define SENSOR_FUSION_TYPE_STRING "vqf"

#include <vqf.h>

#include "../motionprocessing/types.h"

namespace SlimeVR::Sensors {

/// Tuned VQF parameters. **Currently unreachable -- no shipped configuration
/// uses these**, which is worth knowing before tuning them further.
///
/// They are read only by the convenience `SensorFusion(gyrTs, ...)` constructor
/// below, and the single call site of that constructor is guarded by
/// `#if !MPU_USE_DMPMAG` (mpu9250sensor.h), while `MPU_USE_DMPMAG` is defined to
/// 1 unconditionally in the same header and is not overridable from
/// platformio.ini. MPU9250, MPU6050 and ICM20948 therefore all run
/// `SensorFusionDMP`, which contains a `DMPMag` and no VQF at all. Every IMU
/// that does run VQF goes through the softfusion path, which passes
/// `SensorType::SensorVQFParams` -- and all ten drivers define that as
/// `VQFParams{}`, i.e. the library's own defaults.
///
/// So the live configuration everywhere is `lib/vqf/vqf.h`'s defaults, not this.
/// Note those are not upstream VQF's defaults either: they were replaced by an
/// optimizer run (see the header) and are themselves tuned.
///
/// ## Units and the one constraint that matters
///
/// `tauAcc` seconds, `restMinT` seconds, `restThGyr` deg/s, `restThAcc` m/s^2.
///
/// `restThAcc` bounds the magnitude of the accelerometer residual against VQF's
/// rest low-pass, checked every sample; a single sample over it resets the rest
/// timer, so rest requires `restMinT / accTs` *consecutive* samples under it.
/// Set it below the accelerometer's noise floor and rest is never detected, so
/// rest-gated gyroscope bias estimation never runs -- and that estimation is
/// worth about three orders of magnitude of heading drift (0.4626 deg/s of
/// measured bias, ~27 deg/min uncorrected, versus 0.0187 deg/min with it
/// running). The failure is silent: nothing in the logs reports it. The symptom
/// is `first_rest_sec == -1` in a fusion-bench replay.
///
/// Because it is a bound on a peak over a window rather than on an RMS, the safe
/// value sits well above the noise floor -- measured at about 3.3x the per-axis
/// sigma, equivalently 1.9x the vector-magnitude sigma, at the accelerometer
/// rates this firmware runs (100-250 Hz). The multiplier is not a constant: it
/// grows with the rest window in samples, so derive it rather than assuming it.
/// `fusion-bench noise CAPTURE.csv` reports the noise floor, the exact minimum
/// threshold a capture admits, and the margin of the configured value.
///
/// Measured margins on an LSM6DSV (per-axis sigma 0.0034/0.0032/0.0051 m/s^2,
/// accelerometer at 120 Hz): the live default of 1.418598 sits about 60x above
/// the cliff, and the 0.06 here would sit about 2.7x above it -- above, but
/// thinly enough that a noisier part, a higher bandwidth setting, or a vibrating
/// mount could cross it. That thinness, not a measured regression, is the reason
/// not to propagate these to the softfusion drivers; see issue #4.
constexpr VQFParams DefaultVQFParams = VQFParams{
	.tauAcc = 2.0f,
	.restMinT = 2.0f,
	.restThGyr = 0.6f,
	.restThAcc = 0.06f,
};

class SensorFusion {
public:
	SensorFusion(
		VQFParams vqfParams,
		sensor_real_t gyrTs,
		sensor_real_t accTs = -1.0,
		sensor_real_t magTs = -1.0
	)
		: gyrTs(gyrTs)
		, accTs((accTs < 0) ? gyrTs : accTs)
		, magTs((magTs < 0) ? gyrTs : magTs)
		, vqfParams(vqfParams)
		, vqf(this->vqfParams,
			  gyrTs,
			  ((accTs < 0) ? gyrTs : accTs),
			  ((magTs < 0) ? gyrTs : magTs)) {}

	explicit SensorFusion(
		sensor_real_t gyrTs,
		sensor_real_t accTs = -1.0,
		sensor_real_t magTs = -1.0
	)
		: SensorFusion(DefaultVQFParams, gyrTs, accTs, magTs) {}

	void update6D(
		sensor_real_t Axyz[3],
		sensor_real_t Gxyz[3],
		sensor_real_t deltat = -1.0f
	);
	void update9D(
		sensor_real_t Axyz[3],
		sensor_real_t Gxyz[3],
		sensor_real_t Mxyz[3],
		sensor_real_t deltat = -1.0f
	);
	void updateAcc(const sensor_real_t Axyz[3], sensor_real_t deltat = -1.0f);
	void updateMag(const sensor_real_t Mxyz[3], sensor_real_t deltat = -1.0f);
	void updateGyro(const sensor_real_t Gxyz[3], sensor_real_t deltat = -1.0f);

	bool isUpdated();
	void clearUpdated();
	sensor_real_t const* getQuaternion();
	Quat getQuaternionQuat();
	sensor_real_t const* getGravityVec();
	sensor_real_t const* getLinearAcc();
	void getLinearAcc(sensor_real_t outLinAccel[3]);
	Vector3 getLinearAccVec();

	static void calcGravityVec(const sensor_real_t qwxyz[4], sensor_real_t gravVec[3]);
	static void calcLinearAcc(
		const sensor_real_t accin[3],
		const sensor_real_t gravVec[3],
		sensor_real_t accout[3]
	);

	void updateBiasForgettingTime(float biasForgettingTime);

	[[nodiscard]] bool getRestDetected() const;

protected:
	sensor_real_t gyrTs;
	sensor_real_t accTs;
	sensor_real_t magTs;

	VQFParams vqfParams;
	VQF vqf;

	// A also used for linear acceleration extraction
	sensor_real_t bAxyz[3]{0.0f, 0.0f, 0.0f};

	bool magExist = false;
	sensor_real_t qwxyz[4]{1.0f, 0.0f, 0.0f, 0.0f};
	bool updated = false;

	bool gravityReady = false;
	sensor_real_t vecGravity[3]{0.0f, 0.0f, 0.0f};
	bool linaccelReady = false;
	sensor_real_t linAccel[3]{0.0f, 0.0f, 0.0f};
#ifdef ESP32
	sensor_real_t linAccel_guard;  // Temporary patch for some weird ESP32 bug
#endif
};
}  // namespace SlimeVR::Sensors

#endif  // SLIMEVR_SENSORFUSION_H
