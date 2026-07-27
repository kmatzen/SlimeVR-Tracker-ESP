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

#ifndef SLIMEVR_RAWSAMPLELOGGER_H
#define SLIMEVR_RAWSAMPLELOGGER_H

#include <Arduino.h>

#include <cstdint>
#include <cstdio>

namespace SlimeVR::Sensors {

/**
 * Streams raw, uncalibrated IMU samples over serial, in the CSV format that
 * `tools/fusion-bench` reads directly. Capturing a dataset is then just
 * redirecting the serial port to a file -- there is no converter step and no
 * host-side dependency.
 *
 * Why raw counts rather than scaled values:
 *
 *  - Raw is what the sensor actually produced. Everything else is derived, so
 *    a raw log can be replayed under *different* calibration, which is the
 *    whole point when the thing being evaluated is the calibration.
 *  - Formatting integers is far cheaper on an ESP8266 than formatting floats,
 *    and this runs in the sample path. Perturbing the timing of the samples
 *    would corrupt the measurement we are trying to take.
 *
 * The scale factors needed to convert are emitted in the header, so no
 * information is lost.
 *
 * Accelerometer and gyroscope generally run at different rates (120 Hz and
 * 240 Hz on an LSM6DSV), so they are logged as separate rows with the unused
 * columns left empty rather than being resampled onto a common tick. Resampling
 * here would bake an assumption into the data instead of leaving it to the
 * analysis.
 *
 * Disabled unless RAW_SAMPLE_LOGGING is defined. The code is compiled either
 * way -- the methods early-return on a `constexpr` condition rather than being
 * `#ifdef`ed out at the call sites -- so the disabled path cannot silently stop
 * compiling.
 */
template <typename Consts>
class RawSampleLogger {
public:
	using RawSensorT = typename Consts::RawSensorT;

#ifdef RAW_SAMPLE_LOGGING
	static constexpr bool Enabled = true;
#else
	static constexpr bool Enabled = false;
#endif

	// Only one sensor is logged. Interleaving two IMUs into one stream would
	// need a discriminator column and gains nothing for the bench tests.
#ifdef RAW_SAMPLE_LOGGING_SENSOR_ID
	static constexpr uint8_t LoggedSensorId = RAW_SAMPLE_LOGGING_SENSOR_ID;
#else
	static constexpr uint8_t LoggedSensorId = 0;
#endif

	/** Real-time marker interval, so nominal and true sample rates can be compared. */
	static constexpr uint32_t MarkerIntervalMicros = 5000000;

	void begin(
		uint8_t sensorId,
		const char* sensorName,
		float accTs,
		float gyrTs,
		float accScale,
		float gyrScale
	) {
		if constexpr (!Enabled) {
			return;
		}
		m_active = (sensorId == LoggedSensorId);
		if (!m_active) {
			return;
		}

		m_accStepNanos = static_cast<uint32_t>(accTs * 1e9f);
		m_gyrStepNanos = static_cast<uint32_t>(gyrTs * 1e9f);
		m_accNanos = 0;
		m_gyrNanos = 0;
		m_accCount = 0;
		m_gyrCount = 0;
		m_lastMarkerMicros = micros();

		char buf[192];
		Serial.println();
		Serial.println(F("# slimevr-imu-log v1"));
		snprintf(buf, sizeof(buf), "# sensor %s", sensorName);
		Serial.println(buf);
		// %.9g keeps enough significant digits that the round trip through text
		// does not perturb the scale factors.
		snprintf(buf, sizeof(buf), "# acc_ts %.9g", static_cast<double>(accTs));
		Serial.println(buf);
		snprintf(buf, sizeof(buf), "# gyr_ts %.9g", static_cast<double>(gyrTs));
		Serial.println(buf);
		snprintf(buf, sizeof(buf), "# acc_scale %.9g", static_cast<double>(accScale));
		Serial.println(buf);
		snprintf(buf, sizeof(buf), "# gyr_scale %.9g", static_cast<double>(gyrScale));
		Serial.println(buf);
		Serial.println(F("# note raw uncalibrated counts; apply *_scale to convert"));
		Serial.println(F("t_us,ax,ay,az,gx,gy,gz"));
	}

	void logAccel(const RawSensorT xyz[3]) {
		if constexpr (!Enabled) {
			return;
		}
		if (!m_active) {
			return;
		}
		m_accNanos += m_accStepNanos;
		m_accCount++;

		char buf[64];
		int n = snprintf(
			buf,
			sizeof(buf),
			"%lu,%ld,%ld,%ld,,,\n",
			static_cast<unsigned long>(m_accNanos / 1000),
			static_cast<long>(xyz[0]),
			static_cast<long>(xyz[1]),
			static_cast<long>(xyz[2])
		);
		if (n > 0) {
			Serial.write(reinterpret_cast<const uint8_t*>(buf), static_cast<size_t>(n));
		}
		emitMarkerIfDue();
	}

	void logGyro(const RawSensorT xyz[3]) {
		if constexpr (!Enabled) {
			return;
		}
		if (!m_active) {
			return;
		}
		m_gyrNanos += m_gyrStepNanos;
		m_gyrCount++;

		char buf[64];
		int n = snprintf(
			buf,
			sizeof(buf),
			"%lu,,,,%ld,%ld,%ld\n",
			static_cast<unsigned long>(m_gyrNanos / 1000),
			static_cast<long>(xyz[0]),
			static_cast<long>(xyz[1]),
			static_cast<long>(xyz[2])
		);
		if (n > 0) {
			Serial.write(reinterpret_cast<const uint8_t*>(buf), static_cast<size_t>(n));
		}
		emitMarkerIfDue();
	}

private:
	/**
	 * Periodic comment carrying the true elapsed time alongside the sample
	 * counts. The row timestamps above are *nominal* -- derived from the
	 * configured sample period -- because that is what the on-device fusion
	 * integrates, so replaying them reproduces what the filter saw. Comparing
	 * the two reveals the difference between the configured and actual ODR,
	 * which is exactly the error the runtime sample-rate calibration exists to
	 * correct, and which is otherwise invisible.
	 */
	void emitMarkerIfDue() {
		const uint32_t now = micros();
		if (now - m_lastMarkerMicros < MarkerIntervalMicros) {
			return;
		}
		m_lastMarkerMicros = now;

		char buf[96];
		int n = snprintf(
			buf,
			sizeof(buf),
			"# t_real %lu acc_n %lu gyr_n %lu\n",
			static_cast<unsigned long>(now),
			static_cast<unsigned long>(m_accCount),
			static_cast<unsigned long>(m_gyrCount)
		);
		if (n > 0) {
			Serial.write(reinterpret_cast<const uint8_t*>(buf), static_cast<size_t>(n));
		}
	}

	bool m_active = false;
	uint32_t m_accStepNanos = 0;
	uint32_t m_gyrStepNanos = 0;
	uint64_t m_accNanos = 0;
	uint64_t m_gyrNanos = 0;
	uint32_t m_accCount = 0;
	uint32_t m_gyrCount = 0;
	uint32_t m_lastMarkerMicros = 0;
};

}  // namespace SlimeVR::Sensors

#endif  // SLIMEVR_RAWSAMPLELOGGER_H
