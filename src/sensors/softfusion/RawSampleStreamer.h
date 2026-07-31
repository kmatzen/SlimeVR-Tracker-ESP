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
#include <cstring>

#include "../../GlobalVars.h"
#include "../../network/packets.h"
#include "rawsamplebatch.h"
#include "rawstreamfeatures.h"

namespace SlimeVR::Sensors {

/**
 * Batches raw, pre-calibration IMU samples and sends them to the server.
 *
 * ## Why this exists next to `RawSampleLogger` rather than instead of it
 *
 * `RawSampleLogger` (#9) captures the right data from the right place -- it is
 * called inside `processAccelSample`/`processGyroSample`, *before*
 * `calibrator.scaleAccelSample`, so a capture holds what the sensor produced
 * rather than what the current calibration made of it. That positioning is the
 * hard part and it was already correct.
 *
 * What it cannot do is take part in a capture session: it is `#ifdef`-gated at
 * compile time, writes CSV over `Serial`, and covers one sensor at a time. A
 * wireless seven-tracker recording cannot use any of that. This is the same data
 * from the same two call sites, batched and sent over the network.
 *
 * Both are kept. The serial logger stays the right tool for a bench capture of
 * one device with a cable attached; this is the right tool for a session.
 *
 * ## Why `.pfr` recordings need it
 *
 * A `.pfr` stores *fused* output, so every recording freezes VQF and its
 * parameters, rest detection, the sensor error model, online estimation and the
 * FIFO configuration. Such a recording answers *"how does the server behave
 * given this tracker behaviour"* permanently, and cannot answer the same
 * question about any other firmware. Raw counts make it re-runnable against any
 * fusion configuration, forever -- which is the difference between recording
 * once and re-shooting after every firmware decision.
 *
 * ## Nominal timestamps, and the real-time marker beside them
 *
 * Sample times are accumulated from the *configured* sample period, not from
 * `micros()`, because the configured period is what the on-device fusion
 * integrates -- replaying nominal timestamps reproduces what the filter actually
 * saw. Every batch also carries the true `micros()` at flush, which is
 * [RawSampleLogger]'s `# t_real` marker by another name: comparing the two is
 * the only way the gap between configured and actual ODR is visible at all.
 *
 * ## Losing a sample is not like losing a rotation
 *
 * A dropped rotation packet is harmless; the next one supersedes it. **A dropped
 * raw sample corrupts every re-fusion run downstream of it**, silently, because
 * the filter integrates a gap it cannot see.
 *
 * So each stream carries a batch sequence number and a cumulative count of
 * samples this side dropped to buffer overrun. Between them the server can tell
 * a complete capture from a holed one, and mark the holes rather than
 * concatenating over them. A capture whose gaps are marked is still useful; one
 * with unmarked gaps is worse than none, because it looks fine.
 */
#if RAW_SAMPLE_STREAMING

template <typename Consts>
class RawSampleStreamer {
public:
	using RawSensorT = typename Consts::RawSensorT;

	static constexpr bool Enabled = true;

	/**
	 * Samples buffered per stream before a flush is forced.
	 *
	 * At the fastest configured gyroscope rate in the tree (LSM6DSV, 240 Hz)
	 * this fills in ~67 ms, so a batch leaves well inside the interval the
	 * server's own bundling already works on. Sized in samples rather than in
	 * time because the buffer is what must not overflow.
	 */
	static constexpr uint16_t BatchCapacity = 16;

	/** How often the stream's metadata is repeated, in microseconds. */
	static constexpr uint32_t InfoIntervalMicros = 2000000;

	/**
	 * Longest a partly-filled batch waits before going out anyway.
	 *
	 * Only a liveness bound. At the configured rates a batch fills in 67 ms
	 * (gyroscope) to 133 ms (accelerometer), so this fires rarely -- it exists so
	 * a slow or stopped stream still delivers what it has.
	 */
	static constexpr uint32_t MaxFlushIntervalMicros = 200000;

	void begin(uint8_t sensorId, const char* sensorName) {
		m_sensorId = sensorId;
		m_sensorName = sensorName;
	}

	/**
	 * Takes the sample periods and scale factors that will describe the stream.
	 *
	 * Read when a capture *starts*, not at `motionSetup`. The runtime
	 * calibration measures the true sample periods some seconds after boot --
	 * the log goes from "Sensor timesteps not calibrated" to "Calibrated
	 * timesteps: Accel 0.008319, Gyro 0.004160" -- so a value captured during
	 * setup is the datasheet nominal, not what the device runs at.
	 *
	 * Measured on hardware, that difference is ~0.15%: 120.18 Hz against a
	 * nominal 120. Over a five-minute recording it is 0.4 s of timeline error,
	 * and since every sample time in the `.imu` is derived from this number, it
	 * would be wrong for the whole file.
	 */
	void setTimebase(float accTs, float gyrTs, float accScale, float gyrScale) {
		m_accTs = accTs;
		m_gyrTs = gyrTs;
		m_accScale = accScale;
		m_gyrScale = gyrScale;
		m_accStepNanos = static_cast<uint32_t>(accTs * 1e9f);
		m_gyrStepNanos = static_cast<uint32_t>(gyrTs * 1e9f);
	}

	/**
	 * Starts or stops streaming.
	 *
	 * Starting resets both streams: nominal time restarts at zero, sequences
	 * restart at zero, and the drop counters clear. A capture is a session, and
	 * carrying counters across sessions would make a fresh capture look like a
	 * continuation of a lossy one.
	 */
	void setStreaming(bool streaming) {
		if (streaming == m_streaming) {
			return;
		}
		m_streaming = streaming;
		if (streaming) {
			m_acc.reset();
			m_gyr.reset();
			m_accNanos = 0;
			m_gyrNanos = 0;
			m_lastInfoMicros = 0;
			m_lastFlushMicros = micros();
		}
	}

	[[nodiscard]] bool isStreaming() const { return m_streaming; }

	void logAccel(const RawSensorT xyz[3]) {
		if (!m_streaming) {
			return;
		}
		m_accNanos += m_accStepNanos;
		push(m_acc, m_accNanos / 1000, xyz);
	}

	void logGyro(const RawSensorT xyz[3]) {
		if (!m_streaming) {
			return;
		}
		m_gyrNanos += m_gyrStepNanos;
		push(m_gyr, m_gyrNanos / 1000, xyz);
	}

	/**
	 * Sends whatever is buffered.
	 *
	 * Called from `sendData()`, so batches ride the bundle the sensor's rotation
	 * data is already going out in -- an inner bundle packet costs three bytes
	 * of framing, and the eight-byte packet number is written once for the whole
	 * bundle rather than once per batch.
	 */
	void flush() {
		if (!m_streaming) {
			return;
		}

		const uint32_t now = micros();
		// Repeated rather than sent once. The transport is UDP, so a server that
		// started listening late, or simply lost the first datagram, would
		// otherwise hold samples it cannot scale into physical units.
		if (m_lastInfoMicros == 0 || now - m_lastInfoMicros >= InfoIntervalMicros) {
			m_lastInfoMicros = now;
			networkConnection.sendRawSampleStreamInfo(
				m_sensorId,
				m_sensorName,
				m_accTs,
				m_gyrTs,
				m_accScale,
				m_gyrScale
			);
		}

		// Send a batch when it is full, or when the oldest pending sample has
		// waited long enough. Flushing on every call was the original behaviour
		// and it was wrong in a way only hardware showed: this runs from
		// `sendData()`, which fires at the network send rate, so batches left
		// with one sample in them.
		//
		// Measured on an ESP8266 with an LSM6DSV: 1529 batches carrying 1536
		// accelerometer samples, and about 370 packets per second from a single
		// tracker against the ~40 the design assumed. 18% of them never arrived.
		// The batching existed and did nothing.
		const bool due = now - m_lastFlushMicros >= MaxFlushIntervalMicros;
		const bool sent = sendStream(RawSampleKind::Accel, m_acc, now, due)
						| sendStream(RawSampleKind::Gyro, m_gyr, now, due);
		if (sent) {
			m_lastFlushMicros = now;
		}
	}

private:
	using Stream = RawSampleBatch<BatchCapacity>;

	static void push(Stream& stream, uint64_t nominalMicros, const RawSensorT xyz[3]) {
		stream.push(
			nominalMicros,
			static_cast<int16_t>(xyz[0]),
			static_cast<int16_t>(xyz[1]),
			static_cast<int16_t>(xyz[2])
		);
	}

	bool sendStream(RawSampleKind kind, Stream& stream, uint32_t realMicros, bool due) {
		if (stream.empty()) {
			return false;
		}
		if (stream.count() < BatchCapacity && !due) {
			return false;
		}
		const bool sent = networkConnection.sendRawSampleBatch(
			m_sensorId,
			kind,
			stream.sequence(),
			stream.dropped(),
			stream.baseMicros(),
			realMicros,
			stream.count(),
			stream.samples()
		);
		stream.advance();
		(void)sent;
		return true;
	}

	bool m_streaming = false;
	uint8_t m_sensorId = 0;
	const char* m_sensorName = "";
	float m_accTs = 0;
	float m_gyrTs = 0;
	float m_accScale = 0;
	float m_gyrScale = 0;
	uint32_t m_accStepNanos = 0;
	uint32_t m_gyrStepNanos = 0;
	uint64_t m_accNanos = 0;
	uint64_t m_gyrNanos = 0;
	uint32_t m_lastInfoMicros = 0;
	uint32_t m_lastFlushMicros = 0;
	Stream m_acc;
	Stream m_gyr;
};

#else

/**
 * The compiled-out variant: no members, no initialisers, no code.
 *
 * Written out as a second class rather than left to `if constexpr` inside the
 * first, because that measurably did not work. `Enabled` does not depend on the
 * template parameter, so a discarded `if constexpr` branch is still fully
 * type-checked -- and, more expensively, the *data members* survive regardless.
 * Two 48-entry sample buffers and a dozen scalars with default initialisers cost
 * **834 bytes of flash on `BOARD_GLOVE_IMU_SLIMEVR_DEV`** in constructor code
 * alone, on a board with about 2 kB spare.
 *
 * That is the tax issue #8 describes -- roughly 350 bytes per feature even when
 * compiled out -- and it is avoidable here, so it is avoided. With this variant
 * the board pays nothing.
 */
template <typename Consts>
class RawSampleStreamer {
public:
	using RawSensorT = typename Consts::RawSensorT;

	static constexpr bool Enabled = false;

	void begin(uint8_t, const char*) {}
	void setTimebase(float, float, float, float) {}
	void setStreaming(bool) {}
	[[nodiscard]] bool isStreaming() const { return false; }
	void logAccel(const RawSensorT*) {}
	void logGyro(const RawSensorT*) {}
	void flush() {}
};

#endif

}  // namespace SlimeVR::Sensors
