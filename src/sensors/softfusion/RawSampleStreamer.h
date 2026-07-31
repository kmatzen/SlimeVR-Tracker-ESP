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
	 * Bytes an inner bundle packet may occupy.
	 *
	 * `Connection::write` buffers inner packets into a 128-byte array and
	 * returns 0 -- failure -- the moment one would not fit. `MUST_TRANSFER_BOOL`
	 * then abandons the send. **Nothing is logged and nothing is counted**: the
	 * batch simply never leaves, and both sides look healthy.
	 *
	 * That is not hypothetical. Adding one four-byte field to the batch header
	 * took it from 125 bytes to 129, and raw capture stopped working entirely --
	 * every capture produced zero samples while rotation data flowed normally.
	 * It cost an afternoon to find, because the symptom is silence.
	 */
	static constexpr uint16_t InnerPacketLimit = 128;

	/** 4-byte packet type, then the fields `sendRawSampleBatch` writes. */
	static constexpr uint16_t BatchHeaderBytes = 4 + 1 + 1 + 1 + 4 + 4 + 4 + 8 + 4 + 2;

	/**
	 * Samples buffered per stream before a flush is forced.
	 *
	 * Bounded by [InnerPacketLimit] rather than chosen: at three `int16` per
	 * sample, this is what fits with room to spare. Twelve fills in ~50 ms at
	 * the LSM6DSV's 240 Hz gyroscope rate, so batches still leave full and the
	 * packet rate stays near the ~20 per second per stream the design assumes.
	 */
	static constexpr uint16_t BatchCapacity = 12;

	static_assert(
		BatchHeaderBytes + BatchCapacity * 3 * sizeof(int16_t) <= InnerPacketLimit,
		"a raw sample batch must fit the inner-bundle buffer, or it is dropped "
		"silently -- see InnerPacketLimit"
	);

	/** How often the stream's metadata is repeated, in microseconds. */
	static constexpr uint32_t InfoIntervalMicros = 2000000;

	/**
	 * How often the counters below are printed while a capture runs.
	 *
	 * Cheap and always on. Every failure of this feature so far has been
	 * diagnosed by inference from what the *server* received, and the last one
	 * resisted that entirely -- a capture produced no raw samples and nothing on
	 * either side said which of "no samples pushed", "no flush called", "nothing
	 * sent" or "sent and lost" was true. These distinguish them.
	 */
	static constexpr uint32_t StatsIntervalMicros = 5000000;

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
	/** Overruns already counted before a capture starts, so they are not blamed on it.
	 */
	void setFifoBaseline(uint32_t total) { m_fifoBaseline = total; }

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
			m_lastStatsMicros = m_lastFlushMicros;
			m_accPushes = 0;
			m_gyrPushes = 0;
			m_flushCalls = 0;
			m_accBatches = 0;
			m_gyrBatches = 0;
			m_refusals = 0;
			m_sendFailures = 0;
			m_pushesWhileStopped = 0;
			// Taken as the baseline rather than zeroed: the counter is cumulative
			// since boot, and a capture only cares about overruns during itself.
			m_fifoDropped = m_fifoBaseline;
			m_fifoPending = m_fifoBaseline;
		}
	}

	[[nodiscard]] bool isStreaming() const { return m_streaming; }

	/**
	 * The sensor's FIFO overran, so samples were lost before this saw them.
	 *
	 * This is the one hole the rest of the design cannot detect. `bulkRead`
	 * reports the sensor's hardware FIFO having already discarded samples, which
	 * means they never reached `processGyroSample` and so never reached here.
	 * Nothing else notices:
	 *
	 * - the buffer drop counter only knows about *this* buffer
	 * - the batch sequence is unbroken, because those batches were produced
	 *   normally
	 * - the nominal clock advances only for samples actually processed, so the
	 *   timeline closes over the hole rather than showing it
	 *
	 * The result would be a capture that reports itself complete while missing
	 * data -- exactly the failure the batching rules exist to prevent, arriving
	 * by a path they do not cover.
	 *
	 * So the count is carried to the server, and the pending batch is sent
	 * *before* the count changes. Placement matters as much as counting: it puts
	 * the hole between two batches, where the server already knows how to mark
	 * it, rather than inside one where the samples either side would look
	 * adjacent.
	 *
	 * **Only the flag is set here.** This runs inside `motionLoop`, straight off
	 * the FIFO drain, and sending a datagram from that path resets the ESP8266 --
	 * measured, not feared: the tracker rebooted repeatedly and a 12 s capture
	 * yielded 35 samples instead of ~4300. The send is left to [flush], which is
	 * the only place this class is allowed to touch the network.
	 *
	 * The cost is a bounded imprecision. Samples pushed between the overrun and
	 * the next flush land in the batch that is about to be closed, so they are
	 * labelled as before the hole when they are really after it. `flush` runs
	 * from `sendData` at the network send rate, so that window is a few
	 * milliseconds -- on the order of one accelerometer and two gyroscope
	 * samples. Worth stating rather than hiding: the hole's *existence* and its
	 * *count* are exact, its position is accurate to a few samples.
	 *
	 * Observed on an idle tracker at roughly 20 overruns per minute, so this is
	 * the ordinary case rather than an edge one.
	 */
	void noteFifoOverruns(uint32_t total) { m_fifoPending = total; }

	void logAccel(const RawSensorT xyz[3]) {
		if (!m_streaming) {
			m_pushesWhileStopped++;
			return;
		}
		m_accPushes++;
		m_accNanos += m_accStepNanos;
		push(m_acc, m_accNanos / 1000, xyz);
	}

	void logGyro(const RawSensorT xyz[3]) {
		if (!m_streaming) {
			m_pushesWhileStopped++;
			return;
		}
		m_gyrPushes++;
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
		m_flushCalls++;

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

		// The new overrun count is adopted only once both buffers are empty --
		// that is, immediately after a send -- so the batch that reports it is
		// the first one containing no pre-hole samples. The server splits on the
		// change, and the split lands on a batch boundary.
		//
		// Deliberately *not* forced. An earlier attempt made a pending overrun
		// force an immediate partial send, and on hardware that fed back: the
		// tracker overruns often enough that nearly every flush became partial,
		// which recreated the packet flood #25 removed, which starved the drain
		// loop, which caused more overruns. A 15 s capture yielded 44 samples.
		//
		// The cost of waiting is that the hole is located to the nearest batch
		// boundary -- 67 ms for the gyroscope, 133 ms for the accelerometer --
		// rather than to the sample. The hole's existence and its size stay
		// exact; only its position is coarse, and it is coarse in a file that
		// marks it rather than in one that hides it.
		if (m_acc.empty() && m_gyr.empty()) {
			m_fifoDropped = m_fifoPending;
		}

		reportStatsIfDue(now);
	}

	/**
	 * Prints what the streamer has actually done, so a failed capture can be
	 * read rather than guessed at.
	 *
	 * The four numbers that matter, in the order they would fail: pushes (did
	 * samples reach the streamer at all), flushes (is the send path being
	 * called), batches (did anything qualify to be sent), and refusals (was it
	 * held back for not being full).
	 */
	void reportStatsIfDue(uint32_t now) {
		if (now - m_lastStatsMicros < StatsIntervalMicros) {
			return;
		}
		m_lastStatsMicros = now;
		Serial.printf(
			"[RawStream:%u] push a=%lu g=%lu | flush=%lu batch a=%lu g=%lu | "
			"refused=%lu | buffered a=%u g=%u | fifo %lu->%lu base=%lu | "
			"stopped-pushes=%lu\n",
			m_sensorId,
			static_cast<unsigned long>(m_accPushes),
			static_cast<unsigned long>(m_gyrPushes),
			static_cast<unsigned long>(m_flushCalls),
			static_cast<unsigned long>(m_accBatches),
			static_cast<unsigned long>(m_gyrBatches),
			static_cast<unsigned long>(m_refusals),
			m_acc.count(),
			m_gyr.count(),
			static_cast<unsigned long>(m_fifoDropped),
			static_cast<unsigned long>(m_fifoPending),
			static_cast<unsigned long>(m_fifoBaseline),
			static_cast<unsigned long>(m_pushesWhileStopped)
		);
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
			m_refusals++;
			return false;
		}
		const bool sent = networkConnection.sendRawSampleBatch(
			m_sensorId,
			kind,
			stream.sequence(),
			stream.dropped(),
			m_fifoDropped - m_fifoBaseline,
			stream.baseMicros(),
			realMicros,
			stream.count(),
			stream.samples()
		);
		stream.advance();
		if (kind == RawSampleKind::Accel) {
			m_accBatches++;
		} else {
			m_gyrBatches++;
		}
		m_sendFailures += sent ? 0 : 1;
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
	uint32_t m_fifoDropped = 0;
	uint32_t m_fifoPending = 0;
	uint32_t m_fifoBaseline = 0;
	uint32_t m_lastStatsMicros = 0;
	uint32_t m_accPushes = 0;
	uint32_t m_gyrPushes = 0;
	uint32_t m_flushCalls = 0;
	uint32_t m_accBatches = 0;
	uint32_t m_gyrBatches = 0;
	uint32_t m_refusals = 0;
	uint32_t m_sendFailures = 0;
	uint32_t m_pushesWhileStopped = 0;
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
	void setFifoBaseline(uint32_t) {}
	void noteFifoOverruns(uint32_t) {}
	void setStreaming(bool) {}
	[[nodiscard]] bool isStreaming() const { return false; }
	void logAccel(const RawSensorT*) {}
	void logGyro(const RawSensorT*) {}
	void flush() {}
};

#endif

}  // namespace SlimeVR::Sensors
