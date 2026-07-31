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

// The accumulation buffer behind raw sample streaming (#23).
//
// Split out from `RawSampleStreamer.h` with no Arduino dependency so it can be
// unit tested on a host -- the same reasoning, and the same treatment, as
// `configuration/gyroscalecmd.h`. What is worth testing here is not the sending;
// it is the one invariant the server cannot check for itself.

namespace SlimeVR::Sensors {

/**
 * One stream's worth of pending samples, plus the accounting that makes a
 * lossy capture detectable.
 *
 * ## The invariant
 *
 * **Every batch handed out contains samples that were contiguous in time.**
 *
 * That is the property the whole design rests on. The server reconstructs
 * sample times by stepping from [baseMicros] at the stream's fixed nominal
 * period, so a batch with a hole in the middle would be reconstructed as though
 * the samples after the hole happened earlier than they did -- and nothing
 * downstream could tell. Every re-fusion run over that recording would then
 * integrate a subtly wrong timeline while looking entirely healthy.
 *
 * So a full buffer drops the *incoming* sample and counts it, rather than
 * overwriting the oldest. Dropping loses data at a known place; overwriting
 * silently corrupts the time base. The count is cumulative and travels with
 * every batch, so the server can mark the gap instead of concatenating over it.
 *
 * Accelerometer and gyroscope keep separate instances. They run at different
 * rates, and a shared buffer could not express "the gyroscope dropped samples
 * and the accelerometer did not", which is exactly what the server needs to
 * mark a gap in the right stream.
 */
template <uint16_t Capacity>
class RawSampleBatch {
public:
	static constexpr uint16_t capacity = Capacity;

	[[nodiscard]] uint16_t count() const { return m_count; }
	[[nodiscard]] uint32_t sequence() const { return m_sequence; }
	[[nodiscard]] uint32_t dropped() const { return m_dropped; }
	[[nodiscard]] uint64_t baseMicros() const { return m_baseMicros; }
	[[nodiscard]] const int16_t* samples() const { return m_samples; }
	[[nodiscard]] bool empty() const { return m_count == 0; }

	/** Clears everything, including the counters. Used when a capture starts. */
	void reset() {
		m_count = 0;
		m_sequence = 0;
		m_dropped = 0;
		m_baseMicros = 0;
	}

	/**
	 * Appends one sample, or counts it as dropped if there is no room.
	 *
	 * @param nominalMicros the sample's time on the nominal timeline -- derived
	 * from the configured sample period rather than read from a clock, because
	 * the configured period is what the on-device fusion integrates.
	 */
	void push(uint64_t nominalMicros, int16_t x, int16_t y, int16_t z) {
		if (m_count >= Capacity) {
			m_dropped++;
			return;
		}
		if (m_count == 0) {
			m_baseMicros = nominalMicros;
		}
		m_samples[m_count * 3 + 0] = x;
		m_samples[m_count * 3 + 1] = y;
		m_samples[m_count * 3 + 2] = z;
		m_count++;
	}

	/**
	 * Marks the current batch as sent and starts the next one.
	 *
	 * The sequence advances whether or not the datagram actually left, because
	 * it numbers batches *produced* rather than batches transmitted. A server
	 * that sees a gap in it has lost data either way, and that is what it needs
	 * to know.
	 *
	 * [m_dropped] deliberately does not reset: it is cumulative for the session,
	 * so a batch that arrives after a lossy stretch still reports the loss even
	 * if the batches that were dropped alongside it never arrived.
	 */
	void advance() {
		m_sequence++;
		m_count = 0;
	}

private:
	uint16_t m_count = 0;
	uint32_t m_sequence = 0;
	uint32_t m_dropped = 0;
	uint64_t m_baseMicros = 0;
	int16_t m_samples[Capacity * 3] = {};
};

}  // namespace SlimeVR::Sensors
