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

// Deliberately free of any Arduino or hardware dependency, so the assembly
// logic can be unit tested on a host. See tools/fusion-bench/tests/selftest.cpp.
//
// This is the part of the sensor-hub path most likely to be quietly wrong: an
// off-by-one in the dummy-byte skip or the split boundary produces numbers that
// look like a magnetometer reading but are not one, and no amount of staring at
// a running tracker would reveal it.

namespace SlimeVR::Sensors::SoftFusion {

/**
 * How a magnetometer's bytes are laid out across sensor-hub FIFO words.
 *
 * The IMU batches each slave's read as a whole number of 6-byte FIFO words, so
 * a 7-byte read occupies two words with the tail padded. Only the leading
 * (dummyBytes + data) bytes of each slave's concatenated words are meaningful.
 */
struct MagFifoConfig {
	bool enabled = false;
	/// Dummy bytes each slave transaction emits before real data. Per
	/// transaction, not per sample -- a split read pays the cost twice.
	uint8_t dummyBytes = 0;
	/// Total magnetometer data bytes: 6 for a 16-bit part, 9 for a 24-bit one.
	uint8_t dataBytes = 0;
	/// Whether the read is split across slave 0 and slave 1.
	bool split = false;
	/// Data bytes carried by slave 0 when split.
	uint8_t firstDataBytes = 0;
	float magTs = 0.0f;

	/// Bytes slave 0 must deliver before its portion is complete.
	[[nodiscard]] uint8_t slave0Needed() const {
		return static_cast<uint8_t>(dummyBytes + (split ? firstDataBytes : dataBytes));
	}

	/// Bytes slave 1 must deliver, or 0 when the read is not split.
	[[nodiscard]] uint8_t slave1Needed() const {
		if (!split) {
			return 0;
		}
		return static_cast<uint8_t>(dummyBytes + (dataBytes - firstDataBytes));
	}
};

/**
 * Reassembles a magnetometer sample from sensor-hub FIFO words.
 *
 * Words arrive interleaved with gyroscope and accelerometer words and can span
 * bulkRead calls, so partial state persists across both.
 */
class MagFifoAssembler {
public:
	static constexpr size_t BufferSize = 24;
	static constexpr size_t WordSize = 6;

	void reset() {
		m_slave0Len = 0;
		m_slave1Len = 0;
	}

	/**
	 * Feeds one 6-byte slave-0 word. Returns true and fills `xyz` when the
	 * sample is complete, which also resets the accumulators.
	 */
	bool feedSlave0(const uint8_t* word, const MagFifoConfig& cfg, int32_t xyz[3]) {
		// A fresh slave-0 word arriving when slave 0 is already complete means
		// the previous sample was never finished -- most likely slave 1 was
		// dropped. Start over rather than concatenating across samples, which
		// would silently emit a mixture of two.
		if (m_slave0Len >= cfg.slave0Needed()) {
			reset();
		}
		append(m_slave0, m_slave0Len, word);
		return tryEmit(cfg, xyz);
	}

	/** Feeds one 6-byte slave-1 word. */
	bool feedSlave1(const uint8_t* word, const MagFifoConfig& cfg, int32_t xyz[3]) {
		append(m_slave1, m_slave1Len, word);
		return tryEmit(cfg, xyz);
	}

	/** Sign-extends a little-endian 24-bit value held in the low 3 bytes. */
	static int32_t signExtend24(const uint8_t* p) {
		int32_t v = static_cast<int32_t>(p[0]) | (static_cast<int32_t>(p[1]) << 8)
				  | (static_cast<int32_t>(p[2]) << 16);
		if (v & 0x00800000) {
			v |= static_cast<int32_t>(0xFF000000u);
		}
		return v;
	}

	static int32_t readLE16(const uint8_t* p) {
		return static_cast<int16_t>(
			static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8)
		);
	}

private:
	bool tryEmit(const MagFifoConfig& cfg, int32_t xyz[3]) {
		if (cfg.dataBytes != 6 && cfg.dataBytes != 9) {
			return false;
		}
		if (m_slave0Len < cfg.slave0Needed()) {
			return false;
		}

		const uint8_t firstData = cfg.split ? cfg.firstDataBytes : cfg.dataBytes;
		uint8_t assembled[12]{};
		std::memcpy(assembled, m_slave0 + cfg.dummyBytes, firstData);

		if (cfg.split) {
			if (m_slave1Len < cfg.slave1Needed()) {
				return false;
			}
			std::memcpy(
				assembled + firstData,
				m_slave1 + cfg.dummyBytes,
				static_cast<size_t>(cfg.dataBytes - firstData)
			);
		}

		if (cfg.dataBytes == 9) {
			xyz[0] = signExtend24(assembled + 0);
			xyz[1] = signExtend24(assembled + 3);
			xyz[2] = signExtend24(assembled + 6);
		} else {
			xyz[0] = readLE16(assembled + 0);
			xyz[1] = readLE16(assembled + 2);
			xyz[2] = readLE16(assembled + 4);
		}

		reset();
		return true;
	}

	static void append(uint8_t* buffer, uint8_t& length, const uint8_t* word) {
		const size_t room = BufferSize - length;
		const size_t count = room < WordSize ? room : WordSize;
		if (count == 0) {
			// More words arrived than the configuration accounts for, so the
			// accumulator is out of sync with the device. Drop it.
			length = 0;
			return;
		}
		std::memcpy(buffer + length, word, count);
		length = static_cast<uint8_t>(length + count);
	}

	uint8_t m_slave0[BufferSize]{};
	uint8_t m_slave1[BufferSize]{};
	uint8_t m_slave0Len = 0;
	uint8_t m_slave1Len = 0;
};

}  // namespace SlimeVR::Sensors::SoftFusion
