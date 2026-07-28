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

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <functional>

#include "../../../sensorinterface/RegisterInterface.h"
#include "callbacks.h"
#include "magfifo.h"

namespace SlimeVR::Sensors::SoftFusion::Drivers {

struct LSM6DSOutputHandler {
	LSM6DSOutputHandler(
		RegisterInterface& registerInterface,
		SlimeVR::Logging::Logger& logger
	)
		: m_RegisterInterface(registerInterface)
		, m_Logger(logger) {}

	RegisterInterface& m_RegisterInterface;
	SlimeVR::Logging::Logger& m_Logger;

#pragma pack(push, 1)
	struct FifoEntryAligned {
		union {
			int16_t xyz[3];
			uint8_t raw[6];
		};
	};
#pragma pack(pop)

	static constexpr size_t FullFifoEntrySize = sizeof(FifoEntryAligned) + 1;

	/**
	 * FIFO tag values, from the LSM6DS family's tagged-FIFO encoding.
	 *
	 * Tags 1-3 are already relied on by the gyro/accel/temperature cases below
	 * and are known-good on LSM6DSV, which is good evidence the rest of the
	 * family numbering carries over. The sensor-hub values are taken from ST's
	 * lsm6dsox_reg.h enum -- they have NOT been observed on an LSM6DSV.
	 */
	enum FifoTag : uint8_t {
		TagGyroNC = 0x01,
		TagAccelNC = 0x02,
		TagTemperature = 0x03,
		TagShubSlave0 = 0x0e,
		TagShubSlave1 = 0x0f,
		TagShubNack = 0x19,
	};

	MagFifoAssembler m_magAssembler;
	bool m_shubNackLogged = false;

	template <typename Regs>
	bool bulkRead(
		DriverCallbacks<int16_t>&& callbacks,
		float GyrTs,
		float AccTs,
		float TempTs,
		const MagFifoConfig& mag
	) {
		constexpr auto FIFO_SAMPLES_MASK = 0x3ff;
		constexpr auto FIFO_OVERRUN_LATCHED_MASK = 0x800;

		const auto fifo_status = m_RegisterInterface.readReg16(Regs::FifoStatus);
		const auto available_axes = fifo_status & FIFO_SAMPLES_MASK;
		const auto fifo_bytes = available_axes * FullFifoEntrySize;
		if (fifo_status & FIFO_OVERRUN_LATCHED_MASK) {
			// FIFO overrun is expected to happen during startup and calibration
			m_Logger.error(
				"FIFO OVERRUN! This occuring during normal usage is an issue."
			);
		}

		std::array<uint8_t, FullFifoEntrySize * 8> read_buffer;  // max 8 readings
		const auto bytes_to_read = std::min(
									   static_cast<size_t>(read_buffer.size()),
									   static_cast<size_t>(fifo_bytes)
								   )
								 / FullFifoEntrySize * FullFifoEntrySize;
		m_RegisterInterface
			.readBytes(Regs::FifoData, bytes_to_read, read_buffer.data());
		for (auto i = 0u; i < bytes_to_read; i += FullFifoEntrySize) {
			FifoEntryAligned entry;
			uint8_t tag = read_buffer[i] >> 3;
			memcpy(
				entry.raw,
				&read_buffer[i + 0x1],
				sizeof(FifoEntryAligned)
			);  // skip fifo header

			switch (tag) {
				case TagGyroNC:
					callbacks.processGyroSample(entry.xyz, GyrTs);
					break;
				case TagAccelNC:
					callbacks.processAccelSample(entry.xyz, AccTs);
					break;
				case TagTemperature:
					callbacks.processTempSample(entry.xyz[0], TempTs);
					break;
				case TagShubSlave0:
					if (mag.enabled) {
						int32_t magXyz[3];
						if (m_magAssembler.feedSlave0(entry.raw, mag, magXyz)
							&& callbacks.processMagSample) {
							callbacks.processMagSample(magXyz, mag.magTs);
						}
					}
					break;
				case TagShubSlave1:
					if (mag.enabled) {
						int32_t magXyz[3];
						if (m_magAssembler.feedSlave1(entry.raw, mag, magXyz)
							&& callbacks.processMagSample) {
							callbacks.processMagSample(magXyz, mag.magTs);
						}
					}
					break;
				case TagShubNack:
					// The auxiliary sensor did not acknowledge. Logged once so a
					// miswired or absent mag is obvious instead of just
					// producing no data.
					if (!m_shubNackLogged) {
						m_shubNackLogged = true;
						m_Logger.error(
							"Sensor hub reported a NACK from the auxiliary "
							"sensor; magnetometer data will not arrive"
						);
					}
					m_magAssembler.reset();
					break;
			}
		}
		return fifo_bytes > bytes_to_read;
	}

	/** For drivers with no auxiliary sensor, where the hub tags never appear. */
	template <typename Regs>
	bool bulkRead(
		DriverCallbacks<int16_t>&& callbacks,
		float GyrTs,
		float AccTs,
		float TempTs
	) {
		return bulkRead<Regs>(
			std::move(callbacks),
			GyrTs,
			AccTs,
			TempTs,
			MagFifoConfig{}
		);
	}
};

}  // namespace SlimeVR::Sensors::SoftFusion::Drivers
