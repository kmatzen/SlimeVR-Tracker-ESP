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

#include "../magdriver.h"
#include "logging/Logger.h"
#include "sensorinterface/RegisterInterface.h"

namespace SlimeVR::Sensors::SoftFusion::Drivers {

/**
 * LSM6DS-family sensor hub (the IMU acting as I2C master for an auxiliary
 * sensor).
 *
 * *** NOT YET VALIDATED ON HARDWARE. ***
 *
 * Register addresses and bit positions are taken from the LSM6DSV datasheet and
 * ST's own `lsm6dsv_reg.h`. The code compiles and the sequences follow the
 * documented procedure, but none of it has been run against a real part. Treat
 * every value here as a hypothesis until the bring-up procedure in
 * tools/fusion-bench/README.md has been walked through.
 *
 * Two things about this interface are worth knowing before changing it:
 *
 *  - **The hub is clocked by the accelerometer.** Transactions only advance
 *    while the accelerometer is running, so nothing here works before the IMU
 *    has been initialised, and a one-shot read blocks until the next XL sample.
 *    This is why every operation has a timeout rather than spinning forever.
 *
 *  - **A single slave can read at most 7 bytes** (`SLAVE0_NUMOP` is 3 bits).
 *    That is enough for a 6-byte magnetometer, but not for a 9-byte one, and
 *    definitely not for a 9-byte one behind two dummy bytes. See
 *    startAuxPolling for how that is split across two slaves.
 */
template <typename Regs>
struct LSM6DSSensorHub {
	LSM6DSSensorHub(RegisterInterface& registerInterface, Logging::Logger& logger)
		: m_shubRegisterInterface(registerInterface)
		, m_shubLogger(logger) {}

	// Main-page registers.
	static constexpr uint8_t FuncCfgAccess = 0x01;
	static constexpr uint8_t FuncCfgShubRegAccess = 1 << 6;
	static constexpr uint8_t IfCfg = 0x03;
	static constexpr uint8_t IfCfgShubPuEn = 1 << 6;
	// STATUS_MASTER mirrored into the main page, so completion can be polled
	// without holding the embedded-function bank open.
	static constexpr uint8_t StatusMasterMainPage = 0x39;
	static constexpr uint8_t StatusMasterEndOp = 1 << 0;

	// Embedded-function bank registers, only reachable with SHUB_REG_ACCESS set.
	static constexpr uint8_t SensorHub1 = 0x02;
	static constexpr uint8_t MasterConfig = 0x14;
	static constexpr uint8_t Slv0Add = 0x15;
	static constexpr uint8_t Slv0SubAdd = 0x16;
	static constexpr uint8_t Slv0Config = 0x17;
	static constexpr uint8_t Slv1Add = 0x18;
	static constexpr uint8_t Slv1SubAdd = 0x19;
	static constexpr uint8_t Slv1Config = 0x1a;
	static constexpr uint8_t DataWriteSlv0 = 0x21;

	static constexpr uint8_t MasterConfigMasterOn = 1 << 2;
	static constexpr uint8_t MasterConfigWriteOnce = 1 << 6;
	static constexpr uint8_t MasterConfigRstMasterRegs = 1 << 7;
	static constexpr uint8_t Slv0ConfigBatchEn = 1 << 3;

	/** Slave address of the auxiliary device, 7-bit. */
	void setAuxId(uint8_t deviceId) { m_auxId = deviceId; }

	uint8_t readAux(uint8_t address) {
		enterShubBank(true);
		// Bit 0 set selects a read.
		writeShub(Slv0Add, static_cast<uint8_t>((m_auxId << 1) | 0x01));
		writeShub(Slv0SubAdd, address);
		writeShub(Slv0Config, 1);  // one byte, not batched into FIFO
		writeShub(MasterConfig, MasterConfigWriteOnce | MasterConfigMasterOn);
		enterShubBank(false);

		const bool ok = waitForEndOp();

		enterShubBank(true);
		const uint8_t value = m_shubRegisterInterface.readReg(SensorHub1);
		writeShub(MasterConfig, 0);
		enterShubBank(false);

		if (!ok) {
			m_shubLogger.error(
				"Aux read of 0x%02x from device 0x%02x timed out",
				address,
				m_auxId
			);
			return 0;
		}
		return value;
	}

	void writeAux(uint8_t address, uint8_t value) {
		enterShubBank(true);
		// Bit 0 clear selects a write.
		writeShub(Slv0Add, static_cast<uint8_t>(m_auxId << 1));
		writeShub(Slv0SubAdd, address);
		writeShub(DataWriteSlv0, value);
		writeShub(Slv0Config, 0);  // no reads
		writeShub(MasterConfig, MasterConfigWriteOnce | MasterConfigMasterOn);
		enterShubBank(false);

		const bool ok = waitForEndOp();

		enterShubBank(true);
		writeShub(MasterConfig, 0);
		enterShubBank(false);

		if (!ok) {
			m_shubLogger.error(
				"Aux write of 0x%02x to 0x%02x on device 0x%02x timed out",
				value,
				address,
				m_auxId
			);
		}
	}

	/**
	 * Configures the hub to read the magnetometer continuously and batch the
	 * result into the IMU FIFO.
	 *
	 * The byte budget is the awkward part. `SLAVE0_NUMOP` is three bits, so one
	 * slave reads at most 7 bytes, and a BMM350 needs 2 dummy bytes plus 9 data
	 * bytes. Each slave issues its own I2C transaction with its own repeated
	 * start, so each incurs its own dummy-byte prefix -- which means the read
	 * cannot simply be split 7 + 4 and concatenated. Instead:
	 *
	 *   slave 0: `dummy + 5` bytes from dataReg      -> X, and Y's low 2 bytes
	 *   slave 1: `dummy + 4` bytes from dataReg + 5  -> Y's high byte, and Z
	 *
	 * For a 6-byte magnetometer with no dummy bytes a single slave suffices and
	 * slave 1 is left disabled.
	 */
	void startAuxPolling(uint8_t dataReg, MagDataWidth dataWidth, uint8_t dummyBytes) {
		const uint8_t dataBytes = dataWidth == MagDataWidth::NineByte ? 9 : 6;
		m_auxDummyBytes = dummyBytes;
		m_auxDataBytes = dataBytes;

		enterShubBank(true);

		if (static_cast<uint8_t>(dummyBytes + dataBytes) <= 7) {
			m_auxSplit = false;
			writeShub(Slv0Add, static_cast<uint8_t>((m_auxId << 1) | 0x01));
			writeShub(Slv0SubAdd, dataReg);
			writeShub(
				Slv0Config,
				static_cast<uint8_t>((dummyBytes + dataBytes) | Slv0ConfigBatchEn)
			);
			writeShub(Slv1Config, 0);
		} else {
			m_auxSplit = true;
			const uint8_t firstData = static_cast<uint8_t>(7 - dummyBytes);
			const uint8_t secondData = static_cast<uint8_t>(dataBytes - firstData);
			m_auxFirstDataBytes = firstData;

			writeShub(Slv0Add, static_cast<uint8_t>((m_auxId << 1) | 0x01));
			writeShub(Slv0SubAdd, dataReg);
			writeShub(
				Slv0Config,
				static_cast<uint8_t>((dummyBytes + firstData) | Slv0ConfigBatchEn)
			);

			writeShub(Slv1Add, static_cast<uint8_t>((m_auxId << 1) | 0x01));
			writeShub(Slv1SubAdd, static_cast<uint8_t>(dataReg + firstData));
			writeShub(
				Slv1Config,
				static_cast<uint8_t>((dummyBytes + secondData) | Slv0ConfigBatchEn)
			);
		}

		// WRITE_ONCE is deliberately not set here: this is a repeating read, not
		// a one-shot configuration write.
		writeShub(MasterConfig, MasterConfigMasterOn);
		enterShubBank(false);

		// Internal pull-ups on the auxiliary bus. Harmless where the board also
		// fits external ones, and necessary where it does not.
		const uint8_t ifCfg = m_shubRegisterInterface.readReg(IfCfg);
		m_shubRegisterInterface.writeReg(IfCfg, ifCfg | IfCfgShubPuEn);

		m_auxPolling = true;
	}

	void stopAuxPolling() {
		enterShubBank(true);
		writeShub(MasterConfig, 0);
		writeShub(Slv0Config, 0);
		writeShub(Slv1Config, 0);
		enterShubBank(false);
		m_auxPolling = false;
	}

	[[nodiscard]] bool auxPolling() const { return m_auxPolling; }
	[[nodiscard]] uint8_t auxDummyBytes() const { return m_auxDummyBytes; }
	[[nodiscard]] uint8_t auxDataBytes() const { return m_auxDataBytes; }
	[[nodiscard]] bool auxSplit() const { return m_auxSplit; }
	[[nodiscard]] uint8_t auxFirstDataBytes() const { return m_auxFirstDataBytes; }

private:
	void enterShubBank(bool enabled) {
		const uint8_t current = m_shubRegisterInterface.readReg(FuncCfgAccess);
		const uint8_t next = enabled
							   ? static_cast<uint8_t>(current | FuncCfgShubRegAccess)
							   : static_cast<uint8_t>(current & ~FuncCfgShubRegAccess);
		m_shubRegisterInterface.writeReg(FuncCfgAccess, next);
	}

	void writeShub(uint8_t reg, uint8_t value) {
		m_shubRegisterInterface.writeReg(reg, value);
	}

	/**
	 * Waits for SENS_HUB_ENDOP. Bounded rather than a bare spin: the hub only
	 * advances on accelerometer samples, so if the accelerometer is not running
	 * this would otherwise hang the tracker forever.
	 */
	bool waitForEndOp() {
		constexpr uint32_t timeoutMillis = 50;
		const uint32_t start = millis();
		while (millis() - start < timeoutMillis) {
			const uint8_t status
				= m_shubRegisterInterface.readReg(StatusMasterMainPage);
			if (status & StatusMasterEndOp) {
				return true;
			}
			delay(1);
		}
		return false;
	}

	RegisterInterface& m_shubRegisterInterface;
	Logging::Logger& m_shubLogger;

	uint8_t m_auxId = 0;
	bool m_auxPolling = false;
	bool m_auxSplit = false;
	uint8_t m_auxDummyBytes = 0;
	uint8_t m_auxDataBytes = 0;
	uint8_t m_auxFirstDataBytes = 0;
};

}  // namespace SlimeVR::Sensors::SoftFusion::Drivers
