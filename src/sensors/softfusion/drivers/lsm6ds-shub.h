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

#include <Wire.h>

#include <cstdint>

#include "../magdriver.h"
#include "logging/Logger.h"
#include "sensorinterface/RegisterInterface.h"

namespace SlimeVR::Sensors::SoftFusion::Drivers {

/**
 * LSM6DS-family sensor hub (the IMU acting as I2C master for an auxiliary
 * sensor).
 *
 * *** PARTIALLY VALIDATED ON HARDWARE (CheeseCake "Blueberry", LSM6DSV). ***
 *
 * Confirmed working on real silicon:
 *   - register bank entry and exit (WHO_AM_I reads 0x70 outside the bank and
 *     0x00 inside it, so the switch demonstrably takes effect both ways);
 *   - every hub configuration write lands (SLV0_ADD, SLV0_SUBADD, SLV0_CONFIG
 *     and MASTER_CONFIG all read back exactly as written);
 *   - the address encoding: a 7-bit slave address shifted left with RW in bit 0.
 *
 * NOT confirmed: that the hub ever performs a transaction. On the board tested,
 * STATUS_MASTER stays 0x00 -- neither SENS_HUB_ENDOP nor any NACK bit is set,
 * which means the master never starts a cycle rather than starting one and
 * failing. A pass-through probe of the auxiliary bus also found no device.
 * See tools/fusion-bench/README.md for the remaining hypotheses.
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

	// Bring-up diagnostics: bank-switch verification, configuration readback and
	// the pass-through bus probe. Off by default -- they are one-shot and cheap,
	// but they are debugging scaffolding, not behaviour. Build with
	// -D LSM6DS_SHUB_DEBUG to enable.
#ifdef LSM6DS_SHUB_DEBUG
	static constexpr bool ShubDebug = true;
#else
	static constexpr bool ShubDebug = false;
#endif

	// Main-page registers.
	static constexpr uint8_t FuncCfgAccess = 0x01;
	static constexpr uint8_t FuncCfgShubRegAccess = 1 << 6;
	// Hands OIS control from the SPI2 pins to the primary interface.
	static constexpr uint8_t FuncCfgOisCtrlFromUi = 1 << 0;
	static constexpr uint8_t FuncCfgSpi2Reset = 1 << 1;
	// UI_CTRL1_OIS from the primary interface. Same address as SPI2_CTRL1_OIS,
	// which is the same register seen from the auxiliary side.
	static constexpr uint8_t UiCtrl1Ois = 0x70;
	static constexpr uint8_t UiCtrl1OisSpi2ReadEn = 1 << 0;
	static constexpr uint8_t UiCtrl1OisGyroEn = 1 << 1;
	static constexpr uint8_t UiCtrl1OisAccelEn = 1 << 2;
	static constexpr uint8_t IfCfg = 0x03;
	static constexpr uint8_t IfCfgShubPuEn = 1 << 6;
	// STATUS_MASTER mirrored into the main page, so completion can be polled
	// without holding the embedded-function bank open.
	//
	// 0x48 per the LSM6DSV datasheet (DS13476 Rev 2, table 390 / register map
	// p50). This is NOT 0x39 as on some other parts in the family: on LSM6DSV
	// 0x39 is UI_OUTZ_H_A_OIS_DualC, an OIS output register, which reads ~0 on
	// a stationary device and therefore looks exactly like a hub that never
	// completes.
	static constexpr uint8_t StatusMasterMainPage = 0x48;
	static constexpr uint8_t StatusMasterEndOp = 1 << 0;
	static constexpr uint8_t StatusMasterSlave0Nack = 1 << 3;

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
	static constexpr uint8_t StatusMaster = 0x22;

	static constexpr uint8_t MasterConfigMasterOn = 1 << 2;
	static constexpr uint8_t MasterConfigPassThrough = 1 << 4;
	static constexpr uint8_t MasterConfigWriteOnce = 1 << 6;
	static constexpr uint8_t MasterConfigRstMasterRegs = 1 << 7;
	static constexpr uint8_t Slv0ConfigBatchEn = 1 << 3;

	// SHUB_ODR occupies SLV0_CONFIG[7:5] and is the rate at which the master
	// communicates: 000 = 1.875 Hz ... 100 = 120 Hz (reset default), 101 = 240
	// Hz. Writing SLV0_CONFIG without these bits silently selects 1.875 Hz --
	// a 533 ms cycle, which reads as a hub that never responds. Matched to the
	// accelerometer ODR.
	static constexpr uint8_t Slv0ConfigOdr120Hz = 0b100 << 5;
	static constexpr uint8_t MasterConfigStartConfig = 1 << 5;

	// START_CONFIG = 0 selects the accelerometer/gyroscope data-ready as the
	// hub trigger; 1 would wait on an external INT2 edge. Confirmed against the
	// datasheet (table 428), so this is not a knob.
	static constexpr uint8_t MasterConfigTrigger = 0;

	/**
	 * One-shot diagnostic: bridge the auxiliary bus onto the main I2C bus and
	 * probe for the magnetometer directly.
	 *
	 * This is the test that separates "my hub configuration is wrong" from "the
	 * auxiliary bus has nothing alive on it". In pass-through the IMU stops
	 * driving SDX/SCX and connects them straight to the host bus, so an ACK
	 * here means the part is present and powered and the fault is in the hub
	 * setup; silence means the hub was never the problem.
	 */
	void probeAuxPassThrough() {
		if (!ShubDebug || m_passThroughProbed) {
			return;
		}
		m_passThroughProbed = true;

		disableOisInterface();
		enableAuxPullups();

		enterShubBank(true);
		writeShub(MasterConfig, 0);  // master off before switching the mux
		enterShubBank(false);
		delay(2);
		enterShubBank(true);
		writeShub(MasterConfig, MasterConfigPassThrough);
		enterShubBank(false);
		delay(5);

		uint8_t found = 0;
		for (uint8_t addr :
			 {uint8_t{0x14},
			  uint8_t{0x15},
			  uint8_t{0x19},
			  uint8_t{0x7c},
			  uint8_t{0x0d},
			  uint8_t{0x30}}) {
			Wire.beginTransmission(addr);
			if (Wire.endTransmission() == 0) {
				m_shubLogger.info("Pass-through: device ACKed at 0x%02x", addr);
				found++;
			}
		}
		if (found == 0) {
			m_shubLogger.error("Pass-through: no device responded on the auxiliary bus"
			);
		}

		enterShubBank(true);
		writeShub(MasterConfig, 0);
		enterShubBank(false);
		delay(2);
	}

	/** Slave address of the auxiliary device, 7-bit. */
	void setAuxId(uint8_t deviceId) {
		m_auxId = deviceId;
		disableOisInterface();
		enableAuxPullups();
	}

	/**
	 * Internal pull-ups on the auxiliary bus.
	 *
	 * Must happen before *any* transaction, not just before continuous polling:
	 * boards commonly fit no external pull-ups on SDX/SCX (the CheeseCake
	 * Blueberry does not), and without them every hub transaction simply never
	 * completes.
	 */
	/**
	 * Releases the SDX/SCX pins from the OIS / SPI2 auxiliary interface.
	 *
	 * On LSM6DSV those two pins are shared: they are either the SPI2 (optical
	 * image stabilisation) slave interface or the sensor hub's I2C master, and
	 * they cannot be both. While SPI2 owns them the hub master has nothing to
	 * drive, which presents as the master never starting a cycle at all --
	 * STATUS_MASTER reading 0x00 with neither an end-of-operation nor a NACK
	 * bit, exactly what this driver saw on hardware.
	 *
	 * OIS_CTRL_FROM_UI moves control of the OIS chain from the SPI2 pins to the
	 * primary interface, which is what makes UI_CTRL1_OIS writable here at all;
	 * clearing that register then switches the chain off.
	 */
	void disableOisInterface() {
		const uint8_t access = m_shubRegisterInterface.readReg(FuncCfgAccess);
		m_shubRegisterInterface.writeReg(
			FuncCfgAccess,
			static_cast<uint8_t>(access | FuncCfgOisCtrlFromUi)
		);
		delay(1);

		// Reset the SPI2 block so it releases the pins, then leave the reset.
		m_shubRegisterInterface.writeReg(
			FuncCfgAccess,
			static_cast<uint8_t>(access | FuncCfgOisCtrlFromUi | FuncCfgSpi2Reset)
		);
		delay(1);
		m_shubRegisterInterface.writeReg(
			FuncCfgAccess,
			static_cast<uint8_t>(access | FuncCfgOisCtrlFromUi)
		);
		delay(1);

		// Accelerometer, gyroscope and SPI2 read paths of the OIS chain, all off.
		m_shubRegisterInterface.writeReg(UiCtrl1Ois, 0x00);
		delay(1);

		if (ShubDebug) {
			const uint8_t ois = m_shubRegisterInterface.readReg(UiCtrl1Ois);
			const uint8_t acc = m_shubRegisterInterface.readReg(FuncCfgAccess);
			m_shubLogger.info(
				"OIS disable: UI_CTRL1_OIS=0x%02x FUNC_CFG_ACCESS=0x%02x",
				ois,
				acc
			);
		}
	}

	void enableAuxPullups() {
		const uint8_t ifCfg = m_shubRegisterInterface.readReg(IfCfg);
		m_shubRegisterInterface.writeReg(IfCfg, ifCfg | IfCfgShubPuEn);
	}

	uint8_t readAux(uint8_t address) {
		enterShubBank(true);
		// Bit 0 set selects a read.
		writeShub(Slv0Add, static_cast<uint8_t>((m_auxId << 1) | 0x01));
		writeShub(Slv0SubAdd, address);
		writeShub(
			Slv0Config,
			static_cast<uint8_t>(Slv0ConfigOdr120Hz | 1)
		);  // one byte, not batched
		writeShub(
			MasterConfig,
			MasterConfigWriteOnce | MasterConfigMasterOn | MasterConfigTrigger
		);

		// Read the configuration back before letting the hub run. This
		// separates "the writes never landed" from "the writes landed but the
		// hub never started", which the STATUS_MASTER value alone cannot.
		if (ShubDebug && !m_configChecked) {
			m_configChecked = true;
			const uint8_t rAdd = m_shubRegisterInterface.readReg(Slv0Add);
			const uint8_t rSub = m_shubRegisterInterface.readReg(Slv0SubAdd);
			const uint8_t rCfg = m_shubRegisterInterface.readReg(Slv0Config);
			const uint8_t rMst = m_shubRegisterInterface.readReg(MasterConfig);
			m_shubLogger.info(
				"Shub cfg readback: SLV0_ADD=0x%02x SLV0_SUBADD=0x%02x "
				"SLV0_CONFIG=0x%02x MASTER_CONFIG=0x%02x",
				rAdd,
				rSub,
				rCfg,
				rMst
			);
		}

		enterShubBank(false);

		uint8_t status = 0;
		const bool ok = waitForEndOp(status);

		// A NACK means the hub ran its cycle and the slave did not acknowledge:
		// the master is fine and there is nothing answering at that address.
		// Distinguishing this from a hub that never started is the whole reason
		// this is logged separately from the timeout path.
		if (status & StatusMasterSlave0Nack) {
			m_shubLogger.error(
				"Aux device 0x%02x did not acknowledge (STATUS_MASTER=0x%02x)",
				m_auxId,
				status
			);
		} else if (ShubDebug) {
			m_shubLogger.info(
				"Aux read 0x%02x from 0x%02x: STATUS_MASTER=0x%02x",
				address,
				m_auxId,
				status
			);
		}

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
		writeShub(
			MasterConfig,
			MasterConfigWriteOnce | MasterConfigMasterOn | MasterConfigTrigger
		);
		enterShubBank(false);

		uint8_t status = 0;
		const bool ok = waitForEndOp(status);
		if (status & StatusMasterSlave0Nack) {
			m_shubLogger.error(
				"Aux device 0x%02x did not acknowledge a write (STATUS_MASTER=0x%02x)",
				m_auxId,
				status
			);
		}

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
				static_cast<uint8_t>(
					(dummyBytes + dataBytes) | Slv0ConfigBatchEn | Slv0ConfigOdr120Hz
				)
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
				static_cast<uint8_t>(
					(dummyBytes + firstData) | Slv0ConfigBatchEn | Slv0ConfigOdr120Hz
				)
			);

			writeShub(Slv1Add, static_cast<uint8_t>((m_auxId << 1) | 0x01));
			writeShub(Slv1SubAdd, static_cast<uint8_t>(dataReg + firstData));
			writeShub(
				Slv1Config,
				static_cast<uint8_t>(
					(dummyBytes + secondData) | Slv0ConfigBatchEn | Slv0ConfigOdr120Hz
				)
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

		// Verified once, because a silently failed bank switch is worse than a
		// failed magnetometer: the sensor-hub register addresses collide with
		// CTRL6/CTRL7/CTRL8 on the main page, so every "hub configuration"
		// write would instead be rewriting the gyroscope and accelerometer
		// full-scale settings.
		if (ShubDebug && enabled && !m_bankChecked) {
			m_bankChecked = true;
			const uint8_t readback = m_shubRegisterInterface.readReg(FuncCfgAccess);
			const uint8_t reg0f = m_shubRegisterInterface.readReg(0x0f);
			// Also verify the *exit*. The hub does not run while the bank is
			// held open, so a silently failed exit would look exactly like a
			// hub that never starts -- and would also make the STATUS_MASTER
			// poll read a hub register rather than the main-page mirror.
			m_shubRegisterInterface.writeReg(
				FuncCfgAccess,
				static_cast<uint8_t>(next & ~FuncCfgShubRegAccess)
			);
			const uint8_t outAccess = m_shubRegisterInterface.readReg(FuncCfgAccess);
			const uint8_t outWho = m_shubRegisterInterface.readReg(0x0f);
			m_shubRegisterInterface.writeReg(FuncCfgAccess, next);

			m_shubLogger.info(
				"Shub bank: in access=0x%02x reg0f=0x%02x | out access=0x%02x "
				"reg0f=0x%02x (expect 0x70)",
				readback,
				reg0f,
				outAccess,
				outWho
			);
		}
	}

	void writeShub(uint8_t reg, uint8_t value) {
		m_shubRegisterInterface.writeReg(reg, value);
	}

	/**
	 * Waits for SENS_HUB_ENDOP. Bounded rather than a bare spin: the hub only
	 * advances on accelerometer samples, so if the accelerometer is not running
	 * this would otherwise hang the tracker forever.
	 */
	bool waitForEndOp(uint8_t& statusOut) {
		statusOut = 0;
		// Generous on purpose. The hub cycles at SHUB_ODR, whose default is not
		// necessarily fast, and a single cycle has to complete before
		// SENS_HUB_ENDOP appears. A timeout shorter than one cycle is
		// indistinguishable from a hub that never runs -- which is precisely
		// the ambiguity this driver spent a bring-up session stuck in.
		constexpr uint32_t timeoutMillis = 500;
		const uint32_t start = millis();
		while (millis() - start < timeoutMillis) {
			const uint8_t status
				= m_shubRegisterInterface.readReg(StatusMasterMainPage);
			if (status & StatusMasterEndOp) {
				// STATUS_MASTER is read-to-clear, so the value has to be
				// captured here. Reading it again after this loop returns zero,
				// which previously made every transaction look clean.
				statusOut = status;
				return true;
			}
			delay(1);
		}

		// On failure, report both status registers. If the main-page mirror is
		// the wrong address for this part it will read as a constant while the
		// in-bank one moves, which distinguishes "polling the wrong register"
		// from "the hub never ran".
		const uint8_t mainPage = m_shubRegisterInterface.readReg(StatusMasterMainPage);
		enterShubBank(true);
		const uint8_t inBank = m_shubRegisterInterface.readReg(StatusMaster);
		enterShubBank(false);
		m_shubLogger.error(
			"Sensor hub timeout: STATUS_MASTER mainpage(0x39)=0x%02x "
			"bank(0x22)=0x%02x",
			mainPage,
			inBank
		);
		return false;
	}

	RegisterInterface& m_shubRegisterInterface;
	Logging::Logger& m_shubLogger;

	uint8_t m_auxId = 0;
	bool m_bankChecked = false;
	bool m_configChecked = false;
	bool m_passThroughProbed = false;
	bool m_auxPolling = false;
	bool m_auxSplit = false;
	uint8_t m_auxDummyBytes = 0;
	uint8_t m_auxDataBytes = 0;
	uint8_t m_auxFirstDataBytes = 0;
};

}  // namespace SlimeVR::Sensors::SoftFusion::Drivers
