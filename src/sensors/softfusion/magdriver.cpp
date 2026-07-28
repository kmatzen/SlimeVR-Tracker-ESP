/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2025 Gorbit99 & SlimeVR Contributors

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

#include "magdriver.h"

namespace SlimeVR::Sensors::SoftFusion {

std::vector<MagDefinition> MagDriver::supportedMags{
	MagDefinition{
		.name = "QMC6309",

		.deviceId = 0x7c,

		.whoAmIReg = 0x00,
		.expectedWhoAmI = 0x90,

		.dataWidth = MagDataWidth::SixByte,
		.dataReg = 0x01,

		.scale = 1.0f / 100.0f,  // 8 gauss full scale over 15 bits, to uT

		.setup =
			[](MagInterface& interface) {
				interface.writeByte(0x0b, 0x80);
				interface.writeByte(0x0b, 0x00);  // Soft reset
				delay(10);
				interface.writeByte(0x0b, 0x48);  // Set/reset on, 8g full range, 200Hz
				interface.writeByte(
					0x0a,
					0x21
				);  // LP filter 2, 8x Oversampling, normal mode
				return true;
			},
	},
	MagDefinition{
		.name = "IST8306",

		.deviceId = 0x19,

		.whoAmIReg = 0x00,
		.expectedWhoAmI = 0x06,

		.dataWidth = MagDataWidth::SixByte,
		.dataReg = 0x11,

		.scale = 1.0f / 13.2f,  // ~13.2 LSB/uT

		.setup =
			[](MagInterface& interface) {
				interface.writeByte(0x32, 0x01);  // Soft reset
				delay(50);
				interface.writeByte(0x30, 0x20);  // Noise suppression: low
				interface.writeByte(0x41, 0x2d);  // Oversampling: 32X
				interface.writeByte(0x31, 0x02);  // Continuous measurement @ 10Hz
				return true;
			},
	},
	MagDefinition{
		// Bosch BMM350. Present on the CheeseCake "Blueberry" LSM6DSV board.
		//
		// NOT YET VALIDATED ON HARDWARE. Register addresses and values are from
		// the Bosch BMM350 datasheet (BST-BMM350-DS001) and the vendor
		// BMM350_SensorAPI; the sequence below has been compiled but never run
		// against a real part. See the bring-up procedure in
		// tools/fusion-bench/README.md before trusting any of it.
		.name = "BMM350",

		// 0x14 with the address pin low, 0x15 with it high. Only the low
		// variant is probed; a board strapping it high will not be detected.
		.deviceId = 0x14,

		.whoAmIReg = 0x00,
		.expectedWhoAmI = 0x33,

		// Three bytes per axis (XLSB, LSB, MSB), nine in total, starting at
		// MAG_X_XLSB.
		.dataWidth = MagDataWidth::NineByte,
		.dataReg = 0x31,

		// 0.1 uT/LSB is a placeholder. The BMM350 needs OTP trim data and a
		// per-axis compensation to give a calibrated magnitude, which is not
		// implemented -- see the note below.
		.scale = 0.1f,

		// The BMM350 emits two dummy bytes before real data on a burst read.
		.dummyBytes = 2,

		.setup =
			[](MagInterface& interface) {
				interface.writeByte(0x7e, 0xb6);  // CMD: soft reset
				delay(25);  // datasheet: 24 ms

				// ODR 100 Hz with 4x averaging. Comfortably above the rate the
				// sensor hub will poll at, so the hub never reads a stale
				// sample twice.
				interface.writeByte(0x04, (0x02 << 4) | 0x04);  // PMU_CMD_AGGR_SET
				delay(2);

				interface.writeByte(0x06, 0x01);  // PMU_CMD: normal mode
				delay(40);  // datasheet: 38 ms suspend->normal

				return true;
			},

		.readTrim =
			[](MagInterface& interface, Bmm350Calibration& out) {
				// Each word costs several sensor-hub transactions, and the hub
				// is clocked by the accelerometer, so all 32 take a noticeable
				// fraction of a second. That is acceptable once at startup and
				// is why this is not attempted per-sample.
				uint16_t otp[kBmm350OtpWordCount] = {0};

				for (uint8_t word = 0; word < kBmm350OtpWordCount; word++) {
					interface.writeByte(
						0x50,  // OTP_CMD
						static_cast<uint8_t>(0x20 /* direct read */ | (word & 0x1f))
					);

					bool done = false;
					for (uint8_t attempt = 0; attempt < 10; attempt++) {
						delay(1);
						const uint8_t status = interface.readByte(0x55);
						if (status & 0xe0) {  // error bits
							return false;
						}
						if (status & 0x01) {  // command done
							done = true;
							break;
						}
					}
					if (!done) {
						return false;
					}

					const uint8_t msb = interface.readByte(0x52);
					const uint8_t lsb = interface.readByte(0x53);
					otp[word] = static_cast<uint16_t>(
						(static_cast<uint16_t>(msb) << 8) | lsb
					);
				}

				// Power the OTP block down once the trim is captured.
				interface.writeByte(0x50, 0x80);

				bmm350DecodeOtp(otp, out);
				return true;
			},
	},
};

float MagDriver::getScale() const { return detectedMag ? detectedMag->scale : 1.0f; }

uint8_t MagDriver::getDummyBytes() const {
	return detectedMag ? detectedMag->dummyBytes : 0;
}

bool MagDriver::init(MagInterface&& interface, bool supports9ByteMags) {
	for (auto& mag : supportedMags) {
		interface.setDeviceId(mag.deviceId);

		logger.info("Trying mag %s!", mag.name);

		uint8_t whoAmI = interface.readByte(mag.whoAmIReg);
		logger.debug(
			"%s: read 0x%02x from reg 0x%02x, expected 0x%02x",
			mag.name,
			whoAmI,
			mag.whoAmIReg,
			mag.expectedWhoAmI
		);
		if (whoAmI != mag.expectedWhoAmI) {
			continue;
		}

		if (!supports9ByteMags && mag.dataWidth == MagDataWidth::NineByte) {
			logger.error("The sensor doesn't support this mag!");
			return false;
		}

		logger.info("Found mag %s! Initializing", mag.name);

		if (!mag.setup(interface)) {
			logger.error("Mag %s failed to initialize!", mag.name);
			return false;
		}

		if (mag.readTrim) {
			if (mag.readTrim(interface, trim)) {
				logger.info("Read %s factory trim data", mag.name);
			} else {
				// Not fatal: bmm350Compensate falls back to nominal scaling, so
				// an unreadable OTP costs accuracy rather than the whole mag.
				logger.error(
					"Failed to read %s trim data; continuing uncompensated",
					mag.name
				);
			}
		}

		detectedMag = mag;

		break;
	}

	this->interface = interface;
	return detectedMag.has_value();
}

void MagDriver::startPolling() const {
	if (!detectedMag) {
		return;
	}

	interface.startPolling(
		detectedMag->dataReg,
		detectedMag->dataWidth,
		detectedMag->dummyBytes
	);
}

void MagDriver::stopPolling() const {
	if (!detectedMag) {
		return;
	}

	interface.stopPolling();
}

const char* MagDriver::getAttachedMagName() const {
	if (!detectedMag) {
		return nullptr;
	}

	return detectedMag->name;
}

}  // namespace SlimeVR::Sensors::SoftFusion
