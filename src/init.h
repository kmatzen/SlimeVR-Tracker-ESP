/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2021 Eiren Rain & SlimeVR contributors

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

#include <Arduino.h>
#ifdef ESP8266
#include <user_interface.h>

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

typedef struct rtc_mem {
	uint32_t version;  // RTC memory version
	uint32_t rebootCount;  // Number of reboots
} rtc_mem_t;

extern "C" void preinit(void) {
	HardwareSerial Serialtemp(0);
	struct rst_info* resetreason;
	rtc_mem_t rtcMem;
	resetreason = ESP.getResetInfoPtr();
	// Offset 33 to avoid eboot command area
	ESP.rtcUserMemoryRead(33, (uint32_t*)&rtcMem, sizeof(struct rtc_mem));
	Serialtemp.begin(115200);
	Serialtemp.println(F("\r\n==== SLVR Boot ===="));
	Serialtemp.println(
		F("Reboot reason code: ") + String(ESP.getResetInfoPtr()->reason)
	);
	Serialtemp.println(F("Core Version: ") + ESP.getCoreVersion());
	Serialtemp.println(F("SDK version: ") + String(ESP.getSdkVersion()));
	Serialtemp.println(F("Sketch MD5: ") + String(ESP.getSketchMD5()));
	Serialtemp.println(F("Reset reason code: ") + String(resetreason->reason));
	Serialtemp.println(F("RTC Memory Version: ") + String(rtcMem.version));
	Serialtemp.println(F("RTC Memory Reboot Count: ") + String(rtcMem.rebootCount));
	Serialtemp.println("");
	Serialtemp.println(F("PRODUCT_NAME: ") + String(PRODUCT_NAME));
	Serialtemp.println(F("VENDOR_NAME: ") + String(VENDOR_NAME));
	Serialtemp.println(F("VENDOR_URL: ") + String(VENDOR_URL));
	Serialtemp.println(F("Firmware update URL: ") + String(UPDATE_ADDRESS));
	Serialtemp.println(F("BOARD: ") + String(BOARD));
	Serialtemp.println(F("HARDWARE_MCU: ") + String(HARDWARE_MCU));
	Serialtemp.println(F("PROTOCOL_VERSION: ") + String(PROTOCOL_VERSION));
	Serialtemp.println(F("FIRMWARE_VERSION: ") + String(FIRMWARE_VERSION));
	Serialtemp.println(F("SENSOR_DESC_LIST: "));
	Serialtemp.println(String(TOSTRING(SENSOR_DESC_LIST)));

	if (rtcMem.version != 0x01) {
		// First boot, initialize RTC memory
		rtcMem.version = 0x01;
		rtcMem.rebootCount = 0;
	}
	if (resetreason->reason != REASON_SOFT_WDT_RST
		&& resetreason->reason != REASON_EXCEPTION_RST
		&& resetreason->reason != REASON_WDT_RST) {
		// Not a crash, reset reboot counter
		rtcMem.rebootCount = 0;
	} else {
		// Crash detected, increment reboot counter
		rtcMem.rebootCount++;

		// If more than 3 consecutive crashes, enter safe mode
		if (rtcMem.rebootCount >= 3) {
			// Boot into UART download mode
			Serialtemp.println(
				F("\r\n\r\nEntering safe mode due to repeated crashes.")
			);
			Serialtemp.println(F("Entering flash mode..."));
			// Serial needs to be stay active for
			// rebootIntoUartDownloadMode to work.
			// Found out over testing.
			delay(1000);
			ESP.rebootIntoUartDownloadMode();
		}
	}
	ESP.rtcUserMemoryWrite(33, (uint32_t*)&rtcMem, sizeof(struct rtc_mem));

	Serialtemp.println(F("=== SLVR Boot end ==="));
	Serialtemp.flush();
	// Deinit UART for main code to reinitialize
	Serialtemp.end();
}
#endif
