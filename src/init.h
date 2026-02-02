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
	Serialtemp.println(F("Reboot reason code: ") + String(resetreason->reason));
	Serialtemp.println(F("Core Version: ") + ESP.getCoreVersion());
	Serialtemp.println(F("SDK version: ") + String(ESP.getSdkVersion()));
	Serialtemp.println(F("Sketch MD5: ") + String(ESP.getSketchMD5()));
	Serialtemp.println(F("RTC Memory Version: ") + String(rtcMem.version));
	Serialtemp.println(F("RTC Memory Reboot Count: ") + String(rtcMem.rebootCount));
	Serialtemp.println();
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
			Serialtemp.println();
			Serialtemp.println();
			Serialtemp.println(F("Entering safe mode due to repeated crashes."));
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

#ifdef ESP32
#include "esp_system.h"
#if defined(CONFIG_IDF_TARGET_ESP32C3)
#include "soc/rtc_cntl_reg.h"
#endif
#include "esp_chip_info.h"
#include "esp_intr_alloc.h"

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

typedef struct rtc_mem {
	uint32_t version;  // RTC memory version
	uint32_t rebootCount;  // Number of reboots
} rtc_mem_t;

// infos from https://circuitlabs.net/rtc-memory-usage-in-esp-idf/
RTC_DATA_ATTR rtc_mem_t rtcMem;

extern "C" void initVariant(void) {
	esp_reset_reason_t resetreason;

	resetreason = esp_reset_reason();

	// don't need to read RTC memory as it is cleard on a WDT reset on ESP32

	Serial.begin(115200);
	Serial.println(F("\r\n==== SLVR Boot ===="));
	Serial.println(String(F("Reboot reason code: ")) + String(resetreason));
	Serial.println(String(F("Core Version: ")) + String(ESP.getCoreVersion()));
	Serial.println(String(F("SDK version: ")) + String(ESP.getSdkVersion()));
	Serial.println(String(F("Sketch MD5: ")) + String(ESP.getSketchMD5()));
	Serial.println(String(F("RTC Memory Version: ")) + String(rtcMem.version));
	Serial.println(String(F("RTC Memory Reboot Count: ")) + String(rtcMem.rebootCount));
	Serial.println();
	Serial.println(String(F("PRODUCT_NAME: ")) + String(PRODUCT_NAME));
	Serial.println(String(F("VENDOR_NAME: ")) + String(VENDOR_NAME));
	Serial.println(String(F("VENDOR_URL: ")) + String(VENDOR_URL));
	Serial.println(String(F("Firmware update URL: ")) + String(UPDATE_ADDRESS));
	Serial.println(String(F("BOARD: ")) + String(BOARD));
	Serial.println(String(F("HARDWARE_MCU: ")) + String(HARDWARE_MCU));
	Serial.println(String(F("PROTOCOL_VERSION: ")) + String(PROTOCOL_VERSION));
	Serial.println(String(F("FIRMWARE_VERSION: ")) + String(FIRMWARE_VERSION));
	Serial.println(F("SENSOR_DESC_LIST: "));
	Serial.println(String(TOSTRING(SENSOR_DESC_LIST)));

	if (rtcMem.version != 0x01) {
		// First boot, initialize RTC memory
		rtcMem.version = 0x01;
		rtcMem.rebootCount = 0;
	}
	if (resetreason != ESP_RST_PANIC && resetreason != ESP_RST_INT_WDT
		&& resetreason != ESP_RST_TASK_WDT && resetreason != ESP_RST_WDT
		&& resetreason != ESP_RST_CPU_LOCKUP) {
		// Not a crash, reset reboot counter
		rtcMem.rebootCount = 0;
	} else {
		// Crash detected
		// ESP32 ram gets cleared on WDT reset, force safe mode
		rtcMem.rebootCount = 4;

		// If more than 3 consecutive crashes, enter safe mode
		if (rtcMem.rebootCount >= 3) {
			// Boot into UART download mode
			Serial.println();
			Serial.println();

#if defined(CONFIG_IDF_TARGET_ESP32C3)
			Serial.println(F("Entering safe mode due to repeated crashes."));
			Serial.println(F("Entering flash mode..."));
			// from https://esp32.com/viewtopic.php?t=33180
			delay(1000);
			REG_WRITE(RTC_CNTL_OPTION1_REG, RTC_CNTL_FORCE_DOWNLOAD_BOOT);
			esp_restart();
#else
			Serial.println(
				F("The Firmware does not support reboot into UART download mode for "
				  "this board.")
			);
			Serial.println(
				F("You need to manually enter flash mode by holding BOOT button while "
				  "resetting.")
			);
			while (true) {
				delay(100);  // Halt
			}
#endif
		}
	}
	// no need to write, rtcMem is in RTC memory it does it automatically
	// ESP.rtcUserMemoryWrite(33, (uint32_t*)&rtcMem, sizeof(struct rtc_mem));

	Serial.println(F("=== SLVR Boot end ==="));
	Serial.flush();
}
#endif
