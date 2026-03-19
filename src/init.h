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

#ifdef ESP32
#include "esp_chip_info.h"
#include "esp_intr_alloc.h"
#include "esp_system.h"
#include "nvs_flash.h"
#if defined(CONFIG_IDF_TARGET_ESP32C3)
#include "soc/rtc_cntl_reg.h"
#endif
#endif

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

typedef struct rtc_mem {
	uint32_t version;  // RTC memory version
	uint32_t rebootCount;  // Number of reboots
} rtc_mem_t;

bool g_safeModeDeferredFactoryResetRequested = false;

void failSafeProductInfo(Stream* Serial) {
	Serial->println(F("==== SLVR Product Info ===="));
	Serial->println(String(F("PRODUCT_NAME: ")) + String(PRODUCT_NAME));
	Serial->println(String(F("VENDOR_NAME: ")) + String(VENDOR_NAME));
	Serial->println(String(F("VENDOR_URL: ")) + String(VENDOR_URL));
	Serial->println(String(F("Firmware update URL: ")) + String(UPDATE_ADDRESS));
	Serial->println(String(F("BOARD: ")) + String(BOARD));
	Serial->println(String(F("BOARD NAME: ")) + String(boardName()));
	Serial->println(String(F("HARDWARE_MCU: ")) + String(HARDWARE_MCU));
	Serial->println(String(F("PROTOCOL_VERSION: ")) + String(PROTOCOL_VERSION));
	Serial->println(String(F("FIRMWARE_VERSION: ")) + String(FIRMWARE_VERSION));
	Serial->println(F("SENSOR_DESC_LIST: "));
	Serial->println(String(TOSTRING(SENSOR_DESC_LIST)));
}

void failSafeBootInfo(Stream* Serial, uint32_t resetreason, rtc_mem_t* rtcMem) {
	Serial->println(F("\r\n==== SLVR Boot ===="));
	Serial->println(String(F("Reboot reason code: ")) + String(resetreason));
	Serial->println(String(F("Core Version: ")) + String(ESP.getCoreVersion()));
	Serial->println(String(F("SDK version: ")) + String(ESP.getSdkVersion()));
	Serial->println(String(F("Sketch MD5: ")) + String(ESP.getSketchMD5()));
	Serial->println(String(F("RTC Memory Version: ")) + String(rtcMem->version));
	Serial->println(
		String(F("RTC Memory Reboot Count: ")) + String(rtcMem->rebootCount)
	);
	Serial->println();
	failSafeProductInfo(Serial);
}

bool cmdSafeModeFlashMode(Stream* Serial) {
#if defined(ESP8266)
	Serial->println(F("Entering flash mode..."));
	// Serial needs to be stay active for
	// rebootIntoUartDownloadMode to work.
	// Found out over testing.
	delay(1000);
	ESP.rebootIntoUartDownloadMode();
	return true;

#elif defined(CONFIG_IDF_TARGET_ESP32C3)
	Serial->println(F("Entering flash mode..."));
	// from https://esp32.com/viewtopic.php?t=33180
	delay(1000);
	REG_WRITE(RTC_CNTL_OPTION1_REG, RTC_CNTL_FORCE_DOWNLOAD_BOOT);
	esp_restart();
	return true;

#else
	Serial->println(
		F("The Firmware does not support reboot into UART download mode for "
		  "this board.")
	);
	Serial->println(
		F("You need to manually enter flash mode by holding BOOT button while "
		  "resetting.")
	);
#endif
	return false;  // Not supported, caller should handle with a message
}

void cmdSafeModeReboot() {
#if defined(ESP8266)
	// During preinit, system_restart() (used by ESP.restart()) is asynchronous —
	// it posts to the SDK task queue which hasn't started yet, causing a hang.
	// ESP.reset() calls __real_system_restart_local() which is a direct hardware reset.
	ESP.reset();
#else
	ESP.restart();
#endif
}

bool cmdSafeModeFactoryReset(Stream* Serial) {
	// WiFi and LittleFS operations are not reliable during preinit() on either
	// platform. Defer filesystem formatting to normal runtime.
	g_safeModeDeferredFactoryResetRequested = true;

#if defined(ESP8266)
	ESP.eraseConfig();  // Erase SDK config sectors (WiFi credentials etc.)
#elif defined(ESP32)
	nvs_flash_erase();  // Erase NVS (equivalent to ESP8266's config)
#endif

	if (Serial) {
		Serial->println(F("LittleFS format deferred to runtime."));
		Serial->println(F("Continuing normal boot for format + final reboot..."));
		Serial->flush();
	}
	return true;
}

void cmdSafeModeHelp(Stream* Serial) {
	Serial->println(F("Available commands:"));
	Serial->println(
		F("  FRST           - Factory reset (clears WiFi credentials "
		  "and config, then reboots)")
	);
	Serial->println(F("  REBOOT         - Restart the device"));
	Serial->println(F("  SET FLASHMODE  - Enter UART download / flash mode"));
	Serial->println(F("  HELP           - Show this help message"));
}

// Maximum input line length — longest valid command is "SET FLASHMODE" (13 chars).
// Anything beyond this is serial noise or garbage; discard to protect the heap.
// This is especially important because mainSave() runs during preinit/initVariant
// before full system initialization.
#define MAINSAVE_CMD_MAXLEN 32

// mainSave is for all the Shell needed loop in case of safe mode
bool mainSave(Stream* Serial) {
	String inputBuffer = "";

	Serial->println();
	Serial->println(F("=== SLVR Emergency Shell ==="));
	Serial->println(F("Safe mode command prompt. Type 'HELP' for available commands."));
	cmdSafeModeHelp(Serial);

	while (true) {
		while (Serial->available()) {
			char c = (char)Serial->read();
			if (c == '\r') {
				continue;  // ignore carriage return
			}
			if (c == '\n') {
				inputBuffer.trim();
				String cmd = inputBuffer;
				cmd.toUpperCase();
				inputBuffer = "";

				if (cmd == "FRST") {
					Serial->println(F("Performing factory reset..."));
					if (cmdSafeModeFactoryReset(Serial)) {
						return true;
					}
				} else if (cmd == "REBOOT") {
					Serial->println(F("Rebooting..."));
					cmdSafeModeReboot();
				} else if (cmd == "SET FLASHMODE") {
					cmdSafeModeFlashMode(Serial);
				} else if (cmd == "HELP") {
					cmdSafeModeHelp(Serial);
				} else if (cmd.length() > 0) {
					Serial->print(F("Unknown command: '"));
					Serial->print(cmd);
					Serial->println(F("'. Type 'HELP' for available commands."));
				}
			} else {
				if (inputBuffer.length() >= MAINSAVE_CMD_MAXLEN) {
					// Buffer overflow — likely serial noise or garbage data. Discard
					// and warn.
					Serial->println(F(""));
					Serial->println(F("Input too long, discarding line."));
					inputBuffer = "";
				} else {
					inputBuffer += c;
				}
			}
		}

#if defined(ESP8266)
		ESP.wdtFeed();  // Feed the watchdog to prevent reset while in this loop
#endif
		delay(1);
	}
}

#ifdef ESP8266
#include <user_interface.h>

extern "C" void preinit(void) {
	HardwareSerial Serial(0);
	struct rst_info* resetreason;
	rtc_mem_t rtcMem;
	resetreason = ESP.getResetInfoPtr();

	Serial.begin(115200);

	// Offset 33 to avoid eboot command area
	bool rtcOk = ESP.rtcUserMemoryRead(33, (uint32_t*)&rtcMem, sizeof(struct rtc_mem));

	if (!rtcOk || rtcMem.version != 0x01) {
		// First boot, initialize RTC memory
		rtcMem.version = 0x01;
		rtcMem.rebootCount = 0;
	}

	failSafeBootInfo(&Serial, resetreason->reason, &rtcMem);

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
			Serial.println();
			Serial.println();
			Serial.println(F("Entering safe mode due to repeated crashes."));
			Serial.println(F("Starting Emergency shell..."));
			if (mainSave(&Serial)) {
				Serial.println(F("Leaving emergency shell and continuing boot..."));
			} else {
				return;
			}
		}
	}
	ESP.rtcUserMemoryWrite(33, (uint32_t*)&rtcMem, sizeof(struct rtc_mem));

	Serial.println(F("=== SLVR Boot end ==="));
	Serial.flush();
	// Deinit UART for main code to reinitialize
	Serial.end();
}
#endif

#ifdef ESP32
// infos from https://circuitlabs.net/rtc-memory-usage-in-esp-idf/
RTC_DATA_ATTR rtc_mem_t rtcMem;

extern "C" void initVariant(void) {
	esp_reset_reason_t resetreason;

	resetreason = esp_reset_reason();

	if (rtcMem.version != 0x01) {
		// First boot, initialize RTC memory
		rtcMem.version = 0x01;
		rtcMem.rebootCount = 0;
	}

	// don't need to read RTC memory as it is cleard on a WDT reset on ESP32
	Serial.begin(115200);
	failSafeBootInfo(&Serial, resetreason, &rtcMem);

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
			mainSave(&Serial);
		}
	}

	Serial.println(F("=== SLVR Boot end ==="));
	Serial.flush();
}
#endif
