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
#include <user_interface.h>  // Include this to define rst_info

typedef struct rtc_mem {
	uint32_t version;  // RTC memory version
	uint32_t rebootCount;  // Number of reboots
} rtc_mem_t;

// Constructor function that runs during static initialization (before setup)
__attribute__((constructor)) void checkrebootcount() {
	Serial.begin(115200);
	struct rst_info* resetreason;
	rtc_mem_t rtcMem;
	resetreason = ESP.getResetInfoPtr();
	Serial.println("Reset reason code: " + String(resetreason->reason));
	// Offset 33 to avoid eboot command area
	ESP.rtcUserMemoryRead(33, (uint32_t*)&rtcMem, sizeof(struct rtc_mem));
	Serial.println("RTC Memory Version: " + String(rtcMem.version));
	Serial.println("RTC Memory Reboot Count: " + String(rtcMem.rebootCount));
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
			// Implement safe mode actions here
			// For example, disable certain features or notify the user
			Serial.begin(115200);
			Serial.println("Entering safe mode due to repeated crashes.");
			// Additional safe mode logic can be added here
#if defined(ESP8266)
			Serial.println("Entering flash mode...");
			delay(1000);
			ESP.rebootIntoUartDownloadMode();
#else
			Serial.println("Flash mode not supported on this platform!");
#endif
		}
	}
	Serial.println("Reboot Count: " + String(rtcMem.rebootCount));
	ESP.rtcUserMemoryWrite(33, (uint32_t*)&rtcMem, sizeof(struct rtc_mem));
}

#endif
