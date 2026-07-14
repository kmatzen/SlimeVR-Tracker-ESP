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
#include "network/wifihandler.h"

#include <memory>

#include "GlobalVars.h"
#include "globals.h"
#if !ESP8266
#include "esp_wifi.h"
#include "esp_wifi_types.h"
#endif

namespace SlimeVR {

void WiFiNetwork::transitionState(std::unique_ptr<WiFiState>&& newState) {
	currentState = std::move(newState);
}

bool WiFiNetwork::isConnected() const {
	return getWiFiState() == WiFiReconnectionStatus::Success;
}

void WiFiNetwork::setWiFiCredentials(const char* SSID, const char* pass) {
	wifiProvisioning.stopProvisioning();
	transitionState(
		std::make_unique<ServerCredentialsAttemptState>(
			*this,
			std::string{SSID},
			std::string{pass}
		)
	);
}

IPAddress WiFiNetwork::getAddress() { return WiFi.localIP(); }

void WiFiNetwork::setUp() {
	logger.info("Setting up WiFi");
	WiFi.persistent(true);
	WiFi.mode(WIFI_STA);
	WiFi.hostname("SlimeVR FBT Tracker");
	logger.info(
		"Loaded credentials for SSID '%s' and pass length %d",
		getSSID().c_str(),
		getPassword().length()
	);

	transitionState(std::make_unique<SavedCredentialsAttemptState>(*this));

#if ESP8266
#if POWERSAVING_MODE == POWER_SAVING_NONE
	WiFi.setSleepMode(WIFI_NONE_SLEEP);
#elif POWERSAVING_MODE == POWER_SAVING_MINIMUM
	WiFi.setSleepMode(WIFI_MODEM_SLEEP);
#elif POWERSAVING_MODE == POWER_SAVING_MODERATE
	WiFi.setSleepMode(WIFI_MODEM_SLEEP, 10);
#elif POWERSAVING_MODE == POWER_SAVING_MAXIMUM
	WiFi.setSleepMode(WIFI_LIGHT_SLEEP, 10);
#error "MAX POWER SAVING NOT WORKING YET, please disable!"
#endif
#else
#if POWERSAVING_MODE == POWER_SAVING_NONE
	WiFi.setSleep(WIFI_PS_NONE);
#elif POWERSAVING_MODE == POWER_SAVING_MINIMUM
	WiFi.setSleep(WIFI_PS_MIN_MODEM);
#elif POWERSAVING_MODE == POWER_SAVING_MODERATE \
	|| POWERSAVING_MODE == POWER_SAVING_MAXIMUM
	wifi_config_t conf;
	if (esp_wifi_get_config(WIFI_IF_STA, &conf) == ESP_OK) {
		conf.sta.listen_interval = 10;
		esp_wifi_set_config(WIFI_IF_STA, &conf);
		WiFi.setSleep(WIFI_PS_MAX_MODEM);
	} else {
		logger.error("Unable to get WiFi config, power saving not enabled!");
	}
#endif
#endif
}

String WiFiNetwork::getSSID() {
#if ESP8266
	return WiFi.SSID();
#else
	// Necessary, because without a WiFi.begin(), ESP32 is not kind enough to load the
	// SSID on its own, for whatever reason
	wifi_config_t wifiConfig;
	esp_wifi_get_config((wifi_interface_t)ESP_IF_WIFI_STA, &wifiConfig);
	return {reinterpret_cast<char*>(wifiConfig.sta.ssid)};
#endif
}

String WiFiNetwork::getPassword() {
#if ESP8266
	return WiFi.psk();
#else
	// Same as above
	wifi_config_t wifiConfig;
	esp_wifi_get_config((wifi_interface_t)ESP_IF_WIFI_STA, &wifiConfig);
	return {reinterpret_cast<char*>(wifiConfig.sta.password)};
#endif
}

WiFiNetwork::WiFiReconnectionStatus WiFiNetwork::getWiFiState() const {
	return currentState->toStatus();
}

void WiFiNetwork::upkeep() {
	wifiProvisioning.upkeepProvisioning();

	currentState->tick();
}

const char* WiFiNetwork::statusToReasonString(wl_status_t status) {
	switch (status) {
		case WL_DISCONNECTED:
			return "Timeout";
#ifdef ESP8266
		case WL_WRONG_PASSWORD:
			return "Wrong password";
		case WL_CONNECT_FAILED:
			return "Connection failed";
#elif ESP32
		case WL_CONNECT_FAILED:
			return "Wrong password";
#endif

		case WL_NO_SSID_AVAIL:
			return "SSID not found";
		default:
			return "Unknown";
	}
}

WiFiNetwork::WiFiFailureReason WiFiNetwork::statusToFailure(wl_status_t status) {
	switch (status) {
		case WL_DISCONNECTED:
			return WiFiFailureReason::Timeout;
#ifdef ESP8266
		case WL_WRONG_PASSWORD:
			return WiFiFailureReason::WrongPassword;
#elif ESP32
		case WL_CONNECT_FAILED:
			return WiFiFailureReason::WrongPassword;
#endif

		case WL_NO_SSID_AVAIL:
			return WiFiFailureReason::SSIDNotFound;
		default:
			return WiFiFailureReason::Unknown;
	}
}

WiFiNetwork::WiFiState::WiFiState(WiFiNetwork& context, WiFiReconnectionStatus status)
	: context{context}
	, status{status} {}

WiFiNetwork::WiFiReconnectionStatus WiFiNetwork::WiFiState::toStatus() const {
	return status;
}

WiFiNetwork::ConnectingState::ConnectingState(
	WiFiNetwork& context,
	WiFiReconnectionStatus status
)
	: WiFiState{context, status} {}

void WiFiNetwork::ConnectingState::reportWifiProgress() {
	if (context.wifiReportTimeout.elapsed()) {
		context.wifiReportTimeout.restart();
		Serial.print(".");
	}
}

void WiFiNetwork::ConnectingState::tick() { this->reportWifiProgress(); }

bool WiFiNetwork::ConnectingState::tryConnecting(
	bool phyModeG,
	const char* SSID,
	const char* pass
) {
#if ESP8266
	if (phyModeG) {
		WiFi.setPhyMode(WIFI_PHY_MODE_11G);
		if constexpr (USE_ATTENUATION) {
			WiFi.setOutputPower(20.0 - ATTENUATION_G);
		}
	} else {
		WiFi.setPhyMode(WIFI_PHY_MODE_11N);
		if constexpr (USE_ATTENUATION) {
			WiFi.setOutputPower(20.0 - ATTENUATION_N);
		}
	}
#else
	if (phyModeG) {
		return false;
	}
#endif

	setStaticIPIfDefined();
	if (SSID == nullptr) {
		WiFi.begin();
	} else {
		WiFi.begin(SSID, pass);
	}
	return true;
}

void WiFiNetwork::ConnectingState::setStaticIPIfDefined() {
#ifdef WIFI_USE_STATICIP
	const IPAddress ip(WIFI_STATIC_IP);
	const IPAddress gateway(WIFI_STATIC_GATEWAY);
	const IPAddress subnet(WIFI_STATIC_SUBNET);
	WiFi.config(ip, gateway, subnet);
#endif
}

void WiFiNetwork::ConnectingState::showConnectionAttemptFailed(const char* type) const {
	context.logger.error(
		"Can't connect from %s credentials, error: %d, reason: %s.",
		type,
		static_cast<int>(statusToFailure(WiFi.status())),
		statusToReasonString(WiFi.status())
	);
}

WiFiNetwork::NotSetupState::NotSetupState(WiFiNetwork& context)
	: WiFiState{context, WiFiReconnectionStatus::NotSetup} {}

void WiFiNetwork::NotSetupState::tick() {
	// Do nothing
}

WiFiNetwork::SavedCredentialsAttemptState::SavedCredentialsAttemptState(
	WiFiNetwork& context
)
	: ConnectingState{context, WiFiReconnectionStatus::SavedAttempt} {}

void WiFiNetwork::SavedCredentialsAttemptState::tick() {
	ConnectingState::tick();

	if (getSSID().length() == 0) {
		context.logger.debug("Skipping saved credentials attempt on 0-length SSID...");
		context.transitionState(
			std::make_unique<HardcodedCredentialsAttemptState>(context)
		);
		return;
	}

	if (!setup) {
		tryConnecting();
		setup = true;
		return;
	}

	if (WiFi.status() == WL_CONNECTED) {
		context.transitionState(std::make_unique<ConnectedState>(context));
		return;
	}

	if (!wifiTimeout.elapsed() && WiFi.status() == WL_DISCONNECTED) {
		return;
	}

	showConnectionAttemptFailed("saved");

	if (WiFi.status() == WL_DISCONNECTED && !retriedOnG) {
		retriedOnG = true;
		context.logger.debug("Trying saved credentials with PHY Mode G...");
		tryConnecting(true);
		wifiTimeout.restart();
		return;
	}

	context.transitionState(
		std::make_unique<HardcodedCredentialsAttemptState>(context)
	);
}

WiFiNetwork::HardcodedCredentialsAttemptState::HardcodedCredentialsAttemptState(
	WiFiNetwork& context
)
	: ConnectingState{context, WiFiReconnectionStatus::HardcodeAttempt} {}

void WiFiNetwork::HardcodedCredentialsAttemptState::tick() {
#if defined(WIFI_CREDS_SSID) && defined(WIFI_CREDS_PASSWD)
	ConnectingState::tick();

	if (!setup) {
		setup = true;
		WiFi.persistent(false);
		auto result = tryConnecting(false, WIFI_CREDS_SSID, WIFI_CREDS_PASSWD);
		WiFi.persistent(true);
		if (!result) {
			context.transitionState(std::make_unique<FailedState>(context));
		}
		return;
	}

	if (WiFi.status() == WL_CONNECTED) {
		context.transitionState(std::make_unique<ConnectedState>(context));
		return;
	}

	if (!wifiTimeout.elapsed() && WiFi.status() == WL_DISCONNECTED) {
		return;
	}

	showConnectionAttemptFailed("hardcoded");

	if (WiFi.status() == WL_DISCONNECTED && !retriedOnG) {
		retriedOnG = true;
		context.logger.debug("Trying hardcoded credentials with PHY Mode G...");
		// Don't need to save hardcoded credentials
		WiFi.persistent(false);
		auto result = tryConnecting(true, WIFI_CREDS_SSID, WIFI_CREDS_PASSWD);
		wifiTimeout.restart();
		WiFi.persistent(true);

		if (!result) {
			context.transitionState(std::make_unique<FailedState>(context));
		}
		return;
	}

	context.transitionState(std::make_unique<FailedState>(context));
#else
	context.transitionState(std::make_unique<FailedState>(context));
#endif
}

WiFiNetwork::ServerCredentialsAttemptState::ServerCredentialsAttemptState(
	WiFiNetwork& context,
	std::string&& SSID,
	std::string&& pass
)
	: ConnectingState{context, WiFiReconnectionStatus::ServerCredAttempt}
	, SSID{std::move(SSID)}
	, pass{std::move(pass)} {
	context.hadWifi = false;
	tryConnecting(false, SSID.c_str(), pass.c_str());
}

void WiFiNetwork::ServerCredentialsAttemptState::tick() {
	ConnectingState::tick();

	if (WiFi.status() == WL_CONNECTED) {
		context.transitionState(std::make_unique<ConnectedState>(context));
		return;
	}

	if (!wifiTimeout.elapsed() && WiFi.status() == WL_DISCONNECTED) {
		return;
	}

	if (WiFi.status() == WL_DISCONNECTED && !retriedOnG) {
		tryConnecting(true);
		retriedOnG = true;
		return;
	}

	context.transitionState(std::make_unique<FailedState>(context));
}

WiFiNetwork::FailedState::FailedState(WiFiNetwork& context)
	: WiFiState{context, WiFiReconnectionStatus::Failed} {}

void WiFiNetwork::FailedState::tick() {
#if ESP8266
	if constexpr (USE_ATTENUATION) {
		WiFi.setOutputPower(20.0 - ATTENUATION_N);
	}
	WiFi.setPhyMode(WIFI_PHY_MODE_11N);
#endif
	// Start smart config
	if (!context.hadWifi && !WiFi.smartConfigDone() && wifiTimeout.elapsed()) {
		if (WiFi.status() != WL_IDLE_STATUS) {
			context.logger.error(
				"Can't connect from any credentials, error: %d, reason: %s.",
				static_cast<int>(statusToFailure(WiFi.status())),
				statusToReasonString(WiFi.status())
			);
			wifiTimeout.restart();
		}
		wifiProvisioning.startProvisioning();
	}
}

WiFiNetwork::ConnectedState::ConnectedState(WiFiNetwork& context)
	: WiFiState{context, WiFiReconnectionStatus::Success} {
	wifiProvisioning.stopProvisioning();
	statusManager.setStatus(SlimeVR::Status::WIFI_CONNECTING, false);
	context.hadWifi = true;
	context.logger.info(
		"Connected successfully to SSID '%s', IP address %s",
		getSSID().c_str(),
		WiFi.localIP().toString().c_str()
	);
}

void WiFiNetwork::ConnectedState::tick() {
	if (WiFi.status() != WL_CONNECTED) {
		statusManager.setStatus(SlimeVR::Status::WIFI_CONNECTING, true);
		context.logger.warn("Connection to WiFi lost, reconnecting...");
		context.transitionState(
			std::make_unique<SavedCredentialsAttemptState>(context)
		);
		return;
	}

	if (rssiTimeout.elapsed()) {
		uint8_t signalStrength = WiFi.RSSI();
		networkConnection.sendSignalStrength(signalStrength);
		rssiTimeout.restart();
	}
}

}  // namespace SlimeVR
