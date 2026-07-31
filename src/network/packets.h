/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2021 Eiren Rain

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

#ifndef SLIMEVR_PACKETS_H_
#define SLIMEVR_PACKETS_H_

#include <cstdint>

#include "../consts.h"
#include "../sensors/sensor.h"

enum class SendPacketType : uint8_t {
	HeartBeat = 0,
	//  Rotation = 1,
	//  Gyro = 2,
	Handshake = 3,
	Accel = 4,
	// Mag = 5,
	// RawCalibrationData = 6,
	// CalibrationFinished = 7,
	// RawMagnetometer = 9,
	Serial = 11,
	BatteryLevel = 12,
	Tap = 13,
	Error = 14,
	SensorInfo = 15,
	// Rotation2 = 16,
	RotationData = 17,
	MagnetometerAccuracy = 18,
	SignalStrength = 19,
	Temperature = 20,
	// UserAction = 21,
	FeatureFlags = 22,
	// RotationAcceleration = 23,
	AcknowledgeConfigChange = 24,
	FlexData = 26,
	// PositionData = 27,
	TimeSync = 28,
	RotationDataTimestamped = 29,
	// Raw pre-calibration IMU samples, batched. Gated on the server
	// advertising PROTOCOL_RAW_SAMPLES -- see issue #23.
	RawSampleBatch = 30,
	Bundle = 100,
	Inspection = 105,
};

enum class ReceivePacketType : uint8_t {
	HeartBeat = 1,
	Vibrate = 2,
	Handshake = 3,
	Command = 4,
	Config = 8,
	PingPong = 10,
	SensorInfo = 15,
	FeatureFlags = 22,
	SetConfigFlag = 25,
	TimeSync = 28,
	// Start or stop raw sample streaming. Deliberately a command rather than a
	// stored setting: nothing about a capture session is persisted, so the
	// feature adds no SensorConfig field and costs nothing on boards that
	// compile it out. See issue #23.
	RawSampleControl = 30,
};

/**
 * Which of a sensor's two raw streams a batch carries.
 *
 * Separate streams rather than one interleaved buffer: they run at different
 * rates, and a single sequence number could not express "the gyroscope dropped
 * samples and the accelerometer did not".
 */
enum class RawSampleKind : uint8_t {
	Accel = 0,
	Gyro = 1,
};

/**
 * Which shape of RawSampleBatch packet follows.
 *
 * Mirrors the InspectionPacketType discriminator rather than spending a second
 * packet id, because the two shapes belong to one stream and are useless apart.
 */
enum class RawSampleBatchType : uint8_t {
	// Scale factors and nominal sample periods. Repeated periodically, since
	// UDP means a server that joined late would otherwise hold counts it cannot
	// convert into physical units.
	StreamInfo = 0,
	// A run of contiguous samples from one stream.
	Samples = 1,
	// What the streamer has actually done since the capture started.
	//
	// Sent over the network rather than printed, because reading the tracker's
	// serial port resets an ESP8266 and so ends the very capture being
	// diagnosed. Carries the firmware's own batch sequence numbers, which is
	// what distinguishes "the tracker skipped one" from "the network lost one".
	Stats = 2,
};

enum class InspectionPacketType : uint8_t {
	RawImuData = 1,
	FusedImuData = 2,
	CorrectionData = 3,
};

enum class InspectionDataType : uint8_t {
	Int = 1,
	Float = 2,
};

// From the SH-2 interface that BNO08x use.
enum class PacketErrorCode : uint8_t {
	NOT_APPLICABLE = 0,
	POWER_ON_RESET = 1,
	INTERNAL_SYSTEM_RESET = 2,
	WATCHDOG_TIMEOUT = 3,
	EXTERNAL_RESET = 4,
	OTHER = 5,
};

#pragma pack(push, 1)

template <typename T>
T swapEndianness(T value) {
	auto* bytes = reinterpret_cast<uint8_t*>(&value);
	std::reverse(bytes, bytes + sizeof(T));
	return value;
}

template <typename T>
struct BigEndian {
	BigEndian() = default;
	explicit(false) BigEndian(T val) { value = swapEndianness(val); }
	explicit(false) operator T() const { return swapEndianness(value); }

	T value{};
};

struct AccelPacket {
	BigEndian<float> x;
	BigEndian<float> y;
	BigEndian<float> z;
	uint8_t sensorId{};
};

struct BatteryLevelPacket {
	BigEndian<float> batteryVoltage;
	BigEndian<float> batteryPercentage;
};

struct TapPacket {
	uint8_t sensorId;
	uint8_t value;
};

struct ErrorPacket {
	uint8_t sensorId;
	uint8_t error;
};

struct SensorInfoPacket {
	uint8_t sensorId{};
	SensorStatus sensorState{};
	SensorTypeID sensorType{};
	BigEndian<SlimeVR::Configuration::SensorConfigBits> sensorConfigData{};
	bool hasCompletedRestCalibration{};
	SensorPosition sensorPosition{};
	SensorDataType sensorDataType{};
	// ADD NEW FIELDS ABOVE THIS COMMENT ^^^^^^^^
	// WARNING! Only for debug purposes and SHOULD ALWAYS BE LAST IN THE PACKET.
	// It WILL BE REMOVED IN THE FUTURE
	// Send TPS
	BigEndian<float> tpsCounterAveragedTps;
	BigEndian<float> dataCounterAveragedTps;
};

struct RotationDataPacket {
	uint8_t sensorId{};
	uint8_t dataType{};
	BigEndian<float> x;
	BigEndian<float> y;
	BigEndian<float> z;
	BigEndian<float> w;
	uint8_t accuracyInfo{};
};

/**
 * Rotation with the tracker's own measurement time attached.
 *
 * Identical to RotationDataPacket with a trailing timestamp, so the server can
 * convert the sample into its own timebase instead of assuming it arrived the
 * instant it was measured.
 *
 * The timestamp is the tracker's raw 32-bit `micros()`, which wraps roughly
 * every 71.6 minutes. It is sent raw and unwrapped by the server, which already
 * tracks the wrap for clock synchronisation -- doing it in one place avoids two
 * implementations that have to agree.
 */
struct RotationDataTimestampedPacket {
	uint8_t sensorId{};
	uint8_t dataType{};
	BigEndian<float> x;
	BigEndian<float> y;
	BigEndian<float> z;
	BigEndian<float> w;
	uint8_t accuracyInfo{};
	BigEndian<uint32_t> timestampMicros;
};

struct MagnetometerAccuracyPacket {
	uint8_t sensorId{};
	BigEndian<float> accuracyInfo;
};

struct SignalStrengthPacket {
	uint8_t sensorId;
	uint8_t signalStrength;
};

struct TemperaturePacket {
	uint8_t sensorId{};
	BigEndian<float> temperature;
};

struct AcknowledgeConfigChangePacket {
	uint8_t sensorId{};
	BigEndian<SensorToggles> configType;
};

struct FlexDataPacket {
	uint8_t sensorId{};
	BigEndian<float> flexLevel;
};

struct IntRawImuDataInspectionPacket {
	InspectionPacketType inspectionPacketType{};
	uint8_t sensorId{};
	InspectionDataType inspectionDataType{};

	BigEndian<uint32_t> rX;
	BigEndian<uint32_t> rY;
	BigEndian<uint32_t> rZ;
	uint8_t rA{};

	BigEndian<uint32_t> aX;
	BigEndian<uint32_t> aY;
	BigEndian<uint32_t> aZ;
	uint8_t aA{};

	BigEndian<uint32_t> mX;
	BigEndian<uint32_t> mY;
	BigEndian<uint32_t> mZ;
	uint8_t mA{};
};

struct FloatRawImuDataInspectionPacket {
	InspectionPacketType inspectionPacketType{};
	uint8_t sensorId{};
	InspectionDataType inspectionDataType{};

	BigEndian<float> rX;
	BigEndian<float> rY;
	BigEndian<float> rZ;
	uint8_t rA{};

	BigEndian<float> aX;
	BigEndian<float> aY;
	BigEndian<float> aZ;
	uint8_t aA{};

	BigEndian<float> mX;
	BigEndian<float> mY;
	BigEndian<float> mZ;
	uint8_t mA{};
};

struct SetConfigFlagPacket {
	uint8_t sensorId{};
	BigEndian<SensorToggles> flag;
	bool newState{};
};

#pragma pack(pop)

#endif  // SLIMEVR_PACKETS_H_
