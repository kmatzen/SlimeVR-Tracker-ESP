// Reader/writer for the IMU log format consumed by fusion-bench.
//
// The format is deliberately plain CSV with a small comment header. It is
// diffable, hand-editable, trivially produced by a firmware serial logger, and
// trivially produced by numpy for imported public datasets.
//
//   # slimevr-imu-log v1
//   # gyr_ts 0.0016
//   # acc_ts 0.0016
//   # note   static bench, tracker flat on desk
//   t_us,ax,ay,az,gx,gy,gz,qw,qx,qy,qz
//   0,0.01,0.02,9.80,0.0011,0.0,0.0,1,0,0,0
//   1600,...
//
// Required columns: t_us, ax, ay, az, gx, gy, gz
// Optional columns: mx, my, mz          (magnetometer, arbitrary units)
//                   qw, qx, qy, qz      (ground-truth orientation, body->world)
//                   temp                (degrees C)
//
// Units: accelerometer m/s^2, gyroscope rad/s. These are the units VQF expects,
// so no scaling is applied anywhere in the harness.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "quatmath.h"

namespace fb {

struct Sample {
	uint64_t tUs = 0;
	Vec3 acc;
	Vec3 gyr;
	Vec3 mag;
	Quat gt;
	double temp = 0;
	// A row may carry only some of the sensors. Real IMUs run their
	// accelerometer and gyroscope at different rates (120 Hz and 240 Hz on an
	// LSM6DSV), so a capture interleaves rows rather than presenting a single
	// synchronised stream. An empty field means "no sample here", which is not
	// the same as a sample that happens to read zero.
	bool hasAcc = false;
	bool hasGyr = false;
	bool hasMag = false;
};

struct Dataset {
	std::string name;
	std::string note;
	// Nominal sample periods in seconds. If absent from the header they are
	// derived from the median timestamp delta.
	double gyrTs = 0;
	double accTs = 0;
	double magTs = 0;
	// Raw-count to physical-unit conversion, as emitted by the firmware's raw
	// sample logger. 1.0 means the file already holds physical units, which is
	// the case for synthetic datasets and for imported public datasets.
	double accScale = 1.0;
	double gyrScale = 1.0;
	double magScale = 1.0;
	bool hasMag = false;
	bool hasGroundTruth = false;
	bool hasTemp = false;
	// Lines that were not data rows -- firmware log output mixed into the
	// capture, or a truncated final row. Reported so a mangled capture cannot
	// pass for a clean one.
	size_t skippedLines = 0;
	std::vector<Sample> samples;

	double durationSec() const {
		if (samples.size() < 2) {
			return 0.0;
		}
		return static_cast<double>(samples.back().tUs - samples.front().tUs) * 1e-6;
	}
};

// Returns false and fills `error` on a malformed file.
bool loadDataset(const std::string& path, Dataset& out, std::string& error);
bool saveDataset(const std::string& path, const Dataset& ds, std::string& error);

}  // namespace fb
