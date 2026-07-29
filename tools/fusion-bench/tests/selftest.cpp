// Self-tests for the benchmark harness itself.
//
// These do not test the firmware. They test that the *measuring instrument* is
// correct -- that the quaternion algebra, the error decomposition, and the
// determinism guarantees hold. A benchmark whose own arithmetic is wrong is
// worse than no benchmark, because it produces confident numbers.
#include <array>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

#include "dataset.h"
#include "gyroscale.h"
#include "metrics.h"
#include "quatmath.h"
#include "synth.h"

// The sensor-hub FIFO assembler, compiled straight from the firmware tree. It
// has no Arduino dependency precisely so it can be tested here.
#include "../../../src/configuration/accelmodel.h"
#include "../../../src/configuration/gyroscalecmd.h"
#include "../../../src/sensors/softfusion/drivers/bmm350comp.h"
#include "../../../src/sensors/softfusion/drivers/magfifo.h"
#include "../../../src/sensors/softfusion/errormodel.h"
#include "../../../src/sensors/softfusion/onlineestimator.h"
#include "../../../src/sensors/softfusion/sixposition.h"

using namespace fb;

namespace {

int gFailures = 0;
int gChecks = 0;

void checkNear(const char* what, double got, double want, double tol, int line) {
	gChecks++;
	if (!(std::fabs(got - want) <= tol)) {
		std::fprintf(
			stderr,
			"FAIL line %d: %s: got %.9f want %.9f (tol %.9f)\n",
			line,
			what,
			got,
			want,
			tol
		);
		gFailures++;
	}
}

void checkTrue(const char* what, bool ok, int line) {
	gChecks++;
	if (!ok) {
		std::fprintf(stderr, "FAIL line %d: %s\n", line, what);
		gFailures++;
	}
}

#define NEAR(got, want, tol) checkNear(#got, (got), (want), (tol), __LINE__)
#define TRUE_(cond) checkTrue(#cond, (cond), __LINE__)

void testQuatBasics() {
	// Identity behaves.
	Quat i{1, 0, 0, 0};
	NEAR(qAngle(i), 0.0, 1e-12);
	NEAR(qHeading(i), 0.0, 1e-12);
	NEAR(qInclination(i), 0.0, 1e-12);

	// A 90 deg yaw rotates +x to +y.
	Quat yaw90 = qFromYaw(deg2rad(90));
	Vec3 v = qRotate(yaw90, Vec3{1, 0, 0});
	NEAR(v.x, 0.0, 1e-9);
	NEAR(v.y, 1.0, 1e-9);
	NEAR(v.z, 0.0, 1e-9);

	// ...and is pure heading: no inclination component.
	NEAR(rad2deg(qHeading(yaw90)), 90.0, 1e-9);
	NEAR(rad2deg(qInclination(yaw90)), 0.0, 1e-9);
	NEAR(rad2deg(qAngle(yaw90)), 90.0, 1e-9);

	// A 30 deg roll about x is pure inclination: no heading component.
	double a = deg2rad(30) / 2;
	Quat roll30{std::cos(a), std::sin(a), 0, 0};
	NEAR(rad2deg(qHeading(roll30)), 0.0, 1e-9);
	NEAR(rad2deg(qInclination(roll30)), 30.0, 1e-9);

	// Inclination must be invariant to any rotation about the world vertical.
	// This is the property that makes the heading/inclination split meaningful.
	for (double psi = -170; psi <= 170; psi += 40) {
		Quat composed = qMul(qFromYaw(deg2rad(psi)), roll30);
		NEAR(rad2deg(qInclination(composed)), 30.0, 1e-6);
	}

	// Conjugate inverts.
	Quat r = qMul(yaw90, qConj(yaw90));
	NEAR(qAngle(r), 0.0, 1e-9);
}

void testIntegration() {
	// Integrating a constant rate about z for 1 s must give exactly that angle.
	const double rate = deg2rad(45.0);
	Quat q{1, 0, 0, 0};
	const int n = 1000;
	for (int i = 0; i < n; i++) {
		q = qIntegrate(q, Vec3{0, 0, rate}, 1.0 / n);
	}
	NEAR(rad2deg(qHeading(q)), 45.0, 1e-6);
	NEAR(rad2deg(qInclination(q)), 0.0, 1e-6);

	// Step count must not matter for a constant rate about a fixed axis.
	Quat q2{1, 0, 0, 0};
	q2 = qIntegrate(q2, Vec3{0, 0, rate}, 1.0);
	NEAR(rad2deg(qHeading(q2)), 45.0, 1e-9);
}

void testMetricsOnPerfectEstimate() {
	SynthParams sp;
	sp.durationSec = 10;
	sp.rateHz = 100;
	// A perfect estimate scores zero on the reference-free tilt check only if
	// the accelerometer itself is perfect, since that check compares the
	// *measured* specific force against vertical. Zero the accel error model so
	// this test isolates the estimator from the sensor.
	sp.accelBias = 0.0;
	sp.accelNoise = 0.0;

	Dataset ds;
	std::string err;
	TRUE_(generate("tumble", sp, ds, err));

	// Feed the ground truth back in as if it were the estimate.
	RunResult run;
	for (const Sample& s : ds.samples) {
		run.est.push_back(s.gt);
	}
	Metrics m = computeMetrics(ds, run);
	TRUE_(m.hasGroundTruth);
	NEAR(m.totalErrorDegRms, 0.0, 1e-4);
	NEAR(m.headingErrorDegRms, 0.0, 1e-4);
	NEAR(m.inclinationErrorDegRms, 0.0, 1e-4);
	// Tumble has no linear acceleration, so with a clean accelerometer the
	// tilt check is exact too.
	NEAR(m.tiltErrorDegRms, 0.0, 1e-4);

	// Conversely, accelerometer bias must show up in the tilt metric even when
	// the orientation estimate is perfect -- that is what the metric is for,
	// and a version of it that ignored sensor error would be useless.
	SynthParams biased = sp;
	biased.accelBias = 0.2;  // m/s^2, ~1.2 deg of apparent tilt
	Dataset bds;
	TRUE_(generate("tumble", biased, bds, err));
	RunResult brun;
	for (const Sample& s : bds.samples) {
		brun.est.push_back(s.gt);
	}
	Metrics bm = computeMetrics(bds, brun);
	TRUE_(bm.tiltErrorDegRms > 0.5);
	// ...while the ground-truth metrics, which do not look at the
	// accelerometer, must remain clean.
	NEAR(bm.totalErrorDegRms, 0.0, 1e-4);
}

void testConstantHeadingOffsetIsRemoved() {
	SynthParams sp;
	sp.durationSec = 10;
	sp.rateHz = 100;
	Dataset ds;
	std::string err;
	TRUE_(generate("tumble", sp, ds, err));

	// A 6-DoF estimator's absolute yaw is arbitrary. Offsetting the whole
	// trajectory by a constant heading must not be scored as error.
	const Quat offset = qFromYaw(deg2rad(37.0));
	RunResult run;
	for (const Sample& s : ds.samples) {
		run.est.push_back(qMul(offset, s.gt));
	}
	Metrics m = computeMetrics(ds, run);
	NEAR(m.totalErrorDegRms, 0.0, 1e-3);
	NEAR(m.headingErrorDegRms, 0.0, 1e-3);
	NEAR(m.inclinationErrorDegRms, 0.0, 1e-3);

	// A constant *tilt* offset, by contrast, must be scored -- it is a real
	// error and there is no reason to forgive it.
	const double a = deg2rad(5.0) / 2;
	const Quat tilt{std::cos(a), std::sin(a), 0, 0};
	RunResult tilted;
	for (const Sample& s : ds.samples) {
		tilted.est.push_back(qMul(tilt, s.gt));
	}
	Metrics mt = computeMetrics(ds, tilted);
	TRUE_(mt.inclinationErrorDegRms > 4.0);
}

void testHeadingDriftIsMeasured() {
	// Construct an estimate that drifts in yaw at a known rate and confirm the
	// metric recovers it.
	SynthParams sp;
	sp.durationSec = 60;
	sp.rateHz = 100;
	Dataset ds;
	std::string err;
	TRUE_(generate("static", sp, ds, err));

	const double driftDegPerMin = 3.0;
	RunResult run;
	for (const Sample& s : ds.samples) {
		double t = static_cast<double>(s.tUs) * 1e-6;
		double yaw = deg2rad(driftDegPerMin * t / 60.0);
		run.est.push_back(qMul(qFromYaw(yaw), s.gt));
	}
	Metrics m = computeMetrics(ds, run);
	NEAR(m.headingDriftDegPerMin, driftDegPerMin, 1e-3);
}

void testDatasetRoundTrip() {
	SynthParams sp;
	sp.durationSec = 2;
	sp.rateHz = 100;
	sp.withMag = true;
	Dataset ds;
	std::string err;
	TRUE_(generate("walk", sp, ds, err));

	const std::string path = "build/_roundtrip.csv";
	TRUE_(saveDataset(path, ds, err));

	Dataset back;
	TRUE_(loadDataset(path, back, err));
	TRUE_(back.samples.size() == ds.samples.size());
	TRUE_(back.hasMag == ds.hasMag);
	TRUE_(back.hasGroundTruth == ds.hasGroundTruth);
	NEAR(back.gyrTs, ds.gyrTs, 1e-9);

	// Values survive the text round trip to the precision we write.
	for (size_t i = 0; i < ds.samples.size(); i += 17) {
		NEAR(back.samples[i].acc.x, ds.samples[i].acc.x, 1e-5);
		NEAR(back.samples[i].gyr.z, ds.samples[i].gyr.z, 1e-8);
		NEAR(back.samples[i].gt.w, ds.samples[i].gt.w, 1e-8);
	}
	std::remove(path.c_str());
}

void testDeterminism() {
	// Same seed must produce a bit-identical dataset, and the same dataset must
	// produce a bit-identical fusion result. Without this, no baseline
	// comparison means anything.
	SynthParams sp;
	sp.durationSec = 5;
	sp.rateHz = 200;
	sp.seed = 42;

	Dataset a;
	Dataset b;
	std::string err;
	TRUE_(generate("tumble", sp, a, err));
	TRUE_(generate("tumble", sp, b, err));
	TRUE_(a.samples.size() == b.samples.size());
	bool identical = true;
	for (size_t i = 0; i < a.samples.size(); i++) {
		identical = identical && a.samples[i].gyr.x == b.samples[i].gyr.x
				 && a.samples[i].acc.z == b.samples[i].acc.z;
	}
	TRUE_(identical);

	BenchParams bp;
	RunResult r1 = runFusion(a, bp);
	RunResult r2 = runFusion(a, bp);
	bool sameRun = r1.est.size() == r2.est.size();
	for (size_t i = 0; sameRun && i < r1.est.size(); i++) {
		sameRun = r1.est[i].w == r2.est[i].w && r1.est[i].x == r2.est[i].x
			   && r1.est[i].y == r2.est[i].y && r1.est[i].z == r2.est[i].z;
	}
	TRUE_(sameRun);

	// A different seed must actually change the data, otherwise the noise model
	// is silently inert and every test above is vacuous.
	SynthParams sp2 = sp;
	sp2.seed = 43;
	Dataset c;
	TRUE_(generate("tumble", sp2, c, err));
	bool differs = false;
	for (size_t i = 0; i < a.samples.size(); i++) {
		if (a.samples[i].gyr.x != c.samples[i].gyr.x) {
			differs = true;
			break;
		}
	}
	TRUE_(differs);
}

void testCleanStaticConverges() {
	// End-to-end: with no bias and no noise, a stationary tracker must not
	// drift and must sit level. This is the weakest possible claim about VQF,
	// which is exactly why it is a good canary -- if this breaks, something is
	// badly wrong in the wiring between the harness and the filter.
	SynthParams sp;
	sp.durationSec = 30;
	sp.rateHz = 250;
	sp.gyroBiasDps = 0.0;
	sp.gyroNoiseDps = 0.0;
	sp.accelBias = 0.0;
	sp.accelNoise = 0.0;

	Dataset ds;
	std::string err;
	TRUE_(generate("static", sp, ds, err));

	Metrics m = computeMetrics(ds, runFusion(ds, BenchParams{}));
	NEAR(m.headingDriftDegPerMin, 0.0, 0.01);
	NEAR(m.tiltErrorDegRms, 0.0, 0.05);
	NEAR(m.totalErrorDegRms, 0.0, 0.05);

	// And rest must be detected, since the tracker is by construction at rest.
	TRUE_(m.firstRestSec >= 0.0);
}

void testGyroBiasProducesDrift() {
	// The converse check: a known uncorrected bias must show up as drift, so we
	// know the metric is sensitive to the thing it claims to measure. Rest bias
	// estimation is disabled here so the bias survives to be observed.
	SynthParams sp;
	sp.durationSec = 30;
	sp.rateHz = 250;
	sp.gyroBiasDps = 1.0;
	sp.gyroNoiseDps = 0.0;
	sp.accelBias = 0.0;
	sp.accelNoise = 0.0;

	Dataset ds;
	std::string err;
	TRUE_(generate("static", sp, ds, err));

	// restMinT longer than the run keeps VQF from ever declaring rest.
	BenchParams bp;
	bp.restMinT = 1e6;
	Metrics m = computeMetrics(ds, runFusion(ds, bp));

	// 1 deg/s about z, uncorrected, is 60 deg/min of heading drift.
	TRUE_(std::fabs(m.headingDriftDegPerMin) > 5.0);
}

void testRawCaptureFormat() {
	// Exactly the shape the firmware's RawSampleLogger emits: raw integer
	// counts, accelerometer and gyroscope on separate rows because they run at
	// different rates, and scale factors in the header.
	const std::string path = "build/_raw.csv";
	{
		std::ofstream o(path);
		o << "# slimevr-imu-log v1\n";
		o << "# sensor LSM6DSV\n";
		o << "# acc_ts 0.00833333\n";
		o << "# gyr_ts 0.00416667\n";
		o << "# acc_scale 0.00119681\n";
		o << "# gyr_scale 0.000610865\n";
		o << "# note raw uncalibrated counts\n";
		o << "t_us,ax,ay,az,gx,gy,gz\n";
		o << "4166,,,,10,-20,30\n";
		o << "8333,100,-200,8192,,,\n";
		o << "# t_real 8400 acc_n 1 gyr_n 2\n";
		o << "8333,,,,11,-21,31\n";
		o << "12500,,,,12,-22,32\n";
		o << "16666,101,-201,8193,,,\n";
		o << "16666,,,,13,-23,33\n";
	}

	Dataset ds;
	std::string err;
	TRUE_(loadDataset(path, ds, err));
	TRUE_(ds.samples.size() == 6);

	// Header-declared rates must win over any inference from row spacing.
	NEAR(ds.gyrTs, 0.00416667, 1e-9);
	NEAR(ds.accTs, 0.00833333, 1e-9);

	// Presence, not zero: an empty field means the sensor did not report here.
	TRUE_(!ds.samples[0].hasAcc);
	TRUE_(ds.samples[0].hasGyr);
	TRUE_(ds.samples[1].hasAcc);
	TRUE_(!ds.samples[1].hasGyr);

	// Raw counts must be converted using the header's scale factors.
	NEAR(ds.samples[0].gyr.x, 10 * 0.000610865, 1e-12);
	NEAR(ds.samples[0].gyr.z, 30 * 0.000610865, 1e-12);
	NEAR(ds.samples[1].acc.z, 8192 * 0.00119681, 1e-9);

	// Comment lines in the middle of the data are skipped, not parsed as rows.
	TRUE_(ds.samples[2].hasGyr);
	NEAR(ds.samples[2].gyr.x, 11 * 0.000610865, 1e-12);

	std::remove(path.c_str());
}

void testInterleavedMatchesSynchronous() {
	// End-to-end: take a clean stationary dataset and re-express it the way a
	// real capture arrives -- accelerometer and gyroscope on separate rows --
	// then confirm the fusion still converges. This is the property the whole
	// raw-logging path depends on, and it is not covered by any synchronous
	// dataset.
	SynthParams sp;
	sp.durationSec = 30;
	sp.rateHz = 250;
	sp.gyroBiasDps = 0.0;
	sp.gyroNoiseDps = 0.0;
	sp.accelBias = 0.0;
	sp.accelNoise = 0.0;

	Dataset sync;
	std::string err;
	TRUE_(generate("static", sp, sync, err));

	Dataset split = sync;
	split.samples.clear();
	split.samples.reserve(sync.samples.size() * 2);
	for (const Sample& s : sync.samples) {
		Sample a = s;
		a.hasAcc = true;
		a.hasGyr = false;
		a.hasMag = false;
		Sample g = s;
		g.hasAcc = false;
		g.hasGyr = true;
		g.hasMag = false;
		split.samples.push_back(a);
		split.samples.push_back(g);
	}

	Metrics ms = computeMetrics(sync, runFusion(sync, BenchParams{}));
	Metrics mi = computeMetrics(split, runFusion(split, BenchParams{}));

	// Both must find the tracker stationary and level.
	TRUE_(mi.hasDriftEstimate);
	NEAR(mi.headingDriftDegPerMin, 0.0, 0.01);
	NEAR(mi.tiltErrorDegRms, 0.0, 0.05);
	TRUE_(mi.firstRestSec >= 0.0);

	// And should agree closely with the synchronous run on the same input.
	NEAR(mi.headingDriftDegPerMin, ms.headingDriftDegPerMin, 0.05);
	NEAR(mi.tiltErrorDegRms, ms.tiltErrorDegRms, 0.05);
}

void testPresenceSurvivesRoundTrip() {
	// Trailing empty fields are the easy thing to lose: a naive CSV split drops
	// them, which would silently turn an accelerometer row into a malformed one.
	SynthParams sp;
	sp.durationSec = 1;
	sp.rateHz = 100;
	Dataset ds;
	std::string err;
	TRUE_(generate("tumble", sp, ds, err));

	for (size_t i = 0; i < ds.samples.size(); i++) {
		const bool accRow = (i % 2) == 0;
		ds.samples[i].hasAcc = accRow;
		ds.samples[i].hasGyr = !accRow;
	}

	const std::string path = "build/_presence.csv";
	TRUE_(saveDataset(path, ds, err));

	Dataset back;
	TRUE_(loadDataset(path, back, err));
	TRUE_(back.samples.size() == ds.samples.size());

	bool ok = true;
	for (size_t i = 0; i < ds.samples.size(); i++) {
		ok = ok && back.samples[i].hasAcc == ds.samples[i].hasAcc
		  && back.samples[i].hasGyr == ds.samples[i].hasGyr;
	}
	TRUE_(ok);
	std::remove(path.c_str());
}

using SlimeVR::Sensors::SoftFusion::MagFifoAssembler;
using SlimeVR::Sensors::SoftFusion::MagFifoConfig;

void testMagFifo24BitSplit() {
	// The BMM350 case: 9 data bytes behind 2 dummy bytes, which cannot fit one
	// slave (SLAVE0_NUMOP caps at 7), so it is split. Each slave transaction
	// carries its own dummy-byte prefix.
	MagFifoConfig cfg;
	cfg.enabled = true;
	cfg.dummyBytes = 2;
	cfg.dataBytes = 9;
	cfg.split = true;
	cfg.firstDataBytes = 5;

	NEAR(cfg.slave0Needed(), 7.0, 0.0);  // 2 dummy + 5 data
	NEAR(cfg.slave1Needed(), 6.0, 0.0);  // 2 dummy + 4 data

	// X = 0x123456, Y = 0x000001, Z = -2 (0xFFFFFE), little-endian per axis.
	const uint8_t data[9] = {
		0x56,
		0x34,
		0x12,  // X
		0x01,
		0x00,
		0x00,  // Y
		0xFE,
		0xFF,
		0xFF,  // Z
	};

	// Slave 0: 2 dummy + data[0..4], padded out to two 6-byte FIFO words.
	uint8_t s0w0[6] = {0xAA, 0xBB, data[0], data[1], data[2], data[3]};
	uint8_t s0w1[6] = {data[4], 0x00, 0x00, 0x00, 0x00, 0x00};
	// Slave 1: 2 dummy + data[5..8], one word.
	uint8_t s1w0[6] = {0xCC, 0xDD, data[5], data[6], data[7], data[8]};

	MagFifoAssembler a;
	int32_t xyz[3] = {0, 0, 0};

	TRUE_(!a.feedSlave0(s0w0, cfg, xyz));  // incomplete
	TRUE_(!a.feedSlave0(s0w1, cfg, xyz));  // slave 0 done, slave 1 outstanding
	TRUE_(a.feedSlave1(s1w0, cfg, xyz));  // now complete

	NEAR(xyz[0], 0x123456, 0.0);
	NEAR(xyz[1], 1.0, 0.0);
	NEAR(xyz[2], -2.0, 0.0);  // sign extension across 24 bits
}

void testMagFifo16BitSingleSlave() {
	// A 6-byte magnetometer with no dummy bytes fits one slave and one word.
	MagFifoConfig cfg;
	cfg.enabled = true;
	cfg.dummyBytes = 0;
	cfg.dataBytes = 6;
	cfg.split = false;

	NEAR(cfg.slave0Needed(), 6.0, 0.0);
	NEAR(cfg.slave1Needed(), 0.0, 0.0);

	// X = 1, Y = -1, Z = 258
	uint8_t word[6] = {0x01, 0x00, 0xFF, 0xFF, 0x02, 0x01};

	MagFifoAssembler a;
	int32_t xyz[3] = {0, 0, 0};
	TRUE_(a.feedSlave0(word, cfg, xyz));
	NEAR(xyz[0], 1.0, 0.0);
	NEAR(xyz[1], -1.0, 0.0);
	NEAR(xyz[2], 258.0, 0.0);
}

void testMagFifoDroppedSlave1Recovers() {
	// If slave 1's word is lost, the next slave-0 word must start a fresh
	// sample rather than being concatenated onto the stale one -- otherwise the
	// assembler would emit a blend of two samples, which looks like plausible
	// data and is not.
	MagFifoConfig cfg;
	cfg.enabled = true;
	cfg.dummyBytes = 2;
	cfg.dataBytes = 9;
	cfg.split = true;
	cfg.firstDataBytes = 5;

	uint8_t s0a[6] = {0xAA, 0xBB, 0x11, 0x11, 0x11, 0x22};
	uint8_t s0b[6] = {0x22, 0, 0, 0, 0, 0};
	uint8_t s0c[6] = {0xAA, 0xBB, 0x56, 0x34, 0x12, 0x01};
	uint8_t s0d[6] = {0x00, 0, 0, 0, 0, 0};
	uint8_t s1[6] = {0xCC, 0xDD, 0x00, 0xFE, 0xFF, 0xFF};

	MagFifoAssembler a;
	int32_t xyz[3] = {0, 0, 0};

	// First sample: slave 1 never arrives.
	TRUE_(!a.feedSlave0(s0a, cfg, xyz));
	TRUE_(!a.feedSlave0(s0b, cfg, xyz));

	// Second sample begins; the stale slave-0 bytes must be discarded.
	TRUE_(!a.feedSlave0(s0c, cfg, xyz));
	TRUE_(!a.feedSlave0(s0d, cfg, xyz));
	TRUE_(a.feedSlave1(s1, cfg, xyz));

	// Must be the *second* sample, uncontaminated by the first.
	NEAR(xyz[0], 0x123456, 0.0);
	NEAR(xyz[1], 1.0, 0.0);
	NEAR(xyz[2], -2.0, 0.0);
}

void testMagFifoRejectsBadConfig() {
	// A zeroed config must never emit. This is what protects the path before
	// startAuxPolling has run.
	MagFifoConfig cfg;
	uint8_t word[6] = {1, 2, 3, 4, 5, 6};
	MagFifoAssembler a;
	int32_t xyz[3] = {0, 0, 0};
	TRUE_(!a.feedSlave0(word, cfg, xyz));
	TRUE_(!a.feedSlave1(word, cfg, xyz));
}

void testSignExtend24Boundaries() {
	const uint8_t zero[3] = {0x00, 0x00, 0x00};
	const uint8_t maxPos[3] = {0xFF, 0xFF, 0x7F};  // 8388607
	const uint8_t minNeg[3] = {0x00, 0x00, 0x80};  // -8388608
	const uint8_t negOne[3] = {0xFF, 0xFF, 0xFF};  // -1
	NEAR(MagFifoAssembler::signExtend24(zero), 0.0, 0.0);
	NEAR(MagFifoAssembler::signExtend24(maxPos), 8388607.0, 0.0);
	NEAR(MagFifoAssembler::signExtend24(minNeg), -8388608.0, 0.0);
	NEAR(MagFifoAssembler::signExtend24(negOne), -1.0, 0.0);
}

using SlimeVR::Sensors::SoftFusion::Bmm350Calibration;
using SlimeVR::Sensors::SoftFusion::bmm350Compensate;
using SlimeVR::Sensors::SoftFusion::bmm350DecodeOtp;
using SlimeVR::Sensors::SoftFusion::bmm350FixSign;
using SlimeVR::Sensors::SoftFusion::bmm350RawToCelsius;
using SlimeVR::Sensors::SoftFusion::kBmm350LsbToUtXY;
using SlimeVR::Sensors::SoftFusion::kBmm350LsbToUtZ;

void testBmm350FixSign() {
	// The OTP packs signed values into 8- and 12-bit fields. Getting the
	// sign extension wrong flips roughly half the trim coefficients, which is
	// invisible without a reference -- so pin the boundaries.
	NEAR(bmm350FixSign(0x00, 8), 0.0, 0.0);
	NEAR(bmm350FixSign(0x7f, 8), 127.0, 0.0);
	NEAR(bmm350FixSign(0x80, 8), -128.0, 0.0);
	NEAR(bmm350FixSign(0xff, 8), -1.0, 0.0);

	NEAR(bmm350FixSign(0x000, 12), 0.0, 0.0);
	NEAR(bmm350FixSign(0x7ff, 12), 2047.0, 0.0);
	NEAR(bmm350FixSign(0x800, 12), -2048.0, 0.0);
	NEAR(bmm350FixSign(0xfff, 12), -1.0, 0.0);

	NEAR(bmm350FixSign(0xffff, 16), -1.0, 0.0);
	NEAR(bmm350FixSign(0x8000, 16), -32768.0, 0.0);
}

void testBmm350UncalibratedPassthrough() {
	// With no trim data the result must be nominal scaling, not zero. A failed
	// OTP read should degrade to "uncompensated", never to "no signal".
	Bmm350Calibration cal;  // valid == false
	const int32_t raw[3] = {1000, -2000, 3000};
	float out[3] = {0, 0, 0};
	bmm350Compensate(raw, 25.0f, cal, out);
	NEAR(out[0], 1000 * kBmm350LsbToUtXY, 1e-6);
	NEAR(out[1], -2000 * kBmm350LsbToUtXY, 1e-6);
	NEAR(out[2], 3000 * kBmm350LsbToUtZ, 1e-6);
}

void testBmm350IdentityCalibration() {
	// A part with all-zero trim must behave exactly like the uncompensated
	// path. If this diverges, the formula has a stray constant in it.
	Bmm350Calibration cal;
	cal.valid = true;
	cal.dutT0 = 23.0f;

	const int32_t raw[3] = {1000, -2000, 3000};
	float out[3] = {0, 0, 0};
	bmm350Compensate(raw, cal.dutT0, cal, out);
	NEAR(out[0], 1000 * kBmm350LsbToUtXY, 1e-6);
	NEAR(out[1], -2000 * kBmm350LsbToUtXY, 1e-6);
	NEAR(out[2], 3000 * kBmm350LsbToUtZ, 1e-6);
}

void testBmm350OffsetAndSensitivity() {
	Bmm350Calibration cal;
	cal.valid = true;
	cal.dutT0 = 23.0f;
	cal.sensX = 0.10f;  // 10% high
	cal.offsetY = 5.0f;  // +5 uT

	const int32_t raw[3] = {1000, 1000, 0};
	float out[3] = {0, 0, 0};
	bmm350Compensate(raw, cal.dutT0, cal, out);

	NEAR(out[0], 1000 * kBmm350LsbToUtXY * 1.10f, 1e-5);
	NEAR(out[1], 1000 * kBmm350LsbToUtXY + 5.0f, 1e-5);
	NEAR(out[2], 0.0, 1e-6);
}

void testBmm350TemperatureTermsVanishAtT0() {
	// TCO and TCS are defined relative to dut_t0, so at that temperature they
	// must contribute nothing. This is what makes it legitimate to pass dutT0
	// when the temperature channel is not being read.
	Bmm350Calibration cal;
	cal.valid = true;
	cal.dutT0 = 23.0f;
	cal.tcoX = 1.5f;
	cal.tcsX = 0.01f;

	const int32_t raw[3] = {1000, 0, 0};
	float atT0[3] = {0, 0, 0};
	float away[3] = {0, 0, 0};
	bmm350Compensate(raw, cal.dutT0, cal, atT0);
	bmm350Compensate(raw, cal.dutT0 + 20.0f, cal, away);

	NEAR(atT0[0], 1000 * kBmm350LsbToUtXY, 1e-5);
	// ...and away from it they must actually do something, or the terms are
	// silently inert.
	TRUE_(std::fabs(away[0] - atT0[0]) > 1.0);
}

void testBmm350CrossAxis() {
	// Cross-axis coupling is the term that matters most for heading: it rotates
	// the measured field vector, and a rotated vector is a wrong heading.
	Bmm350Calibration cal;
	cal.valid = true;
	cal.dutT0 = 23.0f;
	cal.crossXY = 0.05f;

	const int32_t raw[3] = {0, 1000, 0};
	float out[3] = {0, 0, 0};
	bmm350Compensate(raw, cal.dutT0, cal, out);

	// A pure Y field must produce a non-zero X correction.
	const float y = 1000 * kBmm350LsbToUtXY;
	NEAR(out[0], (0.0f - 0.05f * y) / (1.0f - 0.0f), 1e-5);
	TRUE_(std::fabs(out[0]) > 0.1);
}

void testBmm350OtpDecoding() {
	// Word layout, field positions and divisors, checked against
	// BMM350_SensorAPI's update_mag_off_sens.
	uint16_t otp[32] = {0};
	otp[0] = 0x0123;  // offset_x = 0x123 = 291
	otp[3] = 0x0a05;  // t_offs = 5/5 = 1 ; t_sens = 10/512
	otp[4] = 0x8000;  // sens_x = -128/256 = -0.5
	otp[5] = 0x0040;  // sens_y = 64/256 = 0.25
	otp[7] = 0x0020;  // tco_x = 32/32 = 1
	otp[10] = 0x0100;  // tcs_x = 1/16384
	otp[13] = 0x0200;  // dut_t0 = 512/512 + 23 = 24
	otp[14] = 0x0008;  // cross_x_y = 8/800 = 0.01

	Bmm350Calibration cal;
	bmm350DecodeOtp(otp, cal);

	TRUE_(cal.valid);
	NEAR(cal.offsetX, 291.0, 1e-6);
	NEAR(cal.tOffs, 1.0, 1e-6);
	NEAR(cal.tSens, 10.0 / 512.0, 1e-9);
	NEAR(cal.sensX, -0.5, 1e-9);
	NEAR(cal.sensY, 0.25, 1e-9);
	NEAR(cal.tcoX, 1.0, 1e-9);
	NEAR(cal.tcsX, 1.0 / 16384.0, 1e-12);
	NEAR(cal.dutT0, 24.0, 1e-6);
	NEAR(cal.crossXY, 0.01, 1e-9);

	// offset_y and offset_z each straddle two OTP words, which is the easiest
	// thing in this decoder to get wrong and the hardest to notice.
	uint16_t off[32] = {0};
	// offset_y = 0xABC: high nibble 0xA in word0 bits 12-15, low byte 0xBC in
	// word1 bits 0-7.
	off[0] = 0xa000;
	off[1] = 0x00bc;
	// offset_z = 0x123: nibble 0x1 in word1 bits 8-11, low byte 0x23 in word2.
	off[1] |= 0x0100;
	off[2] = 0x0023;
	Bmm350Calibration ocal;
	bmm350DecodeOtp(off, ocal);
	// 0xABC has bit 11 set, so it is negative: 0xABC - 0x1000 = -1348.
	NEAR(ocal.offsetY, -1348.0, 1e-6);
	NEAR(ocal.offsetZ, 291.0, 1e-6);  // 0x123

	// Negative coefficients must come out negative.
	uint16_t neg[32] = {0};
	neg[4] = 0xff00;  // sens_x = -1/256
	neg[13] = 0xfe00;  // dut_t0 = -512/512 + 23 = 22
	Bmm350Calibration ncal;
	bmm350DecodeOtp(neg, ncal);
	NEAR(ncal.sensX, -1.0 / 256.0, 1e-9);
	NEAR(ncal.dutT0, 22.0, 1e-6);
}

void testBmm350TemperatureConversion() {
	// The vendor conversion is piecewise around zero, which is unusual enough
	// to be worth pinning.
	NEAR(bmm350RawToCelsius(0), 0.0, 1e-9);
	const float pos = bmm350RawToCelsius(30000);
	NEAR(pos, 30000 * 0.000981282 - 25.49, 1e-4);
	const float neg = bmm350RawToCelsius(-30000);
	NEAR(neg, -30000 * 0.000981282 + 25.49, 1e-4);
}

using SlimeVR::Sensors::SoftFusion::ErrorModel;
using SlimeVR::Sensors::SoftFusion::fitErrorModel;

// Applies a known error model *forward*, i.e. simulates what a sensor with that
// error would report for a true field of the given direction.
static void corrupt(
	const double trueXyz[3],
	const double gain[3],
	const double crossXY,
	const double bias[3],
	float out[3]
) {
	// Inverse of the correction: raw = M^-1 * true + bias, with a simple
	// upper-triangular M so the inverse is easy to write by hand.
	const double t1 = trueXyz[1] / gain[1];
	const double t0 = (trueXyz[0] - crossXY * trueXyz[1]) / gain[0];
	const double t2 = trueXyz[2] / gain[2];
	out[0] = static_cast<float>(t0 + bias[0]);
	out[1] = static_cast<float>(t1 + bias[1]);
	out[2] = static_cast<float>(t2 + bias[2]);
}

void testErrorModelIdentityByDefault() {
	// A device with no fitted model must behave exactly as before, which is
	// what allows the storage to land ahead of the estimation.
	ErrorModel m;
	TRUE_(m.isIdentity());
	const float in[3] = {1.5f, -2.5f, 9.8f};
	float out[3];
	m.apply(in, out);
	NEAR(out[0], 1.5, 1e-6);
	NEAR(out[1], -2.5, 1e-6);
	NEAR(out[2], 9.8, 1e-6);
}

void testErrorModelApply() {
	ErrorModel m;
	m.bias[0] = 1.0f;
	m.m[0] = 2.0f;  // x gain
	m.m[1] = 0.5f;  // x picks up y: misalignment
	const float in[3] = {3.0f, 4.0f, 0.0f};
	float out[3];
	m.apply(in, out);
	// (3-1)*2 + 4*0.5 = 6
	NEAR(out[0], 6.0, 1e-6);
	NEAR(out[1], 4.0, 1e-6);
	TRUE_(!m.isIdentity());
}

// Builds a set of orientations covering the sphere reasonably evenly.
static void sphereDirections(std::vector<std::array<double, 3>>& dirs, int n) {
	dirs.clear();
	// Fibonacci sphere: deterministic and well spread, which is what the fit
	// needs. Clustered samples leave it under-determined.
	const double ga = kPi * (3.0 - std::sqrt(5.0));
	for (int i = 0; i < n; i++) {
		const double y = 1.0 - (2.0 * i) / (n - 1);
		const double r = std::sqrt(std::max(0.0, 1.0 - y * y));
		const double th = ga * i;
		dirs.push_back({std::cos(th) * r, y, std::sin(th) * r});
	}
}

void testFitRecoversBiasScaleAndMisalignment() {
	// The whole point of the issue: scale factor and misalignment are invisible
	// to a bias-only calibration, and both are systematic so they accumulate.
	constexpr double g = 9.80665;
	const double gain[3] = {1.03, 0.97, 1.01};  // +-3% scale error
	const double crossXY = 0.02;  // 2% cross-axis coupling
	const double bias[3] = {0.12, -0.08, 0.05};

	std::vector<std::array<double, 3>> dirs;
	sphereDirections(dirs, 200);

	std::vector<float> samples;
	for (const auto& d : dirs) {
		const double t[3] = {d[0] * g, d[1] * g, d[2] * g};
		float raw[3];
		corrupt(t, gain, crossXY, bias, raw);
		samples.push_back(raw[0]);
		samples.push_back(raw[1]);
		samples.push_back(raw[2]);
	}

	ErrorModel fitted;
	TRUE_(fitErrorModel(samples.data(), dirs.size(), static_cast<float>(g), fitted));

	// Bias is recovered directly.
	NEAR(fitted.bias[0], 0.12, 5e-3);
	NEAR(fitted.bias[1], -0.08, 5e-3);
	NEAR(fitted.bias[2], 0.05, 5e-3);

	// The real test is behavioural: corrected magnitudes must be g regardless
	// of orientation. The individual matrix entries are only determined up to a
	// rotation, so comparing them directly would be testing the wrong thing.
	double worst = 0;
	for (const auto& d : dirs) {
		const double t[3] = {d[0] * g, d[1] * g, d[2] * g};
		float raw[3];
		corrupt(t, gain, crossXY, bias, raw);
		float cor[3];
		fitted.apply(raw, cor);
		const double mag
			= std::sqrt(cor[0] * cor[0] + cor[1] * cor[1] + cor[2] * cor[2]);
		worst = std::max(worst, std::fabs(mag - g));
	}
	TRUE_(worst < 0.01);

	// And it must be a real improvement over doing nothing.
	double worstRaw = 0;
	for (const auto& d : dirs) {
		const double t[3] = {d[0] * g, d[1] * g, d[2] * g};
		float raw[3];
		corrupt(t, gain, crossXY, bias, raw);
		const double mag
			= std::sqrt(raw[0] * raw[0] + raw[1] * raw[1] + raw[2] * raw[2]);
		worstRaw = std::max(worstRaw, std::fabs(mag - g));
	}
	TRUE_(worstRaw > 0.2);
	TRUE_(worst < worstRaw / 10.0);
}

void testFitIsNearIdentityForAPerfectSensor() {
	// A sensor with no error must not have error invented for it.
	constexpr double g = 9.80665;
	const double gain[3] = {1.0, 1.0, 1.0};
	const double bias[3] = {0.0, 0.0, 0.0};

	std::vector<std::array<double, 3>> dirs;
	sphereDirections(dirs, 100);
	std::vector<float> samples;
	for (const auto& d : dirs) {
		const double t[3] = {d[0] * g, d[1] * g, d[2] * g};
		float raw[3];
		corrupt(t, gain, 0.0, bias, raw);
		samples.push_back(raw[0]);
		samples.push_back(raw[1]);
		samples.push_back(raw[2]);
	}

	ErrorModel fitted;
	TRUE_(fitErrorModel(samples.data(), dirs.size(), static_cast<float>(g), fitted));
	NEAR(fitted.m[0], 1.0, 1e-3);
	NEAR(fitted.m[4], 1.0, 1e-3);
	NEAR(fitted.m[8], 1.0, 1e-3);
	NEAR(fitted.bias[0], 0.0, 1e-3);
	NEAR(fitted.bias[1], 0.0, 1e-3);
	NEAR(fitted.bias[2], 0.0, 1e-3);
}

void testFitRejectsInsufficientData() {
	ErrorModel m;
	float few[12] = {0};
	TRUE_(!fitErrorModel(few, 4, 9.8f, m));
	TRUE_(!fitErrorModel(few, 4, 0.0f, m));
	// Model must be untouched on failure.
	TRUE_(m.isIdentity());
}

void testFitRejectsDegenerateOrientations() {
	// Enough samples for the quadric to be nominally determined, but all in one
	// plane. The solve would succeed and return a confident, wrong model, which
	// then multiplies every sample -- worse than having no calibration at all.
	constexpr double g = 9.80665;
	std::vector<float> planar;
	for (int i = 0; i < 60; i++) {
		const double th = 2.0 * kPi * i / 60.0;
		planar.push_back(static_cast<float>(g * std::cos(th)));
		planar.push_back(static_cast<float>(g * std::sin(th)));
		planar.push_back(0.0f);
	}
	ErrorModel m;
	TRUE_(!fitErrorModel(planar.data(), 60, static_cast<float>(g), m));
	TRUE_(m.isIdentity());  // untouched on failure

	// Tightly clustered about one attitude: also plenty of samples, also
	// unobservable.
	std::vector<float> clustered;
	for (int i = 0; i < 60; i++) {
		const double th = 0.001 * i;
		clustered.push_back(static_cast<float>(g * std::sin(th)));
		clustered.push_back(static_cast<float>(g * std::cos(th)));
		clustered.push_back(0.0f);
	}
	ErrorModel m2;
	TRUE_(!fitErrorModel(clustered.data(), 60, static_cast<float>(g), m2));
	TRUE_(m2.isIdentity());
}

void testFitRejectsNearDegenerateCoverage() {
	// The case the conditioning check actually exists for, and the one a
	// count-only guard misses entirely.
	//
	// A narrow band of latitudes has plenty of samples and is not exactly
	// coplanar, so the solve succeeds -- but the model it produces is badly
	// wrong on any direction outside the band it was trained on. That is the
	// dangerous failure: it looks like a successful calibration.
	//
	// Builds a physically correct set: the true field always has magnitude g,
	// corrupted by a real gain and bias, so the only thing varying is coverage.
	constexpr double g = 9.80665;
	const double gain[3] = {1.05, 0.98, 1.10};
	const double bias[3] = {0.15, -0.10, 0.25};

	auto build = [&](double tilt) {
		std::vector<float> out;
		for (int i = 0; i < 180; i++) {
			const double th = 2.0 * kPi * i / 180.0;
			const double ph = tilt * std::sin(3.0 * th);
			const double x = std::cos(th) * std::cos(ph);
			const double y = std::sin(th) * std::cos(ph);
			const double z = std::sin(ph);
			out.push_back(static_cast<float>(g * x / gain[0] + bias[0]));
			out.push_back(static_cast<float>(g * y / gain[1] + bias[1]));
			out.push_back(static_cast<float>(g * z / gain[2] + bias[2]));
		}
		return out;
	};

	// Error the fitted model makes on a direction it never saw: straight up.
	auto errorOnUnseenDirection = [&](const ErrorModel& m) {
		const float in[3] = {
			static_cast<float>(bias[0]),
			static_cast<float>(bias[1]),
			static_cast<float>(g / gain[2] + bias[2]),
		};
		float out[3];
		m.apply(in, out);
		const double mag
			= std::sqrt(out[0] * out[0] + out[1] * out[1] + out[2] * out[2]);
		return std::fabs(mag - g);
	};

	// A narrow band must be refused.
	const std::vector<float> narrow = build(0.15);
	ErrorModel bad;
	TRUE_(!fitErrorModel(narrow.data(), narrow.size() / 3, static_cast<float>(g), bad));
	TRUE_(bad.isIdentity());

	// And the refusal is justified: forcing the same data through a fit
	// produces a model that is wrong by a large margin off-band. Verified here
	// by checking that wider coverage of the *same* error model succeeds and is
	// accurate, so the difference is coverage rather than the data being
	// unfittable.
	const std::vector<float> wide = build(0.9);
	ErrorModel good;
	TRUE_(fitErrorModel(wide.data(), wide.size() / 3, static_cast<float>(g), good));
	TRUE_(errorOnUnseenDirection(good) < 0.01);
}

void testFitAcceptsSixPositionCoverage() {
	// The classic procedure -- each axis up and down -- must be accepted, and
	// with realistic hand-placement error rather than perfect alignment. A
	// check that rejected this would make the whole calibration unusable.
	constexpr double g = 9.80665;
	const double dirs[6][3]
		= {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
	Rng rng(7);
	std::vector<float> samples;
	for (int rep = 0; rep < 10; rep++) {
		for (const auto& d : dirs) {
			// Up to ~10 degrees of misplacement per axis.
			double v[3] = {
				d[0] + 0.17 * rng.normal() * 0.5,
				d[1] + 0.17 * rng.normal() * 0.5,
				d[2] + 0.17 * rng.normal() * 0.5,
			};
			const double n = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
			samples.push_back(static_cast<float>(g * v[0] / n));
			samples.push_back(static_cast<float>(g * v[1] / n));
			samples.push_back(static_cast<float>(g * v[2] / n));
		}
	}
	ErrorModel m;
	TRUE_(fitErrorModel(samples.data(), 60, static_cast<float>(g), m));
}

void testSmallestEigenvalueOfKnownMatrices() {
	using SlimeVR::Sensors::SoftFusion::detail::smallestEigenvalue3;
	// Diagonal: the smallest entry.
	const double diag[9] = {3, 0, 0, 0, 1, 0, 0, 0, 2};
	NEAR(smallestEigenvalue3(diag), 1.0, 1e-9);
	// Isotropic direction coverage gives 1/3 on every axis.
	const double iso[9] = {1.0 / 3, 0, 0, 0, 1.0 / 3, 0, 0, 0, 1.0 / 3};
	NEAR(smallestEigenvalue3(iso), 1.0 / 3.0, 1e-9);
	// Coplanar directions: no variance out of plane.
	const double planar[9] = {0.5, 0, 0, 0, 0.5, 0, 0, 0, 0};
	NEAR(smallestEigenvalue3(planar), 0.0, 1e-9);
	// Non-diagonal, known eigenvalues 1 and 3.
	const double rot[9] = {2, 1, 0, 1, 2, 0, 0, 0, 2};
	NEAR(smallestEigenvalue3(rot), 1.0, 1e-9);
}

// Builds a rest-move-rest capture with a known gyroscope scale error.
//
// `readsHigh` is the factor by which the gyroscope over-reports each axis, so
// the estimator should recover scale = 1/readsHigh.
static Dataset makeScaleCapture(
	const double readsHigh[3],
	const std::vector<std::array<double, 4>>& moves,  // axis xyz + angle (rad)
	double rateHz = 200.0
) {
	Dataset ds;
	ds.gyrTs = 1.0 / rateHz;
	ds.accTs = 1.0 / rateHz;
	ds.hasGroundTruth = false;
	ds.name = "synthetic-scale";

	Quat q{1, 0, 0, 0};
	uint64_t t = 0;
	const double dt = 1.0 / rateHz;

	auto emit = [&](const Vec3& omegaTrue) {
		Sample s;
		s.tUs = t;
		s.hasAcc = true;
		s.hasGyr = true;
		// Accelerometer sees gravity, which is +z in the world frame.
		const Vec3 g = qRotateInv(q, Vec3{0, 0, kGravity});
		s.acc = g;
		s.gyr = Vec3{
			omegaTrue.x * readsHigh[0],
			omegaTrue.y * readsHigh[1],
			omegaTrue.z * readsHigh[2],
		};
		ds.samples.push_back(s);
		q = qIntegrate(q, omegaTrue, dt);
		t += static_cast<uint64_t>(std::llround(dt * 1e6));
	};

	auto rest = [&](double seconds) {
		const int n = static_cast<int>(seconds * rateHz);
		for (int i = 0; i < n; i++) {
			emit(Vec3{0, 0, 0});
		}
	};

	rest(1.0);
	for (const auto& m : moves) {
		const double seconds = 1.0;
		const int n = static_cast<int>(seconds * rateHz);
		const double rate = m[3] / seconds;
		for (int i = 0; i < n; i++) {
			emit(Vec3{m[0] * rate, m[1] * rate, m[2] * rate});
		}
		rest(1.0);
	}
	return ds;
}

void testGyroScaleRecoversKnownError() {
	// The term bias calibration cannot see: it only shows up while rotating.
	const double readsHigh[3] = {1.03, 0.98, 1.015};
	// Rotations about each axis, tilting the tracker so gravity actually moves.
	const std::vector<std::array<double, 4>> moves = {
		{1, 0, 0, deg2rad(90)},
		{0, 1, 0, deg2rad(90)},
		{1, 0, 0, deg2rad(-90)},
		{0, 1, 0, deg2rad(-90)},
		{1, 0, 0, deg2rad(120)},
		{0, 1, 0, deg2rad(120)},
		{1, 0, 0, deg2rad(-120)},
		{0, 1, 0, deg2rad(-120)},
		{0, 0, 1, deg2rad(150)},
		{1, 0, 0, deg2rad(80)},
		{0, 0, 1, deg2rad(-150)},
		{0, 1, 0, deg2rad(80)},
	};
	const Dataset ds = makeScaleCapture(readsHigh, moves);

	const GyroScaleResult r = estimateGyroScale(ds);
	TRUE_(r.valid);
	TRUE_(r.segments >= 8);
	// Recovering the correction to within 0.3% is far tighter than the ~1%
	// errors this exists to catch.
	NEAR(r.scale[0], 1.0 / readsHigh[0], 0.003);
	NEAR(r.scale[1], 1.0 / readsHigh[1], 0.003);
	NEAR(r.scale[2], 1.0 / readsHigh[2], 0.003);
	// And the fit must actually improve the gravity prediction.
	TRUE_(r.residualAfterDeg < r.residualBeforeDeg / 3.0);
}

void testGyroScaleIsUnityForAPerfectSensor() {
	// A clean sensor must not have error invented for it.
	const double perfect[3] = {1.0, 1.0, 1.0};
	const std::vector<std::array<double, 4>> moves = {
		{1, 0, 0, deg2rad(90)},
		{0, 1, 0, deg2rad(90)},
		{1, 0, 0, deg2rad(-90)},
		{0, 1, 0, deg2rad(-90)},
		{0, 0, 1, deg2rad(140)},
		{1, 0, 0, deg2rad(100)},
		{0, 0, 1, deg2rad(-140)},
		{0, 1, 0, deg2rad(100)},
	};
	const Dataset ds = makeScaleCapture(perfect, moves);
	const GyroScaleResult r = estimateGyroScale(ds);
	TRUE_(r.valid);
	NEAR(r.scale[0], 1.0, 0.004);
	NEAR(r.scale[1], 1.0, 0.004);
	NEAR(r.scale[2], 1.0, 0.004);
}

void testGyroScaleRefusesWhenGravityNeverMoves() {
	// Rotation about the gravity vector does not move gravity, so it carries no
	// scale information at all. A capture of a tracker spun while sitting flat
	// is worthless for this however long it runs -- and the estimator has to
	// say so rather than return a confident number built from noise.
	const double readsHigh[3] = {1.03, 1.03, 1.03};
	const std::vector<std::array<double, 4>> spinOnly = {
		{0, 0, 1, deg2rad(180)},
		{0, 0, 1, deg2rad(-180)},
		{0, 0, 1, deg2rad(180)},
		{0, 0, 1, deg2rad(-180)},
		{0, 0, 1, deg2rad(180)},
		{0, 0, 1, deg2rad(-180)},
	};
	const Dataset ds = makeScaleCapture(readsHigh, spinOnly);
	const GyroScaleResult r = estimateGyroScale(ds);
	TRUE_(!r.valid);
	TRUE_(!r.reason.empty());
}

void testGyroScaleRefusesWithoutRestPeriods() {
	// Gravity is only a usable reference when the tracker is still; without
	// pauses there is nothing to compare the integration against.
	Dataset ds;
	ds.gyrTs = 1.0 / 200.0;
	ds.accTs = ds.gyrTs;
	Quat q{1, 0, 0, 0};
	for (int i = 0; i < 4000; i++) {
		Sample s;
		s.tUs = static_cast<uint64_t>(i * 5000);
		s.hasAcc = true;
		s.hasGyr = true;
		s.acc = qRotateInv(q, Vec3{0, 0, kGravity});
		s.gyr = Vec3{deg2rad(60), 0, 0};
		ds.samples.push_back(s);
		q = qIntegrate(q, s.gyr, ds.gyrTs);
	}
	const GyroScaleResult r = estimateGyroScale(ds);
	TRUE_(!r.valid);
	TRUE_(!r.reason.empty());
}

}  // namespace

// ---------------------------------------------------------------------------
// SET GYROSCALE serial command validation
// ---------------------------------------------------------------------------

void testGyroScaleArgParsing() {
	using namespace SlimeVR::Configuration;
	float v = 0;

	TRUE_(parseGyroScaleValue("1.0", v));
	NEAR(v, 1.0, 1e-9);
	TRUE_(parseGyroScaleValue("0.9873", v));
	NEAR(v, 0.9873, 1e-6);

	// `atof` would return 0.0 for each of these and report no error, which is
	// the exact failure this parse exists to close.
	TRUE_(!parseGyroScaleValue("abc", v));
	TRUE_(!parseGyroScaleValue("", v));
	TRUE_(!parseGyroScaleValue(nullptr, v));
	// Trailing garbage: a value that "looks right" but was mistyped.
	TRUE_(!parseGyroScaleValue("1.0x", v));
	TRUE_(!parseGyroScaleValue("nan", v));
	TRUE_(!parseGyroScaleValue("inf", v));
}

void testGyroScaleRangeRejection() {
	using namespace SlimeVR::Configuration;
	float scale[3] = {0, 0, 0};
	int bad = -1;

	const char* good[3] = {"1.002", "0.9950", "1.0100"};
	TRUE_(parseGyroScale(good, scale, bad) == GyroScaleStatus::Ok);
	TRUE_(bad == -1);
	NEAR(scale[0], 1.002, 1e-6);
	NEAR(scale[2], 1.01, 1e-6);

	// The misplaced decimal point. Accepting this would scale the gyroscope by
	// ten and present as a hardware fault rather than a bad calibration.
	const char* typo[3] = {"1.0", "10", "1.0"};
	TRUE_(parseGyroScale(typo, scale, bad) == GyroScaleStatus::OutOfRange);
	TRUE_(bad == 1);

	// Zero would silence the axis entirely.
	const char* zero[3] = {"0", "1.0", "1.0"};
	TRUE_(parseGyroScale(zero, scale, bad) == GyroScaleStatus::OutOfRange);
	TRUE_(bad == 0);

	// Negative would invert it.
	const char* negative[3] = {"1.0", "1.0", "-1.0"};
	TRUE_(parseGyroScale(negative, scale, bad) == GyroScaleStatus::OutOfRange);
	TRUE_(bad == 2);

	// The bounds themselves are inclusive.
	const char* edges[3] = {"0.90", "1.10", "1.0"};
	TRUE_(parseGyroScale(edges, scale, bad) == GyroScaleStatus::Ok);

	const char* justOver[3] = {"1.0", "1.0", "1.1001"};
	TRUE_(parseGyroScale(justOver, scale, bad) == GyroScaleStatus::OutOfRange);
	TRUE_(bad == 2);

	// Unparseable is reported as such rather than being folded into range.
	const char* junk[3] = {"1.0", "banana", "1.0"};
	TRUE_(parseGyroScale(junk, scale, bad) == GyroScaleStatus::Unparseable);
	TRUE_(bad == 1);
}

void testGyroScaleModelNormalisesAccelMatrix() {
	using namespace SlimeVR::Configuration;
	// The hazard this guards. A config that never had a model fitted carries an
	// all-zero A_M. That is harmless while errorModelValid is false, because the
	// loader substitutes identity -- but storing a gyroscope scale sets that
	// flag, and the flag governs both matrices. Without normalisation the zero
	// matrix goes live and multiplies every accelerometer sample to zero.
	float aM[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
	float gM[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
	const float scale[3] = {1.01f, 0.99f, 1.0f};

	const bool discarded = buildGyroScaleModel(scale, false, aM, gM);
	TRUE_(!discarded);

	NEAR(aM[0], 1.0, 1e-9);
	NEAR(aM[4], 1.0, 1e-9);
	NEAR(aM[8], 1.0, 1e-9);
	NEAR(aM[1], 0.0, 1e-9);
	NEAR(aM[5], 0.0, 1e-9);

	NEAR(gM[0], 1.01, 1e-6);
	NEAR(gM[4], 0.99, 1e-6);
	NEAR(gM[8], 1.0, 1e-6);
	NEAR(gM[1], 0.0, 1e-9);
	NEAR(gM[3], 0.0, 1e-9);
}

void testGyroScaleModelPreservesFittedAccelMatrix() {
	using namespace SlimeVR::Configuration;
	// A previously fitted accelerometer model must survive a gyroscope-only
	// update; the two are independent measurements.
	float aM[9] = {1.02f, 0.01f, 0, -0.01f, 0.98f, 0, 0, 0, 1.01f};
	float gM[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
	const float scale[3] = {1.005f, 1.0f, 0.995f};

	buildGyroScaleModel(scale, true, aM, gM);

	NEAR(aM[0], 1.02, 1e-6);
	NEAR(aM[1], 0.01, 1e-6);
	NEAR(aM[4], 0.98, 1e-6);
	NEAR(aM[8], 1.01, 1e-6);
	NEAR(gM[0], 1.005, 1e-6);
	NEAR(gM[8], 0.995, 1e-6);
}

void testGyroScaleReportsDiscardedMisalignment() {
	using namespace SlimeVR::Configuration;
	// Overwriting a fitted gyroscope model with a pure diagonal loses its
	// cross-axis terms. That is the intended semantic -- the estimator has
	// nothing to say about misalignment -- but it must not happen silently.
	float aM[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
	float withCross[9] = {1.01f, 0.02f, 0, -0.02f, 1.0f, 0, 0, 0, 1.0f};
	TRUE_(gyroModelHasMisalignment(withCross));

	const float scale[3] = {1.0f, 1.0f, 1.0f};
	TRUE_(buildGyroScaleModel(scale, true, aM, withCross));
	TRUE_(!gyroModelHasMisalignment(withCross));

	// A pure diagonal has nothing to lose, so no warning is due.
	float diagonal[9] = {1.01f, 0, 0, 0, 1.0f, 0, 0, 0, 1.0f};
	TRUE_(!gyroModelHasMisalignment(diagonal));
	TRUE_(!buildGyroScaleModel(scale, true, aM, diagonal));

	// Neither does an unset one, even though it is not literally diagonal.
	float unset[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
	TRUE_(!buildGyroScaleModel(scale, false, aM, unset));
}

void testGyroScaleModelIsActuallyApplied() {
	using namespace SlimeVR::Configuration;
	// applyErrorMatrix short-circuits two cases: an exact identity and an
	// all-zero diagonal. A stored scale must fall into neither, or it would be
	// written to flash, read back, and then silently ignored.
	float aM[9] = {0};
	float gM[9] = {0};
	const float scale[3] = {1.013f, 0.994f, 1.001f};
	buildGyroScaleModel(scale, false, aM, gM);

	const bool unsetLike = gM[0] == 0.0f && gM[4] == 0.0f && gM[8] == 0.0f;
	const bool identityLike = gM[0] == 1.0f && gM[4] == 1.0f && gM[8] == 1.0f;
	TRUE_(!unsetLike);
	TRUE_(!identityLike);

	// And the arithmetic it will receive is a plain per-axis scaling.
	const float in[3] = {2.0f, -3.0f, 0.5f};
	float out[3];
	out[0] = gM[0] * in[0] + gM[1] * in[1] + gM[2] * in[2];
	out[1] = gM[3] * in[0] + gM[4] * in[1] + gM[5] * in[2];
	out[2] = gM[6] * in[0] + gM[7] * in[1] + gM[8] * in[2];
	NEAR(out[0], 2.0 * 1.013, 1e-5);
	NEAR(out[1], -3.0 * 0.994, 1e-5);
	NEAR(out[2], 0.5 * 1.001, 1e-5);

	// Unity is the one case that *should* short-circuit: RESET must genuinely
	// restore no-op behaviour, not merely a matrix that multiplies out to one.
	const float unity[3] = {1.0f, 1.0f, 1.0f};
	buildGyroScaleModel(unity, false, aM, gM);
	const bool nowIdentity = gM[0] == 1.0f && gM[4] == 1.0f && gM[8] == 1.0f
						  && gM[1] == 0.0f && gM[2] == 0.0f && gM[3] == 0.0f
						  && gM[5] == 0.0f && gM[6] == 0.0f && gM[7] == 0.0f;
	TRUE_(nowIdentity);
}

// ---------------------------------------------------------------------------
// Guided six-position accelerometer calibration.
// ---------------------------------------------------------------------------

using SlimeVR::Sensors::SoftFusion::kBlocksPerPosition;
using SlimeVR::Sensors::SoftFusion::kDwellSamples;
using SlimeVR::Sensors::SoftFusion::kSamplesPerBlock;
using SlimeVR::Sensors::SoftFusion::kSixPositionCount;
using SlimeVR::Sensors::SoftFusion::SixPositionCollector;
using SlimeVR::Sensors::SoftFusion::SixPositionEvent;

constexpr double kG = 9.80665;

/// Samples needed to take one position from "just arrived" to captured.
constexpr size_t kSamplesPerPosition
	= kDwellSamples + kBlocksPerPosition * kSamplesPerBlock;

struct HoldTally {
	int started = 0;
	int captured = 0;
	int disturbed = 0;
	bool complete = false;
};

/// The raw reading a corrupted sensor gives when held in `position`.
static void rawForPosition(
	int position,
	const double gain[3],
	double crossXY,
	const double bias[3],
	float out[3]
) {
	double truth[3] = {0, 0, 0};
	truth[position / 2] = (position % 2 == 0) ? kG : -kG;
	corrupt(truth, gain, crossXY, bias, out);
}

static void feedN(
	SixPositionCollector& c,
	const float accel[3],
	bool atRest,
	size_t n,
	HoldTally& tally
) {
	for (size_t i = 0; i < n; i++) {
		switch (c.feed(accel, atRest, static_cast<float>(kG))) {
			case SixPositionEvent::Started:
				tally.started++;
				break;
			case SixPositionEvent::Captured:
				tally.captured++;
				break;
			case SixPositionEvent::Complete:
				tally.captured++;
				tally.complete = true;
				break;
			case SixPositionEvent::Disturbed:
				tally.disturbed++;
				break;
			case SixPositionEvent::None:
				break;
		}
	}
}

void testSixPositionClassifiesAxes() {
	const float g = static_cast<float>(kG);

	// Each axis up and down lands on its own bin, in the documented order.
	const float axes[6][3] = {
		{g, 0, 0},
		{-g, 0, 0},
		{0, g, 0},
		{0, -g, 0},
		{0, 0, g},
		{0, 0, -g},
	};
	for (int i = 0; i < 6; i++) {
		TRUE_(SixPositionCollector::classify(axes[i], g) == i);
	}

	// Held 15 degrees off: still the same position, because a user cannot do
	// better than this by hand and the fit does not need them to.
	const float tilted[3] = {
		static_cast<float>(kG * std::cos(deg2rad(15.0))),
		static_cast<float>(kG * std::sin(deg2rad(15.0))),
		0,
	};
	TRUE_(SixPositionCollector::classify(tilted, g) == 0);

	// Held 25 degrees off: past tolerance, so the user is told rather than
	// silently given a worse calibration.
	const float sloppy[3] = {
		static_cast<float>(kG * std::cos(deg2rad(25.0))),
		static_cast<float>(kG * std::sin(deg2rad(25.0))),
		0,
	};
	TRUE_(SixPositionCollector::classify(sloppy, g) < 0);

	// Halfway between two axes is not a position at all.
	const float diagonal[3] = {g * 0.7071f, 0, g * 0.7071f};
	TRUE_(SixPositionCollector::classify(diagonal, g) < 0);

	// Magnitude far from gravity: a steady reading that is not gravity.
	const float weak[3] = {g * 0.5f, 0, 0};
	const float strong[3] = {g * 1.5f, 0, 0};
	const float zero[3] = {0, 0, 0};
	TRUE_(SixPositionCollector::classify(weak, g) < 0);
	TRUE_(SixPositionCollector::classify(strong, g) < 0);
	TRUE_(SixPositionCollector::classify(zero, g) < 0);
}

void testSixPositionCapturesAndFits() {
	// The headline: turn a tracker through six positions and the error it was
	// built with comes back out.
	const double gain[3] = {1.03, 0.97, 1.01};
	const double bias[3] = {0.12, -0.08, 0.05};
	constexpr double crossXY = 0.0;

	SixPositionCollector c;
	c.begin();
	TRUE_(c.isRunning());
	TRUE_(c.nextPosition() == 0);

	HoldTally tally;
	for (int p = 0; p < 6; p++) {
		float raw[3];
		rawForPosition(p, gain, crossXY, bias, raw);
		feedN(c, raw, true, kSamplesPerPosition, tally);
	}

	TRUE_(tally.complete);
	TRUE_(tally.started == 6);
	TRUE_(tally.captured == 6);
	TRUE_(tally.disturbed == 0);
	TRUE_(c.capturedCount() == kSixPositionCount);
	TRUE_(c.capturedMask() == 0x3F);
	TRUE_(c.nextPosition() == -1);
	TRUE_(!c.isRunning());

	ErrorModel fitted;
	TRUE_(c.fit(static_cast<float>(kG), fitted));

	NEAR(fitted.bias[0], bias[0], 1e-3);
	NEAR(fitted.bias[1], bias[1], 1e-3);
	NEAR(fitted.bias[2], bias[2], 1e-3);
	NEAR(fitted.m[0], gain[0], 1e-3);
	NEAR(fitted.m[4], gain[1], 1e-3);
	NEAR(fitted.m[8], gain[2], 1e-3);

	// Off-diagonal terms are exactly zero: this fit does not guess at
	// misalignment, and inventing cross-axis coupling would be worse than
	// leaving it alone.
	const int offDiagonal[6] = {1, 2, 3, 5, 6, 7};
	for (const int i : offDiagonal) {
		NEAR(fitted.m[i], 0.0, 0.0);
	}

	// Behavioural check on directions the procedure never visited: a corrected
	// reading must have magnitude g wherever the sensor points.
	std::vector<std::array<double, 3>> dirs;
	sphereDirections(dirs, 100);
	double worst = 0;
	double worstRaw = 0;
	for (const auto& d : dirs) {
		const double t[3] = {d[0] * kG, d[1] * kG, d[2] * kG};
		float raw[3];
		corrupt(t, gain, crossXY, bias, raw);
		float cor[3];
		fitted.apply(raw, cor);
		worst = std::max(
			worst,
			std::fabs(
				std::sqrt(cor[0] * cor[0] + cor[1] * cor[1] + cor[2] * cor[2]) - kG
			)
		);
		worstRaw = std::max(
			worstRaw,
			std::fabs(
				std::sqrt(raw[0] * raw[0] + raw[1] * raw[1] + raw[2] * raw[2]) - kG
			)
		);
	}
	TRUE_(worstRaw > 0.2);
	TRUE_(worst < 0.01);
}

void testSixPositionFullFitIsSingularOnPerfectPositions() {
	// The reason the guided flow fits a diagonal. Six exactly axis-aligned
	// positions zero every cross-product column of the full quadric, so the
	// misalignment terms are not merely noisy, they are undetermined -- and the
	// full solve says so by failing outright.
	//
	// Without this, "six positions determines bias, scale and misalignment"
	// reads as true, and the on-device fit would be reporting how badly the
	// tracker was held as though it were a property of the part.
	const double gain[3] = {1.03, 0.97, 1.01};
	const double bias[3] = {0.12, -0.08, 0.05};

	std::vector<float> samples;
	for (int rep = 0; rep < 4; rep++) {
		for (int p = 0; p < 6; p++) {
			float raw[3];
			rawForPosition(p, gain, 0.0, bias, raw);
			samples.push_back(raw[0]);
			samples.push_back(raw[1]);
			samples.push_back(raw[2]);
		}
	}

	ErrorModel full;
	TRUE_(!fitErrorModel(samples.data(), 24, static_cast<float>(kG), full));

	// The diagonal fit takes the same data and succeeds.
	ErrorModel diagonal;
	TRUE_(fitErrorModelDiagonal(samples.data(), 24, static_cast<float>(kG), diagonal));
	NEAR(diagonal.m[0], gain[0], 1e-4);
	NEAR(diagonal.bias[2], bias[2], 1e-4);
}

void testDiagonalFitHelpsEvenWhenTheSensorIsMisaligned() {
	// The obvious worry about fitting a diagonal: real sensors do have
	// cross-axis coupling, so what does a correction that cannot represent it
	// do to one? It could plausibly bend the scale factors to soak up the
	// misalignment and come out worse than doing nothing.
	//
	// It does not. With 2% cross-coupling on top of 3% scale error, the
	// diagonal fit removes the terms it can model and leaves the one it cannot,
	// which is a large net improvement. That is what makes declining to guess
	// at misalignment a safe default rather than a compromise.
	constexpr double crossXY = 0.02;
	const double gain[3] = {1.03, 0.97, 1.01};
	const double bias[3] = {0.12, -0.08, 0.05};

	std::vector<float> samples;
	for (int rep = 0; rep < 4; rep++) {
		for (int p = 0; p < 6; p++) {
			float raw[3];
			rawForPosition(p, gain, crossXY, bias, raw);
			samples.push_back(raw[0]);
			samples.push_back(raw[1]);
			samples.push_back(raw[2]);
		}
	}

	ErrorModel fitted;
	TRUE_(fitErrorModelDiagonal(samples.data(), 24, static_cast<float>(kG), fitted));
	TRUE_(
		SlimeVR::Configuration::checkAccelModel(fitted, static_cast<float>(kG))
		== SlimeVR::Configuration::AccelModelStatus::Ok
	);

	std::vector<std::array<double, 3>> dirs;
	sphereDirections(dirs, 100);
	double worst = 0;
	double worstRaw = 0;
	for (const auto& d : dirs) {
		const double t[3] = {d[0] * kG, d[1] * kG, d[2] * kG};
		float raw[3];
		corrupt(t, gain, crossXY, bias, raw);
		float cor[3];
		fitted.apply(raw, cor);
		worst = std::max(
			worst,
			std::fabs(
				std::sqrt(cor[0] * cor[0] + cor[1] * cor[1] + cor[2] * cor[2]) - kG
			)
		);
		worstRaw = std::max(
			worstRaw,
			std::fabs(
				std::sqrt(raw[0] * raw[0] + raw[1] * raw[1] + raw[2] * raw[2]) - kG
			)
		);
	}
	// Strictly better, and by a wide margin rather than marginally.
	TRUE_(worst < worstRaw / 4.0);
}

void testDiagonalFitIsUnityForAPerfectSensor() {
	// A sensor with no error must come back as identity, or the calibration
	// would be manufacturing a correction out of nothing.
	std::vector<float> samples;
	for (int rep = 0; rep < 4; rep++) {
		for (int p = 0; p < 6; p++) {
			float raw[3] = {0, 0, 0};
			raw[p / 2] = static_cast<float>((p % 2 == 0) ? kG : -kG);
			samples.push_back(raw[0]);
			samples.push_back(raw[1]);
			samples.push_back(raw[2]);
		}
	}
	ErrorModel m;
	TRUE_(fitErrorModelDiagonal(samples.data(), 24, static_cast<float>(kG), m));
	TRUE_(m.isIdentity());
}

void testDiagonalFitRejectsPoorCoverage() {
	// Five positions plus a repeat is not six orientations, and this is the
	// case direction spread cannot see. Dropping -Z while keeping +Z twice
	// leaves the direction covariance at a perfect 1/3 on every axis, so the
	// spread check is entirely happy -- yet the fit's Z unknowns appear only as
	// `a3 z^2 - 2 c3 z`, the in-plane rows contribute nothing to either, and
	// the surviving +Z rows are identical. One equation, two unknowns.
	//
	// Left to the linear solve this came out differently on different
	// compilers: clang admitted it and returned a confidently wrong Z scale,
	// gcc refused it. Hence a direct check rather than a pivot threshold.
	const int positions[6] = {0, 1, 2, 3, 4, 4};
	std::vector<float> samples;
	for (int rep = 0; rep < 4; rep++) {
		for (const int p : positions) {
			float raw[3] = {0, 0, 0};
			raw[p / 2] = static_cast<float>((p % 2 == 0) ? kG : -kG);
			samples.push_back(raw[0]);
			samples.push_back(raw[1]);
			samples.push_back(raw[2]);
		}
	}
	ErrorModel m;
	TRUE_(!fitErrorModelDiagonal(samples.data(), 24, static_cast<float>(kG), m));

	// The spread check really is blind to it, which is why the second guard
	// has to exist. Asserting this keeps the two from being confused for one.
	TRUE_(SlimeVR::Sensors::SoftFusion::detail::spansThreeDimensions(
		samples.data(),
		24,
		6,
		SlimeVR::Sensors::SoftFusion::kMinDirectionSpread
	));

	// A set confined to one plane is not.
	std::vector<float> planar;
	for (int rep = 0; rep < 8; rep++) {
		for (int p = 0; p < 4; p++) {
			float raw[3] = {0, 0, 0};
			raw[p / 2] = static_cast<float>((p % 2 == 0) ? kG : -kG);
			planar.push_back(raw[0]);
			planar.push_back(raw[1]);
			planar.push_back(raw[2]);
		}
	}
	TRUE_(!fitErrorModelDiagonal(planar.data(), 32, static_cast<float>(kG), m));

	// And too few samples is refused before anything else happens.
	TRUE_(!fitErrorModelDiagonal(planar.data(), 5, static_cast<float>(kG), m));
}

/// Four in-plane positions plus an antipodal pair lifted `elevationDeg` out of
/// the XY plane -- a set whose out-of-plane coverage can be dialled from
/// "barely there" to "the full six positions" at 90 degrees.
///
/// `noise` is added per stored sample, standing in for what survives averaging
/// a block of raw readings.
static void thinCoverageSamples(
	double elevationDeg,
	const double gain[3],
	const double bias[3],
	double noise,
	uint64_t seed,
	std::vector<float>& out
) {
	const double c = std::cos(deg2rad(elevationDeg));
	const double s = std::sin(deg2rad(elevationDeg));
	const double dirs[6][3] = {
		{1, 0, 0},
		{-1, 0, 0},
		{0, 1, 0},
		{0, -1, 0},
		{c, 0, s},
		{-c, 0, -s},
	};
	Rng rng(seed);
	out.clear();
	for (int rep = 0; rep < 4; rep++) {
		for (const auto& d : dirs) {
			const double t[3] = {d[0] * kG, d[1] * kG, d[2] * kG};
			float raw[3];
			corrupt(t, gain, 0.0, bias, raw);
			for (int k = 0; k < 3; k++) {
				out.push_back(raw[k] + static_cast<float>(noise * rng.normal()));
			}
		}
	}
}

void testDiagonalFitRejectsMarginalCoverage() {
	// The case that distinguishes "the spread check works" from "the solve
	// happens to fail". The obvious degenerate sets -- coplanar, or clustered
	// -- are refused by the solve going singular whether or not the check
	// exists, so they prove nothing about it.
	//
	// What the check is actually protecting against turned out not to be
	// singularity at all. Given noiseless samples, four in-plane positions plus
	// a pair lifted only 20 degrees out of plane fits *exactly* -- every
	// unknown is determined and the answer is right to the last digit. The
	// damage is noise amplification: with 0.02 m/s^2 of noise surviving the
	// block average, that same geometry inflates the worst Z-scale error from
	// 0.23% at full coverage to 3.3%, which is larger than the scale error the
	// calibration exists to remove. Thin coverage does not fail loudly, it
	// quietly makes the tracker worse.
	const double gain[3] = {1.03, 0.97, 1.01};
	const double bias[3] = {0.12, -0.08, 0.05};

	std::vector<float> thin;
	thinCoverageSamples(20.0, gain, bias, 0.0, 1, thin);
	ErrorModel m;
	TRUE_(!fitErrorModelDiagonal(thin.data(), 24, static_cast<float>(kG), m));

	// Lift the same pair to 35 degrees and the set is admitted -- and with
	// clean data it is accurate. Without this half the test would only show the
	// check refusing everything.
	std::vector<float> adequate;
	thinCoverageSamples(35.0, gain, bias, 0.0, 1, adequate);
	ErrorModel fitted;
	TRUE_(fitErrorModelDiagonal(adequate.data(), 24, static_cast<float>(kG), fitted));
	NEAR(fitted.m[0], gain[0], 1e-3);
	NEAR(fitted.m[4], gain[1], 1e-3);
	NEAR(fitted.m[8], gain[2], 1e-3);
	NEAR(fitted.bias[2], bias[2], 1e-3);

	// And the amplification itself, measured between two sets the check both
	// accepts: the axis with the least coverage is the axis noise hurts most.
	// This is the trend the threshold is placed against, so it is worth an
	// assertion rather than only a comment.
	constexpr double noise = 0.02;
	double worstThin = 0;
	double worstFull = 0;
	for (uint64_t seed = 1; seed <= 40; seed++) {
		std::vector<float> a;
		std::vector<float> b;
		thinCoverageSamples(35.0, gain, bias, noise, seed, a);
		thinCoverageSamples(90.0, gain, bias, noise, seed, b);
		ErrorModel ma;
		ErrorModel mb;
		TRUE_(fitErrorModelDiagonal(a.data(), 24, static_cast<float>(kG), ma));
		TRUE_(fitErrorModelDiagonal(b.data(), 24, static_cast<float>(kG), mb));
		worstThin = std::max(worstThin, std::fabs(ma.m[8] - gain[2]));
		worstFull = std::max(worstFull, std::fabs(mb.m[8] - gain[2]));
	}
	TRUE_(worstThin > 2.0 * worstFull);
	// Full six-position coverage keeps the same noise well under the error
	// being corrected, which is the whole reason the procedure is six
	// positions and not four.
	TRUE_(worstFull < 0.005);
}

void testSixPositionRequiresRest() {
	// Rest detection is the difference between "held still in a position" and
	// "waved past one". Without it a capture would average motion.
	const double unityGain[3] = {1, 1, 1};
	const double noBias[3] = {0, 0, 0};

	SixPositionCollector c;
	c.begin();
	HoldTally tally;
	for (int p = 0; p < 6; p++) {
		float raw[3];
		rawForPosition(p, unityGain, 0.0, noBias, raw);
		feedN(c, raw, false, kSamplesPerPosition * 2, tally);
	}
	TRUE_(tally.started == 0);
	TRUE_(tally.captured == 0);
	TRUE_(c.capturedCount() == 0);
	TRUE_(c.isRunning());
}

void testSixPositionRetriesAfterMovement() {
	const double unityGain[3] = {1, 1, 1};
	const double noBias[3] = {0, 0, 0};
	float raw[3];
	rawForPosition(0, unityGain, 0.0, noBias, raw);

	SixPositionCollector c;
	c.begin();
	HoldTally tally;

	// Settle in, start capturing, then move before the position is finished.
	feedN(c, raw, true, kDwellSamples + kSamplesPerBlock, tally);
	TRUE_(tally.started == 1);
	TRUE_(c.activePosition() == 0);
	feedN(c, raw, false, 1, tally);
	TRUE_(tally.disturbed == 1);
	TRUE_(tally.captured == 0);
	TRUE_(c.capturedCount() == 0);
	TRUE_(c.activePosition() == -1);

	// The partial capture is discarded rather than resumed: hold it properly
	// and the full sample count is required again.
	feedN(c, raw, true, kSamplesPerPosition - 1, tally);
	TRUE_(tally.captured == 0);
	feedN(c, raw, true, 1, tally);
	TRUE_(tally.captured == 1);
	TRUE_(c.capturedCount() == 1);
}

void testSixPositionWillNotCaptureTwiceWithoutMoving() {
	// A tracker left sitting in one position must not fill the procedure with
	// six copies of the same orientation -- which would fit nothing, and would
	// do it confidently.
	const double unityGain[3] = {1, 1, 1};
	const double noBias[3] = {0, 0, 0};
	float raw[3];
	rawForPosition(4, unityGain, 0.0, noBias, raw);

	SixPositionCollector c;
	c.begin();
	HoldTally tally;
	feedN(c, raw, true, kSamplesPerPosition * 8, tally);

	TRUE_(tally.captured == 1);
	TRUE_(c.capturedCount() == 1);
	TRUE_(c.capturedMask() == (1u << 4));
	TRUE_(c.nextPosition() == 0);
	TRUE_(!tally.complete);
}

void testSixPositionDwellRejectsBriefTouches() {
	// The dwell requirement, exercised at its boundary: one sample short of it
	// nothing starts, one sample later it does. A capture that began the
	// instant a reading looked right would start while a hand is still moving.
	const double unityGain[3] = {1, 1, 1};
	const double noBias[3] = {0, 0, 0};
	float raw[3];
	rawForPosition(2, unityGain, 0.0, noBias, raw);

	SixPositionCollector c;
	c.begin();
	HoldTally tally;
	feedN(c, raw, true, kDwellSamples - 1, tally);
	TRUE_(tally.started == 0);
	feedN(c, raw, true, 1, tally);
	TRUE_(tally.started == 1);

	// Repeated brief touches never accumulate: the dwell counter restarts every
	// time the tracker leaves the position.
	SixPositionCollector brief;
	brief.begin();
	HoldTally briefTally;
	const float away[3] = {0, 0, 0};
	for (int i = 0; i < 40; i++) {
		feedN(brief, raw, true, kDwellSamples - 1, briefTally);
		feedN(brief, away, true, 1, briefTally);
	}
	TRUE_(briefTally.started == 0);
	TRUE_(brief.capturedCount() == 0);
}

void testSixPositionRejectsSloppyHolds() {
	// Held 35 degrees off every axis, the procedure refuses to advance rather
	// than quietly producing a worse calibration.
	SixPositionCollector c;
	c.begin();
	HoldTally tally;
	for (int p = 0; p < 6; p++) {
		float raw[3] = {0, 0, 0};
		const int axis = p / 2;
		const double sign = (p % 2 == 0) ? 1.0 : -1.0;
		raw[axis] = static_cast<float>(sign * kG * std::cos(deg2rad(35.0)));
		raw[(axis + 1) % 3] = static_cast<float>(kG * std::sin(deg2rad(35.0)));
		feedN(c, raw, true, kSamplesPerPosition * 2, tally);
	}
	TRUE_(tally.started == 0);
	TRUE_(c.capturedCount() == 0);
}

void testSixPositionAbortAndRestartAreClean() {
	const double unityGain[3] = {1, 1, 1};
	const double noBias[3] = {0, 0, 0};
	float raw[3];
	rawForPosition(0, unityGain, 0.0, noBias, raw);

	SixPositionCollector c;
	c.begin();
	HoldTally tally;
	feedN(c, raw, true, kSamplesPerPosition, tally);
	TRUE_(c.capturedCount() == 1);

	// Aborting stops it consuming samples at all.
	c.abort();
	TRUE_(!c.isRunning());
	const int before = tally.started;
	rawForPosition(2, unityGain, 0.0, noBias, raw);
	feedN(c, raw, true, kSamplesPerPosition, tally);
	TRUE_(tally.started == before);

	// Restarting discards the earlier progress rather than resuming it, so a
	// half-finished session cannot be mixed with a later one.
	c.begin();
	TRUE_(c.capturedCount() == 0);
	TRUE_(c.capturedMask() == 0);
	TRUE_(c.nextPosition() == 0);
}

void testAccelModelBoundsRejectImplausibleFits() {
	using namespace SlimeVR::Configuration;
	const float g = static_cast<float>(kG);

	ErrorModel ok;
	ok.m[0] = 1.02f;
	ok.m[4] = 0.98f;
	ok.m[8] = 1.0f;
	ok.bias[0] = 0.2f;
	TRUE_(checkAccelModel(ok, g) == AccelModelStatus::Ok);

	// A scale factor no accelerometer has: this is a failed measurement, not a
	// badly calibrated part, and applying it would look like a hardware fault.
	ErrorModel bigScale = ok;
	bigScale.m[4] = 1.4f;
	TRUE_(checkAccelModel(bigScale, g) == AccelModelStatus::ScaleOutOfRange);

	ErrorModel tinyScale = ok;
	tinyScale.m[8] = 0.4f;
	TRUE_(checkAccelModel(tinyScale, g) == AccelModelStatus::ScaleOutOfRange);

	ErrorModel skewed = ok;
	skewed.m[1] = 0.3f;
	TRUE_(checkAccelModel(skewed, g) == AccelModelStatus::MisalignmentOutOfRange);

	ErrorModel drifted = ok;
	drifted.bias[2] = 4.0f;
	TRUE_(checkAccelModel(drifted, g) == AccelModelStatus::BiasOutOfRange);

	ErrorModel diverged = ok;
	diverged.m[0] = std::numeric_limits<float>::quiet_NaN();
	TRUE_(checkAccelModel(diverged, g) == AccelModelStatus::NotFinite);

	ErrorModel infinite = ok;
	infinite.bias[1] = std::numeric_limits<float>::infinity();
	TRUE_(checkAccelModel(infinite, g) == AccelModelStatus::NotFinite);

	// An untouched default model is identity, which must be acceptable.
	ErrorModel identity;
	TRUE_(checkAccelModel(identity, g) == AccelModelStatus::Ok);
}

void testAccelModelStoreNormalisesGyroMatrix() {
	using namespace SlimeVR::Configuration;

	// The dead-tracker hazard, mirrored. Storing an accelerometer model sets
	// errorModelValid, and that one flag governs both matrices -- so a G_M that
	// was never fitted, and is therefore all zero, would go live and multiply
	// every gyroscope sample to zero.
	float aM[9] = {0};
	float gM[9] = {0};
	float aOff[3] = {0, 0, 0};
	bool accelCalibrated[3] = {false, false, false};

	ErrorModel model;
	model.m[0] = 1.02f;
	model.m[4] = 0.98f;
	model.m[8] = 1.01f;
	model.bias[0] = 0.1f;
	model.bias[1] = -0.2f;
	model.bias[2] = 0.05f;

	storeAccelModel(model, false, aM, gM, aOff, accelCalibrated);

	const bool gyroIsIdentity = gM[0] == 1.0f && gM[4] == 1.0f && gM[8] == 1.0f
							 && gM[1] == 0.0f && gM[2] == 0.0f && gM[3] == 0.0f
							 && gM[5] == 0.0f && gM[6] == 0.0f && gM[7] == 0.0f;
	TRUE_(gyroIsIdentity);

	// A gyroscope sample survives the round trip rather than being zeroed.
	const float gyro[3] = {1.0f, -2.0f, 3.0f};
	float out[3];
	ErrorModel stored;
	for (int i = 0; i < 9; i++) {
		stored.m[i] = gM[i];
	}
	stored.apply(gyro, out);
	NEAR(out[0], 1.0, 1e-6);
	NEAR(out[1], -2.0, 1e-6);
	NEAR(out[2], 3.0, 1e-6);
}

void testAccelModelStorePreservesFittedGyroMatrix() {
	using namespace SlimeVR::Configuration;

	// A gyroscope scale measured earlier with SET GYROSCALE must survive an
	// accelerometer calibration; the two are independent measurements and
	// re-running one is not a reason to discard the other.
	float aM[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
	float gM[9] = {1.005f, 0, 0, 0, 0.997f, 0, 0, 0, 1.002f};
	float aOff[3] = {0, 0, 0};
	bool accelCalibrated[3] = {false, false, false};

	ErrorModel model;
	model.m[0] = 1.02f;
	model.bias[0] = 0.1f;

	storeAccelModel(model, true, aM, gM, aOff, accelCalibrated);

	NEAR(gM[0], 1.005, 1e-6);
	NEAR(gM[4], 0.997, 1e-6);
	NEAR(gM[8], 1.002, 1e-6);
}

void testAccelModelStoreIsActuallyApplied() {
	using namespace SlimeVR::Configuration;

	// End to end through the shape the sample path uses:
	// corrected = A_M * (raw * AScale - A_off).
	const double gain[3] = {1.03, 0.97, 1.01};
	const double bias[3] = {0.12, -0.08, 0.05};

	std::vector<float> samples;
	for (int rep = 0; rep < 4; rep++) {
		for (int p = 0; p < 6; p++) {
			float raw[3];
			rawForPosition(p, gain, 0.0, bias, raw);
			samples.push_back(raw[0]);
			samples.push_back(raw[1]);
			samples.push_back(raw[2]);
		}
	}

	ErrorModel fitted;
	TRUE_(fitErrorModelDiagonal(samples.data(), 24, static_cast<float>(kG), fitted));
	TRUE_(checkAccelModel(fitted, static_cast<float>(kG)) == AccelModelStatus::Ok);

	float aM[9] = {0};
	float gM[9] = {0};
	float aOff[3] = {0, 0, 0};
	bool accelCalibrated[3] = {false, false, false};
	storeAccelModel(fitted, false, aM, gM, aOff, accelCalibrated);

	TRUE_(accelCalibrated[0] && accelCalibrated[1] && accelCalibrated[2]);

	double worst = 0;
	for (int p = 0; p < 6; p++) {
		float raw[3];
		rawForPosition(p, gain, 0.0, bias, raw);
		const float d[3] = {raw[0] - aOff[0], raw[1] - aOff[1], raw[2] - aOff[2]};
		const float corrected[3] = {
			aM[0] * d[0] + aM[1] * d[1] + aM[2] * d[2],
			aM[3] * d[0] + aM[4] * d[1] + aM[5] * d[2],
			aM[6] * d[0] + aM[7] * d[1] + aM[8] * d[2],
		};
		worst = std::max(
			worst,
			std::fabs(
				std::sqrt(
					corrected[0] * corrected[0] + corrected[1] * corrected[1]
					+ corrected[2] * corrected[2]
				)
				- kG
			)
		);
	}
	TRUE_(worst < 1e-3);
}

// ---------------------------------------------------------------------------
// Online (recursive) error-model estimation.
// ---------------------------------------------------------------------------

using SlimeVR::Sensors::SoftFusion::DiagonalNormalEquations;
using SlimeVR::Sensors::SoftFusion::kOnlineBlockSamples;
using SlimeVR::Sensors::SoftFusion::kOnlineMinObservations;
using SlimeVR::Sensors::SoftFusion::OnlineErrorEstimator;

/// Feeds one whole block pointing along `dir`, through a corrupted sensor.
static OnlineErrorEstimator::Result feedBlock(
	OnlineErrorEstimator& est,
	const double dir[3],
	const double gain[3],
	const double bias[3],
	bool atRest = true,
	double noise = 0.0,
	Rng* rng = nullptr
) {
	const double n = std::sqrt(dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2]);
	const double t[3] = {dir[0] / n * kG, dir[1] / n * kG, dir[2] / n * kG};
	float raw[3];
	corrupt(t, gain, 0.0, bias, raw);

	auto result = OnlineErrorEstimator::Result::Accumulating;
	for (size_t i = 0; i < kOnlineBlockSamples; i++) {
		float s[3] = {raw[0], raw[1], raw[2]};
		if (noise > 0.0 && rng != nullptr) {
			for (float& v : s) {
				v += static_cast<float>(noise * rng->normal());
			}
		}
		result = est.feed(s, atRest, static_cast<float>(kG));
	}
	return result;
}

/// The six axis directions, which are what ordinary handling eventually covers.
static const double kAxisDirs[6][3]
	= {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};

void testStreamingNormalEquationsMatchBatch() {
	// The refactor's whole claim: accumulating one sample at a time is the same
	// computation as the batch fit, not merely a similar one. If this drifts,
	// the online estimator and the guided flow would silently disagree about
	// the same sensor.
	const double gain[3] = {1.03, 0.97, 1.01};
	const double bias[3] = {0.12, -0.08, 0.05};

	std::vector<float> samples;
	for (int rep = 0; rep < 4; rep++) {
		for (int p = 0; p < 6; p++) {
			float raw[3];
			rawForPosition(p, gain, 0.0, bias, raw);
			samples.push_back(raw[0]);
			samples.push_back(raw[1]);
			samples.push_back(raw[2]);
		}
	}

	ErrorModel batch;
	TRUE_(fitErrorModelDiagonal(samples.data(), 24, static_cast<float>(kG), batch));

	DiagonalNormalEquations streamed;
	for (int i = 0; i < 24; i++) {
		streamed.add(&samples[i * 3]);
	}
	ErrorModel online;
	TRUE_(streamed.solve(static_cast<float>(kG), online));

	for (int i = 0; i < 9; i++) {
		NEAR(online.m[i], batch.m[i], 1e-9);
	}
	for (int i = 0; i < 3; i++) {
		NEAR(online.bias[i], batch.bias[i], 1e-9);
	}
}

void testOnlineRecoversBiasAndScale() {
	// The headline: no procedure, no stored samples -- just a tracker that has
	// been left in a variety of positions -- and the error it was built with
	// comes back out.
	const double gain[3] = {1.03, 0.97, 1.01};
	const double bias[3] = {0.12, -0.08, 0.05};

	OnlineErrorEstimator est;
	TRUE_(!est.isReady());

	for (int rep = 0; rep < 4; rep++) {
		for (const auto& d : kAxisDirs) {
			feedBlock(est, d, gain, bias);
		}
	}

	TRUE_(est.isReady());
	ErrorModel m;
	TRUE_(est.solve(static_cast<float>(kG), m));
	NEAR(m.bias[0], bias[0], 5e-3);
	NEAR(m.bias[1], bias[1], 5e-3);
	NEAR(m.bias[2], bias[2], 5e-3);
	NEAR(m.m[0], gain[0], 5e-3);
	NEAR(m.m[4], gain[1], 5e-3);
	NEAR(m.m[8], gain[2], 5e-3);

	// And it is a model the plausibility check would accept.
	TRUE_(
		SlimeVR::Configuration::checkAccelModel(m, static_cast<float>(kG))
		== SlimeVR::Configuration::AccelModelStatus::Ok
	);
}

void testOnlineIgnoresATrackerThatNeverMoves() {
	// The failure this estimator exists to avoid. A tracker left on a desk
	// overnight is at rest for millions of samples in one orientation. Counting
	// them would give an enormous, beautifully conditioned, completely useless
	// system -- and every health signal except coverage would look excellent.
	const double gain[3] = {1.0, 1.0, 1.0};
	const double bias[3] = {0, 0, 0};
	const double flat[3] = {0, 0, 1};

	OnlineErrorEstimator est;
	int observed = 0;
	int rejected = 0;
	for (int block = 0; block < 2000; block++) {
		const auto r = feedBlock(est, flat, gain, bias);
		if (r == OnlineErrorEstimator::Result::Observed) {
			observed++;
		} else if (r == OnlineErrorEstimator::Result::NotNovel) {
			rejected++;
		}
	}

	// Exactly one observation from 64000 rest samples in one place.
	TRUE_(observed == 1);
	TRUE_(rejected == 1999);
	TRUE_(!est.isReady());
	NEAR(est.observations(), 1.0, 1e-9);
}

void testOnlineNoveltyGateAcceptsRealMovement() {
	// The gate must not be so strict that ordinary handling stops registering.
	// 20 degrees apart is a different position by any reasonable reading.
	const double gain[3] = {1.0, 1.0, 1.0};
	const double bias[3] = {0, 0, 0};

	OnlineErrorEstimator est;
	int observed = 0;
	for (int i = 0; i < 8; i++) {
		const double angle = deg2rad(20.0 * i);
		const double d[3] = {std::sin(angle), 0, std::cos(angle)};
		if (feedBlock(est, d, gain, bias) == OnlineErrorEstimator::Result::Observed) {
			observed++;
		}
	}
	TRUE_(observed == 8);

	// But 5 degrees apart is the same position with a hand resting on it.
	OnlineErrorEstimator tiny;
	int tinyObserved = 0;
	for (int i = 0; i < 8; i++) {
		const double angle = deg2rad(5.0 * i);
		const double d[3] = {std::sin(angle), 0, std::cos(angle)};
		if (feedBlock(tiny, d, gain, bias) == OnlineErrorEstimator::Result::Observed) {
			tinyObserved++;
		}
	}
	TRUE_(tinyObserved < 4);
}

void testOnlineRequiresRest() {
	// Motion is not gravity. Without the rest gate the estimator would fit the
	// wrong vector entirely.
	const double gain[3] = {1.0, 1.0, 1.0};
	const double bias[3] = {0, 0, 0};

	OnlineErrorEstimator est;
	for (int rep = 0; rep < 8; rep++) {
		for (const auto& d : kAxisDirs) {
			feedBlock(est, d, gain, bias, /*atRest=*/false);
		}
	}
	NEAR(est.observations(), 0.0, 1e-12);
	TRUE_(!est.isReady());

	// A block interrupted partway through is discarded rather than completed
	// with whatever arrives next.
	OnlineErrorEstimator interrupted;
	const float flat[3] = {0, 0, static_cast<float>(kG)};
	for (size_t i = 0; i < kOnlineBlockSamples - 1; i++) {
		interrupted.feed(flat, true, static_cast<float>(kG));
	}
	interrupted.feed(flat, false, static_cast<float>(kG));
	for (size_t i = 0; i < kOnlineBlockSamples - 1; i++) {
		interrupted.feed(flat, true, static_cast<float>(kG));
	}
	NEAR(interrupted.observations(), 0.0, 1e-12);
}

void testOnlineRejectsSteadyNonGravity() {
	// Rest detection says "not moving", which is not the same as "measuring
	// gravity". A sensor mid-range-change reads steady and wrong.
	const double gain[3] = {1.0, 1.0, 1.0};
	const double bias[3] = {0, 0, 0};

	OnlineErrorEstimator est;
	for (int rep = 0; rep < 4; rep++) {
		for (const auto& d : kAxisDirs) {
			// Half-magnitude: steady, at rest, and not gravity.
			const double half[3] = {d[0] * 0.5, d[1] * 0.5, d[2] * 0.5};
			const double n
				= std::sqrt(half[0] * half[0] + half[1] * half[1] + half[2] * half[2]);
			float raw[3] = {
				static_cast<float>(half[0] / n * kG * 0.5),
				static_cast<float>(half[1] / n * kG * 0.5),
				static_cast<float>(half[2] / n * kG * 0.5),
			};
			for (size_t i = 0; i < kOnlineBlockSamples; i++) {
				est.feed(raw, true, static_cast<float>(kG));
			}
		}
	}
	NEAR(est.observations(), 0.0, 1e-12);
}

void testOnlineFollowsDriftingBias() {
	// Why forgetting exists. Bias moves with temperature; scale does not. An
	// estimator that weights a reading from hours ago equally with one from now
	// reports an average over conditions that have passed.
	const double gain[3] = {1.0, 1.0, 1.0};
	const double early[3] = {0.20, 0.0, 0.0};
	const double late[3] = {-0.20, 0.0, 0.0};

	OnlineErrorEstimator est;
	for (int rep = 0; rep < 8; rep++) {
		for (const auto& d : kAxisDirs) {
			feedBlock(est, d, gain, early);
		}
	}
	ErrorModel m;
	TRUE_(est.solve(static_cast<float>(kG), m));
	NEAR(m.bias[0], early[0], 5e-3);

	// The bias moves. After enough new observations the estimate follows it
	// rather than splitting the difference.
	for (int rep = 0; rep < 40; rep++) {
		for (const auto& d : kAxisDirs) {
			feedBlock(est, d, gain, late);
		}
	}
	TRUE_(est.solve(static_cast<float>(kG), m));
	NEAR(m.bias[0], late[0], 1e-2);
}

void testOnlineDoesNotConvergeFromWearAlone() {
	// The limitation worth knowing before anyone relies on this, and the reason
	// it supplements the guided flow rather than replacing it.
	//
	// A tracker strapped to a shin sees a narrow band of orientations: gravity
	// stays within a cone about one sensor axis however much the wearer walks.
	// That band never points -Z at the sky, so the axis whose scale and bias are
	// confounded stays confounded, and no amount of walking separates them.
	//
	// The estimator must refuse rather than return the confident answer a
	// well-conditioned-looking system would otherwise produce.
	const double gain[3] = {1.03, 0.97, 1.01};
	const double bias[3] = {0.12, -0.08, 0.05};

	OnlineErrorEstimator worn;
	Rng rng(11);
	for (int i = 0; i < 400; i++) {
		// Gravity within a 40 degree cone about +Z: a leg swinging.
		const double tilt = deg2rad(40.0) * rng.uniform();
		const double az = 2.0 * kPi * rng.uniform();
		const double d[3]
			= {std::sin(tilt) * std::cos(az),
			   std::sin(tilt) * std::sin(az),
			   std::cos(tilt)};
		feedBlock(worn, d, gain, bias);
	}

	// Plenty of observations, and they are genuinely spread in two dimensions.
	TRUE_(worn.observations() > kOnlineMinObservations);
	// But not in three, and never both ways along any axis.
	TRUE_(!worn.eachAxisBothWays());
	TRUE_(!worn.isReady());
	ErrorModel refused;
	TRUE_(!worn.solve(static_cast<float>(kG), refused));

	// Now the tracker comes off and spends the night on a shelf, on each of its
	// faces in turn -- which is what actually completes the picture. This is the
	// other half of the test: without it, the first half would only show an
	// estimator that never converges at all.
	for (int rep = 0; rep < 3; rep++) {
		for (const auto& d : kAxisDirs) {
			feedBlock(worn, d, gain, bias);
		}
	}
	TRUE_(worn.eachAxisBothWays());
	TRUE_(worn.isReady());

	ErrorModel m;
	TRUE_(worn.solve(static_cast<float>(kG), m));
	NEAR(m.bias[0], bias[0], 2e-2);
	NEAR(m.m[8], gain[2], 2e-2);
}

void testOnlineReadinessConditionsAreIndependent() {
	// `isReady` asks three questions, and it is easy to write a test suite in
	// which any one of them could be deleted without a failure -- because the
	// natural sample sets satisfy all three or none. Each half below is
	// constructed so exactly one condition fails.
	const double gain[3] = {1.0, 1.0, 1.0};
	const double bias[3] = {0, 0, 0};

	// Count alone. Six axis directions are perfect coverage by both other
	// measures, and still too few observations to act on.
	OnlineErrorEstimator sparse;
	for (const auto& d : kAxisDirs) {
		feedBlock(sparse, d, gain, bias);
	}
	TRUE_(sparse.eachAxisBothWays());
	TRUE_(
		sparse.directionSpread() >= SlimeVR::Sensors::SoftFusion::kMinDirectionSpread
	);
	TRUE_(sparse.observations() < kOnlineMinObservations);
	TRUE_(!sparse.isReady());

	// Spread alone. A tracker rocked back and forth through a shallow angle
	// sees every axis both ways -- |z| reaches 0.22, past the 0.2 that counts as
	// seeing an axis -- while barely leaving the xy plane. Both-ways is
	// satisfied and the directions still do not span three dimensions.
	OnlineErrorEstimator shallow;
	constexpr double lift = 0.22;
	const double ring = std::sqrt(1.0 - lift * lift);
	for (int i = 0; i < 24; i++) {
		const double th = deg2rad(30.0 * i);
		const double z = (i % 2 == 0) ? lift : -lift;
		const double d[3] = {ring * std::cos(th), ring * std::sin(th), z};
		feedBlock(shallow, d, gain, bias);
	}
	TRUE_(shallow.observations() >= kOnlineMinObservations);
	TRUE_(shallow.eachAxisBothWays());
	TRUE_(
		shallow.directionSpread() < SlimeVR::Sensors::SoftFusion::kMinDirectionSpread
	);
	TRUE_(!shallow.isReady());
}

void testOnlineStateIsBounded() {
	// The property that makes this viable on a microcontroller: the estimator
	// costs the same after a million samples as after one. If this ever grows a
	// buffer, it stops being usable on the boards that need it most -- and
	// those are precisely the multi-IMU boards where it would be worth the
	// most, so the ceiling is worth pinning rather than trusting.
	//
	// 368 bytes as written, 224 of which is the normal equations themselves.
	// For scale, the guided collector's sample buffer alone is 288 bytes and it
	// holds only 24 observations; this holds an unbounded history in less.
	TRUE_(sizeof(OnlineErrorEstimator) <= 512);

	// And it stays numerically sane over a long run rather than saturating.
	const double gain[3] = {1.02, 0.99, 1.0};
	const double bias[3] = {0.05, 0.05, -0.05};
	OnlineErrorEstimator est;
	for (int rep = 0; rep < 2000; rep++) {
		for (const auto& d : kAxisDirs) {
			feedBlock(est, d, gain, bias);
		}
	}
	ErrorModel m;
	TRUE_(est.solve(static_cast<float>(kG), m));
	NEAR(m.m[0], gain[0], 5e-3);
	NEAR(m.bias[2], bias[2], 5e-3);
	// Forgetting keeps the effective count near its window rather than letting
	// it run to 12000, which is what would eventually destroy the precision.
	TRUE_(est.observations() < 100.0);
}

void testOnlineSurvivesNoise() {
	// Realistic block-mean noise must not push the estimate outside what the
	// plausibility check accepts, or the feature would spend its life refusing
	// its own output.
	const double gain[3] = {1.03, 0.97, 1.01};
	const double bias[3] = {0.12, -0.08, 0.05};
	Rng rng(5);

	OnlineErrorEstimator est;
	for (int rep = 0; rep < 20; rep++) {
		for (const auto& d : kAxisDirs) {
			feedBlock(est, d, gain, bias, true, 0.02, &rng);
		}
	}
	ErrorModel m;
	TRUE_(est.solve(static_cast<float>(kG), m));
	TRUE_(
		SlimeVR::Configuration::checkAccelModel(m, static_cast<float>(kG))
		== SlimeVR::Configuration::AccelModelStatus::Ok
	);
	NEAR(m.m[0], gain[0], 1e-2);
	NEAR(m.bias[1], bias[1], 2e-2);
}

int main() {
	testQuatBasics();
	testIntegration();
	testMetricsOnPerfectEstimate();
	testConstantHeadingOffsetIsRemoved();
	testHeadingDriftIsMeasured();
	testDatasetRoundTrip();
	testDeterminism();
	testCleanStaticConverges();
	testGyroBiasProducesDrift();
	testRawCaptureFormat();
	testInterleavedMatchesSynchronous();
	testPresenceSurvivesRoundTrip();
	testMagFifo24BitSplit();
	testMagFifo16BitSingleSlave();
	testMagFifoDroppedSlave1Recovers();
	testMagFifoRejectsBadConfig();
	testSignExtend24Boundaries();
	testBmm350FixSign();
	testBmm350UncalibratedPassthrough();
	testBmm350IdentityCalibration();
	testBmm350OffsetAndSensitivity();
	testBmm350TemperatureTermsVanishAtT0();
	testBmm350CrossAxis();
	testBmm350OtpDecoding();
	testBmm350TemperatureConversion();
	testErrorModelIdentityByDefault();
	testErrorModelApply();
	testFitRecoversBiasScaleAndMisalignment();
	testFitIsNearIdentityForAPerfectSensor();
	testFitRejectsInsufficientData();
	testFitRejectsDegenerateOrientations();
	testFitRejectsNearDegenerateCoverage();
	testFitAcceptsSixPositionCoverage();
	testGyroScaleRecoversKnownError();
	testGyroScaleIsUnityForAPerfectSensor();
	testGyroScaleRefusesWhenGravityNeverMoves();
	testGyroScaleRefusesWithoutRestPeriods();
	testSmallestEigenvalueOfKnownMatrices();
	testGyroScaleArgParsing();
	testGyroScaleRangeRejection();
	testGyroScaleModelNormalisesAccelMatrix();
	testGyroScaleModelPreservesFittedAccelMatrix();
	testGyroScaleReportsDiscardedMisalignment();
	testGyroScaleModelIsActuallyApplied();
	testSixPositionClassifiesAxes();
	testSixPositionCapturesAndFits();
	testSixPositionFullFitIsSingularOnPerfectPositions();
	testDiagonalFitHelpsEvenWhenTheSensorIsMisaligned();
	testDiagonalFitIsUnityForAPerfectSensor();
	testDiagonalFitRejectsPoorCoverage();
	testDiagonalFitRejectsMarginalCoverage();
	testSixPositionRequiresRest();
	testSixPositionRetriesAfterMovement();
	testSixPositionWillNotCaptureTwiceWithoutMoving();
	testSixPositionDwellRejectsBriefTouches();
	testSixPositionRejectsSloppyHolds();
	testSixPositionAbortAndRestartAreClean();
	testAccelModelBoundsRejectImplausibleFits();
	testAccelModelStoreNormalisesGyroMatrix();
	testAccelModelStorePreservesFittedGyroMatrix();
	testAccelModelStoreIsActuallyApplied();
	testStreamingNormalEquationsMatchBatch();
	testOnlineRecoversBiasAndScale();
	testOnlineIgnoresATrackerThatNeverMoves();
	testOnlineNoveltyGateAcceptsRealMovement();
	testOnlineRequiresRest();
	testOnlineRejectsSteadyNonGravity();
	testOnlineFollowsDriftingBias();
	testOnlineDoesNotConvergeFromWearAlone();
	testOnlineReadinessConditionsAreIndependent();
	testOnlineStateIsBounded();
	testOnlineSurvivesNoise();

	if (gFailures == 0) {
		std::printf("selftest: %d checks passed\n", gChecks);
		return 0;
	}
	std::fprintf(stderr, "selftest: %d of %d checks FAILED\n", gFailures, gChecks);
	return 1;
}
