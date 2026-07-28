// Self-tests for the benchmark harness itself.
//
// These do not test the firmware. They test that the *measuring instrument* is
// correct -- that the quaternion algebra, the error decomposition, and the
// determinism guarantees hold. A benchmark whose own arithmetic is wrong is
// worse than no benchmark, because it produces confident numbers.
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

#include "dataset.h"
#include "metrics.h"
#include "quatmath.h"
#include "synth.h"

// The sensor-hub FIFO assembler, compiled straight from the firmware tree. It
// has no Arduino dependency precisely so it can be tested here.
#include "../../../src/sensors/softfusion/drivers/magfifo.h"

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

}  // namespace

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

	if (gFailures == 0) {
		std::printf("selftest: %d checks passed\n", gChecks);
		return 0;
	}
	std::fprintf(stderr, "selftest: %d of %d checks FAILED\n", gFailures, gChecks);
	return 1;
}
