// Self-tests for the benchmark harness itself.
//
// These do not test the firmware. They test that the *measuring instrument* is
// correct -- that the quaternion algebra, the error decomposition, and the
// determinism guarantees hold. A benchmark whose own arithmetic is wrong is
// worse than no benchmark, because it produces confident numbers.
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "dataset.h"
#include "metrics.h"
#include "quatmath.h"
#include "synth.h"

using namespace fb;

namespace {

int gFailures = 0;
int gChecks = 0;

void checkNear(
	const char* what, double got, double want, double tol, int line
) {
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

	if (gFailures == 0) {
		std::printf("selftest: %d checks passed\n", gChecks);
		return 0;
	}
	std::fprintf(stderr, "selftest: %d of %d checks FAILED\n", gFailures, gChecks);
	return 1;
}
