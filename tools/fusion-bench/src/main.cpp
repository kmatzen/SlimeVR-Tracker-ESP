// fusion-bench -- host-side benchmark for the tracker's orientation fusion.
//
//   fusion-bench suite [--baseline FILE] [--write-baseline FILE] [--json FILE]
//   fusion-bench gen TRAJECTORY OUT.csv [--duration S] [--rate HZ] [--seed N]
//                                       [--gyro-bias DPS] [--gyro-noise DPS]
//                                       [--gyro-scale FRAC] [--accel-bias MS2]
//                                       [--accel-noise MS2] [--with-mag]
//   fusion-bench run DATASET.csv [--json FILE] [--stock] [--mag]
//                                [--tau-acc X] [--rest-th-gyr X]
//                                [--rest-th-acc X] [--rest-min-t X]
//   fusion-bench sweep DATASET.csv PARAM FROM TO STEPS
//   fusion-bench noise DATASET.csv [--rest-min-t X] [--rest-th-acc X] [--stock]
//
// See README.md for what the numbers mean and how to produce real datasets.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "dataset.h"
#include "gyroscale.h"
#include "metrics.h"
#include "noisefloor.h"
#include "synth.h"

using namespace fb;

namespace {

int usage() {
	std::string trajectories;
	listTrajectories(trajectories);
	std::fprintf(
		stderr,
		"usage:\n"
		"  fusion-bench suite [--baseline FILE] [--write-baseline FILE] "
		"[--json FILE]\n"
		"  fusion-bench gen TRAJECTORY OUT.csv [options]\n"
		"  fusion-bench run DATASET.csv [options]\n"
		"  fusion-bench sweep DATASET.csv PARAM FROM TO STEPS\n"
		"  fusion-bench gyro-scale DATASET.csv\n"
		"  fusion-bench noise DATASET.csv [options]\n"
		"\ntrajectories: %s\n"
		"sweep params: tau-acc, rest-th-gyr, rest-th-acc, rest-min-t\n"
		"gen options:  --duration S --rate HZ --seed N --gyro-bias DPS\n"
		"              --gyro-noise DPS --gyro-scale FRAC --accel-bias MS2\n"
		"              --accel-noise MS2 --with-mag\n"
		"run options:  --json FILE --stock --mag --tau-acc X --rest-th-gyr X\n"
		"              --rest-th-acc X --rest-min-t X\n",
		trajectories.c_str()
	);
	return 2;
}

bool hasFlag(int argc, char** argv, const char* name) {
	for (int i = 0; i < argc; i++) {
		if (std::strcmp(argv[i], name) == 0) {
			return true;
		}
	}
	return false;
}

const char* flagValue(int argc, char** argv, const char* name) {
	for (int i = 0; i + 1 < argc; i++) {
		if (std::strcmp(argv[i], name) == 0) {
			return argv[i + 1];
		}
	}
	return nullptr;
}

double flagDouble(int argc, char** argv, const char* name, double def) {
	const char* v = flagValue(argc, argv, name);
	return v ? std::strtod(v, nullptr) : def;
}

// --- The standard suite ---------------------------------------------------
//
// Each case isolates something. Keep them small and fast: the whole suite
// should run in a couple of seconds so it can sit in CI on every PR.
struct SuiteCase {
	const char* name;
	const char* trajectory;
	double durationSec;
	double gyroBiasDps;
	double gyroScaleErr;
	bool useTunedParams;  // false => VQF stock defaults
};

const SuiteCase kSuite[] = {
	// Drift with a realistic residual bias -- the headline regression test.
	//
	// This row is also the harness's rest-cliff regression guard, though it was
	// not designed as one. It runs at the synthetic default accelerometer noise
	// of 0.02 m/s^2 per axis, where the cliff sits at 0.0668, and the tuned
	// restThAcc of 0.06 falls 10% under it -- so rest is never detected here and
	// the baseline drift is -3.577 deg/min rather than the -0.018 the other rows
	// see. If a change moves this row toward zero, the cliff moved, and that is
	// worth knowing.
	{"static-tuned", "static", 120.0, 0.15, 0.0, true},
	// Same input, stock VQF parameters -- and stock is what every softfusion IMU
	// actually runs, so this row is the realistic one.
	//
	// The delta between these two rows was read as the cost of issue #4 (tuned
	// params never reaching softfusion IMUs). Measured, it is not: it is the
	// cliff above, and it exists only because the synthetic noise here is about
	// 5x the 0.004 m/s^2 per-axis noise measured on a real LSM6DSV. At real
	// noise both parameter sets clear the cliff and perform identically, which is
	// what Bench Test A found on hardware. `fusion-bench noise` reports the
	// margin for any capture.
	{"static-stock", "static", 120.0, 0.15, 0.0, false},
	// Tilt tracking away from level.
	{"tilted", "static-tilted", 60.0, 0.15, 0.0, true},
	// Heading tracking through sustained yaw motion.
	{"yaw-sweep", "yaw-sweep", 60.0, 0.05, 0.0, true},
	// General multi-axis motion.
	{"tumble", "tumble", 60.0, 0.05, 0.0, true},
	// Net-zero rotation: whatever heading error remains is accumulated error.
	{"return-to-origin", "return-to-origin", 60.0, 0.05, 0.0, true},
	// 1% gyro scale error with no bias. Bias calibration cannot see this, so it
	// is the acceptance test for issue #5 (calibration models bias only).
	{"scale-error-1pct", "yaw-sweep", 60.0, 0.0, 0.01, true},
	// Limb-like motion with linear acceleration disturbing the gravity
	// reference.
	{"walk", "walk", 60.0, 0.05, 0.0, true},
};

struct BaselineEntry {
	double value;
	double tolerance;
};

bool readBaseline(
	const std::string& path,
	std::map<std::string, BaselineEntry>& out,
	std::string& error
) {
	std::ifstream in(path);
	if (!in) {
		error = "cannot open baseline " + path;
		return false;
	}
	std::string line;
	while (std::getline(in, line)) {
		size_t hash = line.find('#');
		if (hash != std::string::npos) {
			line = line.substr(0, hash);
		}
		std::istringstream is(line);
		std::string key;
		double value = 0;
		double tol = 0;
		if (!(is >> key >> value >> tol)) {
			continue;
		}
		out[key] = BaselineEntry{value, tol};
	}
	return true;
}

// Metrics that go into the baseline, per case. Deliberately a short list:
// a baseline nobody reads is a baseline nobody maintains.
void collectKeys(
	const std::string& caseName,
	const Metrics& m,
	std::vector<std::pair<std::string, double>>& out
) {
	if (m.hasDriftEstimate) {
		out.emplace_back(
			caseName + "/heading_drift_deg_per_min",
			m.headingDriftDegPerMin
		);
		out.emplace_back(caseName + "/jitter_deg_rms", m.jitterDegRms);
	}
	out.emplace_back(caseName + "/tilt_error_deg_rms", m.tiltErrorDegRms);
	if (m.hasGroundTruth) {
		out.emplace_back(caseName + "/total_error_deg_rms", m.totalErrorDegRms);
		out.emplace_back(caseName + "/heading_error_deg_rms", m.headingErrorDegRms);
		out.emplace_back(
			caseName + "/inclination_error_deg_rms",
			m.inclinationErrorDegRms
		);
		out.emplace_back(caseName + "/final_heading_error_deg", m.finalHeadingErrorDeg);
	}
}

int cmdSuite(int argc, char** argv) {
	const char* baselinePath = flagValue(argc, argv, "--baseline");
	const char* writeBaselinePath = flagValue(argc, argv, "--write-baseline");
	const char* jsonPath = flagValue(argc, argv, "--json");

	std::vector<std::pair<std::string, double>> values;
	std::ostringstream json;
	json << "{\n  \"results\": {\n";

	for (size_t i = 0; i < sizeof(kSuite) / sizeof(kSuite[0]); i++) {
		const SuiteCase& c = kSuite[i];

		SynthParams sp;
		sp.durationSec = c.durationSec;
		sp.rateHz = 250.0;
		sp.seed = 1000 + i;
		sp.gyroBiasDps = c.gyroBiasDps;
		sp.gyroScaleErr = c.gyroScaleErr;

		Dataset ds;
		std::string err;
		if (!generate(c.trajectory, sp, ds, err)) {
			std::fprintf(stderr, "generate %s: %s\n", c.name, err.c_str());
			return 1;
		}
		ds.name = c.name;

		BenchParams bp;
		bp.stock = !c.useTunedParams;

		RunResult run = runFusion(ds, bp);
		Metrics m = computeMetrics(ds, run);
		collectKeys(c.name, m, values);

		json << "    \"" << c.name << "\": " << metricsToJson(m);
		json << ((i + 1 < sizeof(kSuite) / sizeof(kSuite[0])) ? ",\n" : "\n");

		char drift[32];
		if (m.hasDriftEstimate) {
			std::snprintf(drift, sizeof(drift), "%8.4f", m.headingDriftDegPerMin);
		} else {
			// No stationary segment long enough to fit a drift rate. Printing a
			// number here would be printing the motion, not the error.
			std::snprintf(drift, sizeof(drift), "%8s", "n/a");
		}
		std::printf(
			"%-18s  drift %s deg/min   tilt %6.3f deg   total %6.3f deg   "
			"heading %6.3f deg   final %6.3f deg\n",
			c.name,
			drift,
			m.tiltErrorDegRms,
			m.totalErrorDegRms,
			m.headingErrorDegRms,
			m.finalHeadingErrorDeg
		);
	}

	json << "  }\n}\n";

	if (jsonPath) {
		std::ofstream o(jsonPath);
		o << json.str();
		std::printf("wrote %s\n", jsonPath);
	}

	if (writeBaselinePath) {
		std::ofstream o(writeBaselinePath);
		o << "# fusion-bench baseline\n"
		  << "# key  value  tolerance\n"
		  << "#\n"
		  << "# Tolerances should come from measured run-to-run spread, not\n"
		  << "# from taste. See README.md, 'Setting tolerances'.\n";
		char buf[256];
		for (const auto& kv : values) {
			// Default tolerance: 5% of magnitude with a small absolute floor,
			// as a starting point to be replaced by measured values.
			double tol = std::fabs(kv.second) * 0.05;
			if (tol < 1e-3) {
				tol = 1e-3;
			}
			std::snprintf(
				buf,
				sizeof(buf),
				"%-46s %12.6f %12.6f\n",
				kv.first.c_str(),
				kv.second,
				tol
			);
			o << buf;
		}
		std::printf("wrote %s\n", writeBaselinePath);
	}

	if (baselinePath) {
		std::map<std::string, BaselineEntry> base;
		std::string err;
		if (!readBaseline(baselinePath, base, err)) {
			std::fprintf(stderr, "%s\n", err.c_str());
			return 1;
		}
		int failures = 0;
		std::printf(
			"\n%-46s %12s %12s %12s\n",
			"metric",
			"baseline",
			"current",
			"delta"
		);
		for (const auto& kv : values) {
			auto it = base.find(kv.first);
			if (it == base.end()) {
				std::printf(
					"%-46s %12s %12.6f   (new)\n",
					kv.first.c_str(),
					"-",
					kv.second
				);
				continue;
			}
			double delta = kv.second - it->second.value;
			bool bad = std::fabs(delta) > it->second.tolerance;
			std::printf(
				"%-46s %12.6f %12.6f %12.6f %s\n",
				kv.first.c_str(),
				it->second.value,
				kv.second,
				delta,
				bad ? "REGRESSION" : ""
			);
			if (bad) {
				failures++;
			}
		}
		if (failures > 0) {
			std::fprintf(stderr, "\n%d metric(s) outside tolerance\n", failures);
			return 1;
		}
		std::printf("\nall metrics within tolerance\n");
	}

	return 0;
}

int cmdGen(int argc, char** argv) {
	if (argc < 4) {
		return usage();
	}
	const std::string traj = argv[2];
	const std::string out = argv[3];

	SynthParams p;
	p.durationSec = flagDouble(argc, argv, "--duration", p.durationSec);
	p.rateHz = flagDouble(argc, argv, "--rate", p.rateHz);
	p.seed = static_cast<uint64_t>(flagDouble(argc, argv, "--seed", 1));
	p.gyroBiasDps = flagDouble(argc, argv, "--gyro-bias", p.gyroBiasDps);
	p.gyroNoiseDps = flagDouble(argc, argv, "--gyro-noise", p.gyroNoiseDps);
	p.gyroScaleErr = flagDouble(argc, argv, "--gyro-scale", p.gyroScaleErr);
	p.accelBias = flagDouble(argc, argv, "--accel-bias", p.accelBias);
	p.accelNoise = flagDouble(argc, argv, "--accel-noise", p.accelNoise);
	p.withMag = hasFlag(argc, argv, "--with-mag");

	Dataset ds;
	std::string err;
	if (!generate(traj, p, ds, err)) {
		std::fprintf(stderr, "%s\n", err.c_str());
		return 1;
	}
	if (!saveDataset(out, ds, err)) {
		std::fprintf(stderr, "%s\n", err.c_str());
		return 1;
	}
	std::printf(
		"wrote %s (%zu samples, %.1f s)\n",
		out.c_str(),
		ds.samples.size(),
		ds.durationSec()
	);
	return 0;
}

BenchParams parseBenchParams(int argc, char** argv) {
	BenchParams bp;
	bp.stock = hasFlag(argc, argv, "--stock");
	bp.useMag = hasFlag(argc, argv, "--mag");
	bp.tauAcc = flagDouble(argc, argv, "--tau-acc", bp.tauAcc);
	bp.restThGyr = flagDouble(argc, argv, "--rest-th-gyr", bp.restThGyr);
	bp.restThAcc = flagDouble(argc, argv, "--rest-th-acc", bp.restThAcc);
	bp.restMinT = flagDouble(argc, argv, "--rest-min-t", bp.restMinT);
	return bp;
}

int cmdRun(int argc, char** argv) {
	if (argc < 3) {
		return usage();
	}
	Dataset ds;
	std::string err;
	if (!loadDataset(argv[2], ds, err)) {
		std::fprintf(stderr, "%s\n", err.c_str());
		return 1;
	}

	// A serial capture can contain firmware log output interleaved with the
	// data. Those lines are skipped, but say so -- a capture that quietly lost
	// half its samples would still produce confident-looking numbers.
	if (ds.skippedLines > 0) {
		std::fprintf(
			stderr,
			"note: skipped %zu non-data line(s) in %s (kept %zu samples)\n",
			ds.skippedLines,
			argv[2],
			ds.samples.size()
		);
	}

	RunResult run = runFusion(ds, parseBenchParams(argc, argv));
	Metrics m = computeMetrics(ds, run);
	std::string j = metricsToJson(m);

	if (const char* jsonPath = flagValue(argc, argv, "--json")) {
		std::ofstream o(jsonPath);
		o << j << "\n";
	}
	std::printf("%s\n", j.c_str());
	return 0;
}

int cmdSweep(int argc, char** argv) {
	if (argc < 7) {
		return usage();
	}
	Dataset ds;
	std::string err;
	if (!loadDataset(argv[2], ds, err)) {
		std::fprintf(stderr, "%s\n", err.c_str());
		return 1;
	}
	const std::string param = argv[3];
	const double from = std::strtod(argv[4], nullptr);
	const double to = std::strtod(argv[5], nullptr);
	const int steps = static_cast<int>(std::strtol(argv[6], nullptr, 10));
	if (steps < 2) {
		std::fprintf(stderr, "steps must be >= 2\n");
		return 1;
	}

	std::printf(
		"%-14s %14s %14s %14s %14s\n",
		param.c_str(),
		"drift/min",
		"tilt_rms",
		"total_rms",
		"heading_rms"
	);
	for (int i = 0; i < steps; i++) {
		double v = from + (to - from) * i / (steps - 1);
		BenchParams bp = parseBenchParams(argc, argv);
		if (param == "tau-acc") {
			bp.tauAcc = v;
		} else if (param == "rest-th-gyr") {
			bp.restThGyr = v;
		} else if (param == "rest-th-acc") {
			bp.restThAcc = v;
		} else if (param == "rest-min-t") {
			bp.restMinT = v;
		} else {
			std::fprintf(stderr, "unknown sweep param '%s'\n", param.c_str());
			return 1;
		}
		Metrics m = computeMetrics(ds, runFusion(ds, bp));
		std::printf(
			"%-14.5f %14.5f %14.5f %14.5f %14.5f\n",
			v,
			m.headingDriftDegPerMin,
			m.tiltErrorDegRms,
			m.totalErrorDegRms,
			m.headingErrorDegRms
		);
	}
	return 0;
}

int cmdGyroScale(int argc, char** argv) {
	if (argc < 3) {
		return usage();
	}
	Dataset ds;
	std::string err;
	if (!loadDataset(argv[2], ds, err)) {
		std::fprintf(stderr, "%s\n", err.c_str());
		return 1;
	}

	const GyroScaleResult r = estimateGyroScale(ds);

	std::printf("rest-to-rest transitions used: %d\n", r.segments);
	std::printf(
		"gravity prediction error: %.3f deg before, %.3f deg after\n",
		r.residualBeforeDeg,
		r.residualAfterDeg
	);
	std::printf(
		"observability (deg per 1%% of scale): x %.3f  y %.3f  z %.3f\n",
		r.observability[0],
		r.observability[1],
		r.observability[2]
	);

	if (!r.valid) {
		// Refusing is the useful answer here. A scale factor fitted from motion
		// that never moved gravity is noise wearing a number's clothes.
		std::fprintf(stderr, "no usable estimate: %s\n", r.reason.c_str());
		return 1;
	}

	std::printf(
		"\nscale (multiply measured rate by this):\n"
		"  x %.5f  y %.5f  z %.5f\n",
		r.scale[0],
		r.scale[1],
		r.scale[2]
	);
	std::printf(
		"gyroscope reads high by: x %+.3f%%  y %+.3f%%  z %+.3f%%\n",
		(1.0 / r.scale[0] - 1.0) * 100.0,
		(1.0 / r.scale[1] - 1.0) * 100.0,
		(1.0 / r.scale[2] - 1.0) * 100.0
	);
	return 0;
}

int cmdNoise(int argc, char** argv) {
	if (argc < 3) {
		return usage();
	}
	Dataset ds;
	std::string err;
	if (!loadDataset(argv[2], ds, err)) {
		std::fprintf(stderr, "%s\n", err.c_str());
		return 1;
	}
	const BenchParams p = parseBenchParams(argc, argv);
	const AccelNoiseResult r = measureAccelNoise(ds, p);

	std::printf(
		"accelerometer noise (m/s^2, 1 sigma):\n"
		"  per axis     x %.5f  y %.5f  z %.5f\n"
		"  vector       %.5f\n",
		r.axisSigma[0],
		r.axisSigma[1],
		r.axisSigma[2],
		r.vectorSigma
	);

	if (!r.valid) {
		std::fprintf(stderr, "no threshold estimate: %s\n", r.reason.c_str());
		return 1;
	}

	const double configured = r.configuredThreshold;
	const double margin
		= r.settledThreshold > 0 ? configured / r.settledThreshold : 0.0;

	std::printf(
		"\nrest residual against VQF's low-pass (the quantity restThAcc bounds):\n"
		"  rms          %.5f\n"
		"  peak         %.5f\n"
		"\nrest window    %zu samples (restMinT %.3f s at %.1f Hz)\n"
		"accel samples  %zu\n",
		r.residualRms,
		r.residualPeak,
		r.restWindowSamples,
		p.restMinT,
		ds.accTs > 0 ? 1.0 / ds.accTs : 0.0,
		r.accelSamples
	);

	std::printf(
		"\nminimum restThAcc that admits rest:\n"
		"  anywhere              %.5f  (includes the filter's startup transient)\n"
		"  once settled          %.5f  (= %.2f x vector sigma) <- configure against "
		"this\n"
		"configured restThAcc:   %.6f  (%.1fx the settled minimum)\n",
		r.requiredThreshold,
		r.settledThreshold,
		r.sigmaMultiple,
		configured,
		margin
	);

	// A threshold between the two bounds catches rest during startup and never
	// again. Worth calling out separately: first_rest_sec looks healthy, so the
	// usual diagnostic says nothing is wrong.
	if (configured >= r.requiredThreshold && configured < r.settledThreshold) {
		std::fprintf(
			stderr,
			"\nFAIL: threshold sits between the two bounds. Rest is reached during\n"
			"the filter's startup transient and never again, so first_rest_sec looks\n"
			"healthy while bias estimation stops after boot. Raise restThAcc to at\n"
			"least %.5f.\n",
			r.settledThreshold
		);
		return 1;
	}

	// Below the cliff the failure is total and silent: rest is never detected,
	// so gyroscope bias estimation never runs. Say so loudly, and exit non-zero,
	// because a capture that cannot reach rest makes every other metric from it
	// meaningless.
	if (configured < r.requiredThreshold) {
		std::fprintf(
			stderr,
			"\nFAIL: configured threshold is below the cliff -- rest will never be\n"
			"detected on this hardware, so gyroscope bias estimation will never run.\n"
			"Nothing in the firmware's logs reports this; first_rest_sec = -1 is the\n"
			"symptom. Raise restThAcc to at least %.5f.\n",
			r.requiredThreshold
		);
		return 1;
	}
	if (margin < kMinThresholdMargin) {
		std::fprintf(
			stderr,
			"\nWARNING: only %.1fx margin over the cliff (want %.1fx). Rest is\n"
			"detected on this capture but a noisier part, a higher bandwidth\n"
			"setting, or a vibrating mount could cross it.\n",
			margin,
			kMinThresholdMargin
		);
	}
	return 0;
}

}  // namespace

int main(int argc, char** argv) {
	if (argc < 2) {
		return usage();
	}
	const std::string cmd = argv[1];
	if (cmd == "suite") {
		return cmdSuite(argc, argv);
	}
	if (cmd == "gen") {
		return cmdGen(argc, argv);
	}
	if (cmd == "run") {
		return cmdRun(argc, argv);
	}
	if (cmd == "sweep") {
		return cmdSweep(argc, argv);
	}
	if (cmd == "gyro-scale") {
		return cmdGyroScale(argc, argv);
	}
	if (cmd == "noise") {
		return cmdNoise(argc, argv);
	}
	return usage();
}
