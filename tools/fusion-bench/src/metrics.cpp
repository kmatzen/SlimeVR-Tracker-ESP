#include "metrics.h"

#include <algorithm>
#include <cstdio>
#include <sstream>

#include "vqf.h"

namespace fb {
namespace {

double rms(const std::vector<double>& v) {
	if (v.empty()) {
		return 0.0;
	}
	double s = 0;
	for (double x : v) {
		s += x * x;
	}
	return std::sqrt(s / static_cast<double>(v.size()));
}

// Least-squares slope of y against x.
double slope(const std::vector<double>& x, const std::vector<double>& y) {
	size_t n = x.size();
	if (n < 2) {
		return 0.0;
	}
	double sx = 0, sy = 0;
	for (size_t i = 0; i < n; i++) {
		sx += x[i];
		sy += y[i];
	}
	double mx = sx / static_cast<double>(n);
	double my = sy / static_cast<double>(n);
	double num = 0, den = 0;
	for (size_t i = 0; i < n; i++) {
		num += (x[i] - mx) * (y[i] - my);
		den += (x[i] - mx) * (x[i] - mx);
	}
	if (den == 0) {
		return 0.0;
	}
	return num / den;
}

}  // namespace

RunResult runFusion(const Dataset& ds, const BenchParams& p) {
	VQFParams params;  // VQF's own defaults
	if (!p.stock) {
		params.tauAcc = static_cast<vqf_real_t>(p.tauAcc);
		params.restMinT = static_cast<vqf_real_t>(p.restMinT);
		params.restThGyr = static_cast<vqf_real_t>(p.restThGyr);
		params.restThAcc = static_cast<vqf_real_t>(p.restThAcc);
	}

	const bool useMag = p.useMag && ds.hasMag;

	VQF vqf(
		params,
		static_cast<vqf_real_t>(ds.gyrTs),
		static_cast<vqf_real_t>(ds.accTs),
		static_cast<vqf_real_t>(ds.magTs)
	);

	RunResult r;
	r.usedMag = useMag;
	r.est.reserve(ds.samples.size());

	size_t magDist = 0;
	uint64_t t0 = ds.samples.front().tUs;
	uint64_t prevGyrT = t0;
	bool haveGyrT = false;

	for (size_t i = 0; i < ds.samples.size(); i++) {
		const Sample& s = ds.samples[i];

		// Use the log's own timestamps rather than the nominal rate, so that
		// jitter and dropped samples in a real capture are modelled rather than
		// silently smoothed away.
		//
		// Measured between consecutive *gyroscope* rows. A real capture
		// interleaves accelerometer and gyroscope rows at different rates, so
		// the gap to the previous row of any kind is not the gyroscope's
		// timestep and feeding it to VQF would misstate the integration
		// interval on every sample.
		double dt = ds.gyrTs;
		if (s.hasGyr && haveGyrT) {
			dt = static_cast<double>(s.tUs - prevGyrT) * 1e-6;
			if (dt <= 0) {
				dt = ds.gyrTs;
			}
		}
		if (s.hasGyr) {
			prevGyrT = s.tUs;
			haveGyrT = true;
		}

		vqf_real_t acc[3] = {
			static_cast<vqf_real_t>(s.acc.x),
			static_cast<vqf_real_t>(s.acc.y),
			static_cast<vqf_real_t>(s.acc.z),
		};
		vqf_real_t gyr[3] = {
			static_cast<vqf_real_t>(s.gyr.x),
			static_cast<vqf_real_t>(s.gyr.y),
			static_cast<vqf_real_t>(s.gyr.z),
		};

		// Order matches SensorFusion::update9D / update6D. Each sensor is
		// updated only on rows that actually carry it -- feeding a repeated
		// accelerometer sample on every gyroscope row would double-count the
		// accelerometer correction and change what the filter does.
		if (useMag && s.hasMag) {
			vqf_real_t mag[3] = {
				static_cast<vqf_real_t>(s.mag.x),
				static_cast<vqf_real_t>(s.mag.y),
				static_cast<vqf_real_t>(s.mag.z),
			};
			vqf.updateMag(mag);
		}
		if (s.hasAcc) {
			vqf.updateAcc(acc);
		}
		if (s.hasGyr) {
			vqf.updateGyr(gyr, static_cast<vqf_real_t>(dt));
		}

		vqf_real_t q[4];
		if (useMag) {
			vqf.getQuat9D(q);
		} else {
			vqf.getQuat6D(q);
		}
		r.est.push_back(qNorm(Quat{q[0], q[1], q[2], q[3]}));

		if (r.firstRestSec < 0 && vqf.getRestDetected()) {
			r.firstRestSec = static_cast<double>(s.tUs - t0) * 1e-6;
		}
		if (useMag && vqf.getMagDistDetected()) {
			magDist++;
		}
	}

	vqf_real_t bias[3];
	vqf.getBiasEstimate(bias);
	r.finalBiasDps = Vec3{rad2deg(bias[0]), rad2deg(bias[1]), rad2deg(bias[2])};
	if (!ds.samples.empty()) {
		r.magDistFraction
			= static_cast<double>(magDist) / static_cast<double>(ds.samples.size());
	}

	return r;
}

Metrics computeMetrics(const Dataset& ds, const RunResult& run) {
	Metrics m;
	m.dataset = ds.name;
	m.sampleCount = ds.samples.size();
	m.durationSec = ds.durationSec();
	m.sampleRateHz = m.durationSec > 0
					   ? static_cast<double>(m.sampleCount - 1) / m.durationSec
					   : 0.0;
	m.firstRestSec = run.firstRestSec;
	m.finalBiasDps = std::sqrt(
		run.finalBiasDps.x * run.finalBiasDps.x
		+ run.finalBiasDps.y * run.finalBiasDps.y
		+ run.finalBiasDps.z * run.finalBiasDps.z
	);

	const size_t n = std::min(ds.samples.size(), run.est.size());
	if (n < 2) {
		return m;
	}

	const uint64_t t0 = ds.samples.front().tUs;

	// --- Locate the longest stationary segment -----------------------------
	// Drift and jitter are only defined while the tracker is not moving.
	//
	// Rest is detected from the *variation* of the gyroscope signal, not its
	// magnitude. A stationary tracker with an uncorrected bias reads a constant
	// non-zero rate -- thresholding the magnitude would classify exactly the
	// trackers we most want to measure as "moving" and silently skip them.
	// Variation is near zero whenever the tracker is still, whatever the bias.
	// Computed over gyroscope-bearing rows only. In an interleaved capture the
	// accelerometer rows carry no angular rate, and treating their zeros as
	// measurements would make every tracker look perfectly still.
	std::vector<size_t> gyrRows;
	gyrRows.reserve(n);
	for (size_t i = 0; i < n; i++) {
		if (ds.samples[i].hasGyr) {
			gyrRows.push_back(i);
		}
	}

	std::vector<bool> atRest(n, false);
	if (!gyrRows.empty()) {
		const size_t g = gyrRows.size();
		const size_t w = std::max<size_t>(
			2,
			static_cast<size_t>(std::lround(0.5 / std::max(ds.gyrTs, 1e-6)))
		);
		std::vector<bool> gyrAtRest(g, false);
		for (size_t j = 0; j < g; j++) {
			const size_t begin = (j > w / 2) ? j - w / 2 : 0;
			const size_t end = std::min(g, begin + w);
			double mx = 0, my = 0, mz = 0;
			for (size_t k = begin; k < end; k++) {
				const Vec3& gv = ds.samples[gyrRows[k]].gyr;
				mx += gv.x;
				my += gv.y;
				mz += gv.z;
			}
			const double cnt = static_cast<double>(end - begin);
			mx /= cnt;
			my /= cnt;
			mz /= cnt;
			double worst = 0;
			for (size_t k = begin; k < end; k++) {
				const Vec3& gv = ds.samples[gyrRows[k]].gyr;
				worst = std::max(worst, std::fabs(gv.x - mx));
				worst = std::max(worst, std::fabs(gv.y - my));
				worst = std::max(worst, std::fabs(gv.z - mz));
			}
			const double meanMagDps = rad2deg(std::sqrt(mx * mx + my * my + mz * mz));
			gyrAtRest[j]
				= rad2deg(worst) < kRestVariationDps && meanMagDps < kRestMagnitudeDps;
		}

		// Project back onto every row: a row inherits the state of the most
		// recent gyroscope row, so a rest segment stays contiguous across the
		// interleaved accelerometer rows inside it.
		bool current = gyrAtRest[0];
		size_t next = 0;
		for (size_t i = 0; i < n; i++) {
			if (next < g && gyrRows[next] == i) {
				current = gyrAtRest[next];
				next++;
			}
			atRest[i] = current;
		}
	}

	size_t bestBegin = 0;
	size_t bestLen = 0;
	{
		size_t curBegin = 0;
		size_t curLen = 0;
		for (size_t i = 0; i < n; i++) {
			if (atRest[i]) {
				if (curLen == 0) {
					curBegin = i;
				}
				curLen++;
				if (curLen > bestLen) {
					bestLen = curLen;
					bestBegin = curBegin;
				}
			} else {
				curLen = 0;
			}
		}
	}

	if (bestLen >= 2) {
		m.restSecondsUsed
			= static_cast<double>(
				  ds.samples[bestBegin + bestLen - 1].tUs - ds.samples[bestBegin].tUs
			  )
			* 1e-6;
	}
	m.hasDriftEstimate = m.restSecondsUsed >= kMinDriftSegmentSec;

	// --- Heading drift -----------------------------------------------------
	// Unwrap the estimated heading over the rest segment and fit a line. The
	// true heading is constant there, so the slope is pure error.
	if (m.hasDriftEstimate) {
		std::vector<double> ts;
		std::vector<double> yaw;
		ts.reserve(bestLen);
		yaw.reserve(bestLen);
		double unwrapped = 0;
		double prev = qHeading(run.est[bestBegin]);
		for (size_t i = bestBegin; i < bestBegin + bestLen; i++) {
			double h = qHeading(run.est[i]);
			unwrapped += wrapPi(h - prev);
			prev = h;
			ts.push_back(static_cast<double>(ds.samples[i].tUs - t0) * 1e-6);
			yaw.push_back(unwrapped);
		}
		m.headingDriftDegPerMin = rad2deg(slope(ts, yaw)) * 60.0;

		// Jitter: angular step between consecutive estimates while stationary,
		// so it isolates high-frequency noise from slow drift.
		std::vector<double> step;
		step.reserve(bestLen - 1);
		for (size_t i = bestBegin + 1; i < bestBegin + bestLen; i++) {
			step.push_back(rad2deg(qAngle(qMul(run.est[i], qConj(run.est[i - 1])))));
		}
		m.jitterDegRms = rms(step);
	}

	// --- Tilt self-consistency --------------------------------------------
	// Rotating the measured specific force into the world frame must yield +z.
	// Any deviation is an error in the estimated inclination. This is exact for
	// a stationary tracker and degrades gracefully under linear acceleration.
	std::vector<double> tilt;
	tilt.reserve(n);
	const Vec3 up{0, 0, 1};
	for (size_t i = 0; i < n; i++) {
		// Only rows that actually carry an accelerometer sample. A zero vector
		// has no direction, so including empty rows would be meaningless.
		if (!ds.samples[i].hasAcc) {
			continue;
		}
		Vec3 world = qRotate(run.est[i], ds.samples[i].acc);
		tilt.push_back(rad2deg(vAngle(world, up)));
	}
	if (!tilt.empty()) {
		m.tiltErrorDegRms = rms(tilt);
		m.tiltErrorDegMax = *std::max_element(tilt.begin(), tilt.end());
	}

	// --- Ground truth ------------------------------------------------------
	if (!ds.hasGroundTruth) {
		return m;
	}
	m.hasGroundTruth = true;

	// Difference rotation in the world frame: d = q_est * q_gt^-1, identity
	// when the estimate is perfect.
	std::vector<Quat> diff;
	diff.reserve(n);
	for (size_t i = 0; i < n; i++) {
		diff.push_back(qMul(run.est[i], qConj(ds.samples[i].gt)));
	}

	// Remove the single best constant heading offset (circular mean of the
	// per-sample heading errors). Without this we would be measuring the
	// arbitrary starting yaw of a 6-DoF estimator, not its accuracy.
	double sumSin = 0, sumCos = 0;
	for (const Quat& d : diff) {
		double h = qHeading(d);
		sumSin += std::sin(h);
		sumCos += std::cos(h);
	}
	const Quat correction = qFromYaw(-std::atan2(sumSin, sumCos));

	std::vector<double> tot;
	std::vector<double> head;
	std::vector<double> inc;
	tot.reserve(n);
	head.reserve(n);
	inc.reserve(n);
	for (const Quat& d : diff) {
		// Applied on the left: a rotation about the world vertical.
		Quat a = qNorm(qMul(correction, d));
		tot.push_back(rad2deg(qAngle(a)));
		head.push_back(rad2deg(std::fabs(qHeading(a))));
		inc.push_back(rad2deg(qInclination(a)));
	}
	m.finalHeadingErrorDeg = head.back();
	m.totalErrorDegRms = rms(tot);
	m.totalErrorDegMax = *std::max_element(tot.begin(), tot.end());
	m.headingErrorDegRms = rms(head);
	m.inclinationErrorDegRms = rms(inc);

	return m;
}

std::string metricsToJson(const Metrics& m) {
	char buf[256];
	std::ostringstream o;
	o << "{\n";

	auto num = [&](const char* key, double v, bool last = false) {
		std::snprintf(buf, sizeof(buf), "  \"%s\": %.6f%s\n", key, v, last ? "" : ",");
		o << buf;
	};

	std::snprintf(buf, sizeof(buf), "  \"dataset\": \"%s\",\n", m.dataset.c_str());
	o << buf;
	std::snprintf(
		buf,
		sizeof(buf),
		"  \"sample_count\": %llu,\n",
		static_cast<unsigned long long>(m.sampleCount)
	);
	o << buf;
	num("duration_sec", m.durationSec);
	num("sample_rate_hz", m.sampleRateHz);
	std::snprintf(
		buf,
		sizeof(buf),
		"  \"has_drift_estimate\": %s,\n",
		m.hasDriftEstimate ? "true" : "false"
	);
	o << buf;
	num("rest_seconds_used", m.restSecondsUsed);
	if (m.hasDriftEstimate) {
		num("heading_drift_deg_per_min", m.headingDriftDegPerMin);
		num("jitter_deg_rms", m.jitterDegRms);
	}
	num("tilt_error_deg_rms", m.tiltErrorDegRms);
	num("tilt_error_deg_max", m.tiltErrorDegMax);
	num("first_rest_sec", m.firstRestSec);
	num("final_bias_dps", m.finalBiasDps);

	std::snprintf(
		buf,
		sizeof(buf),
		"  \"has_ground_truth\": %s%s\n",
		m.hasGroundTruth ? "true" : "false",
		m.hasGroundTruth ? "," : ""
	);
	o << buf;
	if (m.hasGroundTruth) {
		num("total_error_deg_rms", m.totalErrorDegRms);
		num("total_error_deg_max", m.totalErrorDegMax);
		num("heading_error_deg_rms", m.headingErrorDegRms);
		num("inclination_error_deg_rms", m.inclinationErrorDegRms);
		num("final_heading_error_deg", m.finalHeadingErrorDeg, true);
	}

	o << "}";
	return o.str();
}

}  // namespace fb
