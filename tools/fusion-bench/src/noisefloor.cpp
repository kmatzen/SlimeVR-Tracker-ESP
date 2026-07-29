#include "noisefloor.h"

#include <cmath>
#include <cstdint>
#include <deque>
#include <vector>

#include "vqf.h"

namespace fb {

namespace {

// Per-axis noise from successive differences. For white noise,
// var(x[n] - x[n-1]) = 2 * var(x), so sigma = stddev(diff) / sqrt(2).
//
// The differencing is what makes this usable on a capture that is not perfectly
// still: a constant bias cancels, and so does any drift slow compared with the
// sample rate. Taking a plain standard deviation of the raw signal instead would
// fold thermal drift and residual tilt into the "noise".
void axisNoiseFromDifferences(const std::vector<double>& v, double& sigma) {
	sigma = 0;
	if (v.size() < 2) {
		return;
	}
	double sum = 0;
	double sumSq = 0;
	const size_t n = v.size() - 1;
	for (size_t i = 1; i < v.size(); i++) {
		const double d = v[i] - v[i - 1];
		sum += d;
		sumSq += d * d;
	}
	const double mean = sum / static_cast<double>(n);
	const double var = sumSq / static_cast<double>(n) - mean * mean;
	sigma = var > 0 ? std::sqrt(var / 2.0) : 0.0;
}

// Sliding-window minimum of the window maximum, over windows of `w` samples.
//
// This is the cliff: rest needs `w` consecutive samples all strictly under the
// threshold, so the smallest threshold that admits rest anywhere in the capture
// is the quietest window's peak. Monotonic deque, O(n).
double minWindowMax(const std::vector<double>& v, size_t w) {
	if (w == 0 || v.size() < w) {
		return 0.0;
	}
	std::deque<size_t> dq;  // indices, values decreasing
	double best = 0;
	bool haveBest = false;
	for (size_t i = 0; i < v.size(); i++) {
		while (!dq.empty() && v[dq.back()] <= v[i]) {
			dq.pop_back();
		}
		dq.push_back(i);
		while (dq.front() + w <= i) {
			dq.pop_front();
		}
		if (i + 1 >= w) {
			const double windowMax = v[dq.front()];
			if (!haveBest || windowMax < best) {
				best = windowMax;
				haveBest = true;
			}
		}
	}
	return haveBest ? best : 0.0;
}

}  // namespace

AccelNoiseResult measureAccelNoise(const Dataset& ds, const BenchParams& p) {
	AccelNoiseResult r;

	VQFParams params;  // VQF's own defaults
	if (!p.stock) {
		params.tauAcc = static_cast<vqf_real_t>(p.tauAcc);
		params.restMinT = static_cast<vqf_real_t>(p.restMinT);
		params.restThGyr = static_cast<vqf_real_t>(p.restThGyr);
		params.restThAcc = static_cast<vqf_real_t>(p.restThAcc);
	}

	r.configuredThreshold = static_cast<double>(params.restThAcc);

	if (ds.samples.size() < 2 || ds.accTs <= 0) {
		r.reason = "capture has no usable accelerometer timing";
		return r;
	}

	// N = restMinT / accTs, the run of consecutive under-threshold samples rest
	// requires. Rounded up: a partial sample does not extend the run.
	r.restWindowSamples
		= static_cast<size_t>(std::ceil(static_cast<double>(params.restMinT) / ds.accTs)
		);

	VQF vqf(
		params,
		static_cast<vqf_real_t>(ds.gyrTs),
		static_cast<vqf_real_t>(ds.accTs),
		static_cast<vqf_real_t>(ds.magTs)
	);

	std::vector<double> ax, ay, az, residual;
	ax.reserve(ds.samples.size());
	ay.reserve(ds.samples.size());
	az.reserve(ds.samples.size());
	residual.reserve(ds.samples.size());

	uint64_t prevGyrT = ds.samples.front().tUs;
	bool haveGyrT = false;

	for (const Sample& s : ds.samples) {
		// Timestep handling mirrors runFusion() in metrics.cpp: measured between
		// consecutive gyroscope rows, because a capture interleaves the two
		// sensors at different rates.
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

		if (s.hasAcc) {
			vqf_real_t acc[3] = {
				static_cast<vqf_real_t>(s.acc.x),
				static_cast<vqf_real_t>(s.acc.y),
				static_cast<vqf_real_t>(s.acc.z),
			};
			vqf.updateAcc(acc);

			// Read the residual back out of VQF rather than recomputing it. The
			// accessor returns the deviation divided by the threshold, so
			// multiplying by the threshold recovers it in m/s^2.
			vqf_real_t dev[2];
			vqf.getRelativeRestDeviations(dev);
			residual.push_back(
				static_cast<double>(dev[1]) * static_cast<double>(params.restThAcc)
			);

			ax.push_back(s.acc.x);
			ay.push_back(s.acc.y);
			az.push_back(s.acc.z);
		}
		if (s.hasGyr) {
			vqf_real_t gyr[3] = {
				static_cast<vqf_real_t>(s.gyr.x),
				static_cast<vqf_real_t>(s.gyr.y),
				static_cast<vqf_real_t>(s.gyr.z),
			};
			vqf.updateGyr(gyr, static_cast<vqf_real_t>(dt));
		}
	}

	r.accelSamples = residual.size();
	if (r.accelSamples < kMinNoiseSamples) {
		r.reason = "too few accelerometer samples to measure noise";
		return r;
	}

	axisNoiseFromDifferences(ax, r.axisSigma[0]);
	axisNoiseFromDifferences(ay, r.axisSigma[1]);
	axisNoiseFromDifferences(az, r.axisSigma[2]);
	r.vectorSigma = std::sqrt(
		r.axisSigma[0] * r.axisSigma[0] + r.axisSigma[1] * r.axisSigma[1]
		+ r.axisSigma[2] * r.axisSigma[2]
	);

	// Skip the filter's initialisation phase before looking at residuals. For
	// the first restFilterTau seconds VQF's "low-pass output" is a plain running
	// mean of every sample so far (filterVec in vqf.cpp), so the residual there
	// measures how far the mean has settled, not sensor noise.
	const size_t skip = static_cast<size_t>(
		std::ceil(static_cast<double>(params.restFilterTau) / ds.accTs)
	);
	std::vector<double> settled;
	if (residual.size() > skip) {
		settled.assign(residual.begin() + static_cast<long>(skip), residual.end());
	}
	if (settled.size() < r.restWindowSamples) {
		r.reason = "capture is shorter than one rest window after filter settling";
		return r;
	}

	double sumSq = 0;
	for (const double d : settled) {
		sumSq += d * d;
		if (d > r.residualPeak) {
			r.residualPeak = d;
		}
	}
	r.residualRms = std::sqrt(sumSq / static_cast<double>(settled.size()));

	// Over the whole series, including the startup transient: this is what
	// decides whether rest is ever reached at all.
	r.requiredThreshold = minWindowMax(residual, r.restWindowSamples);
	// Over settled samples only: the number to configure against.
	r.settledThreshold = minWindowMax(settled, r.restWindowSamples);

	r.sigmaMultiple = r.vectorSigma > 0 ? r.settledThreshold / r.vectorSigma : 0.0;
	r.valid = r.requiredThreshold > 0 && r.settledThreshold > 0;
	if (!r.valid) {
		r.reason = "no window produced a positive residual bound";
	}
	return r;
}

}  // namespace fb
