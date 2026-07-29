#include "gyroscale.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include "metrics.h"
#include "quatmath.h"

namespace fb {
namespace {

struct GyroSample {
	Vec3 omega;
	double dt = 0;
};

/// One rest-to-rest transition: gravity direction at each end, and the
/// gyroscope samples in between.
struct Transition {
	Vec3 gravityStart;
	Vec3 gravityEnd;
	std::vector<GyroSample> samples;
	double totalRotationRad = 0;
};

Vec3 normalise(const Vec3& v) {
	const double n = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
	if (n <= 0) {
		return Vec3{0, 0, 0};
	}
	return Vec3{v.x / n, v.y / n, v.z / n};
}

/// Marks gyroscope-bearing rows as at rest, using the same two-criteria test as
/// the drift metric: low variation about the local mean *and* low magnitude.
/// Neither works alone -- a constant bias defeats the magnitude test and a
/// steady turn defeats the variation test.
std::vector<bool> findRest(const Dataset& ds, const std::vector<size_t>& gyrRows) {
	const size_t g = gyrRows.size();
	std::vector<bool> rest(g, false);
	if (g == 0) {
		return rest;
	}
	const size_t w = std::max<size_t>(
		2,
		static_cast<size_t>(std::lround(0.25 / std::max(ds.gyrTs, 1e-6)))
	);
	for (size_t j = 0; j < g; j++) {
		const size_t begin = (j > w / 2) ? j - w / 2 : 0;
		const size_t end = std::min(g, begin + w);
		double mx = 0, my = 0, mz = 0;
		for (size_t k = begin; k < end; k++) {
			const Vec3& v = ds.samples[gyrRows[k]].gyr;
			mx += v.x;
			my += v.y;
			mz += v.z;
		}
		const double cnt = static_cast<double>(end - begin);
		mx /= cnt;
		my /= cnt;
		mz /= cnt;
		double worst = 0;
		for (size_t k = begin; k < end; k++) {
			const Vec3& v = ds.samples[gyrRows[k]].gyr;
			worst = std::max(worst, std::fabs(v.x - mx));
			worst = std::max(worst, std::fabs(v.y - my));
			worst = std::max(worst, std::fabs(v.z - mz));
		}
		const double magDps = rad2deg(std::sqrt(mx * mx + my * my + mz * mz));
		rest[j] = rad2deg(worst) < kRestVariationDps && magDps < kRestMagnitudeDps;
	}
	return rest;
}

/// Mean accelerometer direction over a row range, or a zero vector if the range
/// carries no accelerometer samples.
Vec3 meanGravity(const Dataset& ds, size_t rowBegin, size_t rowEnd) {
	Vec3 sum{0, 0, 0};
	size_t n = 0;
	for (size_t i = rowBegin; i <= rowEnd && i < ds.samples.size(); i++) {
		if (!ds.samples[i].hasAcc) {
			continue;
		}
		sum.x += ds.samples[i].acc.x;
		sum.y += ds.samples[i].acc.y;
		sum.z += ds.samples[i].acc.z;
		n++;
	}
	if (n == 0) {
		return Vec3{0, 0, 0};
	}
	return normalise(Vec3{sum.x / n, sum.y / n, sum.z / n});
}

/// Mean gravity-prediction error over all transitions, in radians.
double residualFor(const std::vector<Transition>& ts, const double scale[3]) {
	double total = 0;
	for (const Transition& t : ts) {
		Quat q{1, 0, 0, 0};
		for (const GyroSample& s : t.samples) {
			const Vec3 w{
				s.omega.x * scale[0],
				s.omega.y * scale[1],
				s.omega.z * scale[2],
			};
			q = qIntegrate(q, w, s.dt);
		}
		// Gravity is fixed in the world, so where it lands in the body frame is
		// determined entirely by how the body turned.
		const Vec3 predicted = qRotateInv(q, t.gravityStart);
		total += vAngle(predicted, t.gravityEnd);
	}
	return ts.empty() ? 0.0 : total / static_cast<double>(ts.size());
}

/// Ternary search on one axis. The residual is smooth and unimodal in the
/// neighbourhood of the true scale, which is all this needs.
void refineAxis(const std::vector<Transition>& ts, double scale[3], int axis) {
	double lo = 0.90;
	double hi = 1.10;
	for (int i = 0; i < 60; i++) {
		const double a = lo + (hi - lo) / 3.0;
		const double b = hi - (hi - lo) / 3.0;
		double trial[3] = {scale[0], scale[1], scale[2]};
		trial[axis] = a;
		const double ra = residualFor(ts, trial);
		trial[axis] = b;
		const double rb = residualFor(ts, trial);
		if (ra < rb) {
			hi = b;
		} else {
			lo = a;
		}
	}
	scale[axis] = 0.5 * (lo + hi);
}

}  // namespace

GyroScaleResult estimateGyroScale(const Dataset& ds) {
	GyroScaleResult r;

	std::vector<size_t> gyrRows;
	gyrRows.reserve(ds.samples.size());
	for (size_t i = 0; i < ds.samples.size(); i++) {
		if (ds.samples[i].hasGyr) {
			gyrRows.push_back(i);
		}
	}
	if (gyrRows.size() < 100) {
		r.reason = "too few gyroscope samples";
		return r;
	}

	const std::vector<bool> rest = findRest(ds, gyrRows);

	// Collapse the per-sample rest flags into runs, keeping only rest periods
	// long enough for the accelerometer reading to have settled.
	struct Run {
		size_t beginIdx;
		size_t endIdx;
	};
	std::vector<Run> restRuns;
	size_t runStart = 0;
	bool inRun = false;
	for (size_t j = 0; j < rest.size(); j++) {
		if (rest[j] && !inRun) {
			inRun = true;
			runStart = j;
		} else if (!rest[j] && inRun) {
			inRun = false;
			const double secs
				= static_cast<double>(j - runStart) * std::max(ds.gyrTs, 1e-9);
			if (secs >= kRestSegmentSeconds) {
				restRuns.push_back({runStart, j - 1});
			}
		}
	}
	if (inRun) {
		const double secs
			= static_cast<double>(rest.size() - runStart) * std::max(ds.gyrTs, 1e-9);
		if (secs >= kRestSegmentSeconds) {
			restRuns.push_back({runStart, rest.size() - 1});
		}
	}

	if (restRuns.size() < 2) {
		r.reason = "fewer than two settled rest periods; the capture needs pauses";
		return r;
	}

	// Build one transition per adjacent pair of rest periods.
	std::vector<Transition> transitions;
	for (size_t k = 0; k + 1 < restRuns.size(); k++) {
		Transition t;
		t.gravityStart = meanGravity(
			ds,
			gyrRows[restRuns[k].beginIdx],
			gyrRows[restRuns[k].endIdx]
		);
		t.gravityEnd = meanGravity(
			ds,
			gyrRows[restRuns[k + 1].beginIdx],
			gyrRows[restRuns[k + 1].endIdx]
		);
		if (t.gravityStart.x == 0 && t.gravityStart.y == 0 && t.gravityStart.z == 0) {
			continue;
		}
		if (t.gravityEnd.x == 0 && t.gravityEnd.y == 0 && t.gravityEnd.z == 0) {
			continue;
		}

		for (size_t j = restRuns[k].endIdx; j <= restRuns[k + 1].beginIdx; j++) {
			const Vec3& w = ds.samples[gyrRows[j]].gyr;
			const double dt = ds.gyrTs;
			t.samples.push_back(GyroSample{w, dt});
			t.totalRotationRad += std::sqrt(w.x * w.x + w.y * w.y + w.z * w.z) * dt;
		}

		// A transition that barely turned carries almost no scale information
		// but full measurement noise, so it would only dilute the fit.
		if (rad2deg(t.totalRotationRad) < kMinTransitionDeg) {
			continue;
		}
		transitions.push_back(std::move(t));
	}

	r.segments = static_cast<int>(transitions.size());
	if (r.segments < kMinScaleSegments) {
		r.reason = "too few rest-to-rest transitions with enough rotation";
		return r;
	}

	const double unity[3] = {1.0, 1.0, 1.0};
	r.residualBeforeDeg = rad2deg(residualFor(transitions, unity));

	double scale[3] = {1.0, 1.0, 1.0};
	// A few rounds of coordinate descent. The axes couple only weakly, so this
	// converges quickly and avoids needing derivatives.
	for (int round = 0; round < 4; round++) {
		for (int axis = 0; axis < 3; axis++) {
			refineAxis(transitions, scale, axis);
		}
	}

	const double fitted = residualFor(transitions, scale);
	r.residualAfterDeg = rad2deg(fitted);
	for (int i = 0; i < 3; i++) {
		r.scale[i] = scale[i];
	}

	// Observability: how much worse the fit gets when an axis is perturbed by
	// 1%. If the answer is "not at all", the capture never constrained that
	// axis, and the number returned for it is meaningless however good the
	// residual looks overall.
	for (int i = 0; i < 3; i++) {
		double probe[3] = {scale[0], scale[1], scale[2]};
		probe[i] = scale[i] * 1.01;
		r.observability[i]
			= rad2deg(residualFor(transitions, probe)) - r.residualAfterDeg;
	}

	for (int i = 0; i < 3; i++) {
		if (r.observability[i] < kMinScaleObservability) {
			static const char* names[3] = {"X", "Y", "Z"};
			r.reason = std::string("axis ") + names[i]
					 + " was never rotated in a way that moves gravity";
			return r;
		}
	}

	r.valid = true;
	return r;
}

}  // namespace fb
