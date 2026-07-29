/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2026 SlimeVR Contributors

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
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
	FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
	IN THE SOFTWARE.
*/

#pragma once

#include <cmath>
#include <cstddef>

// Free of any Arduino or hardware dependency so the arithmetic can be unit
// tested on a host. See tools/fusion-bench/tests/selftest.cpp.

namespace SlimeVR::Sensors::SoftFusion {

/**
 * The full first-order sensor error model: `corrected = M * (raw - bias)`.
 *
 * The runtime calibration currently models bias only. Bias is the largest term
 * and its treatment here is good -- rest-gated estimation with temperature
 * compensation removes about three orders of magnitude of gyroscope drift, as
 * measured. But bias is the term that dominates a *stationary* sensor, and a
 * body tracker is strapped to a moving limb.
 *
 * The terms this adds are the ones that dominate when moving:
 *
 *  - **Scale factor** produces error proportional to rotation *angle*, not to
 *    time. Turn 180 degrees with a 1% scale error and you are 1.8 degrees out
 *    immediately, however good the bias estimate is. Rest-gated calibration
 *    cannot see this at all, because at rest there is no rotation to scale.
 *
 *  - **Axis misalignment** couples motion between axes, turning pitch and roll
 *    into spurious yaw. That is the most damaging possible direction for the
 *    error to go, because yaw is the unobservable axis on a 6-DoF tracker.
 *
 * Both are systematic, so they accumulate; noise does not.
 *
 * `M` defaults to identity, so a device with no fitted model behaves exactly as
 * before and the storage can land ahead of the estimation.
 */
struct ErrorModel {
	/// Row-major 3x3. Identity by default.
	float m[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
	float bias[3] = {0, 0, 0};

	[[nodiscard]] bool isIdentity() const {
		constexpr float eps = 1e-6f;
		const float ident[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
		for (int i = 0; i < 9; i++) {
			if (std::fabs(m[i] - ident[i]) > eps) {
				return false;
			}
		}
		return std::fabs(bias[0]) <= eps && std::fabs(bias[1]) <= eps
			&& std::fabs(bias[2]) <= eps;
	}

	void apply(const float in[3], float out[3]) const {
		const float d0 = in[0] - bias[0];
		const float d1 = in[1] - bias[1];
		const float d2 = in[2] - bias[2];
		out[0] = m[0] * d0 + m[1] * d1 + m[2] * d2;
		out[1] = m[3] * d0 + m[4] * d1 + m[5] * d2;
		out[2] = m[6] * d0 + m[7] * d1 + m[8] * d2;
	}
};

namespace detail {

/// Solves `a x = b` for a small dense system. Returns false if singular.
inline bool solveLinear(double* a, double* b, int n) {
	for (int col = 0; col < n; col++) {
		int pivot = col;
		for (int r = col; r < n; r++) {
			if (std::fabs(a[r * n + col]) > std::fabs(a[pivot * n + col])) {
				pivot = r;
			}
		}
		if (std::fabs(a[pivot * n + col]) < 1e-14) {
			return false;
		}
		if (pivot != col) {
			for (int k = 0; k < n; k++) {
				const double t = a[col * n + k];
				a[col * n + k] = a[pivot * n + k];
				a[pivot * n + k] = t;
			}
			const double t = b[col];
			b[col] = b[pivot];
			b[pivot] = t;
		}
		for (int r = 0; r < n; r++) {
			if (r == col) {
				continue;
			}
			const double f = a[r * n + col] / a[col * n + col];
			if (f == 0.0) {
				continue;
			}
			for (int k = col; k < n; k++) {
				a[r * n + k] -= f * a[col * n + k];
			}
			b[r] -= f * b[col];
		}
	}
	for (int i = 0; i < n; i++) {
		b[i] /= a[i * n + i];
	}
	return true;
}

/// Lower-triangular Cholesky of a symmetric positive-definite 3x3.
inline bool cholesky3(const double q[9], double l[9]) {
	for (int i = 0; i < 9; i++) {
		l[i] = 0.0;
	}
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j <= i; j++) {
			double sum = q[i * 3 + j];
			for (int k = 0; k < j; k++) {
				sum -= l[i * 3 + k] * l[j * 3 + k];
			}
			if (i == j) {
				if (sum <= 0.0) {
					return false;
				}
				l[i * 3 + j] = std::sqrt(sum);
			} else {
				l[i * 3 + j] = sum / l[j * 3 + j];
			}
		}
	}
	return true;
}

/**
 * Smallest eigenvalue of a symmetric positive-semidefinite 3x3, closed form.
 *
 * Used to ask how well the sample directions span three dimensions. An
 * iterative solver would be overkill for a 3x3, and this runs once at
 * calibration time rather than in the sample path.
 */
inline double smallestEigenvalue3(const double a[9]) {
	const double p1 = a[1] * a[1] + a[2] * a[2] + a[5] * a[5];
	const double tr = a[0] + a[4] + a[8];
	if (p1 <= 0.0) {
		double m = a[0];
		if (a[4] < m) {
			m = a[4];
		}
		if (a[8] < m) {
			m = a[8];
		}
		return m;
	}
	const double q = tr / 3.0;
	const double p2 = (a[0] - q) * (a[0] - q) + (a[4] - q) * (a[4] - q)
					+ (a[8] - q) * (a[8] - q) + 2.0 * p1;
	const double p = std::sqrt(p2 / 6.0);
	if (p <= 0.0) {
		return q;
	}
	double b[9];
	for (int i = 0; i < 9; i++) {
		b[i] = a[i] / p;
	}
	b[0] -= q / p;
	b[4] -= q / p;
	b[8] -= q / p;
	const double detB = b[0] * (b[4] * b[8] - b[5] * b[7])
					  - b[1] * (b[3] * b[8] - b[5] * b[6])
					  + b[2] * (b[3] * b[7] - b[4] * b[6]);
	double r = detB / 2.0;
	if (r < -1.0) {
		r = -1.0;
	} else if (r > 1.0) {
		r = 1.0;
	}
	const double phi = std::acos(r) / 3.0;
	constexpr double twoPiOver3 = 2.0943951023931953;
	return q + 2.0 * p * std::cos(phi + twoPiOver3);
}

/**
 * True if the sample directions span three dimensions well enough to fit.
 *
 * Directions rather than magnitudes, so a sensor with a large bias is not
 * mistaken for poor coverage: it is where the sensor was *pointed* that
 * determines whether the ellipsoid is observable.
 *
 * @param minUsed  Samples with a usable magnitude required, i.e. the number of
 *                 unknowns the caller's fit has.
 */
inline bool spansThreeDimensions(
	const float* samples,
	size_t count,
	size_t minUsed,
	double minSpread
) {
	double cov[9] = {0};
	size_t used = 0;
	for (size_t i = 0; i < count; i++) {
		const double x = samples[i * 3 + 0];
		const double y = samples[i * 3 + 1];
		const double z = samples[i * 3 + 2];
		const double len = std::sqrt(x * x + y * y + z * z);
		if (len < 1e-9) {
			continue;
		}
		const double d[3] = {x / len, y / len, z / len};
		for (int r = 0; r < 3; r++) {
			for (int c = 0; c < 3; c++) {
				cov[r * 3 + c] += d[r] * d[c];
			}
		}
		used++;
	}
	if (used < minUsed) {
		return false;
	}
	for (int i = 0; i < 9; i++) {
		cov[i] /= static_cast<double>(used);
	}
	return smallestEigenvalue3(cov) >= minSpread;
}

/**
 * True if every axis was measured pointing both up and down.
 *
 * This exists because direction spread, which is what the quadric fit checks,
 * cannot see the failure it guards. Take the six-position procedure and drop
 * one position -- say -Z, keeping +Z twice. The direction covariance is
 * unchanged at a perfect 1/3 on every axis, so the spread check is entirely
 * happy. The diagonal fit is nonetheless singular: its Z unknowns appear only
 * as `a3 z^2 - 2 c3 z`, the in-plane rows have `z = 0` and contribute nothing
 * to either, and the surviving `+Z` rows are all identical -- one equation for
 * two unknowns.
 *
 * Whether that is caught by the linear solve is a matter of floating-point
 * luck, and it duly differed between compilers: clang admitted the set and
 * returned a confidently wrong Z scale, gcc refused it. Either behaviour is
 * unacceptable for something applied to every subsequent sample, so the
 * condition is tested directly rather than left to a pivot threshold.
 *
 * Slightly stricter than the algebra strictly requires -- two distinct
 * same-sign magnitudes would also separate the unknowns, just badly -- and
 * deliberately so, because "each axis up and each axis down" is exactly the
 * procedure being asked of the user.
 *
 * @param minComponent  How large a coordinate must be to count as pointing
 *                      along that axis rather than lying across it.
 */
inline bool
eachAxisSeenBothWays(const float* samples, size_t count, float minComponent) {
	bool positive[3] = {false, false, false};
	bool negative[3] = {false, false, false};
	for (size_t i = 0; i < count; i++) {
		for (int axis = 0; axis < 3; axis++) {
			const float v = samples[i * 3 + axis];
			if (v > minComponent) {
				positive[axis] = true;
			} else if (v < -minComponent) {
				negative[axis] = true;
			}
		}
	}
	for (int axis = 0; axis < 3; axis++) {
		if (!positive[axis] || !negative[axis]) {
			return false;
		}
	}
	return true;
}

}  // namespace detail

/**
 * Fraction of the expected magnitude a coordinate must reach to count as
 * measuring that axis.
 *
 * 0.2 -- about 11.5 degrees above horizontal. Far below what the six-position
 * procedure produces (a hold within 20 degrees of vertical puts 0.94 on the
 * axis) and far above the rounding noise of a coordinate that is nominally
 * zero, so it separates "pointed along this axis" from "lay across it" without
 * being a second alignment tolerance.
 */
constexpr float kMinAxisComponent = 0.2f;

/**
 * Minimum directional spread required before a fit is accepted.
 *
 * The smallest eigenvalue of the covariance of the sample directions: 1/3 for
 * perfectly isotropic coverage, 0 when the directions are coplanar. 0.05
 * admits the classic six-position procedure with generous hand-placement error
 * while rejecting a set that never left one plane.
 */
constexpr double kMinDirectionSpread = 0.05;

/**
 * Fits an error model to static measurements taken in many orientations.
 *
 * The physical fact this exploits: however the sensor is oriented, a stationary
 * accelerometer measures a vector of constant magnitude. Uncorrected readings
 * therefore trace an *ellipsoid* rather than a sphere, and the ellipsoid's
 * centre is the bias while its shape is the combined scale and misalignment.
 * Recovering both is a quadric fit.
 *
 * Requires genuinely varied orientations. Samples clustered near one attitude
 * leave the fit under-determined -- it will still return numbers, so the caller
 * has to supply good data rather than relying on a failure. The classic
 * procedure is six positions, each axis up and down.
 *
 * @param samples  `3 * count` floats, xyz per sample, in raw scaled units.
 * @param count    Number of samples. At least 9 for the quadric to be
 *                 determined; far more in practice.
 * @param norm     Expected magnitude, e.g. gravity in m/s^2.
 */
inline bool
fitErrorModel(const float* samples, size_t count, float norm, ErrorModel& out) {
	if (count < 9 || norm <= 0.0f) {
		return false;
	}

	// Reject sample sets that do not actually span three dimensions.
	//
	// The quadric has nine unknowns, so nine samples make it *nominally*
	// determined -- but samples clustered near one attitude, or confined to a
	// plane, leave it near-singular. The solve still succeeds and returns a
	// confidently wrong model, which is worse than no calibration at all
	// because it is then applied to every subsequent sample.
	if (!detail::spansThreeDimensions(samples, count, 9, kMinDirectionSpread)) {
		return false;
	}

	// Quadric: x'Qx + u'x = 1, nine unknowns after fixing the constant. Solving
	// the normal equations directly is adequate here -- the design matrix is
	// well conditioned once the orientations are varied, and this runs once at
	// calibration time rather than in the sample path.
	double ata[81] = {0};
	double atb[9] = {0};
	for (size_t s = 0; s < count; s++) {
		const double x = samples[s * 3 + 0];
		const double y = samples[s * 3 + 1];
		const double z = samples[s * 3 + 2];
		const double row[9]
			= {x * x, y * y, z * z, 2 * x * y, 2 * x * z, 2 * y * z, x, y, z};
		for (int i = 0; i < 9; i++) {
			for (int j = 0; j < 9; j++) {
				ata[i * 9 + j] += row[i] * row[j];
			}
			atb[i] += row[i];
		}
	}
	if (!detail::solveLinear(ata, atb, 9)) {
		return false;
	}

	const double qh[9] = {
		atb[0],
		atb[3],
		atb[4],
		atb[3],
		atb[1],
		atb[5],
		atb[4],
		atb[5],
		atb[2],
	};
	const double uh[3] = {atb[6], atb[7], atb[8]};

	// Centre: b = -0.5 * Q^-1 u. The unknown overall scale cancels here, which
	// is why the bias can be recovered before the scale is known.
	double qinv[9];
	for (int i = 0; i < 9; i++) {
		qinv[i] = qh[i];
	}
	double rhs[3] = {-0.5 * uh[0], -0.5 * uh[1], -0.5 * uh[2]};
	if (!detail::solveLinear(qinv, rhs, 3)) {
		return false;
	}
	const double b[3] = {rhs[0], rhs[1], rhs[2]};

	// Recover the scale the constant-term normalisation threw away.
	double bqb = 0.0;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			bqb += b[i] * qh[i * 3 + j] * b[j];
		}
	}
	const double denom = 1.0 + bqb;
	if (std::fabs(denom) < 1e-12) {
		return false;
	}
	const double scale = static_cast<double>(norm) * norm / denom;

	double q[9];
	for (int i = 0; i < 9; i++) {
		q[i] = qh[i] * scale;
	}

	// M with M'M = Q. Cholesky picks one of the many valid square roots; they
	// differ by a rotation, which is unobservable from magnitudes alone and is
	// handled by the existing mounting calibration rather than here.
	double l[9];
	if (!detail::cholesky3(q, l)) {
		return false;
	}

	// M = L', so that M'M = L L' = Q.
	out.m[0] = static_cast<float>(l[0]);
	out.m[1] = static_cast<float>(l[3]);
	out.m[2] = static_cast<float>(l[6]);
	out.m[3] = 0.0f;
	out.m[4] = static_cast<float>(l[4]);
	out.m[5] = static_cast<float>(l[7]);
	out.m[6] = 0.0f;
	out.m[7] = 0.0f;
	out.m[8] = static_cast<float>(l[8]);
	out.bias[0] = static_cast<float>(b[0]);
	out.bias[1] = static_cast<float>(b[1]);
	out.bias[2] = static_cast<float>(b[2]);
	return true;
}

/**
 * The normal equations for the diagonal (bias + scale) fit, in streaming form.
 *
 * The batch fit builds these by summing one rank-1 update per sample and then
 * solving. Nothing in that sum depends on the samples being available together,
 * which is the whole reason on-device online estimation is possible at all: the
 * accumulator *is* the recursive estimator. Twenty-seven numbers replace an
 * unbounded sample history, and a tracker can keep learning across a session it
 * has no room to record.
 *
 * `double` is not defensive here. The rows contain `x^2`, so `A'A` contains
 * `x^4` -- around 9200 for a 1 g reading in m/s^2. Accumulate a few hundred
 * thousand of those in `float` and the running sum crosses the point where
 * adding one more changes nothing, and the estimate silently freezes. That
 * failure is invisible: no overflow, no NaN, just an estimator that stops
 * responding.
 */
struct DiagonalNormalEquations {
	/// Symmetric 6x6, upper triangle packed by rows.
	double ata[21] = {0};
	double atb[6] = {0};
	/// Effective sample count. Fractional once forgetting has been applied.
	double weight = 0.0;

	static constexpr int packedIndex(int i, int j) {
		// Row-major upper triangle: row i starts after the rows above it.
		return i * 6 - (i * (i - 1)) / 2 + (j - i);
	}

	/// Adds one sample, in the same units as the `norm` later passed to `solve`.
	void add(const float sample[3], double sampleWeight = 1.0) {
		const double x = sample[0];
		const double y = sample[1];
		const double z = sample[2];
		const double row[6] = {x * x, y * y, z * z, -2 * x, -2 * y, -2 * z};
		for (int i = 0; i < 6; i++) {
			for (int j = i; j < 6; j++) {
				ata[packedIndex(i, j)] += sampleWeight * row[i] * row[j];
			}
			atb[i] += sampleWeight * row[i];
		}
		weight += sampleWeight;
	}

	/**
	 * Discounts everything accumulated so far by `factor`.
	 *
	 * Exponential forgetting, the standard recursive-least-squares device. It
	 * matters because the two things being estimated do not age alike: scale
	 * factor is a property of the part and stable over its life, while bias
	 * drifts with temperature and time. Without forgetting, a reading taken
	 * hours ago at a different temperature counts exactly as much as one taken
	 * now, and the bias estimate is an average over conditions that no longer
	 * hold rather than an estimate of the current one.
	 */
	void forget(double factor) {
		for (double& v : ata) {
			v *= factor;
		}
		for (double& v : atb) {
			v *= factor;
		}
		weight *= factor;
	}

	void reset() { *this = DiagonalNormalEquations{}; }

	/**
	 * Solves for bias and scale.
	 *
	 * Returns false if the accumulated system is singular or implies a
	 * non-physical sensor. Says nothing about whether the *coverage* was
	 * adequate -- that is a separate question, asked separately, because a
	 * well-conditioned solve on a badly covered sample set is exactly the
	 * failure mode that looks like success.
	 *
	 * @param norm  Expected magnitude, e.g. gravity in m/s^2.
	 */
	[[nodiscard]] bool solve(float norm, ErrorModel& out) const {
		if (!(norm > 0.0f) || weight <= 0.0) {
			return false;
		}

		double full[36];
		for (int i = 0; i < 6; i++) {
			for (int j = i; j < 6; j++) {
				const double v = ata[packedIndex(i, j)];
				full[i * 6 + j] = v;
				full[j * 6 + i] = v;
			}
		}
		double rhs[6];
		for (int i = 0; i < 6; i++) {
			rhs[i] = atb[i];
		}
		if (!detail::solveLinear(full, rhs, 6)) {
			return false;
		}

		const double a[3] = {rhs[0], rhs[1], rhs[2]};
		const double c[3] = {rhs[3], rhs[4], rhs[5]};
		for (const double v : a) {
			// a_i is s_i^2 up to a positive factor. Non-positive means the fit
			// did not find an ellipsoid, so there is no real scale to report.
			if (!(v > 0.0)) {
				return false;
			}
		}

		const double b[3] = {c[0] / a[0], c[1] / a[1], c[2] / a[2]};

		double sum = 0.0;
		for (int i = 0; i < 3; i++) {
			sum += a[i] * b[i] * b[i];
		}
		const double denom = 1.0 + sum;
		if (std::fabs(denom) < 1e-12) {
			return false;
		}

		for (int i = 0; i < 9; i++) {
			out.m[i] = 0.0f;
		}
		for (int i = 0; i < 3; i++) {
			const double scaleSquared = a[i] * static_cast<double>(norm) * norm / denom;
			if (!(scaleSquared > 0.0)) {
				return false;
			}
			out.m[i * 4] = static_cast<float>(std::sqrt(scaleSquared));
			out.bias[i] = static_cast<float>(b[i]);
		}
		return true;
	}
};

/**
 * Fits bias and per-axis scale only, leaving the matrix diagonal.
 *
 * This exists because of a fact about the six-position procedure that is easy
 * to state and easy to miss: **six perfectly executed axis-aligned positions
 * carry no information about misalignment at all.**
 *
 * The full quadric's design matrix has columns for `xy`, `xz` and `yz`. Hold
 * the sensor with exactly one axis vertical and the other two read zero, so
 * every one of those products is zero, and all three columns vanish. The
 * solve is then singular -- not merely ill-conditioned. What rescues it in
 * practice is hand-placement error, which means the misalignment terms a
 * six-position fit reports are estimated from how badly the user held the
 * tracker. That is not a measurement, and cross-axis terms fitted from noise
 * are the most damaging possible kind of error to invent: they turn pitch and
 * roll into spurious yaw, the unobservable axis on a 6-DoF tracker.
 *
 * So the guided on-device flow fits what six positions genuinely determine --
 * bias and scale, six unknowns from six distinct orientations, well conditioned
 * -- and declines to guess the rest. Misalignment remains available through the
 * host path (`SET LOGRAW` plus `fusion-bench`), where a capture can cover
 * orientations off the axes and the cross terms are actually observable.
 *
 * The algebra is the same trick as the full fit. A stationary accelerometer
 * satisfies `sum_i s_i^2 (x_i - b_i)^2 = norm^2`; expanding gives a linear
 * system in `s_i^2` and `s_i^2 b_i` once the constant is normalised away, and
 * the overall scale that normalisation discards is recovered afterwards.
 *
 * @param samples  `3 * count` floats, xyz per sample, in raw scaled units.
 * @param count    Number of samples. At least 6.
 * @param norm     Expected magnitude, e.g. gravity in m/s^2.
 */
inline bool
fitErrorModelDiagonal(const float* samples, size_t count, float norm, ErrorModel& out) {
	if (count < 6 || norm <= 0.0f) {
		return false;
	}
	if (!detail::spansThreeDimensions(samples, count, 6, kMinDirectionSpread)) {
		return false;
	}
	if (!detail::eachAxisSeenBothWays(samples, count, kMinAxisComponent * norm)) {
		return false;
	}

	DiagonalNormalEquations normal;
	for (size_t s = 0; s < count; s++) {
		normal.add(&samples[s * 3]);
	}
	return normal.solve(norm, out);
}

}  // namespace SlimeVR::Sensors::SoftFusion
